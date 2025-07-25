// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2007 Oracle.  All rights reserved.
 */

#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/writeback.h>
#include <linux/pagemap.h>
#include <linux/blkdev.h>
#include <linux/uuid.h>
#include "misc.h"
#include "ctree.h"
#include "disk-io.h"
#include "transaction.h"
#include "locking.h"
#include "tree-log.h"
#include "inode-map.h"
#include "volumes.h"
#include "dev-replace.h"
#include "qgroup.h"
#include "block-group.h"
#include "space-info.h"

#define BTRFS_ROOT_TRANS_TAG 0

static const unsigned int btrfs_blocked_trans_types[TRANS_STATE_MAX] = {
	[TRANS_STATE_RUNNING]		= 0U,
	[TRANS_STATE_COMMIT_START]	= (__TRANS_START | __TRANS_ATTACH),
	[TRANS_STATE_COMMIT_DOING]	= (__TRANS_START |
					   __TRANS_ATTACH |
					   __TRANS_JOIN |
					   __TRANS_JOIN_NOSTART),
	[TRANS_STATE_UNBLOCKED]		= (__TRANS_START |
					   __TRANS_ATTACH |
					   __TRANS_JOIN |
					   __TRANS_JOIN_NOLOCK |
					   __TRANS_JOIN_NOSTART),
	[TRANS_STATE_COMPLETED]		= (__TRANS_START |
					   __TRANS_ATTACH |
					   __TRANS_JOIN |
					   __TRANS_JOIN_NOLOCK |
					   __TRANS_JOIN_NOSTART),
};

void btrfs_put_transaction(struct btrfs_transaction *transaction)
{
	WARN_ON(refcount_read(&transaction->use_count) == 0);
	if (refcount_dec_and_test(&transaction->use_count)) {
		BUG_ON(!list_empty(&transaction->list));
		WARN_ON(!RB_EMPTY_ROOT(
				&transaction->delayed_refs.href_root.rb_root));
		WARN_ON(!RB_EMPTY_ROOT(
				&transaction->delayed_refs.dirty_extent_root));
		if (transaction->delayed_refs.pending_csums)
			btrfs_err(transaction->fs_info,
				  "pending csums is %llu",
				  transaction->delayed_refs.pending_csums);
		/*
		 * If any block groups are found in ->deleted_bgs then it's
		 * because the transaction was aborted and a commit did not
		 * happen (things failed before writing the new superblock
		 * and calling btrfs_finish_extent_commit()), so we can not
		 * discard the physical locations of the block groups.
		 */
		while (!list_empty(&transaction->deleted_bgs)) {
			struct btrfs_block_group_cache *cache;

			cache = list_first_entry(&transaction->deleted_bgs,
						 struct btrfs_block_group_cache,
						 bg_list);
			list_del_init(&cache->bg_list);
			btrfs_put_block_group_trimming(cache);
			btrfs_put_block_group(cache);
		}
		WARN_ON(!list_empty(&transaction->dev_update_list));
		kfree(transaction);
	}
}

static noinline void switch_commit_roots(struct btrfs_trans_handle *trans)
{
	struct btrfs_transaction *cur_trans = trans->transaction;
	struct btrfs_fs_info *fs_info = trans->fs_info;
	struct btrfs_root *root, *tmp;

	down_write(&fs_info->commit_root_sem);
	list_for_each_entry_safe(root, tmp, &cur_trans->switch_commits,
				 dirty_list) {
		list_del_init(&root->dirty_list);
		free_extent_buffer(root->commit_root);
		root->commit_root = btrfs_root_node(root);
		if (is_fstree(root->root_key.objectid))
			btrfs_unpin_free_ino(root);
		extent_io_tree_release(&root->dirty_log_pages);
		btrfs_qgroup_clean_swapped_blocks(root);
	}

	/* We can free old roots now. */
	spin_lock(&cur_trans->dropped_roots_lock);
	while (!list_empty(&cur_trans->dropped_roots)) {
		root = list_first_entry(&cur_trans->dropped_roots,
					struct btrfs_root, root_list);
		list_del_init(&root->root_list);
		spin_unlock(&cur_trans->dropped_roots_lock);
		btrfs_free_log(trans, root);
		btrfs_drop_and_free_fs_root(fs_info, root);
		spin_lock(&cur_trans->dropped_roots_lock);
	}
	spin_unlock(&cur_trans->dropped_roots_lock);
	up_write(&fs_info->commit_root_sem);
}

static inline void extwriter_counter_inc(struct btrfs_transaction *trans,
					 unsigned int type)
{
	if (type & TRANS_EXTWRITERS)
		atomic_inc(&trans->num_extwriters);
}

static inline void extwriter_counter_dec(struct btrfs_transaction *trans,
					 unsigned int type)
{
	if (type & TRANS_EXTWRITERS)
		atomic_dec(&trans->num_extwriters);
}

static inline void extwriter_counter_init(struct btrfs_transaction *trans,
					  unsigned int type)
{
	atomic_set(&trans->num_extwriters, ((type & TRANS_EXTWRITERS) ? 1 : 0));
}

static inline int extwriter_counter_read(struct btrfs_transaction *trans)
{
	return atomic_read(&trans->num_extwriters);
}

/*
 * To be called after all the new block groups attached to the transaction
 * handle have been created (btrfs_create_pending_block_groups()).
 */
void btrfs_trans_release_chunk_metadata(struct btrfs_trans_handle *trans)
{
	struct btrfs_fs_info *fs_info = trans->fs_info;

	if (!trans->chunk_bytes_reserved)
		return;

	WARN_ON_ONCE(!list_empty(&trans->new_bgs));

	btrfs_block_rsv_release(fs_info, &fs_info->chunk_block_rsv,
				trans->chunk_bytes_reserved);
	trans->chunk_bytes_reserved = 0;
}

/*
 * either allocate a new transaction or hop into the existing one
 */
static noinline int join_transaction(struct btrfs_fs_info *fs_info,
				     unsigned int type)
{
	struct btrfs_transaction *cur_trans;

	spin_lock(&fs_info->trans_lock);
loop:
	/* The file system has been taken offline. No new transactions. */
	if (test_bit(BTRFS_FS_STATE_ERROR, &fs_info->fs_state)) {
		spin_unlock(&fs_info->trans_lock);
		return -EROFS;
	}

	cur_trans = fs_info->running_transaction;
	if (cur_trans) {
		if (TRANS_ABORTED(cur_trans)) {
			const int abort_error = cur_trans->aborted;

			spin_unlock(&fs_info->trans_lock);
			return abort_error;
		}
		if (btrfs_blocked_trans_types[cur_trans->state] & type) {
			spin_unlock(&fs_info->trans_lock);
			return -EBUSY;
		}
		refcount_inc(&cur_trans->use_count);
		atomic_inc(&cur_trans->num_writers);
		extwriter_counter_inc(cur_trans, type);
		spin_unlock(&fs_info->trans_lock);
		return 0;
	}
	spin_unlock(&fs_info->trans_lock);

	/*
	 * If we are ATTACH or TRANS_JOIN_NOSTART, we just want to catch the
	 * current transaction, and commit it. If there is no transaction, just
	 * return ENOENT.
	 */
	if (type == TRANS_ATTACH || type == TRANS_JOIN_NOSTART)
		return -ENOENT;

	/*
	 * JOIN_NOLOCK only happens during the transaction commit, so
	 * it is impossible that ->running_transaction is NULL
	 */
	BUG_ON(type == TRANS_JOIN_NOLOCK);

	cur_trans = kmalloc(sizeof(*cur_trans), GFP_NOFS);
	if (!cur_trans)
		return -ENOMEM;

	spin_lock(&fs_info->trans_lock);
	if (fs_info->running_transaction) {
		/*
		 * someone started a transaction after we unlocked.  Make sure
		 * to redo the checks above
		 */
		kfree(cur_trans);
		goto loop;
	} else if (test_bit(BTRFS_FS_STATE_ERROR, &fs_info->fs_state)) {
		spin_unlock(&fs_info->trans_lock);
		kfree(cur_trans);
		return -EROFS;
	}

	cur_trans->fs_info = fs_info;
	atomic_set(&cur_trans->num_writers, 1);
	extwriter_counter_init(cur_trans, type);
	init_waitqueue_head(&cur_trans->writer_wait);
	init_waitqueue_head(&cur_trans->commit_wait);
	cur_trans->state = TRANS_STATE_RUNNING;
	/*
	 * One for this trans handle, one so it will live on until we
	 * commit the transaction.
	 */
	refcount_set(&cur_trans->use_count, 2);
	cur_trans->flags = 0;
	cur_trans->start_time = ktime_get_seconds();

	memset(&cur_trans->delayed_refs, 0, sizeof(cur_trans->delayed_refs));

	cur_trans->delayed_refs.href_root = RB_ROOT_CACHED;
	cur_trans->delayed_refs.dirty_extent_root = RB_ROOT;
	atomic_set(&cur_trans->delayed_refs.num_entries, 0);

	/*
	 * although the tree mod log is per file system and not per transaction,
	 * the log must never go across transaction boundaries.
	 */
	smp_mb();
	if (!list_empty(&fs_info->tree_mod_seq_list))
		WARN(1, KERN_ERR "BTRFS: tree_mod_seq_list not empty when creating a fresh transaction\n");
	if (!RB_EMPTY_ROOT(&fs_info->tree_mod_log))
		WARN(1, KERN_ERR "BTRFS: tree_mod_log rb tree not empty when creating a fresh transaction\n");
	atomic64_set(&fs_info->tree_mod_seq, 0);

	spin_lock_init(&cur_trans->delayed_refs.lock);

	INIT_LIST_HEAD(&cur_trans->pending_snapshots);
	INIT_LIST_HEAD(&cur_trans->dev_update_list);
	INIT_LIST_HEAD(&cur_trans->switch_commits);
	INIT_LIST_HEAD(&cur_trans->dirty_bgs);
	INIT_LIST_HEAD(&cur_trans->io_bgs);
	INIT_LIST_HEAD(&cur_trans->dropped_roots);
	mutex_init(&cur_trans->cache_write_mutex);
	spin_lock_init(&cur_trans->dirty_bgs_lock);
	INIT_LIST_HEAD(&cur_trans->deleted_bgs);
	spin_lock_init(&cur_trans->dropped_roots_lock);
	list_add_tail(&cur_trans->list, &fs_info->trans_list);
	extent_io_tree_init(fs_info, &cur_trans->dirty_pages,
			IO_TREE_TRANS_DIRTY_PAGES, fs_info->btree_inode);
	fs_info->generation++;
	cur_trans->transid = fs_info->generation;
	fs_info->running_transaction = cur_trans;
	cur_trans->aborted = 0;
	spin_unlock(&fs_info->trans_lock);

	return 0;
}

/*
 * this does all the record keeping required to make sure that a reference
 * counted root is properly recorded in a given transaction.  This is required
 * to make sure the old root from before we joined the transaction is deleted
 * when the transaction commits
 */
static int record_root_in_trans(struct btrfs_trans_handle *trans,
			       struct btrfs_root *root,
			       int force)
{
	struct btrfs_fs_info *fs_info = root->fs_info;

	if ((test_bit(BTRFS_ROOT_REF_COWS, &root->state) &&
	    root->last_trans < trans->transid) || force) {
		WARN_ON(root == fs_info->extent_root);
		WARN_ON(!force && root->commit_root != root->node);

		/*
		 * see below for IN_TRANS_SETUP usage rules
		 * we have the reloc mutex held now, so there
		 * is only one writer in this function
		 */
		set_bit(BTRFS_ROOT_IN_TRANS_SETUP, &root->state);

		/* make sure readers find IN_TRANS_SETUP before
		 * they find our root->last_trans update
		 */
		smp_wmb();

		spin_lock(&fs_info->fs_roots_radix_lock);
		if (root->last_trans == trans->transid && !force) {
			spin_unlock(&fs_info->fs_roots_radix_lock);
			return 0;
		}
		radix_tree_tag_set(&fs_info->fs_roots_radix,
				   (unsigned long)root->root_key.objectid,
				   BTRFS_ROOT_TRANS_TAG);
		spin_unlock(&fs_info->fs_roots_radix_lock);
		root->last_trans = trans->transid;

		/* this is pretty tricky.  We don't want to
		 * take the relocation lock in btrfs_record_root_in_trans
		 * unless we're really doing the first setup for this root in
		 * this transaction.
		 *
		 * Normally we'd use root->last_trans as a flag to decide
		 * if we want to take the expensive mutex.
		 *
		 * But, we have to set root->last_trans before we
		 * init the relocation root, otherwise, we trip over warnings
		 * in ctree.c.  The solution used here is to flag ourselves
		 * with root IN_TRANS_SETUP.  When this is 1, we're still
		 * fixing up the reloc trees and everyone must wait.
		 *
		 * When this is zero, they can trust root->last_trans and fly
		 * through btrfs_record_root_in_trans without having to take the
		 * lock.  smp_wmb() makes sure that all the writes above are
		 * done before we pop in the zero below
		 */
		btrfs_init_reloc_root(trans, root);
		smp_mb__before_atomic();
		clear_bit(BTRFS_ROOT_IN_TRANS_SETUP, &root->state);
	}
	return 0;
}


void btrfs_add_dropped_root(struct btrfs_trans_handle *trans,
			    struct btrfs_root *root)
{
	struct btrfs_fs_info *fs_info = root->fs_info;
	struct btrfs_transaction *cur_trans = trans->transaction;

	/* Add ourselves to the transaction dropped list */
	spin_lock(&cur_trans->dropped_roots_lock);
	list_add_tail(&root->root_list, &cur_trans->dropped_roots);
	spin_unlock(&cur_trans->dropped_roots_lock);

	/* Make sure we don't try to update the root at commit time */
	spin_lock(&fs_info->fs_roots_radix_lock);
	radix_tree_tag_clear(&fs_info->fs_roots_radix,
			     (unsigned long)root->root_key.objectid,
			     BTRFS_ROOT_TRANS_TAG);
	spin_unlock(&fs_info->fs_roots_radix_lock);
}

int btrfs_record_root_in_trans(struct btrfs_trans_handle *trans,
			       struct btrfs_root *root)
{
	struct btrfs_fs_info *fs_info = root->fs_info;

	if (!test_bit(BTRFS_ROOT_REF_COWS, &root->state))
		return 0;

	/*
	 * see record_root_in_trans for comments about IN_TRANS_SETUP usage
	 * and barriers
	 */
	smp_rmb();
	if (root->last_trans == trans->transid &&
	    !test_bit(BTRFS_ROOT_IN_TRANS_SETUP, &root->state))
		return 0;

	mutex_lock(&fs_info->reloc_mutex);
	record_root_in_trans(trans, root, 0);
	mutex_unlock(&fs_info->reloc_mutex);

	return 0;
}

static inline int is_transaction_blocked(struct btrfs_transaction *trans)
{
	return (trans->state >= TRANS_STATE_COMMIT_START &&
		trans->state < TRANS_STATE_UNBLOCKED &&
		!TRANS_ABORTED(trans));
}

/* wait for commit against the current transaction to become unblocked
 * when this is done, it is safe to start a new transaction, but the current
 * transaction might not be fully on disk.
 */
static void wait_current_trans(struct btrfs_fs_info *fs_info)
{
	struct btrfs_transaction *cur_trans;

	spin_lock(&fs_info->trans_lock);
	cur_trans = fs_info->running_transaction;
	if (cur_trans && is_transaction_blocked(cur_trans)) {
		refcount_inc(&cur_trans->use_count);
		spin_unlock(&fs_info->trans_lock);

		wait_event(fs_info->transaction_wait,
			   cur_trans->state >= TRANS_STATE_UNBLOCKED ||
			   TRANS_ABORTED(cur_trans));
		btrfs_put_transaction(cur_trans);
	} else {
		spin_unlock(&fs_info->trans_lock);
	}
}

static int may_wait_transaction(struct btrfs_fs_info *fs_info, int type)
{
	if (test_bit(BTRFS_FS_LOG_RECOVERING, &fs_info->flags))
		return 0;

	if (type == TRANS_START)
		return 1;

	return 0;
}

static inline bool need_reserve_reloc_root(struct btrfs_root *root)
{
	struct btrfs_fs_info *fs_info = root->fs_info;

	if (!fs_info->reloc_ctl ||
	    !test_bit(BTRFS_ROOT_REF_COWS, &root->state) ||
	    root->root_key.objectid == BTRFS_TREE_RELOC_OBJECTID ||
	    root->reloc_root)
		return false;

	return true;
}

static struct btrfs_trans_handle *
start_transaction(struct btrfs_root *root, unsigned int num_items,
		  unsigned int type, enum btrfs_reserve_flush_enum flush,
		  bool enforce_qgroups)
{
	struct btrfs_fs_info *fs_info = root->fs_info;
	struct btrfs_block_rsv *delayed_refs_rsv = &fs_info->delayed_refs_rsv;
	struct btrfs_trans_handle *h;
	struct btrfs_transaction *cur_trans;
	u64 num_bytes = 0;
	u64 qgroup_reserved = 0;
	bool reloc_reserved = false;
	bool do_chunk_alloc = false;
	int ret;

	/* Send isn't supposed to start transactions. */
	ASSERT(current->journal_info != BTRFS_SEND_TRANS_STUB);

	if (test_bit(BTRFS_FS_STATE_ERROR, &fs_info->fs_state))
		return ERR_PTR(-EROFS);

	if (current->journal_info) {
		WARN_ON(type & TRANS_EXTWRITERS);
		h = current->journal_info;
		refcount_inc(&h->use_count);
		WARN_ON(refcount_read(&h->use_count) > 2);
		h->orig_rsv = h->block_rsv;
		h->block_rsv = NULL;
		goto got_it;
	}

	/*
	 * Do the reservation before we join the transaction so we can do all
	 * the appropriate flushing if need be.
	 */
	if (num_items && root != fs_info->chunk_root) {
		struct btrfs_block_rsv *rsv = &fs_info->trans_block_rsv;
		u64 delayed_refs_bytes = 0;

		qgroup_reserved = num_items * fs_info->nodesize;
		ret = btrfs_qgroup_reserve_meta_pertrans(root, qgroup_reserved,
				enforce_qgroups);
		if (ret)
			return ERR_PTR(ret);

		/*
		 * We want to reserve all the bytes we may need all at once, so
		 * we only do 1 enospc flushing cycle per transaction start.  We
		 * accomplish this by simply assuming we'll do 2 x num_items
		 * worth of delayed refs updates in this trans handle, and
		 * refill that amount for whatever is missing in the reserve.
		 */
		num_bytes = btrfs_calc_insert_metadata_size(fs_info, num_items);
		if (flush == BTRFS_RESERVE_FLUSH_ALL &&
		    delayed_refs_rsv->full == 0) {
			delayed_refs_bytes = num_bytes;
			num_bytes <<= 1;
		}

		/*
		 * Do the reservation for the relocation root creation
		 */
		if (need_reserve_reloc_root(root)) {
			num_bytes += fs_info->nodesize;
			reloc_reserved = true;
		}

		ret = btrfs_block_rsv_add(root, rsv, num_bytes, flush);
		if (ret)
			goto reserve_fail;
		if (delayed_refs_bytes) {
			btrfs_migrate_to_delayed_refs_rsv(fs_info, rsv,
							  delayed_refs_bytes);
			num_bytes -= delayed_refs_bytes;
		}

		if (rsv->space_info->force_alloc)
			do_chunk_alloc = true;
	} else if (num_items == 0 && flush == BTRFS_RESERVE_FLUSH_ALL &&
		   !delayed_refs_rsv->full) {
		/*
		 * Some people call with btrfs_start_transaction(root, 0)
		 * because they can be throttled, but have some other mechanism
		 * for reserving space.  We still want these guys to refill the
		 * delayed block_rsv so just add 1 items worth of reservation
		 * here.
		 */
		ret = btrfs_delayed_refs_rsv_refill(fs_info, flush);
		if (ret)
			goto reserve_fail;
	}
again:
	h = kmem_cache_zalloc(btrfs_trans_handle_cachep, GFP_NOFS);
	if (!h) {
		ret = -ENOMEM;
		goto alloc_fail;
	}

	/*
	 * If we are JOIN_NOLOCK we're already committing a transaction and
	 * waiting on this guy, so we don't need to do the sb_start_intwrite
	 * because we're already holding a ref.  We need this because we could
	 * have raced in and did an fsync() on a file which can kick a commit
	 * and then we deadlock with somebody doing a freeze.
	 *
	 * If we are ATTACH, it means we just want to catch the current
	 * transaction and commit it, so we needn't do sb_start_intwrite(). 
	 */
	if (type & __TRANS_FREEZABLE)
		sb_start_intwrite(fs_info->sb);

	if (may_wait_transaction(fs_info, type))
		wait_current_trans(fs_info);

	do {
		ret = join_transaction(fs_info, type);
		if (ret == -EBUSY) {
			wait_current_trans(fs_info);
			if (unlikely(type == TRANS_ATTACH ||
				     type == TRANS_JOIN_NOSTART))
				ret = -ENOENT;
		}
	} while (ret == -EBUSY);

	if (ret < 0)
		goto join_fail;

	cur_trans = fs_info->running_transaction;

	h->transid = cur_trans->transid;
	h->transaction = cur_trans;
	h->root = root;
	refcount_set(&h->use_count, 1);
	h->fs_info = root->fs_info;

	h->type = type;
	h->can_flush_pending_bgs = true;
	INIT_LIST_HEAD(&h->new_bgs);

	smp_mb();
	if (cur_trans->state >= TRANS_STATE_COMMIT_START &&
	    may_wait_transaction(fs_info, type)) {
		current->journal_info = h;
		btrfs_commit_transaction(h);
		goto again;
	}

	if (num_bytes) {
		trace_btrfs_space_reservation(fs_info, "transaction",
					      h->transid, num_bytes, 1);
		h->block_rsv = &fs_info->trans_block_rsv;
		h->bytes_reserved = num_bytes;
		h->reloc_reserved = reloc_reserved;
	}

got_it:
	if (!current->journal_info)
		current->journal_info = h;

	/*
	 * If the space_info is marked ALLOC_FORCE then we'll get upgraded to
	 * ALLOC_FORCE the first run through, and then we won't allocate for
	 * anybody else who races in later.  We don't care about the return
	 * value here.
	 */
	if (do_chunk_alloc && num_bytes) {
		u64 flags = h->block_rsv->space_info->flags;

		btrfs_chunk_alloc(h, btrfs_get_alloc_profile(fs_info, flags),
				  CHUNK_ALLOC_NO_FORCE);
	}

	/*
	 * btrfs_record_root_in_trans() needs to alloc new extents, and may
	 * call btrfs_join_transaction() while we're also starting a
	 * transaction.
	 *
	 * Thus it need to be called after current->journal_info initialized,
	 * or we can deadlock.
	 */
	btrfs_record_root_in_trans(h, root);

	return h;

join_fail:
	if (type & __TRANS_FREEZABLE)
		sb_end_intwrite(fs_info->sb);
	kmem_cache_free(btrfs_trans_handle_cachep, h);
alloc_fail:
	if (num_bytes)
		btrfs_block_rsv_release(fs_info, &fs_info->trans_block_rsv,
					num_bytes);
reserve_fail:
	btrfs_qgroup_free_meta_pertrans(root, qgroup_reserved);
	return ERR_PTR(ret);
}

struct btrfs_trans_handle *btrfs_start_transaction(struct btrfs_root *root,
						   unsigned int num_items)
{
	return start_transaction(root, num_items, TRANS_START,
				 BTRFS_RESERVE_FLUSH_ALL, true);
}

struct btrfs_trans_handle *btrfs_start_transaction_fallback_global_rsv(
					struct btrfs_root *root,
					unsigned int num_items)
{
	return start_transaction(root, num_items, TRANS_START,
				 BTRFS_RESERVE_FLUSH_ALL_STEAL, false);
}

struct btrfs_trans_handle *btrfs_join_transaction(struct btrfs_root *root)
{
	return start_transaction(root, 0, TRANS_JOIN, BTRFS_RESERVE_NO_FLUSH,
				 true);
}

struct btrfs_trans_handle *btrfs_join_transaction_nolock(struct btrfs_root *root)
{
	return start_transaction(root, 0, TRANS_JOIN_NOLOCK,
				 BTRFS_RESERVE_NO_FLUSH, true);
}

/*
 * Similar to regular join but it never starts a transaction when none is
 * running or after waiting for the current one to finish.
 */
struct btrfs_trans_handle *btrfs_join_transaction_nostart(struct btrfs_root *root)
{
	return start_transaction(root, 0, TRANS_JOIN_NOSTART,
				 BTRFS_RESERVE_NO_FLUSH, true);
}

/*
 * btrfs_attach_transaction() - catch the running transaction
 *
 * It is used when we want to commit the current the transaction, but
 * don't want to start a new one.
 *
 * Note: If this function return -ENOENT, it just means there is no
 * running transaction. But it is possible that the inactive transaction
 * is still in the memory, not fully on disk. If you hope there is no
 * inactive transaction in the fs when -ENOENT is returned, you should
 * invoke
 *     btrfs_attach_transaction_barrier()
 */
struct btrfs_trans_handle *btrfs_attach_transaction(struct btrfs_root *root)
{
	return start_transaction(root, 0, TRANS_ATTACH,
				 BTRFS_RESERVE_NO_FLUSH, true);
}

/*
 * btrfs_attach_transaction_barrier() - catch the running transaction
 *
 * It is similar to the above function, the difference is this one
 * will wait for all the inactive transactions until they fully
 * complete.
 */
struct btrfs_trans_handle *
btrfs_attach_transaction_barrier(struct btrfs_root *root)
{
	struct btrfs_trans_handle *trans;

	trans = start_transaction(root, 0, TRANS_ATTACH,
				  BTRFS_RESERVE_NO_FLUSH, true);
	if (trans == ERR_PTR(-ENOENT)) {
		int ret;

		ret = btrfs_wait_for_commit(root->fs_info, 0);
		if (ret)
			return ERR_PTR(ret);
	}

	return trans;
}

/* wait for a transaction commit to be fully complete */
static noinline void wait_for_commit(struct btrfs_transaction *commit)
{
	wait_event(commit->commit_wait, commit->state == TRANS_STATE_COMPLETED);
}

int btrfs_wait_for_commit(struct btrfs_fs_info *fs_info, u64 transid)
{
	struct btrfs_transaction *cur_trans = NULL, *t;
	int ret = 0;

	if (transid) {
		if (transid <= fs_info->last_trans_committed)
			goto out;

		/* find specified transaction */
		spin_lock(&fs_info->trans_lock);
		list_for_each_entry(t, &fs_info->trans_list, list) {
			if (t->transid == transid) {
				cur_trans = t;
				refcount_inc(&cur_trans->use_count);
				ret = 0;
				break;
			}
			if (t->transid > transid) {
				ret = 0;
				break;
			}
		}
		spin_unlock(&fs_info->trans_lock);

		/*
		 * The specified transaction doesn't exist, or we
		 * raced with btrfs_commit_transaction
		 */
		if (!cur_trans) {
			if (transid > fs_info->last_trans_committed)
				ret = -EINVAL;
			goto out;
		}
	} else {
		/* find newest transaction that is committing | committed */
		spin_lock(&fs_info->trans_lock);
		list_for_each_entry_reverse(t, &fs_info->trans_list,
					    list) {
			if (t->state >= TRANS_STATE_COMMIT_START) {
				if (t->state == TRANS_STATE_COMPLETED)
					break;
				cur_trans = t;
				refcount_inc(&cur_trans->use_count);
				break;
			}
		}
		spin_unlock(&fs_info->trans_lock);
		if (!cur_trans)
			goto out;  /* nothing committing|committed */
	}

	wait_for_commit(cur_trans);
	ret = cur_trans->aborted;
	btrfs_put_transaction(cur_trans);
out:
	return ret;
}

void btrfs_throttle(struct btrfs_fs_info *fs_info)
{
	wait_current_trans(fs_info);
}

static int should_end_transaction(struct btrfs_trans_handle *trans)
{
	struct btrfs_fs_info *fs_info = trans->fs_info;

	if (btrfs_check_space_for_delayed_refs(fs_info))
		return 1;

	return !!btrfs_block_rsv_check(&fs_info->global_block_rsv, 5);
}

int btrfs_should_end_transaction(struct btrfs_trans_handle *trans)
{
	struct btrfs_transaction *cur_trans = trans->transaction;

	smp_mb();
	if (cur_trans->state >= TRANS_STATE_COMMIT_START ||
	    cur_trans->delayed_refs.flushing)
		return 1;

	return should_end_transaction(trans);
}

static void btrfs_trans_release_metadata(struct btrfs_trans_handle *trans)

{
	struct btrfs_fs_info *fs_info = trans->fs_info;

	if (!trans->block_rsv) {
		ASSERT(!trans->bytes_reserved);
		return;
	}

	if (!trans->bytes_reserved)
		return;

	ASSERT(trans->block_rsv == &fs_info->trans_block_rsv);
	trace_btrfs_space_reservation(fs_info, "transaction",
				      trans->transid, trans->bytes_reserved, 0);
	btrfs_block_rsv_release(fs_info, trans->block_rsv,
				trans->bytes_reserved);
	trans->bytes_reserved = 0;
}

static int __btrfs_end_transaction(struct btrfs_trans_handle *trans,
				   int throttle)
{
	struct btrfs_fs_info *info = trans->fs_info;
	struct btrfs_transaction *cur_trans = trans->transaction;
	int err = 0;

	if (refcount_read(&trans->use_count) > 1) {
		refcount_dec(&trans->use_count);
		trans->block_rsv = trans->orig_rsv;
		return 0;
	}

	btrfs_trans_release_metadata(trans);
	trans->block_rsv = NULL;

	btrfs_create_pending_block_groups(trans);

	btrfs_trans_release_chunk_metadata(trans);

	if (trans->type & __TRANS_FREEZABLE)
		sb_end_intwrite(info->sb);

	WARN_ON(cur_trans != info->running_transaction);
	WARN_ON(atomic_read(&cur_trans->num_writers) < 1);
	atomic_dec(&cur_trans->num_writers);
	extwriter_counter_dec(cur_trans, trans->type);

	cond_wake_up(&cur_trans->writer_wait);
	btrfs_put_transaction(cur_trans);

	if (current->journal_info == trans)
		current->journal_info = NULL;

	if (throttle)
		btrfs_run_delayed_iputs(info);

	if (TRANS_ABORTED(trans) ||
	    test_bit(BTRFS_FS_STATE_ERROR, &info->fs_state)) {
		wake_up_process(info->transaction_kthread);
		if (TRANS_ABORTED(trans))
			err = trans->aborted;
		else
			err = -EROFS;
	}

	kmem_cache_free(btrfs_trans_handle_cachep, trans);
	return err;
}

int btrfs_end_transaction(struct btrfs_trans_handle *trans)
{
	return __btrfs_end_transaction(trans, 0);
}

int btrfs_end_transaction_throttle(struct btrfs_trans_handle *trans)
{
	return __btrfs_end_transaction(trans, 1);
}

/*
 * when btree blocks are allocated, they have some corresponding bits set for
 * them in one of two extent_io trees.  This is used to make sure all of
 * those extents are sent to disk but does not wait on them
 */
int btrfs_write_marked_extents(struct btrfs_fs_info *fs_info,
			       struct extent_io_tree *dirty_pages, int mark)
{
	int err = 0;
	int werr = 0;
	struct address_space *mapping = fs_info->btree_inode->i_mapping;
	struct extent_state *cached_state = NULL;
	u64 start = 0;
	u64 end;

	atomic_inc(&BTRFS_I(fs_info->btree_inode)->sync_writers);
	while (!find_first_extent_bit(dirty_pages, start, &start, &end,
				      mark, &cached_state)) {
		bool wait_writeback = false;

		err = convert_extent_bit(dirty_pages, start, end,
					 EXTENT_NEED_WAIT,
					 mark, &cached_state);
		/*
		 * convert_extent_bit can return -ENOMEM, which is most of the
		 * time a temporary error. So when it happens, ignore the error
		 * and wait for writeback of this range to finish - because we
		 * failed to set the bit EXTENT_NEED_WAIT for the range, a call
		 * to __btrfs_wait_marked_extents() would not know that
		 * writeback for this range started and therefore wouldn't
		 * wait for it to finish - we don't want to commit a
		 * superblock that points to btree nodes/leafs for which
		 * writeback hasn't finished yet (and without errors).
		 * We cleanup any entries left in the io tree when committing
		 * the transaction (through extent_io_tree_release()).
		 */
		if (err == -ENOMEM) {
			err = 0;
			wait_writeback = true;
		}
		if (!err)
			err = filemap_fdatawrite_range(mapping, start, end);
		if (err)
			werr = err;
		else if (wait_writeback)
			werr = filemap_fdatawait_range(mapping, start, end);
		free_extent_state(cached_state);
		cached_state = NULL;
		cond_resched();
		start = end + 1;
	}
	atomic_dec(&BTRFS_I(fs_info->btree_inode)->sync_writers);
	return werr;
}

/*
 * when btree blocks are allocated, they have some corresponding bits set for
 * them in one of two extent_io trees.  This is used to make sure all of
 * those extents are on disk for transaction or log commit.  We wait
 * on all the pages and clear them from the dirty pages state tree
 */
static int __btrfs_wait_marked_extents(struct btrfs_fs_info *fs_info,
				       struct extent_io_tree *dirty_pages)
{
	int err = 0;
	int werr = 0;
	struct address_space *mapping = fs_info->btree_inode->i_mapping;
	struct extent_state *cached_state = NULL;
	u64 start = 0;
	u64 end;

	while (!find_first_extent_bit(dirty_pages, start, &start, &end,
				      EXTENT_NEED_WAIT, &cached_state)) {
		/*
		 * Ignore -ENOMEM errors returned by clear_extent_bit().
		 * When committing the transaction, we'll remove any entries
		 * left in the io tree. For a log commit, we don't remove them
		 * after committing the log because the tree can be accessed
		 * concurrently - we do it only at transaction commit time when
		 * it's safe to do it (through extent_io_tree_release()).
		 */
		err = clear_extent_bit(dirty_pages, start, end,
				       EXTENT_NEED_WAIT, 0, 0, &cached_state);
		if (err == -ENOMEM)
			err = 0;
		if (!err)
			err = filemap_fdatawait_range(mapping, start, end);
		if (err)
			werr = err;
		free_extent_state(cached_state);
		cached_state = NULL;
		cond_resched();
		start = end + 1;
	}
	if (err)
		werr = err;
	return werr;
}

int btrfs_wait_extents(struct btrfs_fs_info *fs_info,
		       struct extent_io_tree *dirty_pages)
{
	bool errors = false;
	int err;

	err = __btrfs_wait_marked_extents(fs_info, dirty_pages);
	if (test_and_clear_bit(BTRFS_FS_BTREE_ERR, &fs_info->flags))
		errors = true;

	if (errors && !err)
		err = -EIO;
	return err;
}

int btrfs_wait_tree_log_extents(struct btrfs_root *log_root, int mark)
{
	struct btrfs_fs_info *fs_info = log_root->fs_info;
	struct extent_io_tree *dirty_pages = &log_root->dirty_log_pages;
	bool errors = false;
	int err;

	ASSERT(log_root->root_key.objectid == BTRFS_TREE_LOG_OBJECTID);

	err = __btrfs_wait_marked_extents(fs_info, dirty_pages);
	if ((mark & EXTENT_DIRTY) &&
	    test_and_clear_bit(BTRFS_FS_LOG1_ERR, &fs_info->flags))
		errors = true;

	if ((mark & EXTENT_NEW) &&
	    test_and_clear_bit(BTRFS_FS_LOG2_ERR, &fs_info->flags))
		errors = true;

	if (errors && !err)
		err = -EIO;
	return err;
}

/*
 * When btree blocks are allocated the corresponding extents are marked dirty.
 * This function ensures such extents are persisted on disk for transaction or
 * log commit.
 *
 * @trans: transaction whose dirty pages we'd like to write
 */
static int btrfs_write_and_wait_transaction(struct btrfs_trans_handle *trans)
{
	int ret;
	int ret2;
	struct extent_io_tree *dirty_pages = &trans->transaction->dirty_pages;
	struct btrfs_fs_info *fs_info = trans->fs_info;
	struct blk_plug plug;

	blk_start_plug(&plug);
	ret = btrfs_write_marked_extents(fs_info, dirty_pages, EXTENT_DIRTY);
	blk_finish_plug(&plug);
	ret2 = btrfs_wait_extents(fs_info, dirty_pages);

	extent_io_tree_release(&trans->transaction->dirty_pages);

	if (ret)
		return ret;
	else if (ret2)
		return ret2;
	else
		return 0;
}

/*
 * this is used to update the root pointer in the tree of tree roots.
 *
 * But, in the case of the extent allocation tree, updating the root
 * pointer may allocate blocks which may change the root of the extent
 * allocation tree.
 *
 * So, this loops and repeats and makes sure the cowonly root didn't
 * change while the root pointer was being updated in the metadata.
 */
static int update_cowonly_root(struct btrfs_trans_handle *trans,
			       struct btrfs_root *root)
{
	int ret;
	u64 old_root_bytenr;
	u64 old_root_used;
	struct btrfs_fs_info *fs_info = root->fs_info;
	struct btrfs_root *tree_root = fs_info->tree_root;

	old_root_used = btrfs_root_used(&root->root_item);

	while (1) {
		old_root_bytenr = btrfs_root_bytenr(&root->root_item);
		if (old_root_bytenr == root->node->start &&
		    old_root_used == btrfs_root_used(&root->root_item))
			break;

		btrfs_set_root_node(&root->root_item, root->node);
		ret = btrfs_update_root(trans, tree_root,
					&root->root_key,
					&root->root_item);
		if (ret)
			return ret;

		old_root_used = btrfs_root_used(&root->root_item);
	}

	return 0;
}

/*
 * update all the cowonly tree roots on disk
 *
 * The error handling in this function may not be obvious. Any of the
 * failures will cause the file system to go offline. We still need
 * to clean up the delayed refs.
 */
static noinline int commit_cowonly_roots(struct btrfs_trans_handle *trans)
{
	struct btrfs_fs_info *fs_info = trans->fs_info;
	struct list_head *dirty_bgs = &trans->transaction->dirty_bgs;
	struct list_head *io_bgs = &trans->transaction->io_bgs;
	struct list_head *next;
	struct extent_buffer *eb;
	int ret;

	eb = btrfs_lock_root_node(fs_info->tree_root);
	ret = btrfs_cow_block(trans, fs_info->tree_root, eb, NULL,
			      0, &eb);
	btrfs_tree_unlock(eb);
	free_extent_buffer(eb);

	if (ret)
		return ret;

	ret = btrfs_run_delayed_refs(trans, (unsigned long)-1);
	if (ret)
		return ret;

	ret = btrfs_run_dev_stats(trans);
	if (ret)
		return ret;
	ret = btrfs_run_dev_replace(trans);
	if (ret)
		return ret;
	ret = btrfs_run_qgroups(trans);
	if (ret)
		return ret;

	ret = btrfs_setup_space_cache(trans);
	if (ret)
		return ret;

	/* run_qgroups might have added some more refs */
	ret = btrfs_run_delayed_refs(trans, (unsigned long)-1);
	if (ret)
		return ret;
again:
	while (!list_empty(&fs_info->dirty_cowonly_roots)) {
		struct btrfs_root *root;
		next = fs_info->dirty_cowonly_roots.next;
		list_del_init(next);
		root = list_entry(next, struct btrfs_root, dirty_list);
		clear_bit(BTRFS_ROOT_DIRTY, &root->state);

		if (root != fs_info->extent_root)
			list_add_tail(&root->dirty_list,
				      &trans->transaction->switch_commits);
		ret = update_cowonly_root(trans, root);
		if (ret)
			return ret;
		ret = btrfs_run_delayed_refs(trans, (unsigned long)-1);
		if (ret)
			return ret;
	}

	while (!list_empty(dirty_bgs) || !list_empty(io_bgs)) {
		ret = btrfs_write_dirty_block_groups(trans);
		if (ret)
			return ret;
		ret = btrfs_run_delayed_refs(trans, (unsigned long)-1);
		if (ret)
			return ret;
	}

	if (!list_empty(&fs_info->dirty_cowonly_roots))
		goto again;

	list_add_tail(&fs_info->extent_root->dirty_list,
		      &trans->transaction->switch_commits);

	/* Update dev-replace pointer once everything is committed */
	fs_info->dev_replace.committed_cursor_left =
		fs_info->dev_replace.cursor_left_last_write_of_item;

	return 0;
}

/*
 * dead roots are old snapshots that need to be deleted.  This allocates
 * a dirty root struct and adds it into the list of dead roots that need to
 * be deleted
 */
void btrfs_add_dead_root(struct btrfs_root *root)
{
	struct btrfs_fs_info *fs_info = root->fs_info;

	spin_lock(&fs_info->trans_lock);
	if (list_empty(&root->root_list))
		list_add_tail(&root->root_list, &fs_info->dead_roots);
	spin_unlock(&fs_info->trans_lock);
}

/*
 * update all the cowonly tree roots on disk
 */
static noinline int commit_fs_roots(struct btrfs_trans_handle *trans)
{
	struct btrfs_fs_info *fs_info = trans->fs_info;
	struct btrfs_root *gang[8];
	int i;
	int ret;

	spin_lock(&fs_info->fs_roots_radix_lock);
	while (1) {
		ret = radix_tree_gang_lookup_tag(&fs_info->fs_roots_radix,
						 (void **)gang, 0,
						 ARRAY_SIZE(gang),
						 BTRFS_ROOT_TRANS_TAG);
		if (ret == 0)
			break;
		for (i = 0; i < ret; i++) {
			struct btrfs_root *root = gang[i];
			int ret2;

			radix_tree_tag_clear(&fs_info->fs_roots_radix,
					(unsigned long)root->root_key.objectid,
					BTRFS_ROOT_TRANS_TAG);
			btrfs_qgroup_free_meta_all_pertrans(root);
			spin_unlock(&fs_info->fs_roots_radix_lock);

			btrfs_free_log(trans, root);
			btrfs_update_reloc_root(trans, root);

			btrfs_save_ino_cache(root, trans);

			/* see comments in should_cow_block() */
			clear_bit(BTRFS_ROOT_FORCE_COW, &root->state);
			smp_mb__after_atomic();

			if (root->commit_root != root->node) {
				list_add_tail(&root->dirty_list,
					&trans->transaction->switch_commits);
				btrfs_set_root_node(&root->root_item,
						    root->node);
			}

			ret2 = btrfs_update_root(trans, fs_info->tree_root,
						&root->root_key,
						&root->root_item);
			if (ret2)
				return ret2;
			spin_lock(&fs_info->fs_roots_radix_lock);
		}
	}
	spin_unlock(&fs_info->fs_roots_radix_lock);
	return 0;
}

/*
 * defrag a given btree.
 * Every leaf in the btree is read and defragged.
 */
int btrfs_defrag_root(struct btrfs_root *root)
{
	struct btrfs_fs_info *info = root->fs_info;
	struct btrfs_trans_handle *trans;
	int ret;

	if (test_and_set_bit(BTRFS_ROOT_DEFRAG_RUNNING, &root->state))
		return 0;

	while (1) {
		trans = btrfs_start_transaction(root, 0);
		if (IS_ERR(trans)) {
			ret = PTR_ERR(trans);
			break;
		}

		ret = btrfs_defrag_leaves(trans, root);

		btrfs_end_transaction(trans);
		btrfs_btree_balance_dirty(info);
		cond_resched();

		if (btrfs_fs_closing(info) || ret != -EAGAIN)
			break;

		if (btrfs_defrag_cancelled(info)) {
			btrfs_debug(info, "defrag_root cancelled");
			ret = -EAGAIN;
			break;
		}
	}
	clear_bit(BTRFS_ROOT_DEFRAG_RUNNING, &root->state);
	return ret;
}

/*
 * Do all special snapshot related qgroup dirty hack.
 *
 * Will do all needed qgroup inherit and dirty hack like switch commit
 * roots inside one transaction and write all btree into disk, to make
 * qgroup works.
 */
static int qgroup_account_snapshot(struct btrfs_trans_handle *trans,
				   struct btrfs_root *src,
				   struct btrfs_root *parent,
				   struct btrfs_qgroup_inherit *inherit,
				   u64 dst_objectid)
{
	struct btrfs_fs_info *fs_info = src->fs_info;
	int ret;

	/*
	 * Save some performance in the case that qgroups are not
	 * enabled. If this check races with the ioctl, rescan will
	 * kick in anyway.
	 */
	if (!test_bit(BTRFS_FS_QUOTA_ENABLED, &fs_info->flags))
		return 0;

	/*
	 * Ensure dirty @src will be committed.  Or, after coming
	 * commit_fs_roots() and switch_commit_roots(), any dirty but not
	 * recorded root will never be updated again, causing an outdated root
	 * item.
	 */
	record_root_in_trans(trans, src, 1);

	/*
	 * We are going to commit transaction, see btrfs_commit_transaction()
	 * comment for reason locking tree_log_mutex
	 */
	mutex_lock(&fs_info->tree_log_mutex);

	ret = commit_fs_roots(trans);
	if (ret)
		goto out;
	ret = btrfs_qgroup_account_extents(trans);
	if (ret < 0)
		goto out;

	/* Now qgroup are all updated, we can inherit it to new qgroups */
	ret = btrfs_qgroup_inherit(trans, src->root_key.objectid, dst_objectid,
				   inherit);
	if (ret < 0)
		goto out;

	/*
	 * Now we do a simplified commit transaction, which will:
	 * 1) commit all subvolume and extent tree
	 *    To ensure all subvolume and extent tree have a valid
	 *    commit_root to accounting later insert_dir_item()
	 * 2) write all btree blocks onto disk
	 *    This is to make sure later btree modification will be cowed
	 *    Or commit_root can be populated and cause wrong qgroup numbers
	 * In this simplified commit, we don't really care about other trees
	 * like chunk and root tree, as they won't affect qgroup.
	 * And we don't write super to avoid half committed status.
	 */
	ret = commit_cowonly_roots(trans);
	if (ret)
		goto out;
	switch_commit_roots(trans);
	ret = btrfs_write_and_wait_transaction(trans);
	if (ret)
		btrfs_handle_fs_error(fs_info, ret,
			"Error while writing out transaction for qgroup");

out:
	mutex_unlock(&fs_info->tree_log_mutex);

	/*
	 * Force parent root to be updated, as we recorded it before so its
	 * last_trans == cur_transid.
	 * Or it won't be committed again onto disk after later
	 * insert_dir_item()
	 */
	if (!ret)
		record_root_in_trans(trans, parent, 1);
	return ret;
}

/*
 * new snapshots need to be created at a very specific time in the
 * transaction commit.  This does the actual creation.
 *
 * Note:
 * If the error which may affect the commitment of the current transaction
 * happens, we should return the error number. If the error which just affect
 * the creation of the pending snapshots, just return 0.
 */
static noinline int create_pending_snapshot(struct btrfs_trans_handle *trans,
				   struct btrfs_pending_snapshot *pending)
{

	struct btrfs_fs_info *fs_info = trans->fs_info;
	struct btrfs_key key;
	struct btrfs_root_item *new_root_item;
	struct btrfs_root *tree_root = fs_info->tree_root;
	struct btrfs_root *root = pending->root;
	struct btrfs_root *parent_root;
	struct btrfs_block_rsv *rsv;
	struct inode *parent_inode;
	struct btrfs_path *path;
	struct btrfs_dir_item *dir_item;
	struct dentry *dentry;
	struct extent_buffer *tmp;
	struct extent_buffer *old;
	struct timespec64 cur_time;
	int ret = 0;
	u64 to_reserve = 0;
	u64 index = 0;
	u64 objectid;
	u64 root_flags;
	uuid_le new_uuid;

	ASSERT(pending->path);
	path = pending->path;

	ASSERT(pending->root_item);
	new_root_item = pending->root_item;

	pending->error = btrfs_find_free_objectid(tree_root, &objectid);
	if (pending->error)
		goto no_free_objectid;

	/*
	 * Make qgroup to skip current new snapshot's qgroupid, as it is
	 * accounted by later btrfs_qgroup_inherit().
	 */
	btrfs_set_skip_qgroup(trans, objectid);

	btrfs_reloc_pre_snapshot(pending, &to_reserve);

	if (to_reserve > 0) {
		pending->error = btrfs_block_rsv_add(root,
						     &pending->block_rsv,
						     to_reserve,
						     BTRFS_RESERVE_NO_FLUSH);
		if (pending->error)
			goto clear_skip_qgroup;
	}

	key.objectid = objectid;
	key.offset = (u64)-1;
	key.type = BTRFS_ROOT_ITEM_KEY;

	rsv = trans->block_rsv;
	trans->block_rsv = &pending->block_rsv;
	trans->bytes_reserved = trans->block_rsv->reserved;
	trace_btrfs_space_reservation(fs_info, "transaction",
				      trans->transid,
				      trans->bytes_reserved, 1);
	dentry = pending->dentry;
	parent_inode = pending->dir;
	parent_root = BTRFS_I(parent_inode)->root;
	record_root_in_trans(trans, parent_root, 0);

	cur_time = current_time(parent_inode);

	/*
	 * insert the directory item
	 */
	ret = btrfs_set_inode_index(BTRFS_I(parent_inode), &index);
	BUG_ON(ret); /* -ENOMEM */

	/* check if there is a file/dir which has the same name. */
	dir_item = btrfs_lookup_dir_item(NULL, parent_root, path,
					 btrfs_ino(BTRFS_I(parent_inode)),
					 dentry->d_name.name,
					 dentry->d_name.len, 0);
	if (dir_item != NULL && !IS_ERR(dir_item)) {
		pending->error = -EEXIST;
		goto dir_item_existed;
	} else if (IS_ERR(dir_item)) {
		ret = PTR_ERR(dir_item);
		btrfs_abort_transaction(trans, ret);
		goto fail;
	}
	btrfs_release_path(path);

	/*
	 * pull in the delayed directory update
	 * and the delayed inode item
	 * otherwise we corrupt the FS during
	 * snapshot
	 */
	ret = btrfs_run_delayed_items(trans);
	if (ret) {	/* Transaction aborted */
		btrfs_abort_transaction(trans, ret);
		goto fail;
	}

	record_root_in_trans(trans, root, 0);
	btrfs_set_root_last_snapshot(&root->root_item, trans->transid);
	memcpy(new_root_item, &root->root_item, sizeof(*new_root_item));
	btrfs_check_and_init_root_item(new_root_item);

	root_flags = btrfs_root_flags(new_root_item);
	if (pending->readonly)
		root_flags |= BTRFS_ROOT_SUBVOL_RDONLY;
	else
		root_flags &= ~BTRFS_ROOT_SUBVOL_RDONLY;
	btrfs_set_root_flags(new_root_item, root_flags);

	btrfs_set_root_generation_v2(new_root_item,
			trans->transid);
	uuid_le_gen(&new_uuid);
	memcpy(new_root_item->uuid, new_uuid.b, BTRFS_UUID_SIZE);
	memcpy(new_root_item->parent_uuid, root->root_item.uuid,
			BTRFS_UUID_SIZE);
	if (!(root_flags & BTRFS_ROOT_SUBVOL_RDONLY)) {
		memset(new_root_item->received_uuid, 0,
		       sizeof(new_root_item->received_uuid));
		memset(&new_root_item->stime, 0, sizeof(new_root_item->stime));
		memset(&new_root_item->rtime, 0, sizeof(new_root_item->rtime));
		btrfs_set_root_stransid(new_root_item, 0);
		btrfs_set_root_rtransid(new_root_item, 0);
	}
	btrfs_set_stack_timespec_sec(&new_root_item->otime, cur_time.tv_sec);
	btrfs_set_stack_timespec_nsec(&new_root_item->otime, cur_time.tv_nsec);
	btrfs_set_root_otransid(new_root_item, trans->transid);

	old = btrfs_lock_root_node(root);
	ret = btrfs_cow_block(trans, root, old, NULL, 0, &old);
	if (ret) {
		btrfs_tree_unlock(old);
		free_extent_buffer(old);
		btrfs_abort_transaction(trans, ret);
		goto fail;
	}

	btrfs_set_lock_blocking_write(old);

	ret = btrfs_copy_root(trans, root, old, &tmp, objectid);
	/* clean up in any case */
	btrfs_tree_unlock(old);
	free_extent_buffer(old);
	if (ret) {
		btrfs_abort_transaction(trans, ret);
		goto fail;
	}
	/* see comments in should_cow_block() */
	set_bit(BTRFS_ROOT_FORCE_COW, &root->state);
	smp_wmb();

	btrfs_set_root_node(new_root_item, tmp);
	/* record when the snapshot was created in key.offset */
	key.offset = trans->transid;
	ret = btrfs_insert_root(trans, tree_root, &key, new_root_item);
	btrfs_tree_unlock(tmp);
	free_extent_buffer(tmp);
	if (ret) {
		btrfs_abort_transaction(trans, ret);
		goto fail;
	}

	/*
	 * insert root back/forward references
	 */
	ret = btrfs_add_root_ref(trans, objectid,
				 parent_root->root_key.objectid,
				 btrfs_ino(BTRFS_I(parent_inode)), index,
				 dentry->d_name.name, dentry->d_name.len);
	if (ret) {
		btrfs_abort_transaction(trans, ret);
		goto fail;
	}

	key.offset = (u64)-1;
	pending->snap = btrfs_read_fs_root_no_name(fs_info, &key);
	if (IS_ERR(pending->snap)) {
		ret = PTR_ERR(pending->snap);
		btrfs_abort_transaction(trans, ret);
		goto fail;
	}

	ret = btrfs_reloc_post_snapshot(trans, pending);
	if (ret) {
		btrfs_abort_transaction(trans, ret);
		goto fail;
	}

	ret = btrfs_run_delayed_refs(trans, (unsigned long)-1);
	if (ret) {
		btrfs_abort_transaction(trans, ret);
		goto fail;
	}

	/*
	 * Do special qgroup accounting for snapshot, as we do some qgroup
	 * snapshot hack to do fast snapshot.
	 * To co-operate with that hack, we do hack again.
	 * Or snapshot will be greatly slowed down by a subtree qgroup rescan
	 */
	ret = qgroup_account_snapshot(trans, root, parent_root,
				      pending->inherit, objectid);
	if (ret < 0)
		goto fail;

	ret = btrfs_insert_dir_item(trans, dentry->d_name.name,
				    dentry->d_name.len, BTRFS_I(parent_inode),
				    &key, BTRFS_FT_DIR, index);
	/* We have check then name at the beginning, so it is impossible. */
	BUG_ON(ret == -EEXIST || ret == -EOVERFLOW);
	if (ret) {
		btrfs_abort_transaction(trans, ret);
		goto fail;
	}

	btrfs_i_size_write(BTRFS_I(parent_inode), parent_inode->i_size +
					 dentry->d_name.len * 2);
	parent_inode->i_mtime = parent_inode->i_ctime =
		current_time(parent_inode);
	ret = btrfs_update_inode_fallback(trans, parent_root, parent_inode);
	if (ret) {
		btrfs_abort_transaction(trans, ret);
		goto fail;
	}
	ret = btrfs_uuid_tree_add(trans, new_uuid.b, BTRFS_UUID_KEY_SUBVOL,
				  objectid);
	if (ret) {
		btrfs_abort_transaction(trans, ret);
		goto fail;
	}
	if (!btrfs_is_empty_uuid(new_root_item->received_uuid)) {
		ret = btrfs_uuid_tree_add(trans, new_root_item->received_uuid,
					  BTRFS_UUID_KEY_RECEIVED_SUBVOL,
					  objectid);
		if (ret && ret != -EEXIST) {
			btrfs_abort_transaction(trans, ret);
			goto fail;
		}
	}

	ret = btrfs_run_delayed_refs(trans, (unsigned long)-1);
	if (ret) {
		btrfs_abort_transaction(trans, ret);
		goto fail;
	}

fail:
	pending->error = ret;
dir_item_existed:
	trans->block_rsv = rsv;
	trans->bytes_reserved = 0;
clear_skip_qgroup:
	btrfs_clear_skip_qgroup(trans);
no_free_objectid:
	kfree(new_root_item);
	pending->root_item = NULL;
	btrfs_free_path(path);
	pending->path = NULL;

	return ret;
}

/*
 * create all the snapshots we've scheduled for creation
 */
static noinline int create_pending_snapshots(struct btrfs_trans_handle *trans)
{
	struct btrfs_pending_snapshot *pending, *next;
	struct list_head *head = &trans->transaction->pending_snapshots;
	int ret = 0;

	list_for_each_entry_safe(pending, next, head, list) {
		list_del(&pending->list);
		ret = create_pending_snapshot(trans, pending);
		if (ret)
			break;
	}
	return ret;
}

static void update_super_roots(struct btrfs_fs_info *fs_info)
{
	struct btrfs_root_item *root_item;
	struct btrfs_super_block *super;

	super = fs_info->super_copy;

	root_item = &fs_info->chunk_root->root_item;
	super->chunk_root = root_item->bytenr;
	super->chunk_root_generation = root_item->generation;
	super->chunk_root_level = root_item->level;

	root_item = &fs_info->tree_root->root_item;
	super->root = root_item->bytenr;
	super->generation = root_item->generation;
	super->root_level = root_item->level;
	if (btrfs_test_opt(fs_info, SPACE_CACHE))
		super->cache_generation = root_item->generation;
	if (test_bit(BTRFS_FS_UPDATE_UUID_TREE_GEN, &fs_info->flags))
		super->uuid_tree_generation = root_item->generation;
}

int btrfs_transaction_in_commit(struct btrfs_fs_info *info)
{
	struct btrfs_transaction *trans;
	int ret = 0;

	spin_lock(&info->trans_lock);
	trans = info->running_transaction;
	if (trans)
		ret = (trans->state >= TRANS_STATE_COMMIT_START);
	spin_unlock(&info->trans_lock);
	return ret;
}

int btrfs_transaction_blocked(struct btrfs_fs_info *info)
{
	struct btrfs_transaction *trans;
	int ret = 0;

	spin_lock(&info->trans_lock);
	trans = info->running_transaction;
	if (trans)
		ret = is_transaction_blocked(trans);
	spin_unlock(&info->trans_lock);
	return ret;
}

/*
 * wait for the current transaction commit to start and block subsequent
 * transaction joins
 */
static void wait_current_trans_commit_start(struct btrfs_fs_info *fs_info,
					    struct btrfs_transaction *trans)
{
	wait_event(fs_info->transaction_blocked_wait,
		   trans->state >= TRANS_STATE_COMMIT_START ||
		   TRANS_ABORTED(trans));
}

/*
 * wait for the current transaction to start and then become unblocked.
 * caller holds ref.
 */
static void wait_current_trans_commit_start_and_unblock(
					struct btrfs_fs_info *fs_info,
					struct btrfs_transaction *trans)
{
	wait_event(fs_info->transaction_wait,
		   trans->state >= TRANS_STATE_UNBLOCKED ||
		   TRANS_ABORTED(trans));
}

/*
 * commit transactions asynchronously. once btrfs_commit_transaction_async
 * returns, any subsequent transaction will not be allowed to join.
 */
struct btrfs_async_commit {
	struct btrfs_trans_handle *newtrans;
	struct work_struct work;
};

static void do_async_commit(struct work_struct *work)
{
	struct btrfs_async_commit *ac =
		container_of(work, struct btrfs_async_commit, work);

	/*
	 * We've got freeze protection passed with the transaction.
	 * Tell lockdep about it.
	 */
	if (ac->newtrans->type & __TRANS_FREEZABLE)
		__sb_writers_acquired(ac->newtrans->fs_info->sb, SB_FREEZE_FS);

	current->journal_info = ac->newtrans;

	btrfs_commit_transaction(ac->newtrans);
	kfree(ac);
}

int btrfs_commit_transaction_async(struct btrfs_trans_handle *trans,
				   int wait_for_unblock)
{
	struct btrfs_fs_info *fs_info = trans->fs_info;
	struct btrfs_async_commit *ac;
	struct btrfs_transaction *cur_trans;

	ac = kmalloc(sizeof(*ac), GFP_NOFS);
	if (!ac)
		return -ENOMEM;

	INIT_WORK(&ac->work, do_async_commit);
	ac->newtrans = btrfs_join_transaction(trans->root);
	if (IS_ERR(ac->newtrans)) {
		int err = PTR_ERR(ac->newtrans);
		kfree(ac);
		return err;
	}

	/* take transaction reference */
	cur_trans = trans->transaction;
	refcount_inc(&cur_trans->use_count);

	btrfs_end_transaction(trans);

	/*
	 * Tell lockdep we've released the freeze rwsem, since the
	 * async commit thread will be the one to unlock it.
	 */
	if (ac->newtrans->type & __TRANS_FREEZABLE)
		__sb_writers_release(fs_info->sb, SB_FREEZE_FS);

	schedule_work(&ac->work);

	/* wait for transaction to start and unblock */
	if (wait_for_unblock)
		wait_current_trans_commit_start_and_unblock(fs_info, cur_trans);
	else
		wait_current_trans_commit_start(fs_info, cur_trans);

	if (current->journal_info == trans)
		current->journal_info = NULL;

	btrfs_put_transaction(cur_trans);
	return 0;
}


static void cleanup_transaction(struct btrfs_trans_handle *trans, int err)
{
	struct btrfs_fs_info *fs_info = trans->fs_info;
	struct btrfs_transaction *cur_trans = trans->transaction;

	WARN_ON(refcount_read(&trans->use_count) > 1);

	btrfs_abort_transaction(trans, err);

	spin_lock(&fs_info->trans_lock);

	/*
	 * If the transaction is removed from the list, it means this
	 * transaction has been committed successfully, so it is impossible
	 * to call the cleanup function.
	 */
	BUG_ON(list_empty(&cur_trans->list));

	list_del_init(&cur_trans->list);
	if (cur_trans == fs_info->running_transaction) {
		cur_trans->state = TRANS_STATE_COMMIT_DOING;
		spin_unlock(&fs_info->trans_lock);
		wait_event(cur_trans->writer_wait,
			   atomic_read(&cur_trans->num_writers) == 1);

		spin_lock(&fs_info->trans_lock);
	}
	spin_unlock(&fs_info->trans_lock);

	btrfs_cleanup_one_transaction(trans->transaction, fs_info);

	spin_lock(&fs_info->trans_lock);
	if (cur_trans == fs_info->running_transaction)
		fs_info->running_transaction = NULL;
	spin_unlock(&fs_info->trans_lock);

	if (trans->type & __TRANS_FREEZABLE)
		sb_end_intwrite(fs_info->sb);
	btrfs_put_transaction(cur_trans);
	btrfs_put_transaction(cur_trans);

	trace_btrfs_transaction_commit(trans->root);

	if (current->journal_info == trans)
		current->journal_info = NULL;
	btrfs_scrub_cancel(fs_info);

	kmem_cache_free(btrfs_trans_handle_cachep, trans);
}

/*
 * Release reserved delayed ref space of all pending block groups of the
 * transaction and remove them from the list
 */
static void btrfs_cleanup_pending_block_groups(struct btrfs_trans_handle *trans)
{
       struct btrfs_fs_info *fs_info = trans->fs_info;
       struct btrfs_block_group_cache *block_group, *tmp;

       list_for_each_entry_safe(block_group, tmp, &trans->new_bgs, bg_list) {
               btrfs_delayed_refs_rsv_release(fs_info, 1);
               list_del_init(&block_group->bg_list);
       }
}

static inline int btrfs_start_delalloc_flush(struct btrfs_trans_handle *trans)
{
	struct btrfs_fs_info *fs_info = trans->fs_info;

	/*
	 * We use writeback_inodes_sb here because if we used
	 * btrfs_start_delalloc_roots we would deadlock with fs freeze.
	 * Currently are holding the fs freeze lock, if we do an async flush
	 * we'll do btrfs_join_transaction() and deadlock because we need to
	 * wait for the fs freeze lock.  Using the direct flushing we benefit
	 * from already being in a transaction and our join_transaction doesn't
	 * have to re-take the fs freeze lock.
	 */
	if (btrfs_test_opt(fs_info, FLUSHONCOMMIT)) {
		writeback_inodes_sb(fs_info->sb, WB_REASON_SYNC);
	} else {
		struct btrfs_pending_snapshot *pending;
		struct list_head *head = &trans->transaction->pending_snapshots;

		/*
		 * Flush dellaloc for any root that is going to be snapshotted.
		 * This is done to avoid a corrupted version of files, in the
		 * snapshots, that had both buffered and direct IO writes (even
		 * if they were done sequentially) due to an unordered update of
		 * the inode's size on disk.
		 */
		list_for_each_entry(pending, head, list) {
			int ret;

			ret = btrfs_start_delalloc_snapshot(pending->root);
			if (ret)
				return ret;
		}
	}
	return 0;
}

static inline void btrfs_wait_delalloc_flush(struct btrfs_trans_handle *trans)
{
	struct btrfs_fs_info *fs_info = trans->fs_info;

	if (btrfs_test_opt(fs_info, FLUSHONCOMMIT)) {
		btrfs_wait_ordered_roots(fs_info, U64_MAX, 0, (u64)-1);
	} else {
		struct btrfs_pending_snapshot *pending;
		struct list_head *head = &trans->transaction->pending_snapshots;

		/*
		 * Wait for any dellaloc that we started previously for the roots
		 * that are going to be snapshotted. This is to avoid a corrupted
		 * version of files in the snapshots that had both buffered and
		 * direct IO writes (even if they were done sequentially).
		 */
		list_for_each_entry(pending, head, list)
			btrfs_wait_ordered_extents(pending->root,
						   U64_MAX, 0, U64_MAX);
	}
}

/*
 * Add a pending snapshot associated with the given transaction handle to the
 * respective handle. This must be called after the transaction commit started
 * and while holding fs_info->trans_lock.
 * This serves to guarantee a caller of btrfs_commit_transaction() that it can
 * safely free the pending snapshot pointer in case btrfs_commit_transaction()
 * returns an error.
 */
static void add_pending_snapshot(struct btrfs_trans_handle *trans)
{
	struct btrfs_transaction *cur_trans = trans->transaction;

	if (!trans->pending_snapshot)
		return;

	lockdep_assert_held(&trans->fs_info->trans_lock);
	ASSERT(cur_trans->state >= TRANS_STATE_COMMIT_START);

	list_add(&trans->pending_snapshot->list, &cur_trans->pending_snapshots);
}

int btrfs_commit_transaction(struct btrfs_trans_handle *trans)
{
	struct btrfs_fs_info *fs_info = trans->fs_info;
	struct btrfs_transaction *cur_trans = trans->transaction;
	struct btrfs_transaction *prev_trans = NULL;
	int ret;

	/*
	 * Some places just start a transaction to commit it.  We need to make
	 * sure that if this commit fails that the abort code actually marks the
	 * transaction as failed, so set trans->dirty to make the abort code do
	 * the right thing.
	 */
	trans->dirty = true;

	/* Stop the commit early if ->aborted is set */
	if (TRANS_ABORTED(cur_trans)) {
		ret = cur_trans->aborted;
		btrfs_end_transaction(trans);
		return ret;
	}

	btrfs_trans_release_metadata(trans);
	trans->block_rsv = NULL;

	/* make a pass through all the delayed refs we have so far
	 * any runnings procs may add more while we are here
	 */
	ret = btrfs_run_delayed_refs(trans, 0);
	if (ret) {
		btrfs_end_transaction(trans);
		return ret;
	}

	cur_trans = trans->transaction;

	/*
	 * set the flushing flag so procs in this transaction have to
	 * start sending their work down.
	 */
	cur_trans->delayed_refs.flushing = 1;
	smp_wmb();

	btrfs_create_pending_block_groups(trans);

	ret = btrfs_run_delayed_refs(trans, 0);
	if (ret) {
		btrfs_end_transaction(trans);
		return ret;
	}

	if (!test_bit(BTRFS_TRANS_DIRTY_BG_RUN, &cur_trans->flags)) {
		int run_it = 0;

		/* this mutex is also taken before trying to set
		 * block groups readonly.  We need to make sure
		 * that nobody has set a block group readonly
		 * after a extents from that block group have been
		 * allocated for cache files.  btrfs_set_block_group_ro
		 * will wait for the transaction to commit if it
		 * finds BTRFS_TRANS_DIRTY_BG_RUN set.
		 *
		 * The BTRFS_TRANS_DIRTY_BG_RUN flag is also used to make sure
		 * only one process starts all the block group IO.  It wouldn't
		 * hurt to have more than one go through, but there's no
		 * real advantage to it either.
		 */
		mutex_lock(&fs_info->ro_block_group_mutex);
		if (!test_and_set_bit(BTRFS_TRANS_DIRTY_BG_RUN,
				      &cur_trans->flags))
			run_it = 1;
		mutex_unlock(&fs_info->ro_block_group_mutex);

		if (run_it) {
			ret = btrfs_start_dirty_block_groups(trans);
			if (ret) {
				btrfs_end_transaction(trans);
				return ret;
			}
		}
	}

	spin_lock(&fs_info->trans_lock);
	if (cur_trans->state >= TRANS_STATE_COMMIT_START) {

		add_pending_snapshot(trans);

		spin_unlock(&fs_info->trans_lock);
		refcount_inc(&cur_trans->use_count);
		ret = btrfs_end_transaction(trans);

		wait_for_commit(cur_trans);

		if (TRANS_ABORTED(cur_trans))
			ret = cur_trans->aborted;

		btrfs_put_transaction(cur_trans);

		return ret;
	}

	cur_trans->state = TRANS_STATE_COMMIT_START;
	wake_up(&fs_info->transaction_blocked_wait);

	if (cur_trans->list.prev != &fs_info->trans_list) {
		prev_trans = list_entry(cur_trans->list.prev,
					struct btrfs_transaction, list);
		if (prev_trans->state != TRANS_STATE_COMPLETED) {
			refcount_inc(&prev_trans->use_count);
			spin_unlock(&fs_info->trans_lock);

			wait_for_commit(prev_trans);
			ret = READ_ONCE(prev_trans->aborted);

			btrfs_put_transaction(prev_trans);
			if (ret)
				goto cleanup_transaction;
		} else {
			spin_unlock(&fs_info->trans_lock);
		}
	} else {
		spin_unlock(&fs_info->trans_lock);
		/*
		 * The previous transaction was aborted and was already removed
		 * from the list of transactions at fs_info->trans_list. So we
		 * abort to prevent writing a new superblock that reflects a
		 * corrupt state (pointing to trees with unwritten nodes/leafs).
		 */
		if (test_bit(BTRFS_FS_STATE_TRANS_ABORTED, &fs_info->fs_state)) {
			ret = -EROFS;
			goto cleanup_transaction;
		}
	}

	extwriter_counter_dec(cur_trans, trans->type);

	ret = btrfs_start_delalloc_flush(trans);
	if (ret)
		goto cleanup_transaction;

	ret = btrfs_run_delayed_items(trans);
	if (ret)
		goto cleanup_transaction;

	wait_event(cur_trans->writer_wait,
		   extwriter_counter_read(cur_trans) == 0);

	/* some pending stuffs might be added after the previous flush. */
	ret = btrfs_run_delayed_items(trans);
	if (ret)
		goto cleanup_transaction;

	btrfs_wait_delalloc_flush(trans);

	btrfs_scrub_pause(fs_info);
	/*
	 * Ok now we need to make sure to block out any other joins while we
	 * commit the transaction.  We could have started a join before setting
	 * COMMIT_DOING so make sure to wait for num_writers to == 1 again.
	 */
	spin_lock(&fs_info->trans_lock);
	add_pending_snapshot(trans);
	cur_trans->state = TRANS_STATE_COMMIT_DOING;
	spin_unlock(&fs_info->trans_lock);
	wait_event(cur_trans->writer_wait,
		   atomic_read(&cur_trans->num_writers) == 1);

	if (TRANS_ABORTED(cur_trans)) {
		ret = cur_trans->aborted;
		goto scrub_continue;
	}
	/*
	 * the reloc mutex makes sure that we stop
	 * the balancing code from coming in and moving
	 * extents around in the middle of the commit
	 */
	mutex_lock(&fs_info->reloc_mutex);

	/*
	 * We needn't worry about the delayed items because we will
	 * deal with them in create_pending_snapshot(), which is the
	 * core function of the snapshot creation.
	 */
	ret = create_pending_snapshots(trans);
	if (ret) {
		mutex_unlock(&fs_info->reloc_mutex);
		goto scrub_continue;
	}

	/*
	 * We insert the dir indexes of the snapshots and update the inode
	 * of the snapshots' parents after the snapshot creation, so there
	 * are some delayed items which are not dealt with. Now deal with
	 * them.
	 *
	 * We needn't worry that this operation will corrupt the snapshots,
	 * because all the tree which are snapshoted will be forced to COW
	 * the nodes and leaves.
	 */
	ret = btrfs_run_delayed_items(trans);
	if (ret) {
		mutex_unlock(&fs_info->reloc_mutex);
		goto scrub_continue;
	}

	ret = btrfs_run_delayed_refs(trans, (unsigned long)-1);
	if (ret) {
		mutex_unlock(&fs_info->reloc_mutex);
		goto scrub_continue;
	}

	/*
	 * make sure none of the code above managed to slip in a
	 * delayed item
	 */
	btrfs_assert_delayed_root_empty(fs_info);

	WARN_ON(cur_trans != trans->transaction);

	/* btrfs_commit_tree_roots is responsible for getting the
	 * various roots consistent with each other.  Every pointer
	 * in the tree of tree roots has to point to the most up to date
	 * root for every subvolume and other tree.  So, we have to keep
	 * the tree logging code from jumping in and changing any
	 * of the trees.
	 *
	 * At this point in the commit, there can't be any tree-log
	 * writers, but a little lower down we drop the trans mutex
	 * and let new people in.  By holding the tree_log_mutex
	 * from now until after the super is written, we avoid races
	 * with the tree-log code.
	 */
	mutex_lock(&fs_info->tree_log_mutex);

	ret = commit_fs_roots(trans);
	if (ret) {
		mutex_unlock(&fs_info->tree_log_mutex);
		mutex_unlock(&fs_info->reloc_mutex);
		goto scrub_continue;
	}

	/*
	 * Since the transaction is done, we can apply the pending changes
	 * before the next transaction.
	 */
	btrfs_apply_pending_changes(fs_info);

	/* commit_fs_roots gets rid of all the tree log roots, it is now
	 * safe to free the root of tree log roots
	 */
	btrfs_free_log_root_tree(trans, fs_info);

	/*
	 * commit_fs_roots() can call btrfs_save_ino_cache(), which generates
	 * new delayed refs. Must handle them or qgroup can be wrong.
	 */
	ret = btrfs_run_delayed_refs(trans, (unsigned long)-1);
	if (ret) {
		mutex_unlock(&fs_info->tree_log_mutex);
		mutex_unlock(&fs_info->reloc_mutex);
		goto scrub_continue;
	}

	/*
	 * Since fs roots are all committed, we can get a quite accurate
	 * new_roots. So let's do quota accounting.
	 */
	ret = btrfs_qgroup_account_extents(trans);
	if (ret < 0) {
		mutex_unlock(&fs_info->tree_log_mutex);
		mutex_unlock(&fs_info->reloc_mutex);
		goto scrub_continue;
	}

	ret = commit_cowonly_roots(trans);
	if (ret) {
		mutex_unlock(&fs_info->tree_log_mutex);
		mutex_unlock(&fs_info->reloc_mutex);
		goto scrub_continue;
	}

	/*
	 * The tasks which save the space cache and inode cache may also
	 * update ->aborted, check it.
	 */
	if (TRANS_ABORTED(cur_trans)) {
		ret = cur_trans->aborted;
		mutex_unlock(&fs_info->tree_log_mutex);
		mutex_unlock(&fs_info->reloc_mutex);
		goto scrub_continue;
	}

	btrfs_prepare_extent_commit(fs_info);

	cur_trans = fs_info->running_transaction;

	btrfs_set_root_node(&fs_info->tree_root->root_item,
			    fs_info->tree_root->node);
	list_add_tail(&fs_info->tree_root->dirty_list,
		      &cur_trans->switch_commits);

	btrfs_set_root_node(&fs_info->chunk_root->root_item,
			    fs_info->chunk_root->node);
	list_add_tail(&fs_info->chunk_root->dirty_list,
		      &cur_trans->switch_commits);

	switch_commit_roots(trans);

	ASSERT(list_empty(&cur_trans->dirty_bgs));
	ASSERT(list_empty(&cur_trans->io_bgs));
	update_super_roots(fs_info);

	btrfs_set_super_log_root(fs_info->super_copy, 0);
	btrfs_set_super_log_root_level(fs_info->super_copy, 0);
	memcpy(fs_info->super_for_commit, fs_info->super_copy,
	       sizeof(*fs_info->super_copy));

	btrfs_commit_device_sizes(cur_trans);

	clear_bit(BTRFS_FS_LOG1_ERR, &fs_info->flags);
	clear_bit(BTRFS_FS_LOG2_ERR, &fs_info->flags);

	btrfs_trans_release_chunk_metadata(trans);

	spin_lock(&fs_info->trans_lock);
	cur_trans->state = TRANS_STATE_UNBLOCKED;
	fs_info->running_transaction = NULL;
	spin_unlock(&fs_info->trans_lock);
	mutex_unlock(&fs_info->reloc_mutex);

	wake_up(&fs_info->transaction_wait);

	ret = btrfs_write_and_wait_transaction(trans);
	if (ret) {
		btrfs_handle_fs_error(fs_info, ret,
				      "Error while writing out transaction");
		mutex_unlock(&fs_info->tree_log_mutex);
		goto scrub_continue;
	}

	ret = write_all_supers(fs_info, 0);
	/*
	 * the super is written, we can safely allow the tree-loggers
	 * to go about their business
	 */
	mutex_unlock(&fs_info->tree_log_mutex);
	if (ret)
		goto scrub_continue;

	btrfs_finish_extent_commit(trans);

	if (test_bit(BTRFS_TRANS_HAVE_FREE_BGS, &cur_trans->flags))
		btrfs_clear_space_info_full(fs_info);

	fs_info->last_trans_committed = cur_trans->transid;
	/*
	 * We needn't acquire the lock here because there is no other task
	 * which can change it.
	 */
	cur_trans->state = TRANS_STATE_COMPLETED;
	wake_up(&cur_trans->commit_wait);

	spin_lock(&fs_info->trans_lock);
	list_del_init(&cur_trans->list);
	spin_unlock(&fs_info->trans_lock);

	btrfs_put_transaction(cur_trans);
	btrfs_put_transaction(cur_trans);

	if (trans->type & __TRANS_FREEZABLE)
		sb_end_intwrite(fs_info->sb);

	trace_btrfs_transaction_commit(trans->root);

	btrfs_scrub_continue(fs_info);

	if (current->journal_info == trans)
		current->journal_info = NULL;

	kmem_cache_free(btrfs_trans_handle_cachep, trans);

	return ret;

scrub_continue:
	btrfs_scrub_continue(fs_info);
cleanup_transaction:
	btrfs_trans_release_metadata(trans);
	btrfs_cleanup_pending_block_groups(trans);
	btrfs_trans_release_chunk_metadata(trans);
	trans->block_rsv = NULL;
	btrfs_warn(fs_info, "Skipping commit of aborted transaction.");
	if (current->journal_info == trans)
		current->journal_info = NULL;
	cleanup_transaction(trans, ret);

	return ret;
}

/*
 * return < 0 if error
 * 0 if there are no more dead_roots at the time of call
 * 1 there are more to be processed, call me again
 *
 * The return value indicates there are certainly more snapshots to delete, but
 * if there comes a new one during processing, it may return 0. We don't mind,
 * because btrfs_commit_super will poke cleaner thread and it will process it a
 * few seconds later.
 */
int btrfs_clean_one_deleted_snapshot(struct btrfs_root *root)
{
	int ret;
	struct btrfs_fs_info *fs_info = root->fs_info;

	spin_lock(&fs_info->trans_lock);
	if (list_empty(&fs_info->dead_roots)) {
		spin_unlock(&fs_info->trans_lock);
		return 0;
	}
	root = list_first_entry(&fs_info->dead_roots,
			struct btrfs_root, root_list);
	list_del_init(&root->root_list);
	spin_unlock(&fs_info->trans_lock);

	btrfs_debug(fs_info, "cleaner removing %llu", root->root_key.objectid);

	btrfs_kill_all_delayed_nodes(root);

	if (btrfs_header_backref_rev(root->node) <
			BTRFS_MIXED_BACKREF_REV)
		ret = btrfs_drop_snapshot(root, NULL, 0, 0);
	else
		ret = btrfs_drop_snapshot(root, NULL, 1, 0);

	return (ret < 0) ? 0 : 1;
}

void btrfs_apply_pending_changes(struct btrfs_fs_info *fs_info)
{
	unsigned long prev;
	unsigned long bit;

	prev = xchg(&fs_info->pending_changes, 0);
	if (!prev)
		return;

	bit = 1 << BTRFS_PENDING_SET_INODE_MAP_CACHE;
	if (prev & bit)
		btrfs_set_opt(fs_info->mount_opt, INODE_MAP_CACHE);
	prev &= ~bit;

	bit = 1 << BTRFS_PENDING_CLEAR_INODE_MAP_CACHE;
	if (prev & bit)
		btrfs_clear_opt(fs_info->mount_opt, INODE_MAP_CACHE);
	prev &= ~bit;

	bit = 1 << BTRFS_PENDING_COMMIT;
	if (prev & bit)
		btrfs_debug(fs_info, "pending commit done");
	prev &= ~bit;

	if (prev)
		btrfs_warn(fs_info,
			"unknown pending changes left 0x%lx, ignoring", prev);
}
