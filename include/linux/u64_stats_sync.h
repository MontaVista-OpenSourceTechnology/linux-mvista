/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_U64_STATS_SYNC_H
#define _LINUX_U64_STATS_SYNC_H

/*
 * To properly implement 64bits network statistics on 32bit and 64bit hosts,
 * we provide a synchronization point, that is a noop on 64bit or UP kernels.
 *
 * Key points :
 * 1) Use a seqcount on SMP 32bits, with low overhead.
 * 2) Whole thing is a noop on 64bit arches or UP kernels.
 * 3) Write side must ensure mutual exclusion or one seqcount update could
 *    be lost, thus blocking readers forever.
 *    If this synchronization point is not a mutex, but a spinlock or
 *    spinlock_bh() or disable_bh() :
 * 3.1) Write side should not sleep.
 * 3.2) Write side should not allow preemption.
 * 3.3) If applicable, interrupts should be disabled.
 *
 * 4) If reader fetches several counters, there is no guarantee the whole values
 *    are consistent (remember point 1) : this is a noop on 64bit arches anyway)
 *
 * 5) readers are allowed to sleep or be preempted/interrupted : They perform
 *    pure reads. But if they have to fetch many values, it's better to not allow
 *    preemptions/interruptions to avoid many retries.
 *
 * 6) If counter might be written by an interrupt, readers should block interrupts.
 *    (On UP, there is no seqcount_t protection, a reader allowing interrupts could
 *     read partial values)
 *
 * 7) For irq and softirq uses, readers can use u64_stats_fetch_begin_irq() and
 *    u64_stats_fetch_retry_irq() helpers
 *
 * Usage :
 *
 * Stats producer (writer) should use following template granted it already got
 * an exclusive access to counters (a lock is already taken, or per cpu
 * data is used [in a non preemptable context])
 *
 *   spin_lock_bh(...) or other synchronization to get exclusive access
 *   ...
 *   u64_stats_update_begin(&stats->syncp);
 *   u64_stats_add(&stats->bytes64, len); // non atomic operation
 *   u64_stats_inc(&stats->packets64);    // non atomic operation
 *   u64_stats_update_end(&stats->syncp);
 *
 * While a consumer (reader) should use following template to get consistent
 * snapshot for each variable (but no guarantee on several ones)
 *
 * u64 tbytes, tpackets;
 * unsigned int start;
 *
 * do {
 *         start = u64_stats_fetch_begin(&stats->syncp);
 *         tbytes = u64_stats_read(&stats->bytes64); // non atomic operation
 *         tpackets = u64_stats_read(&stats->packets64); // non atomic operation
 * } while (u64_stats_fetch_retry(&stats->syncp, start));
 *
 *
 * Example of use in drivers/net/loopback.c, using per_cpu containers,
 * in BH disabled context.
 */
#include <linux/seqlock.h>

struct u64_stats_sync {
#if BITS_PER_LONG==32 && defined(CONFIG_SMP)
	seqcount_t	seq;
#endif
};

#if BITS_PER_LONG == 64
#include <asm/local64.h>

typedef struct {
	local64_t	v;
} u64_stats_t ;

static inline u64 u64_stats_read(const u64_stats_t *p)
{
	return local64_read(&p->v);
}

static inline void u64_stats_set(u64_stats_t *p, u64 val)
{
	local64_set(&p->v, val);
}

static inline void u64_stats_add(u64_stats_t *p, unsigned long val)
{
	local64_add(val, &p->v);
}

static inline void u64_stats_inc(u64_stats_t *p)
{
	local64_inc(&p->v);
}

#else

typedef struct {
	u64		v;
} u64_stats_t;

static inline u64 u64_stats_read(const u64_stats_t *p)
{
	return p->v;
}

static inline void u64_stats_set(u64_stats_t *p, u64 val)
{
	p->v = val;
}

static inline void u64_stats_add(u64_stats_t *p, unsigned long val)
{
	p->v += val;
}

static inline void u64_stats_inc(u64_stats_t *p)
{
	p->v++;
}
#endif

#if BITS_PER_LONG == 32 && defined(CONFIG_SMP)
#define u64_stats_init(syncp)				\
	do {						\
		struct u64_stats_sync *__s = (syncp);	\
		seqcount_init(&__s->seq);		\
	} while (0)
#else
static inline void u64_stats_init(struct u64_stats_sync *syncp)
{
}
#endif

static inline void u64_stats_update_begin(struct u64_stats_sync *syncp)
{
#if BITS_PER_LONG==32 && defined(CONFIG_SMP)
	write_seqcount_begin(&syncp->seq);
#endif
}

static inline void u64_stats_update_end(struct u64_stats_sync *syncp)
{
#if BITS_PER_LONG==32 && defined(CONFIG_SMP)
	write_seqcount_end(&syncp->seq);
#endif
}

static inline unsigned long
u64_stats_update_begin_irqsave(struct u64_stats_sync *syncp)
{
	unsigned long flags = 0;

#if BITS_PER_LONG==32 && defined(CONFIG_SMP)
	local_irq_save(flags);
	write_seqcount_begin(&syncp->seq);
#endif
	return flags;
}

static inline void
u64_stats_update_end_irqrestore(struct u64_stats_sync *syncp,
				unsigned long flags)
{
#if BITS_PER_LONG==32 && defined(CONFIG_SMP)
	write_seqcount_end(&syncp->seq);
	local_irq_restore(flags);
#endif
}

static inline unsigned int __u64_stats_fetch_begin(const struct u64_stats_sync *syncp)
{
#if BITS_PER_LONG==32 && defined(CONFIG_SMP)
	return read_seqcount_begin(&syncp->seq);
#else
	return 0;
#endif
}

static inline unsigned int u64_stats_fetch_begin(const struct u64_stats_sync *syncp)
{
#if BITS_PER_LONG==32 && !defined(CONFIG_SMP)
	preempt_disable();
#endif
	return __u64_stats_fetch_begin(syncp);
}

static inline bool __u64_stats_fetch_retry(const struct u64_stats_sync *syncp,
					 unsigned int start)
{
#if BITS_PER_LONG==32 && defined(CONFIG_SMP)
	return read_seqcount_retry(&syncp->seq, start);
#else
	return false;
#endif
}

static inline bool u64_stats_fetch_retry(const struct u64_stats_sync *syncp,
					 unsigned int start)
{
#if BITS_PER_LONG==32 && !defined(CONFIG_SMP)
	preempt_enable();
#endif
	return __u64_stats_fetch_retry(syncp, start);
}

/*
 * In case irq handlers can update u64 counters, readers can use following helpers
 * - SMP 32bit arches use seqcount protection, irq safe.
 * - UP 32bit must disable irqs.
 * - 64bit have no problem atomically reading u64 values, irq safe.
 */
static inline unsigned int u64_stats_fetch_begin_irq(const struct u64_stats_sync *syncp)
{
#if BITS_PER_LONG==32 && !defined(CONFIG_SMP)
	local_irq_disable();
#endif
	return __u64_stats_fetch_begin(syncp);
}

static inline bool u64_stats_fetch_retry_irq(const struct u64_stats_sync *syncp,
					     unsigned int start)
{
#if BITS_PER_LONG==32 && !defined(CONFIG_SMP)
	local_irq_enable();
#endif
	return __u64_stats_fetch_retry(syncp, start);
}

#endif /* _LINUX_U64_STATS_SYNC_H */
