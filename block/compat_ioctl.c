// SPDX-License-Identifier: GPL-2.0
#include <linux/blkdev.h>
#include <linux/blkpg.h>
#include <linux/blktrace_api.h>
#include <linux/cdrom.h>
#include <linux/compat.h>
#include <linux/elevator.h>
#include <linux/hdreg.h>
#include <linux/pr.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/types.h>
#include <linux/uaccess.h>

static int compat_put_ushort(unsigned long arg, unsigned short val)
{
	return put_user(val, (unsigned short __user *)compat_ptr(arg));
}

static int compat_put_int(unsigned long arg, int val)
{
	return put_user(val, (compat_int_t __user *)compat_ptr(arg));
}

static int compat_put_uint(unsigned long arg, unsigned int val)
{
	return put_user(val, (compat_uint_t __user *)compat_ptr(arg));
}

static int compat_put_long(unsigned long arg, long val)
{
	return put_user(val, (compat_long_t __user *)compat_ptr(arg));
}

static int compat_put_ulong(unsigned long arg, compat_ulong_t val)
{
	return put_user(val, (compat_ulong_t __user *)compat_ptr(arg));
}

static int compat_put_u64(unsigned long arg, u64 val)
{
	return put_user(val, (compat_u64 __user *)compat_ptr(arg));
}

struct compat_hd_geometry {
	unsigned char heads;
	unsigned char sectors;
	unsigned short cylinders;
	u32 start;
};

static int compat_hdio_getgeo(struct gendisk *disk, struct block_device *bdev,
			struct compat_hd_geometry __user *ugeo)
{
	struct hd_geometry geo;
	int ret;

	if (!ugeo)
		return -EINVAL;
	if (!disk->fops->getgeo)
		return -ENOTTY;

	memset(&geo, 0, sizeof(geo));
	/*
	 * We need to set the startsect first, the driver may
	 * want to override it.
	 */
	geo.start = get_start_sect(bdev);
	ret = disk->fops->getgeo(bdev, &geo);
	if (ret)
		return ret;

	ret = copy_to_user(ugeo, &geo, 4);
	ret |= put_user(geo.start, &ugeo->start);
	if (ret)
		ret = -EFAULT;

	return ret;
}

struct compat_blkpg_ioctl_arg {
	compat_int_t op;
	compat_int_t flags;
	compat_int_t datalen;
	compat_caddr_t data;
};

static int compat_blkpg_ioctl(struct block_device *bdev, fmode_t mode,
		unsigned int cmd, struct compat_blkpg_ioctl_arg __user *ua32)
{
	struct blkpg_ioctl_arg __user *a = compat_alloc_user_space(sizeof(*a));
	compat_caddr_t udata;
	compat_int_t n;
	int err;

	err = get_user(n, &ua32->op);
	err |= put_user(n, &a->op);
	err |= get_user(n, &ua32->flags);
	err |= put_user(n, &a->flags);
	err |= get_user(n, &ua32->datalen);
	err |= put_user(n, &a->datalen);
	err |= get_user(udata, &ua32->data);
	err |= put_user(compat_ptr(udata), &a->data);
	if (err)
		return err;

	return blkdev_ioctl(bdev, mode, cmd, (unsigned long)a);
}

#define BLKBSZGET_32		_IOR(0x12, 112, int)
#define BLKBSZSET_32		_IOW(0x12, 113, int)
#define BLKGETSIZE64_32		_IOR(0x12, 114, int)

/* Most of the generic ioctls are handled in the normal fallback path.
   This assumes the blkdev's low level compat_ioctl always returns
   ENOIOCTLCMD for unknown ioctls. */
long compat_blkdev_ioctl(struct file *file, unsigned cmd, unsigned long arg)
{
	int ret = -ENOIOCTLCMD;
	struct inode *inode = file->f_mapping->host;
	struct block_device *bdev = inode->i_bdev;
	struct gendisk *disk = bdev->bd_disk;
	fmode_t mode = file->f_mode;
	loff_t size;
	unsigned int max_sectors;

	/*
	 * O_NDELAY can be altered using fcntl(.., F_SETFL, ..), so we have
	 * to updated it before every ioctl.
	 */
	if (file->f_flags & O_NDELAY)
		mode |= FMODE_NDELAY;
	else
		mode &= ~FMODE_NDELAY;

	switch (cmd) {
	case HDIO_GETGEO:
		return compat_hdio_getgeo(disk, bdev, compat_ptr(arg));
	case BLKPBSZGET:
		return compat_put_uint(arg, bdev_physical_block_size(bdev));
	case BLKIOMIN:
		return compat_put_uint(arg, bdev_io_min(bdev));
	case BLKIOOPT:
		return compat_put_uint(arg, bdev_io_opt(bdev));
	case BLKALIGNOFF:
		return compat_put_int(arg, bdev_alignment_offset(bdev));
	case BLKDISCARDZEROES:
		return compat_put_uint(arg, 0);
	case BLKFLSBUF:
	case BLKROSET:
	case BLKDISCARD:
	case BLKSECDISCARD:
	case BLKZEROOUT:
	/*
	 * the ones below are implemented in blkdev_locked_ioctl,
	 * but we call blkdev_ioctl, which gets the lock for us
	 */
	case BLKRRPART:
	case BLKREPORTZONE:
	case BLKRESETZONE:
		return blkdev_ioctl(bdev, mode, cmd,
				(unsigned long)compat_ptr(arg));
	case BLKBSZSET_32:
		return blkdev_ioctl(bdev, mode, BLKBSZSET,
				(unsigned long)compat_ptr(arg));
	case BLKPG:
		return compat_blkpg_ioctl(bdev, mode, cmd, compat_ptr(arg));
	case BLKRAGET:
	case BLKFRAGET:
		if (!arg)
			return -EINVAL;
		return compat_put_long(arg,
			       (bdev->bd_bdi->ra_pages * PAGE_SIZE) / 512);
	case BLKROGET: /* compatible */
		return compat_put_int(arg, bdev_read_only(bdev) != 0);
	case BLKBSZGET_32: /* get the logical block size (cf. BLKSSZGET) */
		return compat_put_int(arg, block_size(bdev));
	case BLKSSZGET: /* get block device hardware sector size */
		return compat_put_int(arg, bdev_logical_block_size(bdev));
	case BLKSECTGET:
		max_sectors = min_t(unsigned int, USHRT_MAX,
				    queue_max_sectors(bdev_get_queue(bdev)));
		return compat_put_ushort(arg, max_sectors);
	case BLKROTATIONAL:
		return compat_put_ushort(arg,
					 !blk_queue_nonrot(bdev_get_queue(bdev)));
	case BLKRASET: /* compatible, but no compat_ptr (!) */
	case BLKFRASET:
		if (!capable(CAP_SYS_ADMIN))
			return -EACCES;
		bdev->bd_bdi->ra_pages = (arg * 512) / PAGE_SIZE;
		return 0;
	case BLKGETSIZE:
		size = i_size_read(bdev->bd_inode);
		if ((size >> 9) > ~(compat_ulong_t)0)
			return -EFBIG;
		return compat_put_ulong(arg, size >> 9);

	case BLKGETSIZE64_32:
		return compat_put_u64(arg, i_size_read(bdev->bd_inode));

	case BLKTRACESETUP32:
	case BLKTRACESTART: /* compatible */
	case BLKTRACESTOP:  /* compatible */
	case BLKTRACETEARDOWN: /* compatible */
		ret = blk_trace_ioctl(bdev, cmd, compat_ptr(arg));
		return ret;
	case IOC_PR_REGISTER:
	case IOC_PR_RESERVE:
	case IOC_PR_RELEASE:
	case IOC_PR_PREEMPT:
	case IOC_PR_PREEMPT_ABORT:
	case IOC_PR_CLEAR:
		return blkdev_ioctl(bdev, mode, cmd,
				(unsigned long)compat_ptr(arg));
	default:
		if (disk->fops->compat_ioctl)
			ret = disk->fops->compat_ioctl(bdev, mode, cmd, arg);
		return ret;
	}
}
