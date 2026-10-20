// SPDX-License-Identifier: GPL-2.0-only
/*
 * Knup KP-RW407/AX firmware-container parser.
 *
 * The stock Realtek stage-2 requires a real rootfs at flash 0x260000, so the
 * firmware container keeps the rootfs there and splits the kernel around it.
 * This parser reads the container header and exposes the real rootfs as
 * "rootfs0" plus the JFFS2 area after the kernel as "rootfs_data".
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/of.h>
#include <linux/byteorder/generic.h>

#include "mtdsplit.h"

#define RW407_MAGIC		0x52574e4b	/* 'KNWR' */
#define RW407_VERSION		1
#define RW407_CVIMG_HDR		0x10
#define RW407_HDR_OFF		0x10000
#define RW407_ROOTFS0_NAME	"rootfs0"
#define RW407_DATA_NAME		"rootfs_data"

struct rw407_header {
	__le32 magic;
	__le32 version;
	__le32 header_size;
	__le32 kernel_a_off;
	__le32 kernel_a_len;
	__le32 kernel_b_off;
	__le32 kernel_b_len;
	__le32 kernel_load;
	__le32 kernel_entry;
	__le32 rootfs_off;
	__le32 rootfs_len;
	__le32 rootfs_data_off;
	__le32 body_len;
	__le32 reserved[3];
} __packed;

static int mtdsplit_parse_rw407(struct mtd_info *master,
				const struct mtd_partition **pparts,
				struct mtd_part_parser_data *data)
{
	struct rw407_header h;
	struct mtd_partition *parts;
	size_t retlen;
	u32 rootfs_off, rootfs_len, data_off;
	int ret;

	ret = mtd_read(master, RW407_CVIMG_HDR + RW407_HDR_OFF,
		       sizeof(h), &retlen, (void *)&h);
	if (ret || retlen != sizeof(h))
		return -EIO;

	if (le32_to_cpu(h.magic) != RW407_MAGIC ||
	    le32_to_cpu(h.version) != RW407_VERSION)
		return -EINVAL;

	rootfs_off = le32_to_cpu(h.rootfs_off) + RW407_CVIMG_HDR;
	rootfs_len = le32_to_cpu(h.rootfs_len);
	data_off = le32_to_cpu(h.rootfs_data_off) + RW407_CVIMG_HDR;

	if (!rootfs_len || rootfs_off >= master->size ||
	    rootfs_off + rootfs_len > master->size ||
	    data_off <= rootfs_off + rootfs_len || data_off >= master->size)
		return -EINVAL;

	ret = mtd_check_rootfs_magic(master, rootfs_off, NULL);
	if (ret)
		return ret;

	parts = kcalloc(2, sizeof(*parts), GFP_KERNEL);
	if (!parts)
		return -ENOMEM;

	parts[0].name = RW407_ROOTFS0_NAME;
	parts[0].offset = rootfs_off;
	/*
	 * The squashfs file is larger than bytes_used (mksquashfs pads it, and
	 * the Realtek trailer follows it).  Give the block device the whole
	 * area up to rootfs_data; squashfs reads only its own bytes_used.
	 */
	parts[0].size = data_off - rootfs_off;

	parts[1].name = RW407_DATA_NAME;
	parts[1].offset = data_off;
	parts[1].size = master->size - data_off;

	*pparts = parts;
	return 2;
}

static const struct of_device_id mtdsplit_rw407_of_match_table[] = {
	{ .compatible = "knup,rw407-firmware" },
	{},
};
MODULE_DEVICE_TABLE(of, mtdsplit_rw407_of_match_table);

static struct mtd_part_parser mtdsplit_rw407_parser = {
	.owner = THIS_MODULE,
	.name = "rw407-firmware",
	.of_match_table = mtdsplit_rw407_of_match_table,
	.parse_fn = mtdsplit_parse_rw407,
	.type = MTD_PARSER_TYPE_FIRMWARE,
};

module_mtd_part_parser(mtdsplit_rw407_parser);
MODULE_LICENSE("GPL");
