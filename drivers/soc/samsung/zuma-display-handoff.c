// SPDX-License-Identifier: GPL-2.0-only
/*
 * Display bootloader handoff for Google Zuma/Husky.
 *
 * The shipping boot chain leaves a working HK3 command-mode display pipeline.
 * Until native Zuma display support exists, sample only the powered handoff
 * state so that framebuffer loss can be distinguished from a power, DECON,
 * DPP, DSIM, or panel failure.  This handoff driver never writes display MMIO.
 */

#include <linux/bitops.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#define ZUMA_PD_STATUS                  0x0004
#define ZUMA_PD_ON                      BIT(0)

#define ZUMA_DPUB_BASE                  0x15461d00
#define ZUMA_DPUF0_BASE                 0x15461d80
#define ZUMA_DPUF1_BASE                 0x15461e00
#define ZUMA_PD_MIN_SIZE                0x20

#define ZUMA_DSIM0_BASE                 0x19440000
#define ZUMA_DSIM0_MIN_SIZE             0x100
#define ZUMA_DSIM_LINK_STATUS0          0x0008
#define ZUMA_DSIM_LINK_STATUS1          0x000c
#define ZUMA_DSIM_LINK_STATUS3          0x0014
#define ZUMA_DSIM_MIPI_STATUS           0x0018
#define ZUMA_DSIM_DPHY_STATUS           0x001c
#define ZUMA_DSIM_CLK_CTRL              0x0020
#define ZUMA_DSIM_RESOL                 0x003c
#define ZUMA_DSIM_CONFIG                0x004c

#define ZUMA_DECON0_BASE                0x19470000
#define ZUMA_DECON0_MIN_SIZE            0x300
#define ZUMA_DECON_VERSION              0x0000
#define ZUMA_DECON_FRAME_COUNT          0x0004
#define ZUMA_DECON_GLOBAL_CON           0x0020
#define ZUMA_DECON_TRIG_CON             0x0030
#define ZUMA_DECON_OF_SIZE_0            0x0290
#define ZUMA_DECON_OF_PIXEL_ORDER       0x02a0

#define ZUMA_DPP0_BASE                  0x19900000
#define ZUMA_DPP0_MIN_SIZE              0x100
#define ZUMA_DPP_RDMA_ENABLE            0x0000
#define ZUMA_DPP_RDMA_IN_CTRL_0         0x0008
#define ZUMA_DPP_RDMA_SRC_WIDTH         0x0010
#define ZUMA_DPP_RDMA_SRC_HEIGHT        0x0014
#define ZUMA_DPP_RDMA_SRC_OFFSET        0x0018
#define ZUMA_DPP_RDMA_IMG_SIZE          0x001c
#define ZUMA_DPP_RDMA_BASEADDR_P0       0x0040
#define ZUMA_DPP_RDMA_SRC_STRIDE_0      0x0050
#define ZUMA_DPP_FORMAT_SHIFT           8
#define ZUMA_DPP_FORMAT_MASK            0x3f

#define ZUMA_HANDOFF_FB_BASE            0xfac00000
#define ZUMA_HANDOFF_FB_WIDTH           1344
#define ZUMA_HANDOFF_FB_HEIGHT          2992
#define ZUMA_HANDOFF_FB_CTRL            0xff400000
#define ZUMA_HANDOFF_FB_PIXELS          \
	(ZUMA_HANDOFF_FB_WIDTH * ZUMA_HANDOFF_FB_HEIGHT)
#define ZUMA_HANDOFF_FB_SIZE            \
	(ZUMA_HANDOFF_FB_PIXELS * sizeof(u32))

extern bool zuma_husky_boot_framebuffer_reserved;

struct zuma_display_block {
	const char *name;
	const char *compatible;
	resource_size_t phys;
	resource_size_t min_size;
	void __iomem *base;
};

static struct zuma_display_block zuma_dpub = {
	.name = "DPUB power domain",
	.compatible = "samsung,exynos-pd",
	.phys = ZUMA_DPUB_BASE,
	.min_size = ZUMA_PD_MIN_SIZE,
};

static struct zuma_display_block zuma_dpuf0 = {
	.name = "DPUF0 power domain",
	.compatible = "samsung,exynos-pd",
	.phys = ZUMA_DPUF0_BASE,
	.min_size = ZUMA_PD_MIN_SIZE,
};

static struct zuma_display_block zuma_dpuf1 = {
	.name = "DPUF1 power domain",
	.compatible = "samsung,exynos-pd",
	.phys = ZUMA_DPUF1_BASE,
	.min_size = ZUMA_PD_MIN_SIZE,
};

static struct zuma_display_block zuma_dsim0 = {
	.name = "DSIM0",
	.compatible = "samsung,exynos-dsim",
	.phys = ZUMA_DSIM0_BASE,
	.min_size = ZUMA_DSIM0_MIN_SIZE,
};

static struct zuma_display_block zuma_decon0 = {
	.name = "DECON0",
	.compatible = "samsung,exynos-decon",
	.phys = ZUMA_DECON0_BASE,
	.min_size = ZUMA_DECON0_MIN_SIZE,
};

static struct zuma_display_block zuma_dpp0 = {
	.name = "DPP0 RDMA",
	.compatible = "samsung,exynos-dpp",
	.phys = ZUMA_DPP0_BASE,
	.min_size = ZUMA_DPP0_MIN_SIZE,
};

static struct zuma_display_block * const zuma_display_blocks[] = {
	&zuma_dpub,
	&zuma_dpuf0,
	&zuma_dpuf1,
	&zuma_dsim0,
	&zuma_decon0,
	&zuma_dpp0,
};

static const unsigned int zuma_snapshot_intervals_ms[] = {
	1000,
	2000,
	3000,
	4000,
};

static const char * const zuma_snapshot_labels[] = {
	"1s",
	"3s",
	"6s",
	"10s",
};

static unsigned int zuma_snapshot_index;
static void zuma_display_snapshot_workfn(struct work_struct *work);
static DECLARE_DELAYED_WORK(zuma_display_snapshot_work,
				 zuma_display_snapshot_workfn);

static int __init zuma_display_map(struct zuma_display_block *block)
{
	struct device_node *np = NULL;
	struct resource res;
	int ret;

	while ((np = of_find_compatible_node(np, NULL, block->compatible))) {
		ret = of_address_to_resource(np, 0, &res);
		if (ret || res.start != block->phys)
			continue;

		if (!of_device_is_available(np)) {
			pr_err("zuma-display-handoff: refusing disabled %s node %pOF\n",
			       block->name, np);
			ret = -ENODEV;
			goto out_put;
		}

		if (resource_size(&res) < block->min_size) {
			pr_err("zuma-display-handoff: refusing short %s resource %pr\n",
			       block->name, &res);
			ret = -EINVAL;
			goto out_put;
		}

		block->base = ioremap(res.start, block->min_size);
		if (!block->base) {
			ret = -ENOMEM;
			goto out_put;
		}

		of_node_put(np);
		return 0;
	}

	pr_err("zuma-display-handoff: exact %s resource at %pa not found\n",
	       block->name, &block->phys);
	return -ENODEV;

out_put:
	of_node_put(np);
	return ret;
}

static void zuma_display_unmap_all(void)
{
	int i;

	for (i = ARRAY_SIZE(zuma_display_blocks) - 1; i >= 0; i--) {
		if (zuma_display_blocks[i]->base) {
			iounmap(zuma_display_blocks[i]->base);
			zuma_display_blocks[i]->base = NULL;
		}
	}
}

static bool zuma_display_domains_on(u32 *dpub, u32 *dpuf0, u32 *dpuf1)
{
	*dpub = readl(zuma_dpub.base + ZUMA_PD_STATUS);
	*dpuf0 = readl(zuma_dpuf0.base + ZUMA_PD_STATUS);
	*dpuf1 = readl(zuma_dpuf1.base + ZUMA_PD_STATUS);

	return (*dpub & ZUMA_PD_ON) && (*dpuf0 & ZUMA_PD_ON);
}

static void zuma_display_snapshot(const char *label)
{
	u32 dpub, dpuf0, dpuf1;
	u32 rdma_ctrl;

	if (!zuma_display_domains_on(&dpub, &dpuf0, &dpuf1)) {
		pr_info("zuma-display-handoff: %s domains DPUB=%#x DPUF0=%#x DPUF1=%#x; display reads skipped\n",
			label, dpub, dpuf0, dpuf1);
		return;
	}

	pr_info("zuma-display-handoff: %s domains DPUB=%#x DPUF0=%#x DPUF1=%#x\n",
		label, dpub, dpuf0, dpuf1);
	pr_info("zuma-display-handoff: %s DECON version=%#x frame=%#x global=%#x trigger=%#x size=%#x order=%#x\n",
		label,
		readl(zuma_decon0.base + ZUMA_DECON_VERSION),
		readl(zuma_decon0.base + ZUMA_DECON_FRAME_COUNT),
		readl(zuma_decon0.base + ZUMA_DECON_GLOBAL_CON),
		readl(zuma_decon0.base + ZUMA_DECON_TRIG_CON),
		readl(zuma_decon0.base + ZUMA_DECON_OF_SIZE_0),
		readl(zuma_decon0.base + ZUMA_DECON_OF_PIXEL_ORDER));

	rdma_ctrl = readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IN_CTRL_0);
	pr_info("zuma-display-handoff: %s DPP0 enable=%#x ctrl=%#x format=%u src=%#xx%#x offset=%#x image=%#x base=%#x stride=%#x\n",
		label,
		readl(zuma_dpp0.base + ZUMA_DPP_RDMA_ENABLE),
		rdma_ctrl,
		(rdma_ctrl >> ZUMA_DPP_FORMAT_SHIFT) & ZUMA_DPP_FORMAT_MASK,
		readl(zuma_dpp0.base + ZUMA_DPP_RDMA_SRC_WIDTH),
		readl(zuma_dpp0.base + ZUMA_DPP_RDMA_SRC_HEIGHT),
		readl(zuma_dpp0.base + ZUMA_DPP_RDMA_SRC_OFFSET),
		readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IMG_SIZE),
		readl(zuma_dpp0.base + ZUMA_DPP_RDMA_BASEADDR_P0),
		readl(zuma_dpp0.base + ZUMA_DPP_RDMA_SRC_STRIDE_0));

	pr_info("zuma-display-handoff: %s DSIM link0=%#x link1=%#x link3=%#x mipi=%#x dphy=%#x clock=%#x resolution=%#x config=%#x\n",
		label,
		readl(zuma_dsim0.base + ZUMA_DSIM_LINK_STATUS0),
		readl(zuma_dsim0.base + ZUMA_DSIM_LINK_STATUS1),
		readl(zuma_dsim0.base + ZUMA_DSIM_LINK_STATUS3),
		readl(zuma_dsim0.base + ZUMA_DSIM_MIPI_STATUS),
		readl(zuma_dsim0.base + ZUMA_DSIM_DPHY_STATUS),
		readl(zuma_dsim0.base + ZUMA_DSIM_CLK_CTRL),
		readl(zuma_dsim0.base + ZUMA_DSIM_RESOL),
		readl(zuma_dsim0.base + ZUMA_DSIM_CONFIG));
}

static void __init zuma_display_scan_framebuffer(void)
{
	const u32 *pixels, *pixel;
	resource_size_t fb_base;
	u64 zero = 0, opaque_black = 0, opaque_white = 0;
	u64 alpha_ff = 0, nonblack = 0;
	u32 dpub, dpuf0, dpuf1;
	u32 ctrl, width, height, offset, image, stride;
	u32 min_x = ZUMA_HANDOFF_FB_WIDTH;
	u32 min_y = ZUMA_HANDOFF_FB_HEIGHT;
	u32 max_x = 0, max_y = 0;
	unsigned long pfn, end_pfn;
	u32 top_left, center, bottom_right;
	u32 x, y;

	if (!zuma_display_domains_on(&dpub, &dpuf0, &dpuf1)) {
		pr_info("zuma-display-handoff: framebuffer scan skipped; display domains are off\n");
		return;
	}

	ctrl = readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IN_CTRL_0);
	width = readl(zuma_dpp0.base + ZUMA_DPP_RDMA_SRC_WIDTH);
	height = readl(zuma_dpp0.base + ZUMA_DPP_RDMA_SRC_HEIGHT);
	offset = readl(zuma_dpp0.base + ZUMA_DPP_RDMA_SRC_OFFSET);
	image = readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IMG_SIZE);
	fb_base = readl(zuma_dpp0.base + ZUMA_DPP_RDMA_BASEADDR_P0);
	stride = readl(zuma_dpp0.base + ZUMA_DPP_RDMA_SRC_STRIDE_0);

	if (fb_base != ZUMA_HANDOFF_FB_BASE ||
	    width != ZUMA_HANDOFF_FB_WIDTH ||
	    height != ZUMA_HANDOFF_FB_HEIGHT ||
	    offset ||
	    image != ((ZUMA_HANDOFF_FB_HEIGHT << 16) |
		      ZUMA_HANDOFF_FB_WIDTH) ||
	    stride ||
	    ctrl != ZUMA_HANDOFF_FB_CTRL) {
		pr_err("zuma-display-handoff: refusing unexpected framebuffer layout base=%pa ctrl=%#x src=%ux%u offset=%#x image=%#x stride=%#x\n",
		       &fb_base, ctrl, width, height, offset, image, stride);
		return;
	}

	if (region_intersects(fb_base, ZUMA_HANDOFF_FB_SIZE,
			      IORESOURCE_SYSTEM_RAM, IORES_DESC_NONE) !=
	    REGION_INTERSECTS) {
		pr_err("zuma-display-handoff: framebuffer candidate %pa+%#zx is not System RAM\n",
		       &fb_base, (size_t)ZUMA_HANDOFF_FB_SIZE);
		return;
	}

	pfn = PHYS_PFN(fb_base);
	end_pfn = PHYS_PFN(fb_base + ZUMA_HANDOFF_FB_SIZE);
	for (; pfn < end_pfn; pfn++) {
		if (!pfn_is_map_memory(pfn)) {
			pr_err("zuma-display-handoff: framebuffer PFN %#lx is not direct-mapped RAM\n",
			       pfn);
			return;
		}
	}

	pixels = memremap(fb_base, ZUMA_HANDOFF_FB_SIZE, MEMREMAP_WB);
	if (!pixels) {
		pr_err("zuma-display-handoff: read-only framebuffer mapping failed\n");
		return;
	}

	pixel = pixels;
	for (y = 0; y < ZUMA_HANDOFF_FB_HEIGHT; y++) {
		for (x = 0; x < ZUMA_HANDOFF_FB_WIDTH; x++, pixel++) {
			u32 value = READ_ONCE(*pixel);

			if (!value)
				zero++;
			if (value == 0xff000000)
				opaque_black++;
			if (value == 0xffffffff)
				opaque_white++;
			if ((value & 0xff000000) == 0xff000000)
				alpha_ff++;
			if (!(value & 0x00ffffff))
				continue;

			nonblack++;
			min_x = min(min_x, x);
			min_y = min(min_y, y);
			max_x = max(max_x, x);
			max_y = max(max_y, y);
		}
	}

	top_left = READ_ONCE(pixels[0]);
	center = READ_ONCE(pixels[(ZUMA_HANDOFF_FB_HEIGHT / 2) *
				 ZUMA_HANDOFF_FB_WIDTH + ZUMA_HANDOFF_FB_WIDTH / 2]);
	bottom_right = READ_ONCE(pixels[ZUMA_HANDOFF_FB_PIXELS - 1]);
	memunmap((void *)pixels);

	pr_info("zuma-display-handoff: framebuffer read-only scan base=%pa size=%#zx pixels=%u zero=%llu opaque-black=%llu opaque-white=%llu alpha-ff=%llu nonblack=%llu\n",
		&fb_base, (size_t)ZUMA_HANDOFF_FB_SIZE,
		ZUMA_HANDOFF_FB_PIXELS,
		(unsigned long long)zero,
		(unsigned long long)opaque_black,
		(unsigned long long)opaque_white,
		(unsigned long long)alpha_ff,
		(unsigned long long)nonblack);
	pr_info("zuma-display-handoff: framebuffer samples top-left=%#x center=%#x bottom-right=%#x\n",
		top_left, center, bottom_right);
	if (nonblack)
		pr_info("zuma-display-handoff: framebuffer nonblack bounds x=%u..%u y=%u..%u\n",
			min_x, max_x, min_y, max_y);
}

static void zuma_display_snapshot_workfn(struct work_struct *work)
{
	unsigned int index = zuma_snapshot_index;

	zuma_display_snapshot(zuma_snapshot_labels[index]);
	index++;
	zuma_snapshot_index = index;

	if (index < ARRAY_SIZE(zuma_snapshot_intervals_ms)) {
		schedule_delayed_work(&zuma_display_snapshot_work,
				      msecs_to_jiffies(zuma_snapshot_intervals_ms[index]));
		return;
	}

	zuma_display_unmap_all();
	pr_info("zuma-display-handoff: completed read-only timed snapshots\n");
}

static int __init zuma_display_handoff_init(void)
{
	struct device_node *root;
	const char *model;
	u32 dpub, dpuf0, dpuf1;
	int i, ret;

	root = of_find_node_by_path("/");
	if (!root)
		return 0;

	if (!of_device_is_compatible(root, "google,zuma") ||
	    of_property_read_string(root, "model", &model) ||
	    strcmp(model, "ZUMA HUSKY MP based on ZUMA")) {
		of_node_put(root);
		return 0;
	}
	of_node_put(root);

	for (i = 0; i < ARRAY_SIZE(zuma_display_blocks); i++) {
		ret = zuma_display_map(zuma_display_blocks[i]);
		if (ret)
			goto out_unmap;
	}

	if (!zuma_display_domains_on(&dpub, &dpuf0, &dpuf1)) {
		pr_info("zuma-display-handoff: early domains DPUB=%#x DPUF0=%#x DPUF1=%#x; refusing display MMIO\n",
			dpub, dpuf0, dpuf1);
		goto out_unmap;
	}

	pr_info("zuma-display-handoff: guarded read-only Zuma/Husky snapshot active\n");
	zuma_display_snapshot("early");
	if (zuma_husky_boot_framebuffer_reserved)
		zuma_display_scan_framebuffer();
	else
		pr_crit("zuma-display-handoff: framebuffer reservation unavailable; scan skipped\n");
	schedule_delayed_work(&zuma_display_snapshot_work,
			      msecs_to_jiffies(zuma_snapshot_intervals_ms[0]));
	return 0;

out_unmap:
	zuma_display_unmap_all();
	return 0;
}
early_initcall(zuma_display_handoff_init);
