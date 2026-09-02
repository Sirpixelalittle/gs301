// SPDX-License-Identifier: GPL-2.0-only
/*
 * Display bootloader handoff for Google Zuma/Husky.
 *
 * The shipping boot chain leaves a working HK3 command-mode display pipeline.
 * Until native Zuma DRM/KMS support exists, preserve that state and expose its
 * reserved linear buffer through fbdev.  Initial adoption may copy and switch
 * only the DPP base; later framebuffer updates only admit a hardware trigger.
 */

#include <linux/bitops.h>
#include <linux/dma-direction.h>
#include <linux/dma-map-ops.h>
#include <linux/fb.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/printk.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/soc/samsung/zuma-display-handoff.h>
#include <linux/workqueue.h>

#include <asm/cache.h>

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
#define ZUMA_DSIM_LINK_CMD_ACTIVE       BIT(26)
#define ZUMA_DSIM_MIPI_FRAME_PROCESSING BIT(29)
#define ZUMA_DSIM_DPHY_STATUS           0x001c
#define ZUMA_DSIM_CLK_CTRL              0x0020
#define ZUMA_DSIM_RESOL                 0x003c
#define ZUMA_DSIM_CONFIG                0x004c

#define ZUMA_DECON0_BASE                0x19470000
#define ZUMA_DECON0_MIN_SIZE            0x300
#define ZUMA_DECON0_WINCON_BASE         0x194a0000
#define ZUMA_DECON0_WINCON_MIN_SIZE     0xe000
#define ZUMA_DECON0_WINCON_RESOURCE     3
#define ZUMA_DECON_ACTIVE_WINDOW        5
#define ZUMA_DECON_WINDOW_COUNT         14
#define ZUMA_DECON_WINCON_STRIDE        0x1000
#define ZUMA_DECON_WINCON_ENABLE        BIT(0)
#define ZUMA_DECON_WINCON_EXPECTED      ZUMA_DECON_WINCON_ENABLE
#define ZUMA_DECON_SHD_REQ_ACTIVE       BIT(ZUMA_DECON_ACTIVE_WINDOW)
#define ZUMA_DECON_VERSION              0x0000
#define ZUMA_DECON_FRAME_COUNT          0x0004
#define ZUMA_DECON_GLOBAL_CON           0x0020
#define ZUMA_DECON_TRIG_CON             0x0030
#define ZUMA_DECON_SHD_REG_UP_REQ       0x0050
#define ZUMA_DECON_INT_EN               0x0060
#define ZUMA_DECON_OF_SIZE_0            0x0290
#define ZUMA_DECON_OF_PIXEL_ORDER       0x02a0

#define ZUMA_DECON_GLOBAL_EXPECTED      0x0133
#define ZUMA_DECON_GLOBAL_IDLE          BIT(5)
#define ZUMA_DECON_TRIG_EXPECTED        0x3070
#define ZUMA_DECON_TRIG_HW_MASK         BIT(4)
#define ZUMA_DECON_TRIG_HW_EN           BIT(0)
#define ZUMA_DECON_INT_EXPECTED         0x3000

#define ZUMA_DPP0_BASE                  0x19900000
#define ZUMA_DPP0_MIN_SIZE              0x500

#define ZUMA_SYSMMU_DPUF0_BASE          0x19840000
#define ZUMA_SYSMMU_DPUF0_MIN_SIZE      0x9000
#define ZUMA_SYSMMU_MMU_CTRL            0x0000
#define ZUMA_SYSMMU_MMU_STATUS          0x0008
#define ZUMA_SYSMMU_MMU_VERSION         0x0034
#define ZUMA_SYSMMU_PMMU_INDICATOR      0x2ffc
#define ZUMA_SYSMMU_PMMU_INFO           0x3000
#define ZUMA_SYSMMU_SWALKER_INFO        0x3004
#define ZUMA_SYSMMU_VM_CTRL             0x8000
#define ZUMA_SYSMMU_VM_FLPT_BASE        0x8404
#define ZUMA_SYSMMU_VM_ATTRIBUTE        0x8408
#define ZUMA_SYSMMU_MMU_CTRL_EXPECTED   0x0
#define ZUMA_SYSMMU_MMU_STATUS_EXPECTED 0x0
#define ZUMA_SYSMMU_VERSION_EXPECTED    0x90000000
#define ZUMA_SYSMMU_VM_CTRL_EXPECTED    0x24
#define ZUMA_SYSMMU_FLPT_EXPECTED       0x0
#define ZUMA_SYSMMU_ATTRIBUTE_EXPECTED  0x0
#define ZUMA_DPP_RDMA_ENABLE            0x0000
#define ZUMA_DPP_RDMA_IN_CTRL_0         0x0008
#define ZUMA_DPP_RDMA_SRC_WIDTH         0x0010
#define ZUMA_DPP_RDMA_SRC_HEIGHT        0x0014
#define ZUMA_DPP_RDMA_SRC_OFFSET        0x0018
#define ZUMA_DPP_RDMA_IMG_SIZE          0x001c
#define ZUMA_DPP_RDMA_BASEADDR_P0       0x0040
#define ZUMA_DPP_RDMA_SHADOW_OFFSET     0x0400
#define ZUMA_DPP_RDMA_SRC_STRIDE_0      0x0050
#define ZUMA_DPP_RDMA_EXPECTED          0x40000000
#define ZUMA_DPP_RDMA_BUSY              BIT(2)
#define ZUMA_DPP_FORMAT_SHIFT           8
#define ZUMA_DPP_FORMAT_MASK            0x3f
#define ZUMA_DPP_FORMAT_FIELD_MASK      \
	(ZUMA_DPP_FORMAT_MASK << ZUMA_DPP_FORMAT_SHIFT)
#define ZUMA_DPP_FORMAT_BGRA8888        0
#define ZUMA_DPP_FORMAT_BGRX8888        4

#define ZUMA_HANDOFF_FB_BASE            0xfac00000
#define ZUMA_HANDOFF_FB_WIDTH           1344
#define ZUMA_HANDOFF_FB_HEIGHT          2992
#define ZUMA_HANDOFF_FB_CTRL            0xff400000
#define ZUMA_HANDOFF_FB_BGRX_CTRL       \
	((ZUMA_HANDOFF_FB_CTRL & ~ZUMA_DPP_FORMAT_FIELD_MASK) | \
	 (ZUMA_DPP_FORMAT_BGRX8888 << ZUMA_DPP_FORMAT_SHIFT))
#define ZUMA_HANDOFF_FB_PIXELS          \
	(ZUMA_HANDOFF_FB_WIDTH * ZUMA_HANDOFF_FB_HEIGHT)
#define ZUMA_HANDOFF_FB_SIZE            \
	(ZUMA_HANDOFF_FB_PIXELS * sizeof(u32))
#define ZUMA_HANDOFF_FB_STRIDE          \
	(ZUMA_HANDOFF_FB_WIDTH * sizeof(u32))
#define ZUMA_FLIP_FB_ALLOC_SIZE         0x01000000
#define ZUMA_FLIP_FB_ALIGNMENT          0x00100000
#define ZUMA_FORMAT_SWITCH_DELAY_MS     2000
#define ZUMA_FB_FLUSH_DELAY_MS          16
#define ZUMA_FB_PALETTE_SIZE            16

struct zuma_display_block {
	const char *name;
	const char *compatible;
	resource_size_t phys;
	resource_size_t min_size;
	unsigned int resource_index;
	void __iomem *base;
};

struct zuma_framebuffer {
	struct fb_info *info;
	/* Protects dirty_start and dirty_end against fbdev and worker access. */
	spinlock_t dirty_lock;
	size_t dirty_start;
	size_t dirty_end;
	u32 palette[ZUMA_FB_PALETTE_SIZE];
	u64 update_count;
};

static DEFINE_MUTEX(zuma_display_mmio_lock);
static bool zuma_framebuffer_validated;
static phys_addr_t zuma_framebuffer_phys = ZUMA_HANDOFF_FB_BASE;
static u32 zuma_framebuffer_ctrl = ZUMA_HANDOFF_FB_CTRL;

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

static struct zuma_display_block zuma_decon0_wincon = {
	.name = "DECON0 window control",
	.compatible = "samsung,exynos-decon",
	.phys = ZUMA_DECON0_WINCON_BASE,
	.min_size = ZUMA_DECON0_WINCON_MIN_SIZE,
	.resource_index = ZUMA_DECON0_WINCON_RESOURCE,
};

static struct zuma_display_block zuma_dpp0 = {
	.name = "DPP0 RDMA",
	.compatible = "samsung,exynos-dpp",
	.phys = ZUMA_DPP0_BASE,
	.min_size = ZUMA_DPP0_MIN_SIZE,
};

static struct zuma_display_block zuma_sysmmu_dpuf0 = {
	.name = "DPUF0 SysMMU",
	.compatible = "samsung,sysmmu-v9",
	.phys = ZUMA_SYSMMU_DPUF0_BASE,
	.min_size = ZUMA_SYSMMU_DPUF0_MIN_SIZE,
};

static struct zuma_display_block * const zuma_display_blocks[] = {
	&zuma_dpub,
	&zuma_dpuf0,
	&zuma_dpuf1,
	&zuma_dsim0,
	&zuma_decon0,
	&zuma_decon0_wincon,
	&zuma_dpp0,
	&zuma_sysmmu_dpuf0,
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
static int zuma_framebuffer_register(void);
static void zuma_display_format_workfn(struct work_struct *work);
static DECLARE_DELAYED_WORK(zuma_display_format_work,
				 zuma_display_format_workfn);
static void zuma_display_snapshot_workfn(struct work_struct *work);
static DECLARE_DELAYED_WORK(zuma_display_snapshot_work,
				 zuma_display_snapshot_workfn);

static int __init zuma_display_map(struct zuma_display_block *block)
{
	struct device_node *np = NULL;
	struct resource res;
	int ret;

	while ((np = of_find_compatible_node(np, NULL, block->compatible))) {
		ret = of_address_to_resource(np, block->resource_index, &res);
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

static u32 zuma_decon_wincon_read(unsigned int window)
{
	return readl(zuma_decon0_wincon.base +
		     window * ZUMA_DECON_WINCON_STRIDE);
}

static bool zuma_display_active_window_valid(void)
{
	unsigned int window;

	if (zuma_decon_wincon_read(ZUMA_DECON_ACTIVE_WINDOW) !=
	    ZUMA_DECON_WINCON_EXPECTED)
		return false;

	for (window = 0; window < ZUMA_DECON_WINDOW_COUNT; window++) {
		if (window != ZUMA_DECON_ACTIVE_WINDOW &&
		    (zuma_decon_wincon_read(window) &
		     ZUMA_DECON_WINCON_ENABLE))
			return false;
	}

	return true;
}

static bool zuma_display_sysmmu_bypassed(void)
{
	return readl(zuma_sysmmu_dpuf0.base + ZUMA_SYSMMU_MMU_CTRL) ==
			ZUMA_SYSMMU_MMU_CTRL_EXPECTED &&
	       readl(zuma_sysmmu_dpuf0.base + ZUMA_SYSMMU_MMU_STATUS) ==
			ZUMA_SYSMMU_MMU_STATUS_EXPECTED &&
	       readl(zuma_sysmmu_dpuf0.base + ZUMA_SYSMMU_MMU_VERSION) ==
			ZUMA_SYSMMU_VERSION_EXPECTED &&
	       readl(zuma_sysmmu_dpuf0.base + ZUMA_SYSMMU_VM_CTRL) ==
			ZUMA_SYSMMU_VM_CTRL_EXPECTED &&
	       readl(zuma_sysmmu_dpuf0.base + ZUMA_SYSMMU_VM_FLPT_BASE) ==
			ZUMA_SYSMMU_FLPT_EXPECTED &&
	       readl(zuma_sysmmu_dpuf0.base + ZUMA_SYSMMU_VM_ATTRIBUTE) ==
			ZUMA_SYSMMU_ATTRIBUTE_EXPECTED;
}

static void zuma_display_snapshot(const char *label)
{
	u32 dpub, dpuf0, dpuf1;
	u32 rdma_ctrl, rdma_shadow_ctrl;

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
	pr_info("zuma-display-handoff: %s DECON active-window=%u wincon=%#x shadow-request=%#x\n",
		label, ZUMA_DECON_ACTIVE_WINDOW,
		zuma_decon_wincon_read(ZUMA_DECON_ACTIVE_WINDOW),
		readl(zuma_decon0.base + ZUMA_DECON_SHD_REG_UP_REQ));

	rdma_ctrl = readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IN_CTRL_0);
	rdma_shadow_ctrl = readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IN_CTRL_0 +
				 ZUMA_DPP_RDMA_SHADOW_OFFSET);
	pr_info("zuma-display-handoff: %s DPP0 enable=%#x ctrl=%#x shadow-ctrl=%#x format=%u src=%#xx%#x offset=%#x image=%#x base=%#x shadow=%#x stride=%#x\n",
		label,
		readl(zuma_dpp0.base + ZUMA_DPP_RDMA_ENABLE),
		rdma_ctrl, rdma_shadow_ctrl,
		(rdma_ctrl >> ZUMA_DPP_FORMAT_SHIFT) & ZUMA_DPP_FORMAT_MASK,
		readl(zuma_dpp0.base + ZUMA_DPP_RDMA_SRC_WIDTH),
		readl(zuma_dpp0.base + ZUMA_DPP_RDMA_SRC_HEIGHT),
		readl(zuma_dpp0.base + ZUMA_DPP_RDMA_SRC_OFFSET),
		readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IMG_SIZE),
		readl(zuma_dpp0.base + ZUMA_DPP_RDMA_BASEADDR_P0),
		readl(zuma_dpp0.base + ZUMA_DPP_RDMA_BASEADDR_P0 +
		      ZUMA_DPP_RDMA_SHADOW_OFFSET),
		readl(zuma_dpp0.base + ZUMA_DPP_RDMA_SRC_STRIDE_0));

	pr_info("zuma-display-handoff: %s DPUF0 SysMMU ctrl=%#x status=%#x version=%#x pmmu_sel=%#x pmmu=%#x swalker=%#x vm_ctrl=%#x flpt=%#x attr=%#x\n",
		label,
		readl(zuma_sysmmu_dpuf0.base + ZUMA_SYSMMU_MMU_CTRL),
		readl(zuma_sysmmu_dpuf0.base + ZUMA_SYSMMU_MMU_STATUS),
		readl(zuma_sysmmu_dpuf0.base + ZUMA_SYSMMU_MMU_VERSION),
		readl(zuma_sysmmu_dpuf0.base + ZUMA_SYSMMU_PMMU_INDICATOR),
		readl(zuma_sysmmu_dpuf0.base + ZUMA_SYSMMU_PMMU_INFO),
		readl(zuma_sysmmu_dpuf0.base + ZUMA_SYSMMU_SWALKER_INFO),
		readl(zuma_sysmmu_dpuf0.base + ZUMA_SYSMMU_VM_CTRL),
		readl(zuma_sysmmu_dpuf0.base + ZUMA_SYSMMU_VM_FLPT_BASE),
		readl(zuma_sysmmu_dpuf0.base + ZUMA_SYSMMU_VM_ATTRIBUTE));

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

static bool __init zuma_display_scan_framebuffer(void)
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
		return false;
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
		return false;
	}

	if (region_intersects(fb_base, ZUMA_HANDOFF_FB_SIZE,
			      IORESOURCE_SYSTEM_RAM, IORES_DESC_NONE) !=
	    REGION_INTERSECTS) {
		pr_err("zuma-display-handoff: framebuffer candidate %pa+%#zx is not System RAM\n",
		       &fb_base, (size_t)ZUMA_HANDOFF_FB_SIZE);
		return false;
	}

	pfn = PHYS_PFN(fb_base);
	end_pfn = PHYS_PFN(fb_base + ZUMA_HANDOFF_FB_SIZE);
	for (; pfn < end_pfn; pfn++) {
		if (!pfn_is_map_memory(pfn)) {
			pr_err("zuma-display-handoff: framebuffer PFN %#lx is not direct-mapped RAM\n",
			       pfn);
			return false;
		}
	}

	pixels = memremap(fb_base, ZUMA_HANDOFF_FB_SIZE, MEMREMAP_WB);
	if (!pixels) {
		pr_err("zuma-display-handoff: read-only framebuffer mapping failed\n");
		return false;
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

	return true;
}

static void zuma_display_set_hw_trigger(bool unmask)
{
	u32 value;

	value = readl(zuma_decon0.base + ZUMA_DECON_TRIG_CON);
	value &= ~(ZUMA_DECON_TRIG_HW_EN | ZUMA_DECON_TRIG_HW_MASK);
	value |= unmask ? ZUMA_DECON_TRIG_HW_EN : ZUMA_DECON_TRIG_HW_MASK;
	writel(value, zuma_decon0.base + ZUMA_DECON_TRIG_CON);
}

static int zuma_display_wait_idle(void)
{
	u32 value;
	int ret;

	ret = readl_poll_timeout_atomic(zuma_decon0.base +
			ZUMA_DECON_GLOBAL_CON, value,
			value & ZUMA_DECON_GLOBAL_IDLE, 10, 100000);
	if (ret)
		return ret;

	ret = readl_poll_timeout_atomic(zuma_dpp0.base +
			ZUMA_DPP_RDMA_ENABLE, value,
			!(value & ZUMA_DPP_RDMA_BUSY), 10, 100000);
	if (ret)
		return ret;

	ret = readl_poll_timeout_atomic(zuma_dsim0.base +
			ZUMA_DSIM_LINK_STATUS1, value,
			!(value & ZUMA_DSIM_LINK_CMD_ACTIVE), 10, 100000);
	if (ret)
		return ret;

	return readl_poll_timeout_atomic(zuma_dsim0.base +
			ZUMA_DSIM_MIPI_STATUS, value,
			!(value & ZUMA_DSIM_MIPI_FRAME_PROCESSING), 10, 100000);
}

static int zuma_display_request_active_window(void)
{
	writel(ZUMA_DECON_SHD_REQ_ACTIVE,
	       zuma_decon0.base + ZUMA_DECON_SHD_REG_UP_REQ);
	if (readl(zuma_decon0.base + ZUMA_DECON_SHD_REG_UP_REQ) !=
	    ZUMA_DECON_SHD_REQ_ACTIVE)
		return -EIO;

	return 0;
}

static int zuma_display_trigger_frame(u32 *frame_before, u32 *frame_after)
{
	u32 value;
	int ret;

	*frame_before = readl(zuma_decon0.base + ZUMA_DECON_FRAME_COUNT);
	*frame_after = *frame_before;
	zuma_display_set_hw_trigger(true);
	ret = readl_poll_timeout_atomic(zuma_decon0.base +
			ZUMA_DECON_FRAME_COUNT, *frame_after,
			*frame_after != *frame_before, 1, 50000);
	zuma_display_set_hw_trigger(false);
	if (ret)
		return ret;

	ret = zuma_display_wait_idle();
	if (ret)
		return ret;

	ret = readl_poll_timeout_atomic(zuma_decon0.base +
			ZUMA_DECON_SHD_REG_UP_REQ, value, !value,
			10, 100000);
	if (ret)
		return ret;

	if (readl(zuma_decon0.base + ZUMA_DECON_TRIG_CON) !=
	    ZUMA_DECON_TRIG_EXPECTED)
		return -EIO;

	return 0;
}

static bool zuma_display_update_ready_for_ctrl(u32 ctrl, u32 shadow_ctrl)
{
	u32 dpub, dpuf0, dpuf1;

	if (!zuma_display_domains_on(&dpub, &dpuf0, &dpuf1))
		return false;

	return readl(zuma_decon0.base + ZUMA_DECON_GLOBAL_CON) ==
			ZUMA_DECON_GLOBAL_EXPECTED &&
	       readl(zuma_decon0.base + ZUMA_DECON_TRIG_CON) ==
			ZUMA_DECON_TRIG_EXPECTED &&
	       !readl(zuma_decon0.base + ZUMA_DECON_SHD_REG_UP_REQ) &&
	       readl(zuma_decon0.base + ZUMA_DECON_INT_EN) ==
			ZUMA_DECON_INT_EXPECTED &&
	       readl(zuma_dpp0.base + ZUMA_DPP_RDMA_ENABLE) ==
			ZUMA_DPP_RDMA_EXPECTED &&
	       readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IN_CTRL_0) == ctrl &&
	       readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IN_CTRL_0 +
		     ZUMA_DPP_RDMA_SHADOW_OFFSET) == shadow_ctrl &&
	       readl(zuma_dpp0.base + ZUMA_DPP_RDMA_BASEADDR_P0) ==
			lower_32_bits(zuma_framebuffer_phys) &&
	       readl(zuma_dpp0.base + ZUMA_DPP_RDMA_BASEADDR_P0 +
		     ZUMA_DPP_RDMA_SHADOW_OFFSET) ==
			lower_32_bits(zuma_framebuffer_phys) &&
	       zuma_display_active_window_valid() &&
	       zuma_display_sysmmu_bypassed() &&
	       !readl(zuma_dsim0.base + ZUMA_DSIM_LINK_STATUS1) &&
	       !readl(zuma_dsim0.base + ZUMA_DSIM_MIPI_STATUS);
}

static bool zuma_display_update_ready(void)
{
	return zuma_display_update_ready_for_ctrl(zuma_framebuffer_ctrl,
						  zuma_framebuffer_ctrl);
}

static bool __init zuma_flip_buffer_valid(void)
{
	phys_addr_t base = zuma_husky_flip_framebuffer_base;
	unsigned long pfn, end_pfn;

	if (!base || !IS_ALIGNED(base, ZUMA_FLIP_FB_ALIGNMENT) ||
	    base > ZUMA_HANDOFF_FB_BASE - ZUMA_FLIP_FB_ALLOC_SIZE ||
	    base >= BIT_ULL(32) ||
	    base > BIT_ULL(32) - ZUMA_FLIP_FB_ALLOC_SIZE) {
		pr_err("zuma-display-handoff: refusing invalid flip framebuffer base %pa\n",
		       &base);
		return false;
	}

	if (region_intersects(base, ZUMA_FLIP_FB_ALLOC_SIZE,
			      IORESOURCE_SYSTEM_RAM, IORES_DESC_NONE) !=
	    REGION_INTERSECTS) {
		pr_err("zuma-display-handoff: flip framebuffer %pa+%#x is not System RAM\n",
		       &base, ZUMA_FLIP_FB_ALLOC_SIZE);
		return false;
	}

	pfn = PHYS_PFN(base);
	end_pfn = PHYS_PFN(base + ZUMA_FLIP_FB_ALLOC_SIZE);
	for (; pfn < end_pfn; pfn++) {
		if (!pfn_is_map_memory(pfn)) {
			pr_err("zuma-display-handoff: flip framebuffer PFN %#lx is not direct-mapped RAM\n",
			       pfn);
			return false;
		}
	}

	return true;
}

static int __init zuma_display_commit_base(phys_addr_t base,
					   u32 *frame_before,
					   u32 *frame_after)
{
	int ret;

	if (upper_32_bits(base))
		return -ERANGE;

	ret = zuma_display_wait_idle();
	if (ret)
		return ret;

	writel(lower_32_bits(base),
	       zuma_dpp0.base + ZUMA_DPP_RDMA_BASEADDR_P0);
	if (readl(zuma_dpp0.base + ZUMA_DPP_RDMA_BASEADDR_P0) !=
	    lower_32_bits(base))
		return -EIO;

	ret = zuma_display_request_active_window();
	if (ret)
		return ret;

	ret = zuma_display_trigger_frame(frame_before, frame_after);
	if (ret)
		return ret;

	if (readl(zuma_dpp0.base + ZUMA_DPP_RDMA_BASEADDR_P0) !=
			lower_32_bits(base) ||
	    readl(zuma_dpp0.base + ZUMA_DPP_RDMA_BASEADDR_P0 +
		  ZUMA_DPP_RDMA_SHADOW_OFFSET) != lower_32_bits(base))
		return -EIO;

	return 0;
}

static int zuma_display_write_ctrl(u32 ctrl)
{
	writel(ctrl, zuma_dpp0.base + ZUMA_DPP_RDMA_IN_CTRL_0);
	if (readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IN_CTRL_0) != ctrl)
		return -EIO;

	return 0;
}

static int zuma_display_commit_format(u32 format, u32 *ctrl_after,
				      u32 *frame_before,
				      u32 *frame_after)
{
	u32 ctrl;
	int ret;

	if (format & ~ZUMA_DPP_FORMAT_MASK)
		return -EINVAL;

	ret = zuma_display_wait_idle();
	if (ret)
		return ret;

	ctrl = readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IN_CTRL_0);
	ctrl &= ~ZUMA_DPP_FORMAT_FIELD_MASK;
	ctrl |= format << ZUMA_DPP_FORMAT_SHIFT;
	ret = zuma_display_write_ctrl(ctrl);
	if (ret)
		return ret;

	ret = zuma_display_request_active_window();
	if (ret)
		return ret;

	ret = zuma_display_trigger_frame(frame_before, frame_after);
	if (ret)
		return ret;

	if (readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IN_CTRL_0) != ctrl ||
	    readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IN_CTRL_0 +
		  ZUMA_DPP_RDMA_SHADOW_OFFSET) != ctrl)
		return -EIO;

	*ctrl_after = ctrl;
	return 0;
}

static int zuma_display_rollback_format(u32 *ctrl_after,
					u32 *frame_before,
					u32 *frame_after)
{
	u32 ctrl, shadow_ctrl;
	int ret;

	ret = zuma_display_wait_idle();
	if (ret)
		return ret;

	ctrl = readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IN_CTRL_0);
	shadow_ctrl = readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IN_CTRL_0 +
			    ZUMA_DPP_RDMA_SHADOW_OFFSET);
	if ((ctrl != ZUMA_HANDOFF_FB_CTRL &&
	     ctrl != ZUMA_HANDOFF_FB_BGRX_CTRL) ||
	    (shadow_ctrl != ZUMA_HANDOFF_FB_CTRL &&
	     shadow_ctrl != ZUMA_HANDOFF_FB_BGRX_CTRL)) {
		pr_err("zuma-display-handoff: refusing format rollback from unknown ctrl=%#x shadow=%#x\n",
		       ctrl, shadow_ctrl);
		return -EIO;
	}

	if (!zuma_display_update_ready_for_ctrl(ctrl, shadow_ctrl)) {
		pr_err("zuma-display-handoff: refusing format rollback from unsafe ctrl=%#x shadow=%#x\n",
		       ctrl, shadow_ctrl);
		return -EIO;
	}

	*frame_before = readl(zuma_decon0.base + ZUMA_DECON_FRAME_COUNT);
	*frame_after = *frame_before;
	if (ctrl == ZUMA_HANDOFF_FB_CTRL &&
	    shadow_ctrl == ZUMA_HANDOFF_FB_CTRL) {
		*ctrl_after = ctrl;
		return 0;
	}

	if (ctrl == ZUMA_HANDOFF_FB_BGRX_CTRL &&
	    shadow_ctrl == ZUMA_HANDOFF_FB_CTRL) {
		ret = zuma_display_write_ctrl(ZUMA_HANDOFF_FB_CTRL);
		if (ret)
			return ret;
		if (!zuma_display_update_ready_for_ctrl(ZUMA_HANDOFF_FB_CTRL,
							ZUMA_HANDOFF_FB_CTRL))
			return -EIO;
		*ctrl_after = ZUMA_HANDOFF_FB_CTRL;
		return 0;
	}

	return zuma_display_commit_format(ZUMA_DPP_FORMAT_BGRA8888,
					  ctrl_after, frame_before,
					  frame_after);
}

static bool __init zuma_display_flip_to_reserved(void)
{
	phys_addr_t new_base = zuma_husky_flip_framebuffer_base;
	void *source, *destination;
	u32 frame_before = 0, frame_after = 0;
	u32 rollback_before = 0, rollback_after = 0;
	int rollback_ret;
	int ret;

	if (!zuma_display_update_ready()) {
		pr_err("zuma-display-handoff: base flip refused by inherited state\n");
		return false;
	}

	if (!zuma_flip_buffer_valid())
		return false;

	source = memremap(ZUMA_HANDOFF_FB_BASE, ZUMA_HANDOFF_FB_SIZE,
			  MEMREMAP_WB);
	if (!source) {
		pr_err("zuma-display-handoff: base flip source mapping failed\n");
		return false;
	}

	destination = memremap(new_base, ZUMA_HANDOFF_FB_SIZE, MEMREMAP_WB);
	if (!destination) {
		pr_err("zuma-display-handoff: base flip destination mapping failed\n");
		memunmap(source);
		return false;
	}

	memcpy(destination, source, ZUMA_HANDOFF_FB_SIZE);
	if (memcmp(destination, source, ZUMA_HANDOFF_FB_SIZE)) {
		pr_err("zuma-display-handoff: base flip copy verification failed\n");
		memunmap(destination);
		memunmap(source);
		return false;
	}
	arch_sync_dma_for_device(new_base, ZUMA_HANDOFF_FB_SIZE, DMA_TO_DEVICE);
	arch_sync_dma_flush();
	memunmap(destination);
	memunmap(source);

	ret = zuma_display_commit_base(new_base, &frame_before, &frame_after);
	if (!ret) {
		zuma_framebuffer_phys = new_base;
		if (zuma_display_update_ready()) {
			pr_info("zuma-display-handoff: DPP0 base flip %#x->%pa frame=%#x->%#x, BGRA format unchanged\n",
				ZUMA_HANDOFF_FB_BASE, &new_base,
				frame_before, frame_after);
			return true;
		}
		ret = -EIO;
	}

	pr_err("zuma-display-handoff: DPP0 base flip failed: %d; attempting rollback\n",
	       ret);
	rollback_ret = zuma_display_commit_base(ZUMA_HANDOFF_FB_BASE,
						&rollback_before,
						&rollback_after);
	zuma_framebuffer_phys = ZUMA_HANDOFF_FB_BASE;
	if (rollback_ret || !zuma_display_update_ready()) {
		pr_crit("zuma-display-handoff: DPP0 base rollback failed: %d\n",
			rollback_ret ?: -EIO);
		return false;
	}

	pr_info("zuma-display-handoff: DPP0 base rollback completed frame=%#x->%#x\n",
		rollback_before, rollback_after);
	return false;
}

static bool zuma_display_switch_to_bgrx(void)
{
	u32 ctrl_after = 0, rollback_ctrl = 0;
	u32 frame_before = 0, frame_after = 0;
	u32 rollback_before = 0, rollback_after = 0;
	int rollback_ret;
	int ret;

	if (zuma_framebuffer_ctrl != ZUMA_HANDOFF_FB_CTRL ||
	    !zuma_display_update_ready()) {
		pr_err("zuma-display-handoff: BGRX format switch refused by BGRA state\n");
		return false;
	}

	ret = zuma_display_commit_format(ZUMA_DPP_FORMAT_BGRX8888,
					 &ctrl_after, &frame_before,
					 &frame_after);
	if (!ret && ctrl_after == ZUMA_HANDOFF_FB_BGRX_CTRL) {
		zuma_framebuffer_ctrl = ctrl_after;
		if (zuma_display_update_ready()) {
			pr_info("zuma-display-handoff: DPP0 format BGRA8888(0)->BGRX8888(4) ctrl=%#x frame=%#x->%#x\n",
				ctrl_after, frame_before, frame_after);
			return true;
		}
		ret = -EIO;
	}

	pr_err("zuma-display-handoff: DPP0 BGRX format switch failed: %d; attempting rollback\n",
	       ret ?: -EIO);
	rollback_ret = zuma_display_rollback_format(&rollback_ctrl,
						    &rollback_before,
						    &rollback_after);
	zuma_framebuffer_ctrl = ZUMA_HANDOFF_FB_CTRL;
	if (rollback_ret || rollback_ctrl != ZUMA_HANDOFF_FB_CTRL ||
	    !zuma_display_update_ready()) {
		pr_crit("zuma-display-handoff: DPP0 BGRA format rollback failed: %d\n",
			rollback_ret ?: -EIO);
		return false;
	}

	pr_info("zuma-display-handoff: DPP0 BGRA format rollback completed ctrl=%#x frame=%#x->%#x\n",
		rollback_ctrl, rollback_before, rollback_after);
	return false;
}

static void zuma_fb_mark_dirty(struct zuma_framebuffer *par,
			       size_t start, size_t length)
{
	unsigned long flags;
	size_t end;

	if (start >= ZUMA_HANDOFF_FB_SIZE || !length)
		return;
	length = min_t(size_t, length, ZUMA_HANDOFF_FB_SIZE - start);
	end = start + length;

	spin_lock_irqsave(&par->dirty_lock, flags);
	par->dirty_start = min(par->dirty_start, start);
	par->dirty_end = max(par->dirty_end, end);
	spin_unlock_irqrestore(&par->dirty_lock, flags);
}

static void zuma_fb_mark_rect(struct zuma_framebuffer *par, u32 x, u32 y,
			      u32 width, u32 height)
{
	size_t start, end;

	if (x >= ZUMA_HANDOFF_FB_WIDTH || y >= ZUMA_HANDOFF_FB_HEIGHT ||
	    !width || !height)
		return;
	width = min(width, ZUMA_HANDOFF_FB_WIDTH - x);
	height = min(height, ZUMA_HANDOFF_FB_HEIGHT - y);
	start = y * ZUMA_HANDOFF_FB_STRIDE + x * sizeof(u32);
	end = (y + height - 1) * ZUMA_HANDOFF_FB_STRIDE +
	      (x + width) * sizeof(u32);
	zuma_fb_mark_dirty(par, start, end - start);
}

static int zuma_fb_submit_update(struct fb_info *info)
{
	struct zuma_framebuffer *par = info->par;
	unsigned long flags;
	size_t start, end;
	u32 frame_before, frame_after;
	int ret = 0;

	mutex_lock(&zuma_display_mmio_lock);

	spin_lock_irqsave(&par->dirty_lock, flags);
	start = par->dirty_start;
	end = par->dirty_end;
	par->dirty_start = ZUMA_HANDOFF_FB_SIZE;
	par->dirty_end = 0;
	spin_unlock_irqrestore(&par->dirty_lock, flags);

	if (start >= end)
		goto out_unlock;

	if (!zuma_display_update_ready()) {
		ret = -EIO;
		goto out_requeue;
	}

	arch_sync_dma_for_device(zuma_framebuffer_phys + start, end - start,
				 DMA_TO_DEVICE);
	arch_sync_dma_flush();

	ret = zuma_display_request_active_window();
	if (ret) {
		pr_err_ratelimited("zuma-display-handoff: framebuffer window update request failed: %d\n",
				   ret);
		goto out_requeue;
	}

	ret = zuma_display_trigger_frame(&frame_before, &frame_after);
	if (ret) {
		pr_err_ratelimited("zuma-display-handoff: framebuffer update did not complete: %d, frame=%#x\n",
				   ret, frame_after);
		goto out_requeue;
	}

	par->update_count++;
	if (par->update_count <= 8)
		pr_info("zuma-display-handoff: fb update %llu bytes=%zu..%zu frame=%#x->%#x\n",
			(unsigned long long)par->update_count, start, end - 1,
			frame_before, frame_after);
	goto out_unlock;

out_requeue:
	zuma_fb_mark_dirty(par, start, end - start);
out_unlock:
	mutex_unlock(&zuma_display_mmio_lock);
	return ret;
}

static void zuma_fb_queue_update(struct fb_info *info)
{
	mod_delayed_work(system_wq, &info->deferred_work,
			 msecs_to_jiffies(ZUMA_FB_FLUSH_DELAY_MS));
}

static ssize_t zuma_fb_read(struct fb_info *info, char __user *buf,
			    size_t count, loff_t *ppos)
{
	return simple_read_from_buffer(buf, count, ppos, info->screen_buffer,
				       info->screen_size);
}

static ssize_t zuma_fb_write(struct fb_info *info, const char __user *buf,
			     size_t count, loff_t *ppos)
{
	struct zuma_framebuffer *par = info->par;
	loff_t start = *ppos;
	ssize_t ret;

	ret = simple_write_to_buffer(info->screen_buffer, info->screen_size,
				     ppos, buf, count);
	if (ret > 0) {
		zuma_fb_mark_dirty(par, start, ret);
		zuma_fb_queue_update(info);
	}
	return ret;
}

static void zuma_fb_fillrect(struct fb_info *info,
			     const struct fb_fillrect *rect)
{
	sys_fillrect(info, rect);
	zuma_fb_mark_rect(info->par, rect->dx, rect->dy,
			  rect->width, rect->height);
	zuma_fb_queue_update(info);
}

static void zuma_fb_copyarea(struct fb_info *info,
			     const struct fb_copyarea *area)
{
	sys_copyarea(info, area);
	zuma_fb_mark_rect(info->par, area->dx, area->dy,
			  area->width, area->height);
	zuma_fb_queue_update(info);
}

static void zuma_fb_imageblit(struct fb_info *info,
			      const struct fb_image *image)
{
	sys_imageblit(info, image);
	zuma_fb_mark_rect(info->par, image->dx, image->dy,
			  image->width, image->height);
	zuma_fb_queue_update(info);
}

static int zuma_fb_sync(struct fb_info *info)
{
	flush_delayed_work(&info->deferred_work);
	return zuma_fb_submit_update(info);
}

static int zuma_fb_ioctl(struct fb_info *info, unsigned int cmd,
			 unsigned long arg)
{
	if (cmd == FBIO_WAITFORVSYNC)
		return zuma_fb_sync(info);
	return -ENOTTY;
}

static int zuma_fb_check_var(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
	if (var->xres != ZUMA_HANDOFF_FB_WIDTH ||
	    var->yres != ZUMA_HANDOFF_FB_HEIGHT ||
	    var->bits_per_pixel != 32)
		return -EINVAL;

	var->xres_virtual = ZUMA_HANDOFF_FB_WIDTH;
	var->yres_virtual = ZUMA_HANDOFF_FB_HEIGHT;
	var->xoffset = 0;
	var->yoffset = 0;
	var->red = info->var.red;
	var->green = info->var.green;
	var->blue = info->var.blue;
	var->transp = info->var.transp;
	return 0;
}

static int zuma_fb_set_par(struct fb_info *info)
{
	return 0;
}

static int zuma_fb_setcolreg(unsigned int regno, unsigned int red,
			     unsigned int green, unsigned int blue,
			     unsigned int transp, struct fb_info *info)
{
	u32 *palette = info->pseudo_palette;
	u32 value;

	if (regno >= ZUMA_FB_PALETTE_SIZE)
		return -EINVAL;

	value = ((red >> 8) << info->var.red.offset) |
		((green >> 8) << info->var.green.offset) |
		((blue >> 8) << info->var.blue.offset);
	if (info->var.transp.length)
		value |= 0xffU << info->var.transp.offset;
	palette[regno] = value;
	return 0;
}

static int zuma_fb_blank(int blank, struct fb_info *info)
{
	return blank == FB_BLANK_UNBLANK ? 0 : -EINVAL;
}

static int zuma_fb_pan_display(struct fb_var_screeninfo *var,
			       struct fb_info *info)
{
	return (var->xoffset || var->yoffset) ? -EINVAL : 0;
}

static void zuma_fb_deferred_io(struct fb_info *info,
				struct list_head *pagelist)
{
	struct zuma_framebuffer *par = info->par;
	struct fb_deferred_io_pageref *pageref;

	list_for_each_entry(pageref, pagelist, list)
		zuma_fb_mark_dirty(par, pageref->offset, PAGE_SIZE);
	zuma_fb_submit_update(info);
}

static const struct fb_ops zuma_fb_ops = {
	.owner = THIS_MODULE,
	.fb_read = zuma_fb_read,
	.fb_write = zuma_fb_write,
	.fb_check_var = zuma_fb_check_var,
	.fb_set_par = zuma_fb_set_par,
	.fb_setcolreg = zuma_fb_setcolreg,
	.fb_blank = zuma_fb_blank,
	.fb_pan_display = zuma_fb_pan_display,
	.fb_fillrect = zuma_fb_fillrect,
	.fb_copyarea = zuma_fb_copyarea,
	.fb_imageblit = zuma_fb_imageblit,
	.fb_sync = zuma_fb_sync,
	.fb_ioctl = zuma_fb_ioctl,
	.fb_mmap = fb_deferred_io_mmap,
};

static struct fb_deferred_io zuma_fb_defio = {
	.deferred_io = zuma_fb_deferred_io,
};

static void zuma_display_format_workfn(struct work_struct *work)
{
	bool register_fb;

	mutex_lock(&zuma_display_mmio_lock);
	if (zuma_framebuffer_validated)
		zuma_framebuffer_validated = zuma_display_switch_to_bgrx();
	register_fb = zuma_framebuffer_validated;
	mutex_unlock(&zuma_display_mmio_lock);

	if (register_fb)
		zuma_framebuffer_register();
}

static void zuma_display_snapshot_workfn(struct work_struct *work)
{
	unsigned int index = zuma_snapshot_index;

	mutex_lock(&zuma_display_mmio_lock);
	zuma_display_snapshot(zuma_snapshot_labels[index]);
	mutex_unlock(&zuma_display_mmio_lock);
	index++;
	zuma_snapshot_index = index;

	if (index < ARRAY_SIZE(zuma_snapshot_intervals_ms)) {
		schedule_delayed_work(&zuma_display_snapshot_work,
				      msecs_to_jiffies(zuma_snapshot_intervals_ms[index]));
		return;
	}

	pr_info("zuma-display-handoff: completed timed snapshots; handoff remains active\n");
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

	pr_info("zuma-display-handoff: guarded Zuma/Husky handoff active\n");
	if (zuma_husky_flip_framebuffer_base)
		pr_info("zuma-display-handoff: reserved flip framebuffer at %pa, size=%#x\n",
			&zuma_husky_flip_framebuffer_base,
			ZUMA_FLIP_FB_ALLOC_SIZE);
	else
		pr_warn("zuma-display-handoff: flip framebuffer reservation unavailable\n");
	mutex_lock(&zuma_display_mmio_lock);
	zuma_display_snapshot("early");
	if (zuma_husky_boot_framebuffer_reserved) {
		zuma_framebuffer_validated = zuma_display_scan_framebuffer();
		if (zuma_framebuffer_validated)
			zuma_framebuffer_validated =
				zuma_display_flip_to_reserved();
	} else {
		pr_crit("zuma-display-handoff: framebuffer reservation unavailable; scan skipped\n");
	}
	mutex_unlock(&zuma_display_mmio_lock);
	if (zuma_framebuffer_validated)
		schedule_delayed_work(&zuma_display_format_work,
				      msecs_to_jiffies(ZUMA_FORMAT_SWITCH_DELAY_MS));
	schedule_delayed_work(&zuma_display_snapshot_work,
			      msecs_to_jiffies(zuma_snapshot_intervals_ms[0]));
	return 0;

out_unmap:
	zuma_display_unmap_all();
	return 0;
}
early_initcall(zuma_display_handoff_init);

static int zuma_framebuffer_register(void)
{
	struct zuma_framebuffer *par;
	struct fb_info *info;
	void *screen_buffer;
	int ret;

	if (!zuma_framebuffer_validated)
		return 0;

	mutex_lock(&zuma_display_mmio_lock);
	if (!zuma_display_update_ready()) {
		mutex_unlock(&zuma_display_mmio_lock);
		pr_err("zuma-display-handoff: framebuffer registration refused by handoff state\n");
		return 0;
	}
	mutex_unlock(&zuma_display_mmio_lock);

	screen_buffer = memremap(zuma_framebuffer_phys,
				 ZUMA_HANDOFF_FB_SIZE, MEMREMAP_WB);
	if (!screen_buffer) {
		pr_err("zuma-display-handoff: framebuffer mapping failed\n");
		return 0;
	}

	info = framebuffer_alloc(sizeof(*par), NULL);
	if (!info) {
		memunmap(screen_buffer);
		return 0;
	}

	par = info->par;
	par->info = info;
	spin_lock_init(&par->dirty_lock);
	par->dirty_start = ZUMA_HANDOFF_FB_SIZE;

	strscpy(info->fix.id, "zuma-handoff", sizeof(info->fix.id));
	info->fix.smem_start = zuma_framebuffer_phys;
	info->fix.smem_len = ZUMA_HANDOFF_FB_SIZE;
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.line_length = ZUMA_HANDOFF_FB_STRIDE;
	info->fix.accel = FB_ACCEL_NONE;

	info->var.xres = ZUMA_HANDOFF_FB_WIDTH;
	info->var.yres = ZUMA_HANDOFF_FB_HEIGHT;
	info->var.xres_virtual = ZUMA_HANDOFF_FB_WIDTH;
	info->var.yres_virtual = ZUMA_HANDOFF_FB_HEIGHT;
	info->var.bits_per_pixel = 32;
	info->var.red.offset = 16;
	info->var.red.length = 8;
	info->var.green.offset = 8;
	info->var.green.length = 8;
	info->var.blue.offset = 0;
	info->var.blue.length = 8;
	info->var.transp.offset = 24;
	info->var.transp.length = 0;
	info->var.height = -1;
	info->var.width = -1;
	info->var.activate = FB_ACTIVATE_NOW;
	info->var.vmode = FB_VMODE_NONINTERLACED;

	info->fbops = &zuma_fb_ops;
	info->screen_buffer = screen_buffer;
	info->screen_size = ZUMA_HANDOFF_FB_SIZE;
	info->pseudo_palette = par->palette;
	info->flags = FBINFO_VIRTFB | FBINFO_READS_FAST |
		      FBINFO_HWACCEL_DISABLED;
	info->skip_panic = true;
	info->fbdefio = &zuma_fb_defio;
	zuma_fb_defio.delay = max_t(unsigned long, 1,
				    msecs_to_jiffies(ZUMA_FB_FLUSH_DELAY_MS));

	ret = fb_deferred_io_init(info);
	if (ret)
		goto out_release;

	ret = register_framebuffer(info);
	if (ret)
		goto out_defio;

	pr_info("zuma-display-handoff: fb%d registered at %pa, %ux%u BGRX8888, stride=%zu\n",
		info->node, &zuma_framebuffer_phys, ZUMA_HANDOFF_FB_WIDTH,
		ZUMA_HANDOFF_FB_HEIGHT, (size_t)ZUMA_HANDOFF_FB_STRIDE);
	return 0;

out_defio:
	fb_deferred_io_cleanup(info);
out_release:
	framebuffer_release(info);
	memunmap(screen_buffer);
	pr_err("zuma-display-handoff: framebuffer registration failed: %d\n", ret);
	return 0;
}
