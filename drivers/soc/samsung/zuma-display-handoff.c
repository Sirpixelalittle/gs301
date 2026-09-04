// SPDX-License-Identifier: GPL-2.0-only
/*
 * Display bootloader handoff for Google Zuma/Husky.
 *
 * The shipping boot chain leaves a working HK3 command-mode display pipeline.
 * Preserve the inherited DECON, DSIM, panel, clocks, and power state behind
 * fixed-mode DRM/KMS. The first real DRM update initializes the proven
 * RDMA/DPP0 fixed path without reset; later updates retain that ownership.
 */

#include <linux/atomic.h>
#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/dma-direction.h>
#include <linux/dma-map-ops.h>
#include <linux/device.h>
#include <linux/dma-fence.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/iosys-map.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/printk.h>
#include <linux/reboot.h>
#include <linux/string.h>
#include <linux/soc/samsung/zuma-display-handoff.h>
#include <linux/workqueue.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_encoder.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_file.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_modes.h>
#include <drm/drm_plane.h>
#include <drm/drm_vblank.h>
#include <drm/drm_probe_helper.h>

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
#define ZUMA_DECON_INT_PEND             0x0070
#define ZUMA_DECON_OF_SIZE_0            0x0290
#define ZUMA_DECON_OF_PIXEL_ORDER       0x02a0

#define ZUMA_DECON_GLOBAL_EXPECTED      0x0133
#define ZUMA_DECON_GLOBAL_IDLE          BIT(5)
#define ZUMA_DECON_TRIG_EXPECTED        0x3070
#define ZUMA_DECON_TRIG_HW_MASK         BIT(4)
#define ZUMA_DECON_TRIG_HW_EN           BIT(0)
#define ZUMA_DECON_INT_MASTER           BIT(0)
#define ZUMA_DECON_INT_FRAME_START      BIT(12)
#define ZUMA_DECON_INT_FRAME_DONE       BIT(13)
#define ZUMA_DECON_INT_FRAME_MASK       \
	(ZUMA_DECON_INT_FRAME_START | ZUMA_DECON_INT_FRAME_DONE)
#define ZUMA_DECON_INT_QUIESCENT        ZUMA_DECON_INT_FRAME_MASK
#define ZUMA_DECON_INT_ACTIVE           \
	(ZUMA_DECON_INT_QUIESCENT | ZUMA_DECON_INT_MASTER)
#define ZUMA_DECON_IRQ_TIMEOUT_MS       100

#define ZUMA_DPP0_BASE                  0x19900000
#define ZUMA_DPP0_MIN_SIZE              0x1000
#define ZUMA_DPP0_DPP_BASE              0x19930000
#define ZUMA_DPP0_SCL_COEF_BASE         0x19940000
#define ZUMA_DPP0_SRAMC_BASE            0x19950000
#define ZUMA_DPP0_HDR_COMM_BASE         0x19960000
#define ZUMA_DPP0_HDR_BASE              0x19980000
#define ZUMA_DPP0_REG_SIZE              0x1000
#define ZUMA_DPP0_SCL_COEF_SIZE         0x4000
#define ZUMA_DPP0_DT_ATTR               0x006d0867
#define ZUMA_DPP0_BL_EFFECTIVE_ATTR     0x004d0047

#define ZUMA_DPP_RDMA_IRQ               0x0004
#define ZUMA_DPP_RDMA_BASEADDR_P1       0x0044
#define ZUMA_DPP_RDMA_AFBC_PARAM        0x0070
#define ZUMA_DPP_RDMA_RECOVERY_CTRL     0x0080
#define ZUMA_DPP_RDMA_DEADLOCK_CTRL     0x0100
#define ZUMA_DPP_RDMA_QOS_LOW           0x0130
#define ZUMA_DPP_RDMA_QOS_HIGH          0x0134
#define ZUMA_DPP_RDMA_DYNAMIC_GATING    0x0140
#define ZUMA_DPP_CORE_SWRST             0x0004
#define ZUMA_DPP_CORE_IRQ_CON           0x0010
#define ZUMA_DPP_CORE_IRQ_MASK          0x0014
#define ZUMA_DPP_CORE_IRQ_STATUS        0x0018
#define ZUMA_DPP_CORE_CONFIG_ERROR      0x001c
#define ZUMA_DPP_CORE_OP_STATUS         0x0030
#define ZUMA_DPP_CORE_IO_CON            0x0038
#define ZUMA_DPP_CORE_INHERITED_IO      0x0080
#define ZUMA_DPP_CORE_IMG_SIZE          0x003c
#define ZUMA_DPP_CORE_SCL_CTRL          0x0080
#define ZUMA_DPP_CORE_SCALED_IMG_SIZE   0x0084
#define ZUMA_DPP_CORE_SCL_HPOSITION     0x0090
#define ZUMA_DPP_CORE_SCL_VPOSITION     0x0094
#define ZUMA_DPP_CORE_SHADOW_OFFSET     0x0100
#define ZUMA_DPP_SRAMC_MODE             0x0010
#define ZUMA_DPP_SRAMC_DST_POSITION     0x0014
#define ZUMA_DPP_SRAMC_SHADOW_OFFSET    0x0800
#define ZUMA_DPP_HDR_COMM_IO_CON        0x000c
#define ZUMA_DPP_HDR_COMM_SIZE          0x0020
#define ZUMA_DPP_HDR_COMM_SHADOW_OFFSET 0x0800
#define ZUMA_DPP_FIXED_IMG_SIZE         0x0bb00540
#define ZUMA_DPP_FIXED_DST_POSITION     0x0baf0000
#define ZUMA_DPP_FIXED_RECOVERY         0x001339e0
#define ZUMA_DPP_FIXED_DEADLOCK         0x03d487a1

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
#define ZUMA_DPP_RDMA_RECOVERY_ENABLE   BIT(0)
#define ZUMA_DPP_RDMA_DEADLOCK_ENABLE   BIT(0)
#define ZUMA_DPP_RDMA_COUNT_MASK        GENMASK(31, 1)
#define ZUMA_DPP_RDMA_IRQ_ENABLE        BIT(0)
#define ZUMA_DPP_RDMA_FRAME_DONE_MASK   BIT(1)
#define ZUMA_DPP_RDMA_ALL_IRQ_MASK      0x00001bf6
#define ZUMA_DPP_RDMA_CONTROL_MASK      \
	(ZUMA_DPP_RDMA_ALL_IRQ_MASK | ZUMA_DPP_RDMA_IRQ_ENABLE)
#define ZUMA_DPP_RDMA_FRAME_DONE_IRQ    BIT(16)
#define ZUMA_DPP_RDMA_DEADLOCK_IRQ      BIT(17)
#define ZUMA_DPP_RDMA_CONFIG_ERR_IRQ    BIT(21)
#define ZUMA_DPP_RDMA_ALL_IRQ_STATUS    0x0dfb0000
#define ZUMA_DPP_RDMA_CONFIG_ERR_STATUS 0x0740
#define ZUMA_DPP_RDMA_INHERITED_IRQ     ZUMA_DPP_RDMA_FRAME_DONE_IRQ
#define ZUMA_DPP_RDMA_MASKED_CONTROL    ZUMA_DPP_RDMA_ALL_IRQ_MASK
#define ZUMA_DPP_RDMA_OWNED_CONTROL     ZUMA_DPP_RDMA_IRQ_ENABLE
#define ZUMA_DPP_CORE_IRQ_ENABLE        BIT(0)
#define ZUMA_DPP_CORE_FRAME_DONE_IRQ    BIT(0)
#define ZUMA_DPP_CORE_CONFIG_ERR_IRQ    BIT(1)
#define ZUMA_DPP_CORE_ALL_IRQ_STATUS    0x3
#define ZUMA_DPP_CORE_INHERITED_MASK    0x4
#define ZUMA_DPP_CORE_MASKED_MASK       0x7
#define ZUMA_DPP_CORE_OWNED_MASK        ZUMA_DPP_CORE_INHERITED_MASK
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
#define ZUMA_SCANOUT_FB_BASE            0xf9c00000
#define ZUMA_FORMAT_SWITCH_DELAY_MS     2000

#define ZUMA_DRM_MODE_CLOCK_KHZ         270882
#define ZUMA_DRM_MODE_HSYNC_START       1424
#define ZUMA_DRM_MODE_HSYNC_END         1448
#define ZUMA_DRM_MODE_HTOTAL            1490
#define ZUMA_DRM_MODE_VSYNC_START       3004
#define ZUMA_DRM_MODE_VSYNC_END         3008
#define ZUMA_DRM_MODE_VTOTAL            3030
#define ZUMA_DRM_MODE_WIDTH_MM          70
#define ZUMA_DRM_MODE_HEIGHT_MM         155

struct zuma_display_block {
	const char *name;
	const char *compatible;
	resource_size_t phys;
	resource_size_t min_size;
	unsigned int resource_index;
	void __iomem *base;
};

struct zuma_dpp0_dt_resource {
	const char *name;
	resource_size_t start;
	resource_size_t size;
};

struct zuma_dpp0_snapshot_reg {
	const char *name;
	u16 live;
	u16 shadow;
};

#define ZUMA_DPP0_NO_SHADOW U16_MAX

enum zuma_dpp0_replay_region {
	ZUMA_DPP0_REPLAY_RDMA,
	ZUMA_DPP0_REPLAY_DPP,
	ZUMA_DPP0_REPLAY_SRAMC,
	ZUMA_DPP0_REPLAY_HDR_COMM,
};

enum zuma_dpp0_replay_stage {
	ZUMA_DPP0_REPLAY_RESETLESS_INIT,
	ZUMA_DPP0_REPLAY_DONE,
};

enum zuma_dpp0_irq_owner_state {
	ZUMA_DPP0_IRQ_INHERITED,
	ZUMA_DPP0_IRQ_ACQUIRING,
	ZUMA_DPP0_IRQ_OWNED,
	ZUMA_DPP0_IRQ_RELEASING,
	ZUMA_DPP0_IRQ_BROKEN,
};

struct zuma_dpp0_profile_reg {
	const char *name;
	enum zuma_dpp0_replay_region region;
	u16 live;
	u16 shadow;
	u32 value;
};

struct zuma_drm {
	struct drm_device drm;
	struct drm_connector connector;
	struct drm_plane primary_plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct completion frame_start_completion;
	struct completion frame_done_completion;
	struct completion dpp_dma_completion;
	struct completion dpp_core_completion;
	atomic64_t frame_start_irq_count;
	atomic64_t frame_done_irq_count;
	atomic64_t dpp_dma_irq_count;
	atomic64_t dpp_core_irq_count;
	atomic_t irq_error;
	atomic_t dpp_dma_error_status;
	atomic_t dpp_core_error_status;
	int frame_start_irq;
	int frame_done_irq;
	int dpp_dma_irq;
	int dpp_core_irq;
	u32 dpp_dma_status;
	u32 dpp_dma_control;
	u32 dpp_core_status;
	u32 dpp_core_control;
	u32 dpp_core_mask;
	u32 dpp_dma_config_error;
	u32 dpp_core_config_error;
	bool irq_proof_armed;
	bool irq_routes_enabled;
	bool irq_proven_once;
	bool dpp_irq_routes_enabled;
	enum zuma_dpp0_replay_stage replay_stage;
	enum zuma_dpp0_irq_owner_state dpp_irq_owner_state;
	struct workqueue_struct *workqueue;
	struct work_struct dpp_irq_fault_work;
	/* Serializes nonblocking queue admission with reboot shutdown. */
	struct mutex commit_admission_lock;
	struct notifier_block reboot_notifier;
};

struct zuma_drm_irq_proof {
	u64 frame_start_before;
	u64 frame_done_before;
	u64 vblank_before;
	u64 frame_start_after;
	u64 frame_done_after;
	u64 vblank_after;
	u64 dpp_dma_before;
	u64 dpp_core_before;
	u64 dpp_dma_after;
	u64 dpp_core_after;
	bool dpp_irq_active;
	bool dpp_irq_released;
};

static DEFINE_MUTEX(zuma_display_mmio_lock);
static DEFINE_MUTEX(zuma_drm_registration_lock);
static bool zuma_framebuffer_validated;
static bool zuma_drm_scanout_ready;
static bool zuma_drm_initcalls_complete;
static bool zuma_drm_update_failed;
static phys_addr_t zuma_framebuffer_phys __ro_after_init =
	ZUMA_HANDOFF_FB_BASE;
static u32 zuma_framebuffer_ctrl = ZUMA_HANDOFF_FB_CTRL;
static u64 zuma_drm_update_count;
static const u32 *zuma_boot_buffer __ro_after_init;
static void *zuma_scanout_buffer __ro_after_init;
static struct device *zuma_drm_root;
static struct zuma_drm *zuma_drm_device;

/*
 * Adopt the boot chain's fixed HK3 mode.  Its existing command-mode stream
 * uses DSC 1.1 with two 672x187 slices; this handoff driver does not rewrite
 * the panel, DSIM or DSC state.
 */
static const struct drm_display_mode zuma_drm_fixed_mode = {
	.clock = ZUMA_DRM_MODE_CLOCK_KHZ,
	.hdisplay = ZUMA_HANDOFF_FB_WIDTH,
	.hsync_start = ZUMA_DRM_MODE_HSYNC_START,
	.hsync_end = ZUMA_DRM_MODE_HSYNC_END,
	.htotal = ZUMA_DRM_MODE_HTOTAL,
	.vdisplay = ZUMA_HANDOFF_FB_HEIGHT,
	.vsync_start = ZUMA_DRM_MODE_VSYNC_START,
	.vsync_end = ZUMA_DRM_MODE_VSYNC_END,
	.vtotal = ZUMA_DRM_MODE_VTOTAL,
	.width_mm = ZUMA_DRM_MODE_WIDTH_MM,
	.height_mm = ZUMA_DRM_MODE_HEIGHT_MM,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
	.name = "1344x2992",
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

static struct zuma_display_block zuma_dpp0_dpp = {
	.name = "DPP0 core",
	.compatible = "samsung,exynos-dpp",
	.phys = ZUMA_DPP0_DPP_BASE,
	.min_size = ZUMA_DPP0_REG_SIZE,
	.resource_index = 1,
};

static struct zuma_display_block zuma_dpp0_sramc = {
	.name = "DPP0 SRAM controller",
	.compatible = "samsung,exynos-dpp",
	.phys = ZUMA_DPP0_SRAMC_BASE,
	.min_size = ZUMA_DPP0_REG_SIZE,
	.resource_index = 3,
};

static struct zuma_display_block zuma_dpp0_hdr_comm = {
	.name = "DPP0 HDR common",
	.compatible = "samsung,exynos-dpp",
	.phys = ZUMA_DPP0_HDR_COMM_BASE,
	.min_size = ZUMA_DPP0_REG_SIZE,
	.resource_index = 4,
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
	&zuma_dpp0_dpp,
	&zuma_dpp0_sramc,
	&zuma_dpp0_hdr_comm,
	&zuma_sysmmu_dpuf0,
};

static const struct zuma_dpp0_dt_resource zuma_dpp0_dt_resources[] = {
	{ "dma", ZUMA_DPP0_BASE, ZUMA_DPP0_REG_SIZE },
	{ "dpp", ZUMA_DPP0_DPP_BASE, ZUMA_DPP0_REG_SIZE },
	{ "scl_coef", ZUMA_DPP0_SCL_COEF_BASE, ZUMA_DPP0_SCL_COEF_SIZE },
	{ "sramc", ZUMA_DPP0_SRAMC_BASE, ZUMA_DPP0_REG_SIZE },
	{ "hdr_comm", ZUMA_DPP0_HDR_COMM_BASE, ZUMA_DPP0_REG_SIZE },
	{ "hdr", ZUMA_DPP0_HDR_BASE, ZUMA_DPP0_REG_SIZE },
};

static const struct zuma_dpp0_snapshot_reg zuma_dpp0_rdma_regs[] = {
	{ "enable", 0x000, ZUMA_DPP0_NO_SHADOW },
	{ "irq", 0x004, ZUMA_DPP0_NO_SHADOW },
	{ "in-ctrl", 0x008, 0x408 },
	{ "src-width", 0x010, 0x410 },
	{ "src-height", 0x014, 0x414 },
	{ "src-offset", 0x018, 0x418 },
	{ "img-size", 0x01c, 0x41c },
	{ "base-p0", 0x040, 0x440 },
	{ "base-p1", 0x044, 0x444 },
	{ "stride-p0", 0x050, 0x450 },
	{ "afbc", 0x070, 0x470 },
	{ "recovery", 0x080, 0x480 },
	{ "deadlock", 0x100, 0x500 },
	{ "qos-low", 0x130, ZUMA_DPP0_NO_SHADOW },
	{ "qos-high", 0x134, ZUMA_DPP0_NO_SHADOW },
	{ "dynamic-gating", 0x140, ZUMA_DPP0_NO_SHADOW },
};

static const struct zuma_dpp0_snapshot_reg zuma_dpp0_core_regs[] = {
	{ "swrst", 0x004, ZUMA_DPP0_NO_SHADOW },
	{ "irq-con", 0x010, ZUMA_DPP0_NO_SHADOW },
	{ "irq-mask", 0x014, ZUMA_DPP0_NO_SHADOW },
	{ "irq-status", 0x018, ZUMA_DPP0_NO_SHADOW },
	{ "cfg-error", 0x01c, ZUMA_DPP0_NO_SHADOW },
	{ "op-status", 0x030, ZUMA_DPP0_NO_SHADOW },
	{ "io-con", 0x038, 0x138 },
	{ "img-size", 0x03c, 0x13c },
	{ "scl-ctrl", 0x080, 0x180 },
	{ "scaled-size", 0x084, 0x184 },
	{ "scl-hpos", 0x090, 0x190 },
	{ "scl-vpos", 0x094, 0x194 },
};

static const struct zuma_dpp0_snapshot_reg zuma_dpp0_sramc_regs[] = {
	{ "mode", 0x010, 0x810 },
	{ "dst-position", 0x014, 0x814 },
};

static const struct zuma_dpp0_snapshot_reg zuma_dpp0_hdr_comm_regs[] = {
	{ "io-con", 0x00c, 0x80c },
	{ "size", 0x020, 0x820 },
};

static const struct zuma_dpp0_profile_reg zuma_dpp0_fixed_profile[] = {
	{ "qos-low", ZUMA_DPP0_REPLAY_RDMA, ZUMA_DPP_RDMA_QOS_LOW,
	  ZUMA_DPP0_NO_SHADOW, 0x44444444 },
	{ "qos-high", ZUMA_DPP0_REPLAY_RDMA, ZUMA_DPP_RDMA_QOS_HIGH,
	  ZUMA_DPP0_NO_SHADOW, 0x44444444 },
	{ "dynamic-gating", ZUMA_DPP0_REPLAY_RDMA,
	  ZUMA_DPP_RDMA_DYNAMIC_GATING, ZUMA_DPP0_NO_SHADOW, 0 },
	{ "alpha-ic-max-format", ZUMA_DPP0_REPLAY_RDMA,
	  ZUMA_DPP_RDMA_IN_CTRL_0,
	  ZUMA_DPP_RDMA_IN_CTRL_0 + ZUMA_DPP_RDMA_SHADOW_OFFSET,
	  ZUMA_HANDOFF_FB_BGRX_CTRL },
	{ "assigned-mo", ZUMA_DPP0_REPLAY_RDMA, ZUMA_DPP_RDMA_ENABLE,
	  ZUMA_DPP0_NO_SHADOW, ZUMA_DPP_RDMA_EXPECTED },
	{ "sramc-dst", ZUMA_DPP0_REPLAY_SRAMC,
	  ZUMA_DPP_SRAMC_DST_POSITION,
	  ZUMA_DPP_SRAMC_DST_POSITION + ZUMA_DPP_SRAMC_SHADOW_OFFSET,
	  ZUMA_DPP_FIXED_DST_POSITION },
	{ "sramc-mode", ZUMA_DPP0_REPLAY_SRAMC, ZUMA_DPP_SRAMC_MODE,
	  ZUMA_DPP_SRAMC_MODE + ZUMA_DPP_SRAMC_SHADOW_OFFSET, 0 },
	{ "scl-disable", ZUMA_DPP0_REPLAY_DPP, ZUMA_DPP_CORE_SCL_CTRL,
	  ZUMA_DPP_CORE_SCL_CTRL + ZUMA_DPP_CORE_SHADOW_OFFSET, 0 },
	{ "scaled-size", ZUMA_DPP0_REPLAY_DPP,
	  ZUMA_DPP_CORE_SCALED_IMG_SIZE,
	  ZUMA_DPP_CORE_SCALED_IMG_SIZE + ZUMA_DPP_CORE_SHADOW_OFFSET, 0 },
	{ "scl-hpos", ZUMA_DPP0_REPLAY_DPP,
	  ZUMA_DPP_CORE_SCL_HPOSITION,
	  ZUMA_DPP_CORE_SCL_HPOSITION + ZUMA_DPP_CORE_SHADOW_OFFSET, 0 },
	{ "scl-vpos", ZUMA_DPP0_REPLAY_DPP,
	  ZUMA_DPP_CORE_SCL_VPOSITION,
	  ZUMA_DPP_CORE_SCL_VPOSITION + ZUMA_DPP_CORE_SHADOW_OFFSET, 0 },
	{ "src-offset", ZUMA_DPP0_REPLAY_RDMA, ZUMA_DPP_RDMA_SRC_OFFSET,
	  ZUMA_DPP_RDMA_SRC_OFFSET + ZUMA_DPP_RDMA_SHADOW_OFFSET, 0 },
	{ "src-width", ZUMA_DPP0_REPLAY_RDMA, ZUMA_DPP_RDMA_SRC_WIDTH,
	  ZUMA_DPP_RDMA_SRC_WIDTH + ZUMA_DPP_RDMA_SHADOW_OFFSET,
	  ZUMA_HANDOFF_FB_WIDTH },
	{ "src-height", ZUMA_DPP0_REPLAY_RDMA, ZUMA_DPP_RDMA_SRC_HEIGHT,
	  ZUMA_DPP_RDMA_SRC_HEIGHT + ZUMA_DPP_RDMA_SHADOW_OFFSET,
	  ZUMA_HANDOFF_FB_HEIGHT },
	{ "rdma-img-size", ZUMA_DPP0_REPLAY_RDMA, ZUMA_DPP_RDMA_IMG_SIZE,
	  ZUMA_DPP_RDMA_IMG_SIZE + ZUMA_DPP_RDMA_SHADOW_OFFSET,
	  ZUMA_DPP_FIXED_IMG_SIZE },
	{ "dpp-img-size", ZUMA_DPP0_REPLAY_DPP, ZUMA_DPP_CORE_IMG_SIZE,
	  ZUMA_DPP_CORE_IMG_SIZE + ZUMA_DPP_CORE_SHADOW_OFFSET,
	  ZUMA_DPP_FIXED_IMG_SIZE },
	{ "hdr-size", ZUMA_DPP0_REPLAY_HDR_COMM, ZUMA_DPP_HDR_COMM_SIZE,
	  ZUMA_DPP_HDR_COMM_SIZE + ZUMA_DPP_HDR_COMM_SHADOW_OFFSET,
	  ZUMA_DPP_FIXED_IMG_SIZE },
	{ "base-p0", ZUMA_DPP0_REPLAY_RDMA, ZUMA_DPP_RDMA_BASEADDR_P0,
	  ZUMA_DPP_RDMA_BASEADDR_P0 + ZUMA_DPP_RDMA_SHADOW_OFFSET,
	  ZUMA_SCANOUT_FB_BASE },
	{ "base-p1", ZUMA_DPP0_REPLAY_RDMA, ZUMA_DPP_RDMA_BASEADDR_P1,
	  ZUMA_DPP_RDMA_BASEADDR_P1 + ZUMA_DPP_RDMA_SHADOW_OFFSET, 0 },
	{ "stride-p0", ZUMA_DPP0_REPLAY_RDMA,
	  ZUMA_DPP_RDMA_SRC_STRIDE_0,
	  ZUMA_DPP_RDMA_SRC_STRIDE_0 + ZUMA_DPP_RDMA_SHADOW_OFFSET, 0 },
	{ "dpp-io", ZUMA_DPP0_REPLAY_DPP, ZUMA_DPP_CORE_IO_CON,
	  ZUMA_DPP_CORE_IO_CON + ZUMA_DPP_CORE_SHADOW_OFFSET, 0 },
	{ "hdr-io", ZUMA_DPP0_REPLAY_HDR_COMM, ZUMA_DPP_HDR_COMM_IO_CON,
	  ZUMA_DPP_HDR_COMM_IO_CON + ZUMA_DPP_HDR_COMM_SHADOW_OFFSET, 0 },
	{ "recovery", ZUMA_DPP0_REPLAY_RDMA,
	  ZUMA_DPP_RDMA_RECOVERY_CTRL,
	  ZUMA_DPP_RDMA_RECOVERY_CTRL + ZUMA_DPP_RDMA_SHADOW_OFFSET,
	  ZUMA_DPP_FIXED_RECOVERY },
	{ "afbc-block-size", ZUMA_DPP0_REPLAY_RDMA,
	  ZUMA_DPP_RDMA_AFBC_PARAM,
	  ZUMA_DPP_RDMA_AFBC_PARAM + ZUMA_DPP_RDMA_SHADOW_OFFSET, 0 },
	{ "deadlock", ZUMA_DPP0_REPLAY_RDMA,
	  ZUMA_DPP_RDMA_DEADLOCK_CTRL,
	  ZUMA_DPP_RDMA_DEADLOCK_CTRL + ZUMA_DPP_RDMA_SHADOW_OFFSET,
	  ZUMA_DPP_FIXED_DEADLOCK },
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
static void zuma_display_format_workfn(struct work_struct *work);
static DECLARE_DELAYED_WORK(zuma_display_format_work,
				 zuma_display_format_workfn);
static void zuma_display_snapshot_workfn(struct work_struct *work);
static DECLARE_DELAYED_WORK(zuma_display_snapshot_work,
				 zuma_display_snapshot_workfn);

static const u32 zuma_drm_primary_plane_formats[] = {
	DRM_FORMAT_XRGB8888,
};

static enum drm_connector_status
zuma_drm_connector_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static int zuma_drm_connector_get_modes(struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector,
						    &zuma_drm_fixed_mode);
}

static int zuma_drm_primary_plane_atomic_check(struct drm_plane *plane,
					       struct drm_atomic_commit *state)
{
	struct zuma_drm *zdev = container_of(plane, struct zuma_drm,
					      primary_plane);
	struct drm_plane_state *old_plane_state;
	struct drm_plane_state *new_plane_state;
	struct drm_crtc_state *new_crtc_state;
	struct drm_framebuffer *fb;
	int ret;

	if (state->async_update)
		return -EOPNOTSUPP;

	old_plane_state = drm_atomic_get_old_plane_state(state, plane);
	new_plane_state = drm_atomic_get_new_plane_state(state, plane);
	if (!!new_plane_state->crtc != !!new_plane_state->fb)
		return -EINVAL;

	if (!new_plane_state->crtc) {
		if (old_plane_state && old_plane_state->crtc) {
			new_crtc_state =
				drm_atomic_get_crtc_state(state, old_plane_state->crtc);
			if (IS_ERR(new_crtc_state))
				return PTR_ERR(new_crtc_state);
		}
		return 0;
	}

	if (new_plane_state->crtc != &zdev->crtc)
		return -EINVAL;

	new_crtc_state =
		drm_atomic_get_crtc_state(state, new_plane_state->crtc);
	if (IS_ERR(new_crtc_state))
		return PTR_ERR(new_crtc_state);

	ret = drm_atomic_helper_check_plane_state(new_plane_state,
						  new_crtc_state,
						  DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING,
						  false, false);
	if (ret)
		return ret;
	if (!new_plane_state->visible)
		return -EINVAL;

	fb = new_plane_state->fb;
	if (fb->format->format != DRM_FORMAT_XRGB8888 ||
	    fb->format->num_planes != 1 ||
	    fb->modifier != DRM_FORMAT_MOD_LINEAR ||
	    fb->width != ZUMA_HANDOFF_FB_WIDTH ||
	    fb->height != ZUMA_HANDOFF_FB_HEIGHT ||
	    fb->pitches[0] != ZUMA_HANDOFF_FB_STRIDE || fb->offsets[0] ||
	    new_plane_state->src_x || new_plane_state->src_y ||
	    new_plane_state->src_w != ZUMA_HANDOFF_FB_WIDTH << 16 ||
	    new_plane_state->src_h != ZUMA_HANDOFF_FB_HEIGHT << 16 ||
	    new_plane_state->crtc_x || new_plane_state->crtc_y ||
	    new_plane_state->crtc_w != ZUMA_HANDOFF_FB_WIDTH ||
	    new_plane_state->crtc_h != ZUMA_HANDOFF_FB_HEIGHT ||
	    new_plane_state->rotation != DRM_MODE_ROTATE_0)
		return -EINVAL;

	return 0;
}

static void
zuma_drm_primary_plane_atomic_update(struct drm_plane *plane,
				     struct drm_atomic_commit *state)
{
	/* The driver commit path owns the fallible copy and frame trigger. */
}

static enum drm_mode_status
zuma_drm_crtc_mode_valid(struct drm_crtc *crtc,
			 const struct drm_display_mode *mode)
{
	return drm_crtc_helper_mode_valid_fixed(crtc, mode,
						&zuma_drm_fixed_mode);
}

static int zuma_drm_crtc_atomic_check(struct drm_crtc *crtc,
				      struct drm_atomic_commit *state)
{
	struct zuma_drm *zdev = container_of(crtc, struct zuma_drm, crtc);
	struct drm_connector_state *connector_state;
	struct drm_plane_state *plane_state;
	struct drm_crtc_state *old_crtc_state;
	struct drm_crtc_state *new_crtc_state;

	old_crtc_state = drm_atomic_get_old_crtc_state(state, crtc);
	new_crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	new_crtc_state->no_vblank = false;
	if (new_crtc_state->async_flip)
		return -EOPNOTSUPP;

	if (!new_crtc_state->enable) {
		if (old_crtc_state->enable || new_crtc_state->active)
			return -EOPNOTSUPP;
		return 0;
	}

	if (!old_crtc_state->enable && !new_crtc_state->active)
		return -EINVAL;
	if (old_crtc_state->enable &&
	    (new_crtc_state->mode_changed ||
	     new_crtc_state->connectors_changed))
		return -EOPNOTSUPP;
	if (old_crtc_state->active && !new_crtc_state->active)
		return -EOPNOTSUPP;
	if (!drm_mode_equal(&new_crtc_state->mode, &zuma_drm_fixed_mode) ||
	    !drm_mode_equal(&new_crtc_state->adjusted_mode,
			    &zuma_drm_fixed_mode))
		return -EINVAL;

	connector_state =
		drm_atomic_get_connector_state(state, &zdev->connector);
	if (IS_ERR(connector_state))
		return PTR_ERR(connector_state);
	if (connector_state->crtc != crtc)
		return -EINVAL;

	plane_state = drm_atomic_get_plane_state(state, &zdev->primary_plane);
	if (IS_ERR(plane_state))
		return PTR_ERR(plane_state);
	if (!old_crtc_state->enable &&
	    (plane_state->crtc != crtc || !plane_state->fb))
		return -EINVAL;

	return 0;
}

static const struct drm_plane_helper_funcs zuma_drm_primary_plane_helper_funcs = {
	DRM_GEM_SHADOW_PLANE_HELPER_FUNCS,
	.atomic_check = zuma_drm_primary_plane_atomic_check,
	.atomic_update = zuma_drm_primary_plane_atomic_update,
};

static const struct drm_plane_funcs zuma_drm_primary_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	DRM_GEM_SHADOW_PLANE_FUNCS,
};

static int zuma_drm_crtc_enable_vblank(struct drm_crtc *crtc);
static void zuma_drm_crtc_disable_vblank(struct drm_crtc *crtc);
static int zuma_drm_enable_irq_routes(struct zuma_drm *zdev);

static const struct drm_crtc_helper_funcs zuma_drm_crtc_helper_funcs = {
	.mode_valid = zuma_drm_crtc_mode_valid,
	.atomic_check = zuma_drm_crtc_atomic_check,
};

static const struct drm_crtc_funcs zuma_drm_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.enable_vblank = zuma_drm_crtc_enable_vblank,
	.disable_vblank = zuma_drm_crtc_disable_vblank,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
};

static const struct drm_encoder_funcs zuma_drm_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static const struct drm_connector_helper_funcs zuma_drm_connector_helper_funcs = {
	.get_modes = zuma_drm_connector_get_modes,
};

static const struct drm_connector_funcs zuma_drm_connector_funcs = {
	.detect = zuma_drm_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int zuma_drm_atomic_commit(struct drm_device *drm,
				  struct drm_atomic_commit *state,
				  bool nonblock);
static void zuma_display_set_hw_trigger(bool unmask);
static void zuma_drm_dpp_irq_fault_work(struct work_struct *work);
static int zuma_drm_reboot_notifier(struct notifier_block *notifier,
				    unsigned long action, void *data);

static const struct drm_mode_config_funcs zuma_drm_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = zuma_drm_atomic_commit,
};

DEFINE_DRM_GEM_FOPS(zuma_drm_fops);

static const struct drm_driver zuma_drm_driver = {
	DRM_GEM_SHMEM_DRIVER_OPS,
	.name = "zuma-display-handoff",
	.desc = "Google Zuma display bootloader handoff",
	.major = 1,
	.minor = 0,
	.driver_features = DRIVER_ATOMIC | DRIVER_GEM | DRIVER_MODESET,
	.fops = &zuma_drm_fops,
};

static bool zuma_drm_irq_proof_is_armed(struct zuma_drm *zdev)
{
	/* Pair with the commit-side publication of completion state. */
	return smp_load_acquire(&zdev->irq_proof_armed);
}

static void zuma_drm_set_irq_proof_armed(struct zuma_drm *zdev, bool armed)
{
	/* Publish proof state before Linux IRQ or DECON master changes. */
	smp_store_release(&zdev->irq_proof_armed, armed);
}

static enum zuma_dpp0_irq_owner_state
zuma_dpp0_irq_owner_state(struct zuma_drm *zdev)
{
	/* Pair with state publication before Linux IRQ route enable. */
	return smp_load_acquire(&zdev->dpp_irq_owner_state);
}

static void
zuma_dpp0_set_irq_owner_state(struct zuma_drm *zdev,
			      enum zuma_dpp0_irq_owner_state state)
{
	/* Keep handlers live until both Linux routes are synchronized. */
	smp_store_release(&zdev->dpp_irq_owner_state, state);
}

static bool zuma_dpp0_irq_handler_live(struct zuma_drm *zdev)
{
	enum zuma_dpp0_irq_owner_state state =
		zuma_dpp0_irq_owner_state(zdev);

	return state == ZUMA_DPP0_IRQ_ACQUIRING ||
	       state == ZUMA_DPP0_IRQ_OWNED ||
	       state == ZUMA_DPP0_IRQ_RELEASING;
}

static void zuma_drm_latch_irq_fault(struct zuma_drm *zdev)
{
	atomic_set(&zdev->irq_error, 1);
	WRITE_ONCE(zuma_drm_update_failed, true);
	complete(&zdev->frame_start_completion);
	complete(&zdev->frame_done_completion);
	complete(&zdev->dpp_dma_completion);
	complete(&zdev->dpp_core_completion);
	if (zdev->workqueue)
		queue_work(zdev->workqueue, &zdev->dpp_irq_fault_work);
}

static irqreturn_t
zuma_drm_handle_frame_irq(struct zuma_drm *zdev, u32 mask,
			  atomic64_t *counter, struct completion *completion,
			  bool account_vblank)
{
	bool proof_armed;
	u32 pending;

	proof_armed = zuma_drm_irq_proof_is_armed(zdev);
	pending = readl(zuma_decon0.base + ZUMA_DECON_INT_PEND);
	if (!(pending & mask)) {
		if (proof_armed || zdev->irq_routes_enabled)
			zuma_drm_latch_irq_fault(zdev);
		return IRQ_NONE;
	}

	writel(mask, zuma_decon0.base + ZUMA_DECON_INT_PEND);
	if (readl(zuma_decon0.base + ZUMA_DECON_INT_PEND) & mask) {
		zuma_drm_latch_irq_fault(zdev);
		return IRQ_HANDLED;
	}

	atomic64_inc(counter);
	if (!proof_armed)
		zuma_drm_latch_irq_fault(zdev);
	if (account_vblank && !drm_crtc_handle_vblank(&zdev->crtc))
		zuma_drm_latch_irq_fault(zdev);
	if (proof_armed)
		complete(completion);
	return IRQ_HANDLED;
}

static irqreturn_t zuma_drm_frame_start_irq(int irq, void *data)
{
	struct zuma_drm *zdev = data;

	return zuma_drm_handle_frame_irq(zdev, ZUMA_DECON_INT_FRAME_START,
					 &zdev->frame_start_irq_count,
					 &zdev->frame_start_completion, true);
}

static irqreturn_t zuma_drm_frame_done_irq(int irq, void *data)
{
	struct zuma_drm *zdev = data;

	return zuma_drm_handle_frame_irq(zdev, ZUMA_DECON_INT_FRAME_DONE,
					 &zdev->frame_done_irq_count,
					 &zdev->frame_done_completion, false);
}

static irqreturn_t zuma_drm_dpp_dma_irq(int irq, void *data)
{
	struct zuma_drm *zdev = data;
	bool proof_armed = zuma_drm_irq_proof_is_armed(zdev);
	bool live = zuma_dpp0_irq_handler_live(zdev);
	u32 control;
	u32 status;
	u32 value;
	bool bad = false;

	value = readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ);
	control = value & ZUMA_DPP_RDMA_CONTROL_MASK;
	status = value & ZUMA_DPP_RDMA_ALL_IRQ_STATUS;
	WRITE_ONCE(zdev->dpp_dma_control, control);
	WRITE_ONCE(zdev->dpp_dma_status, status);
	if (value & ~(ZUMA_DPP_RDMA_ALL_IRQ_STATUS |
		      ZUMA_DPP_RDMA_CONTROL_MASK))
		bad = true;
	if (status & ZUMA_DPP_RDMA_CONFIG_ERR_IRQ)
		WRITE_ONCE(zdev->dpp_dma_config_error,
			   readl(zuma_dpp0.base +
				 ZUMA_DPP_RDMA_CONFIG_ERR_STATUS));
	if (status)
		writel((live ? ZUMA_DPP_RDMA_OWNED_CONTROL : 0) | status,
		       zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ);
	if (!status || !live || control != ZUMA_DPP_RDMA_OWNED_CONTROL ||
	    readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ) !=
		ZUMA_DPP_RDMA_OWNED_CONTROL)
		bad = true;
	if (status & ZUMA_DPP_RDMA_FRAME_DONE_IRQ) {
		atomic64_inc(&zdev->dpp_dma_irq_count);
		if (proof_armed)
			complete(&zdev->dpp_dma_completion);
		else
			bad = true;
	}
	if (status & ~ZUMA_DPP_RDMA_FRAME_DONE_IRQ)
		bad = true;
	if (bad) {
		atomic_or(status, &zdev->dpp_dma_error_status);
		zuma_drm_latch_irq_fault(zdev);
	}

	return status ? IRQ_HANDLED : IRQ_NONE;
}

static irqreturn_t zuma_drm_dpp_core_irq(int irq, void *data)
{
	struct zuma_drm *zdev = data;
	bool proof_armed = zuma_drm_irq_proof_is_armed(zdev);
	bool live = zuma_dpp0_irq_handler_live(zdev);
	u32 control;
	u32 mask;
	u32 status;
	bool bad = false;

	control = readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_CON);
	mask = readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_MASK);
	status = readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_STATUS);
	WRITE_ONCE(zdev->dpp_core_control, control);
	WRITE_ONCE(zdev->dpp_core_mask, mask);
	WRITE_ONCE(zdev->dpp_core_status, status);
	if (status & ZUMA_DPP_CORE_CONFIG_ERR_IRQ)
		WRITE_ONCE(zdev->dpp_core_config_error,
			   readl(zuma_dpp0_dpp.base +
				 ZUMA_DPP_CORE_CONFIG_ERROR));
	if (status & ZUMA_DPP_CORE_ALL_IRQ_STATUS)
		writel(status & ZUMA_DPP_CORE_ALL_IRQ_STATUS,
		       zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_STATUS);
	if (!status || !live || control != ZUMA_DPP_CORE_IRQ_ENABLE ||
	    mask != ZUMA_DPP_CORE_OWNED_MASK ||
	    readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_STATUS))
		bad = true;
	if (status & ZUMA_DPP_CORE_FRAME_DONE_IRQ) {
		atomic64_inc(&zdev->dpp_core_irq_count);
		if (proof_armed)
			complete(&zdev->dpp_core_completion);
		else
			bad = true;
	}
	if (status & ~ZUMA_DPP_CORE_FRAME_DONE_IRQ)
		bad = true;
	if (bad) {
		atomic_or(status, &zdev->dpp_core_error_status);
		zuma_drm_latch_irq_fault(zdev);
	}

	return status ? IRQ_HANDLED : IRQ_NONE;
}

static struct device_node *zuma_drm_find_decon_node(void)
{
	struct device_node *np = NULL;
	struct resource res;

	while ((np = of_find_compatible_node(np, NULL,
					     "samsung,exynos-decon"))) {
		if (!of_address_to_resource(np, 0, &res) &&
		    res.start == ZUMA_DECON0_BASE)
			return np;
	}

	return NULL;
}

static struct device_node *zuma_drm_find_dpp0_node(void)
{
	struct device_node *np = NULL;
	struct resource res;

	while ((np = of_find_compatible_node(np, NULL,
					     "samsung,exynos-dpp"))) {
		if (!of_address_to_resource(np, 0, &res) &&
		    res.start == ZUMA_DPP0_BASE)
			return np;
	}

	return NULL;
}

static int zuma_drm_setup_irqs(struct zuma_drm *zdev)
{
	struct device_node *np;
	int ret;

	init_completion(&zdev->frame_start_completion);
	init_completion(&zdev->frame_done_completion);
	init_completion(&zdev->dpp_dma_completion);
	init_completion(&zdev->dpp_core_completion);
	atomic64_set(&zdev->frame_start_irq_count, 0);
	atomic64_set(&zdev->frame_done_irq_count, 0);
	atomic64_set(&zdev->dpp_dma_irq_count, 0);
	atomic64_set(&zdev->dpp_core_irq_count, 0);
	atomic_set(&zdev->irq_error, 0);
	atomic_set(&zdev->dpp_dma_error_status, 0);
	atomic_set(&zdev->dpp_core_error_status, 0);
	INIT_WORK(&zdev->dpp_irq_fault_work, zuma_drm_dpp_irq_fault_work);
	zuma_dpp0_set_irq_owner_state(zdev, ZUMA_DPP0_IRQ_INHERITED);
	zdev->frame_start_irq = -1;
	zdev->frame_done_irq = -1;
	zdev->dpp_dma_irq = -1;
	zdev->dpp_core_irq = -1;

	np = zuma_drm_find_decon_node();
	if (!np)
		return -ENODEV;

	zdev->frame_start_irq = of_irq_get_byname(np, "frame_start");
	zdev->frame_done_irq = of_irq_get_byname(np, "frame_done");
	of_node_put(np);
	if (zdev->frame_start_irq < 0)
		return zdev->frame_start_irq;
	if (zdev->frame_done_irq < 0)
		return zdev->frame_done_irq;
	if (zdev->frame_start_irq == zdev->frame_done_irq)
		return -EINVAL;

	ret = devm_request_irq(zuma_drm_root, zdev->frame_start_irq,
			       zuma_drm_frame_start_irq, IRQF_NO_AUTOEN,
			       "zuma-decon-frame-start", zdev);
	if (ret)
		return ret;

	ret = devm_request_irq(zuma_drm_root, zdev->frame_done_irq,
			       zuma_drm_frame_done_irq, IRQF_NO_AUTOEN,
			       "zuma-decon-frame-done", zdev);
	if (ret)
		return ret;

	np = zuma_drm_find_dpp0_node();
	if (!np)
		return -ENODEV;
	zdev->dpp_dma_irq = of_irq_get_byname(np, "dma");
	zdev->dpp_core_irq = of_irq_get_byname(np, "dpp");
	of_node_put(np);
	if (zdev->dpp_dma_irq < 0)
		return zdev->dpp_dma_irq;
	if (zdev->dpp_core_irq < 0)
		return zdev->dpp_core_irq;
	if (zdev->dpp_dma_irq == zdev->dpp_core_irq ||
	    zdev->dpp_dma_irq == zdev->frame_start_irq ||
	    zdev->dpp_dma_irq == zdev->frame_done_irq ||
	    zdev->dpp_core_irq == zdev->frame_start_irq ||
	    zdev->dpp_core_irq == zdev->frame_done_irq)
		return -EINVAL;

	ret = devm_request_irq(zuma_drm_root, zdev->dpp_dma_irq,
			       zuma_drm_dpp_dma_irq, IRQF_NO_AUTOEN,
			       "zuma-dpp0-dma", zdev);
	if (ret)
		return ret;
	ret = devm_request_irq(zuma_drm_root, zdev->dpp_core_irq,
			       zuma_drm_dpp_core_irq, IRQF_NO_AUTOEN,
			       "zuma-dpp0-core", zdev);
	if (ret)
		return ret;

	pr_info("zuma-display-handoff: requested disabled DECON frame IRQs start=%d done=%d\n",
		zdev->frame_start_irq, zdev->frame_done_irq);
	pr_info("zuma-display-handoff: requested disabled DPP0 IRQs dma=%d dpp=%d\n",
		zdev->dpp_dma_irq, zdev->dpp_core_irq);
	return 0;
}

static void zuma_drm_destroy_workqueue(void *data)
{
	destroy_workqueue(data);
}

static int zuma_drm_register(void)
{
	const struct drm_vblank_crtc_config vblank_config = {
		.offdelay_ms = -1,
	};
	struct drm_connector *connector;
	struct drm_device *drm;
	struct zuma_drm *zdev;
	int ret;

	if (zuma_drm_device)
		return -EALREADY;

	zuma_drm_root = root_device_register("zuma-display-handoff");
	if (IS_ERR(zuma_drm_root)) {
		ret = PTR_ERR(zuma_drm_root);
		zuma_drm_root = NULL;
		return ret;
	}

	zdev = devm_drm_dev_alloc(zuma_drm_root, &zuma_drm_driver,
				  struct zuma_drm, drm);
	if (IS_ERR(zdev)) {
		ret = PTR_ERR(zdev);
		goto err_unregister_root;
	}
	mutex_init(&zdev->commit_admission_lock);
	zdev->workqueue =
		alloc_ordered_workqueue("zuma-display-handoff", WQ_MEM_RECLAIM);
	if (!zdev->workqueue) {
		ret = -ENOMEM;
		goto err_unregister_root;
	}
	ret = devm_add_action_or_reset(zuma_drm_root,
				       zuma_drm_destroy_workqueue,
				       zdev->workqueue);
	if (ret)
		goto err_unregister_root;

	drm = &zdev->drm;
	ret = drmm_mode_config_init(drm);
	if (ret)
		goto err_unregister_root;

	drm->mode_config.min_width = ZUMA_HANDOFF_FB_WIDTH;
	drm->mode_config.max_width = ZUMA_HANDOFF_FB_WIDTH;
	drm->mode_config.min_height = ZUMA_HANDOFF_FB_HEIGHT;
	drm->mode_config.max_height = ZUMA_HANDOFF_FB_HEIGHT;
	drm->mode_config.preferred_depth = 24;
	drm->mode_config.funcs = &zuma_drm_mode_config_funcs;

	ret = drm_universal_plane_init(drm, &zdev->primary_plane, 0,
				       &zuma_drm_primary_plane_funcs,
				       zuma_drm_primary_plane_formats,
				       ARRAY_SIZE(zuma_drm_primary_plane_formats),
				       NULL, DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		goto err_unregister_root;

	drm_plane_helper_add(&zdev->primary_plane,
			     &zuma_drm_primary_plane_helper_funcs);
	drm_plane_enable_fb_damage_clips(&zdev->primary_plane);

	ret = drm_crtc_init_with_planes(drm, &zdev->crtc,
					&zdev->primary_plane, NULL,
					&zuma_drm_crtc_funcs, NULL);
	if (ret)
		goto err_unregister_root;

	drm_crtc_helper_add(&zdev->crtc, &zuma_drm_crtc_helper_funcs);

	ret = drm_vblank_init(drm, 1);
	if (ret)
		goto err_unregister_root;

	ret = drm_encoder_init(drm, &zdev->encoder, &zuma_drm_encoder_funcs,
			       DRM_MODE_ENCODER_DSI, NULL);
	if (ret)
		goto err_unregister_root;

	zdev->encoder.possible_crtcs = drm_crtc_mask(&zdev->crtc);

	connector = &zdev->connector;
	ret = drm_connector_init(drm, connector, &zuma_drm_connector_funcs,
				 DRM_MODE_CONNECTOR_DSI);
	if (ret)
		goto err_unregister_root;

	drm_connector_helper_add(connector, &zuma_drm_connector_helper_funcs);
	connector->display_info.width_mm = ZUMA_DRM_MODE_WIDTH_MM;
	connector->display_info.height_mm = ZUMA_DRM_MODE_HEIGHT_MM;

	ret = drm_connector_attach_encoder(connector, &zdev->encoder);
	if (ret)
		goto err_unregister_root;

	ret = zuma_drm_setup_irqs(zdev);
	if (ret)
		goto err_unregister_root;
	zdev->reboot_notifier.notifier_call = zuma_drm_reboot_notifier;
	ret = devm_register_reboot_notifier(zuma_drm_root,
					    &zdev->reboot_notifier);
	if (ret)
		goto err_unregister_root;
	ret = zuma_drm_enable_irq_routes(zdev);
	if (ret)
		goto err_unregister_root;
	if (READ_ONCE(zuma_drm_update_failed)) {
		ret = -EIO;
		goto err_disable_irq_routes;
	}

	drm_mode_config_reset(drm);
	drm_crtc_vblank_on_config(&zdev->crtc, &vblank_config);
	dev_set_drvdata(zuma_drm_root, zdev);

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto err_disable_irq_routes;

	zuma_drm_device = zdev;
	pr_info("zuma-display-handoff: registered DRM-only fixed-mode shadow updates with persistent DPP IRQ lifecycle and ordered nonblocking flips\n");
	return 0;

err_disable_irq_routes:
	disable_irq_nosync(zdev->frame_done_irq);
	disable_irq_nosync(zdev->frame_start_irq);
	zdev->irq_routes_enabled = false;
	synchronize_irq(zdev->frame_done_irq);
	synchronize_irq(zdev->frame_start_irq);
err_unregister_root:
	root_device_unregister(zuma_drm_root);
	zuma_drm_root = NULL;
	return ret;
}

static void zuma_drm_try_register(void)
{
	int ret = 0;

	mutex_lock(&zuma_drm_registration_lock);
	if (zuma_drm_scanout_ready && zuma_drm_initcalls_complete &&
	    !zuma_drm_device)
		ret = zuma_drm_register();
	mutex_unlock(&zuma_drm_registration_lock);

	if (ret)
		pr_err("zuma-display-handoff: DRM/KMS scaffold registration failed: %d\n",
		       ret);
}

/* The built-in DRM core initializes at device-initcall level. */
static int __init zuma_drm_late_init(void)
{
	mutex_lock(&zuma_drm_registration_lock);
	zuma_drm_initcalls_complete = true;
	mutex_unlock(&zuma_drm_registration_lock);

	zuma_drm_try_register();
	return 0;
}
late_initcall(zuma_drm_late_init);

static bool __init
zuma_dpp0_dt_u32_matches(struct device_node *np, const char *name, u32 expected)
{
	const struct property *property;
	u32 value;
	int length;

	property = of_find_property(np, name, &length);
	if (!property || length != sizeof(__be32) ||
	    of_property_read_u32(np, name, &value) || value != expected) {
		pr_err("zuma-display-handoff: R34 DPP0 DT invalid %s property\n",
		       name);
		return false;
	}

	pr_info("zuma-display-handoff: R34 DPP0 DT %s=%#x length=%d\n",
		name, value, length);
	return true;
}

static bool __init zuma_dpp0_dt_contract_valid(void)
{
	static const char * const irq_names[] = { "dma", "dpp" };
	static const u32 irq_cells_expected[] = {
		0, 0x100, 0x4, 0,
		0, 0x109, 0x4, 0,
	};
	struct device_node *decon = NULL;
	struct device_node *irq_parent = NULL;
	struct device_node *member;
	struct device_node *np = NULL;
	const __be32 *irq_cells;
	const __be32 *dpp_cells;
	const char *name;
	struct resource res;
	resource_size_t size;
	unsigned int route_count;
	unsigned int route_index = UINT_MAX;
	unsigned int route_matches = 0;
	int count, length = 0;
	unsigned int i;
	bool valid = false;
	u32 cells;

	while ((np = of_find_compatible_node(np, NULL, "samsung,exynos-dpp"))) {
		if (!of_address_to_resource(np, 0, &res) &&
		    res.start == ZUMA_DPP0_BASE)
			break;
	}
	if (!np) {
		pr_err("zuma-display-handoff: R34 exact DPP0 DT node not found\n");
		return false;
	}
	if (!of_device_is_available(np)) {
		pr_err("zuma-display-handoff: R34 DPP0 DT node is disabled\n");
		goto out;
	}

	count = of_property_count_strings(np, "compatible");
	if (count != 1 ||
	    of_property_read_string_index(np, "compatible", 0, &name) ||
	    strcmp(name, "samsung,exynos-dpp")) {
		pr_err("zuma-display-handoff: R34 DPP0 DT compatible mismatch\n");
		goto out;
	}

	count = of_address_count(np);
	if (count != (int)ARRAY_SIZE(zuma_dpp0_dt_resources)) {
		pr_err("zuma-display-handoff: R34 DPP0 DT resource count=%d expected=%zu\n",
		       count, ARRAY_SIZE(zuma_dpp0_dt_resources));
		goto out;
	}
	if (of_property_count_strings(np, "reg-names") != count) {
		pr_err("zuma-display-handoff: R34 DPP0 DT reg-names count mismatch\n");
		goto out;
	}
	for (i = 0; i < (unsigned int)count; i++) {
		if (of_property_read_string_index(np, "reg-names", i, &name) ||
		    strcmp(name, zuma_dpp0_dt_resources[i].name) ||
		    of_address_to_resource(np, i, &res)) {
			pr_err("zuma-display-handoff: R34 DPP0 DT resource %u invalid\n",
			       i);
			goto out;
		}
		size = resource_size(&res);
		if (res.start != zuma_dpp0_dt_resources[i].start ||
		    size != zuma_dpp0_dt_resources[i].size) {
			pr_err("zuma-display-handoff: R34 DPP0 DT resource %u %s mismatch %pr\n",
			       i, name, &res);
			goto out;
		}
		pr_info("zuma-display-handoff: R34 DPP0 DT resource[%u]=%s %pr\n",
			i, name, &res);
	}

	if (!zuma_dpp0_dt_u32_matches(np, "attr", ZUMA_DPP0_DT_ATTR) ||
	    !zuma_dpp0_dt_u32_matches(np, "port", 0) ||
	    !zuma_dpp0_dt_u32_matches(np, "scale_down", 4) ||
	    !zuma_dpp0_dt_u32_matches(np, "scale_up", 8) ||
	    !zuma_dpp0_dt_u32_matches(np, "dpp,id", 0))
		goto out;

	if (of_property_count_strings(np, "interrupt-names") !=
	    (int)ARRAY_SIZE(irq_names)) {
		pr_err("zuma-display-handoff: R34 DPP0 DT interrupt-names count mismatch\n");
		goto out;
	}
	for (i = 0; i < ARRAY_SIZE(irq_names); i++) {
		if (of_property_read_string_index(np, "interrupt-names", i,
						  &name) ||
		    strcmp(name, irq_names[i])) {
			pr_err("zuma-display-handoff: R34 DPP0 DT interrupt name %u invalid\n",
			       i);
			goto out;
		}
	}

	irq_parent = of_irq_find_parent(np);
	if (!irq_parent ||
	    of_property_read_u32(irq_parent, "#interrupt-cells", &cells) ||
	    cells != 4) {
		pr_err("zuma-display-handoff: R34 DPP0 DT interrupt parent invalid\n");
		goto out;
	}
	irq_cells = of_get_property(np, "interrupts", &length);
	if (!irq_cells || length != sizeof(irq_cells_expected)) {
		pr_err("zuma-display-handoff: R34 DPP0 DT interrupts length=%d invalid\n",
		       length);
		goto out;
	}
	for (i = 0; i < ARRAY_SIZE(irq_cells_expected); i++) {
		if (be32_to_cpup(irq_cells + i) != irq_cells_expected[i]) {
			pr_err("zuma-display-handoff: R34 DPP0 DT interrupt cell %u invalid\n",
			       i);
			goto out;
		}
	}
	pr_info("zuma-display-handoff: R34 DPP0 DT irq dma=<0 %#x %#x 0> dpp=<0 %#x %#x 0>\n",
		irq_cells_expected[1], irq_cells_expected[2],
		irq_cells_expected[5], irq_cells_expected[6]);

	while ((decon = of_find_compatible_node(decon, NULL, "samsung,exynos-decon"))) {
		if (!of_address_to_resource(decon, 0, &res) &&
		    res.start == ZUMA_DECON0_BASE)
			break;
	}
	if (!decon || !of_device_is_available(decon)) {
		pr_err("zuma-display-handoff: R34 exact DECON0 DT node unavailable\n");
		goto out;
	}
	dpp_cells = of_get_property(decon, "dpps", &length);
	if (!dpp_cells || length <= 0 || length % sizeof(*dpp_cells)) {
		pr_err("zuma-display-handoff: R34 DECON0 dpps property invalid\n");
		goto out;
	}
	route_count = length / sizeof(*dpp_cells);
	for (i = 0; i < route_count; i++) {
		member = of_parse_phandle(decon, "dpps", i);
		if (!member) {
			pr_err("zuma-display-handoff: R34 DECON0 dpps[%u] invalid\n",
			       i);
			goto out;
		}
		if (member == np) {
			route_matches++;
			route_index = i;
		}
		of_node_put(member);
	}
	if (route_matches != 1 || route_index != 0) {
		pr_err("zuma-display-handoff: R34 DPP0 DECON0 membership matches=%u index=%u invalid\n",
		       route_matches, route_index);
		goto out;
	}

	pr_info("zuma-display-handoff: R34 DPP0 DT contract ok; DECON0 membership index=%u count=%u\n",
		route_index, route_count);
	pr_info("zuma-display-handoff: R34 DPP0 coverage dma=0x19900000+0x1000 dpp=0x19930000+0x1000 sramc=0x19950000+0x1000 hdr_comm=0x19960000+0x1000; scl_coef,hdr validated-unmapped\n");
	pr_info("zuma-display-handoff: R40 DPP0 resetless fixed-init contract dt-capabilities=%#x active-profile=%#x excluded=%#x\n",
		ZUMA_DPP0_DT_ATTR, ZUMA_DPP0_BL_EFFECTIVE_ATTR,
		ZUMA_DPP0_DT_ATTR ^ ZUMA_DPP0_BL_EFFECTIVE_ATTR);
	valid = true;

out:
	of_node_put(decon);
	of_node_put(irq_parent);
	of_node_put(np);
	return valid;
}

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

static void
zuma_dpp0_read_snapshot(void __iomem *base,
			const struct zuma_dpp0_snapshot_reg *regs,
			size_t count, u32 *live, u32 *shadow)
{
	size_t i;

	for (i = 0; i < (unsigned int)count; i++) {
		live[i] = readl(base + regs[i].live);
		if (regs[i].shadow != ZUMA_DPP0_NO_SHADOW)
			shadow[i] = readl(base + regs[i].shadow);
	}
}

static void
zuma_dpp0_log_snapshot(const char *label, const char *region,
		       const struct zuma_dpp0_snapshot_reg *regs,
		       size_t count, const u32 *live, const u32 *shadow)
{
	size_t i;

	for (i = 0; i < (unsigned int)count; i++) {
		if (regs[i].shadow == ZUMA_DPP0_NO_SHADOW)
			pr_info("zuma-display-handoff: R34 %s %s %s[%#x]=%#x\n",
				label, region, regs[i].name, regs[i].live,
				live[i]);
		else
			pr_info("zuma-display-handoff: R34 %s %s %s[%#x]=%#x shadow[%#x]=%#x\n",
				label, region, regs[i].name, regs[i].live,
				live[i], regs[i].shadow, shadow[i]);
	}
}

static bool zuma_dpp0_preflight_snapshot(const char *label)
{
	u32 rdma_live[ARRAY_SIZE(zuma_dpp0_rdma_regs)];
	u32 rdma_shadow[ARRAY_SIZE(zuma_dpp0_rdma_regs)] = { 0 };
	u32 core_live[ARRAY_SIZE(zuma_dpp0_core_regs)];
	u32 core_shadow[ARRAY_SIZE(zuma_dpp0_core_regs)] = { 0 };
	u32 sramc_live[ARRAY_SIZE(zuma_dpp0_sramc_regs)];
	u32 sramc_shadow[ARRAY_SIZE(zuma_dpp0_sramc_regs)] = { 0 };
	u32 hdr_live[ARRAY_SIZE(zuma_dpp0_hdr_comm_regs)];
	u32 hdr_shadow[ARRAY_SIZE(zuma_dpp0_hdr_comm_regs)] = { 0 };
	u32 frame_before, frame_after;
	u32 request_before, request_after;
	u32 config_error = 0;
	u32 dpub, dpuf0, dpuf1;

	if (!zuma_display_domains_on(&dpub, &dpuf0, &dpuf1)) {
		pr_info("zuma-display-handoff: R34 %s DPP0 snapshot skipped; domains DPUB=%#x DPUF0=%#x DPUF1=%#x\n",
			label, dpub, dpuf0, dpuf1);
		return false;
	}

	frame_before = readl(zuma_decon0.base + ZUMA_DECON_FRAME_COUNT);
	request_before = readl(zuma_decon0.base + ZUMA_DECON_SHD_REG_UP_REQ);
	if (request_before) {
		pr_info("zuma-display-handoff: R34 %s DPP0 snapshot rejected; frame=%#x shadow-request=%#x\n",
			label, frame_before, request_before);
		return false;
	}

	zuma_dpp0_read_snapshot(zuma_dpp0.base, zuma_dpp0_rdma_regs,
				ARRAY_SIZE(zuma_dpp0_rdma_regs), rdma_live,
				rdma_shadow);
	if (rdma_live[1] & ZUMA_DPP_RDMA_CONFIG_ERR_IRQ)
		config_error = readl(zuma_dpp0.base +
				     ZUMA_DPP_RDMA_CONFIG_ERR_STATUS);
	zuma_dpp0_read_snapshot(zuma_dpp0_dpp.base, zuma_dpp0_core_regs,
				ARRAY_SIZE(zuma_dpp0_core_regs), core_live,
				core_shadow);
	zuma_dpp0_read_snapshot(zuma_dpp0_sramc.base, zuma_dpp0_sramc_regs,
				ARRAY_SIZE(zuma_dpp0_sramc_regs), sramc_live,
				sramc_shadow);
	zuma_dpp0_read_snapshot(zuma_dpp0_hdr_comm.base,
				zuma_dpp0_hdr_comm_regs,
				ARRAY_SIZE(zuma_dpp0_hdr_comm_regs), hdr_live,
				hdr_shadow);

	frame_after = readl(zuma_decon0.base + ZUMA_DECON_FRAME_COUNT);
	request_after = readl(zuma_decon0.base + ZUMA_DECON_SHD_REG_UP_REQ);
	if (frame_after != frame_before || request_after) {
		pr_info("zuma-display-handoff: R34 %s DPP0 snapshot incoherent; frame=%#x->%#x shadow-request=%#x->%#x\n",
			label, frame_before, frame_after, request_before,
			request_after);
		return false;
	}

	pr_info("zuma-display-handoff: R34 %s DPP0 coherent frame=%#x shadow-request=0\n",
		label, frame_after);
	zuma_dpp0_log_snapshot(label, "RDMA", zuma_dpp0_rdma_regs,
			       ARRAY_SIZE(zuma_dpp0_rdma_regs), rdma_live,
			       rdma_shadow);
	if (rdma_live[1] & ZUMA_DPP_RDMA_CONFIG_ERR_IRQ)
		pr_info("zuma-display-handoff: R34 %s RDMA config-error[%#x]=%#x\n",
			label, ZUMA_DPP_RDMA_CONFIG_ERR_STATUS, config_error);
	zuma_dpp0_log_snapshot(label, "DPP", zuma_dpp0_core_regs,
			       ARRAY_SIZE(zuma_dpp0_core_regs), core_live,
			       core_shadow);
	zuma_dpp0_log_snapshot(label, "SRAMC", zuma_dpp0_sramc_regs,
			       ARRAY_SIZE(zuma_dpp0_sramc_regs), sramc_live,
			       sramc_shadow);
	zuma_dpp0_log_snapshot(label, "HDR-COMM", zuma_dpp0_hdr_comm_regs,
			       ARRAY_SIZE(zuma_dpp0_hdr_comm_regs), hdr_live,
			       hdr_shadow);
	return true;
}

static const char *
zuma_dpp0_replay_stage_name(enum zuma_dpp0_replay_stage stage)
{
	switch (stage) {
	case ZUMA_DPP0_REPLAY_RESETLESS_INIT:
		return "resetless-init";
	case ZUMA_DPP0_REPLAY_DONE:
		return "done";
	}

	return "invalid";
}

static void __iomem *
zuma_dpp0_replay_base(enum zuma_dpp0_replay_region region)
{
	switch (region) {
	case ZUMA_DPP0_REPLAY_RDMA:
		return zuma_dpp0.base;
	case ZUMA_DPP0_REPLAY_DPP:
		return zuma_dpp0_dpp.base;
	case ZUMA_DPP0_REPLAY_SRAMC:
		return zuma_dpp0_sramc.base;
	case ZUMA_DPP0_REPLAY_HDR_COMM:
		return zuma_dpp0_hdr_comm.base;
	}

	return NULL;
}

static int
zuma_dpp0_profile_state_ready(bool include_shadow, bool inherited_boot)
{
	size_t i;

	lockdep_assert_held(&zuma_display_mmio_lock);

	for (i = 0; i < ARRAY_SIZE(zuma_dpp0_fixed_profile); i++) {
		const struct zuma_dpp0_profile_reg *reg =
			&zuma_dpp0_fixed_profile[i];
		void __iomem *base = zuma_dpp0_replay_base(reg->region);
		u32 value = reg->value;

		if (inherited_boot && reg->region == ZUMA_DPP0_REPLAY_DPP &&
		    reg->live == ZUMA_DPP_CORE_IO_CON)
			value = ZUMA_DPP_CORE_INHERITED_IO;
		if (readl(base + reg->live) != value)
			return -EIO;
		if (include_shadow && reg->shadow != ZUMA_DPP0_NO_SHADOW &&
		    readl(base + reg->shadow) != value)
			return -EIO;
	}

	return 0;
}

static int zuma_dpp0_inherited_boot_profile_ready(void)
{
	u32 rdma_irq;

	lockdep_assert_held(&zuma_display_mmio_lock);
	if (zuma_dpp0_profile_state_ready(true, true))
		return -EIO;
	rdma_irq = readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ);
	if (rdma_irq != 0 && rdma_irq != ZUMA_DPP_RDMA_INHERITED_IRQ)
		return -EIO;

	return readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_SWRST) ||
	       readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_CON) ||
	       readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_MASK) !=
		ZUMA_DPP_CORE_INHERITED_MASK ||
	       readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_STATUS) ||
	       readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_CONFIG_ERROR) ||
	       readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_OP_STATUS) ?
		-EIO : 0;
}

static int zuma_dpp0_resetless_fixed_live_ready(const char *phase)
{
	size_t i;

	lockdep_assert_held(&zuma_display_mmio_lock);
	for (i = 0; i < ARRAY_SIZE(zuma_dpp0_fixed_profile); i++) {
		const struct zuma_dpp0_profile_reg *reg =
			&zuma_dpp0_fixed_profile[i];
		void __iomem *base = zuma_dpp0_replay_base(reg->region);
		u32 actual = readl(base + reg->live);

		if (actual != reg->value) {
			pr_err("zuma-display-handoff: resetless %s %s live[%#x]=%#x expected=%#x\n",
			       phase, reg->name, reg->live, actual, reg->value);
			return -EIO;
		}
	}

	return 0;
}

static int
zuma_dpp0_resetless_mixed_profile_ready(const char *phase)
{
	size_t i;

	lockdep_assert_held(&zuma_display_mmio_lock);
	if (zuma_dpp0_resetless_fixed_live_ready(phase))
		return -EIO;
	for (i = 0; i < ARRAY_SIZE(zuma_dpp0_fixed_profile); i++) {
		const struct zuma_dpp0_profile_reg *reg =
			&zuma_dpp0_fixed_profile[i];
		void __iomem *base = zuma_dpp0_replay_base(reg->region);
		u32 actual;
		u32 expected;

		if (reg->shadow == ZUMA_DPP0_NO_SHADOW)
			continue;
		expected = reg->value;
		if (reg->region == ZUMA_DPP0_REPLAY_DPP &&
		    reg->live == ZUMA_DPP_CORE_IO_CON)
			expected = ZUMA_DPP_CORE_INHERITED_IO;
		actual = readl(base + reg->shadow);
		if (actual != expected) {
			pr_err("zuma-display-handoff: resetless %s %s shadow[%#x]=%#x expected=%#x\n",
			       phase, reg->name, reg->shadow, actual, expected);
			return -EIO;
		}
	}

	return 0;
}

static int
zuma_dpp0_resetless_irq_ready(const char *phase, bool inherited)
{
	u32 config_error;
	u32 irq_con;
	u32 irq_mask;
	u32 irq_status;
	u32 op_status;
	u32 rdma_irq;
	u32 swrst;
	bool rdma_ready;

	lockdep_assert_held(&zuma_display_mmio_lock);
	if (zuma_dpp0_resetless_mixed_profile_ready(phase))
		return -EIO;

	rdma_irq = readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ);
	swrst = readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_SWRST);
	irq_con = readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_CON);
	irq_mask = readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_MASK);
	irq_status = readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_STATUS);
	config_error = readl(zuma_dpp0_dpp.base +
			     ZUMA_DPP_CORE_CONFIG_ERROR);
	op_status = readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_OP_STATUS);
	rdma_ready = inherited ?
		(rdma_irq == 0 || rdma_irq == ZUMA_DPP_RDMA_INHERITED_IRQ) :
		rdma_irq == ZUMA_DPP_RDMA_OWNED_CONTROL;
	if (!rdma_ready || swrst ||
	    irq_con != (inherited ? 0 : ZUMA_DPP_CORE_IRQ_ENABLE) ||
	    irq_mask != (inherited ? ZUMA_DPP_CORE_INHERITED_MASK :
				     ZUMA_DPP_CORE_OWNED_MASK) ||
	    irq_status || config_error || op_status) {
		pr_err("zuma-display-handoff: resetless %s IRQ gate rdma=%#x expected=%s swrst=%#x con=%#x mask=%#x status=%#x config=%#x op=%#x\n",
		       phase, rdma_irq,
		       inherited ? "0|frame-done" : "owned", swrst,
		       irq_con, irq_mask, irq_status, config_error, op_status);
		return -EIO;
	}

	return 0;
}

static int zuma_dpp0_resetless_pretrigger_ready(void)
{
	return zuma_dpp0_resetless_irq_ready("entry", true);
}

static int zuma_dpp0_resetless_owned_pretrigger_ready(void)
{
	return zuma_dpp0_resetless_irq_ready("owned", false);
}

static int
zuma_dpp0_irq_profile_ready(u32 rdma_irq, u32 dpp_irq_con,
			    u32 dpp_irq_mask)
{
	lockdep_assert_held(&zuma_display_mmio_lock);

	if (zuma_dpp0_profile_state_ready(true, false))
		return -EIO;

	return readl(zuma_dpp0.base + ZUMA_DPP_RDMA_ENABLE) !=
			ZUMA_DPP_RDMA_EXPECTED ||
	       readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ) != rdma_irq ||
	       readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_SWRST) != 0 ||
	       readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_CON) !=
			dpp_irq_con ||
	       readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_MASK) !=
			dpp_irq_mask ||
	       readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_STATUS) != 0 ||
	       readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_CONFIG_ERROR) != 0 ||
	       readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_OP_STATUS) != 0 ?
		-EIO : 0;
}

static int zuma_dpp0_irq_inherited_ready(void)
{
	if (!zuma_dpp0_irq_profile_ready(0, 0,
					 ZUMA_DPP_CORE_INHERITED_MASK))
		return 0;
	return zuma_dpp0_irq_profile_ready(ZUMA_DPP_RDMA_INHERITED_IRQ, 0,
					   ZUMA_DPP_CORE_INHERITED_MASK);
}

static int zuma_dpp0_irq_restored_ready(void)
{
	return zuma_dpp0_irq_profile_ready(0, 0,
					   ZUMA_DPP_CORE_INHERITED_MASK);
}

static int zuma_dpp0_irq_owned_ready(void)
{
	return zuma_dpp0_irq_profile_ready(ZUMA_DPP_RDMA_OWNED_CONTROL,
					   ZUMA_DPP_CORE_IRQ_ENABLE,
					   ZUMA_DPP_CORE_OWNED_MASK);
}

static void zuma_dpp0_resetless_write_profile(void)
{
	size_t i;

	lockdep_assert_held(&zuma_display_mmio_lock);

	for (i = 0; i < ARRAY_SIZE(zuma_dpp0_fixed_profile); i++) {
		const struct zuma_dpp0_profile_reg *reg =
			&zuma_dpp0_fixed_profile[i];

		writel(reg->value,
		       zuma_dpp0_replay_base(reg->region) + reg->live);
	}
}

static int
zuma_dpp0_resetless_quiesce_irqs(struct zuma_drm *zdev,
				 bool allow_inherited_frame_done)
{
	u32 core_control;
	u32 core_mask;
	u32 core_status;
	u32 dma_control;
	u32 dma_status;
	u32 unexpected;
	u32 value;
	int ret = 0;

	value = readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ);
	dma_control = value & ZUMA_DPP_RDMA_CONTROL_MASK;
	dma_status = value & ZUMA_DPP_RDMA_ALL_IRQ_STATUS;
	if (dma_status & ZUMA_DPP_RDMA_CONFIG_ERR_IRQ)
		WRITE_ONCE(zdev->dpp_dma_config_error,
			   readl(zuma_dpp0.base +
				 ZUMA_DPP_RDMA_CONFIG_ERR_STATUS));
	unexpected = dma_status;
	if (allow_inherited_frame_done)
		unexpected &= ~ZUMA_DPP_RDMA_FRAME_DONE_IRQ;
	if (unexpected ||
	    (value & ~(ZUMA_DPP_RDMA_ALL_IRQ_STATUS |
		       ZUMA_DPP_RDMA_CONTROL_MASK)) ||
	    (dma_control != 0 &&
	     dma_control != ZUMA_DPP_RDMA_MASKED_CONTROL)) {
		atomic_or(unexpected, &zdev->dpp_dma_error_status);
		ret = -EIO;
	}
	writel(ZUMA_DPP_RDMA_MASKED_CONTROL | dma_status,
	       zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ);
	if (readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ) !=
	    ZUMA_DPP_RDMA_MASKED_CONTROL)
		ret = -EIO;

	core_control = readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_CON);
	core_mask = readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_MASK);
	core_status = readl(zuma_dpp0_dpp.base +
			    ZUMA_DPP_CORE_IRQ_STATUS);
	if (core_control || (core_mask & ~ZUMA_DPP_CORE_MASKED_MASK))
		ret = -EIO;
	if (core_status & ZUMA_DPP_CORE_CONFIG_ERR_IRQ)
		WRITE_ONCE(zdev->dpp_core_config_error,
			   readl(zuma_dpp0_dpp.base +
				 ZUMA_DPP_CORE_CONFIG_ERROR));
	if (core_status) {
		atomic_or(core_status, &zdev->dpp_core_error_status);
		ret = -EIO;
	}
	writel(ZUMA_DPP_CORE_MASKED_MASK,
	       zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_MASK);
	if (core_status & ZUMA_DPP_CORE_ALL_IRQ_STATUS)
		writel(core_status & ZUMA_DPP_CORE_ALL_IRQ_STATUS,
		       zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_STATUS);
	writel(0, zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_CON);
	if (readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_MASK) !=
		ZUMA_DPP_CORE_MASKED_MASK ||
	    readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_STATUS) ||
	    readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_CON))
		ret = -EIO;

	return ret;
}

static int
zuma_dpp0_resetless_init(struct zuma_drm *zdev, bool *profile_started)
{
	u64 deadlock;
	u32 rcv_ctrl;
	u32 rcv_num;
	u32 rdma_irq;
	int ret;

	lockdep_assert_held(&zuma_display_mmio_lock);
	if (zuma_dpp0_irq_owner_state(zdev) != ZUMA_DPP0_IRQ_INHERITED ||
	    zdev->dpp_irq_routes_enabled ||
	    readl(zuma_dpp0.base + ZUMA_DPP_RDMA_ENABLE) &
		ZUMA_DPP_RDMA_BUSY ||
	    readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_OP_STATUS))
		return -EIO;

	rdma_irq = readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ);
	if (rdma_irq & ZUMA_DPP_RDMA_DEADLOCK_IRQ) {
		atomic_or(ZUMA_DPP_RDMA_DEADLOCK_IRQ,
			  &zdev->dpp_dma_error_status);
		return -EIO;
	}
	if (zuma_dpp0_inherited_boot_profile_ready())
		return -EIO;

	rcv_ctrl = readl(zuma_dpp0.base + ZUMA_DPP_RDMA_RECOVERY_CTRL);
	if ((rcv_ctrl & ZUMA_DPP_RDMA_RECOVERY_ENABLE) ||
	    !(rcv_ctrl & ZUMA_DPP_RDMA_COUNT_MASK))
		return -EIO;
	rcv_num = (rcv_ctrl & ZUMA_DPP_RDMA_COUNT_MASK) >> 1;
	deadlock = ((u64)rcv_num * 51 << 1) |
		   ZUMA_DPP_RDMA_DEADLOCK_ENABLE;
	if (deadlock > U32_MAX ||
	    readl(zuma_dpp0.base + ZUMA_DPP_RDMA_DEADLOCK_CTRL) !=
		(u32)deadlock)
		return -EIO;

	zuma_dpp0_set_irq_owner_state(zdev, ZUMA_DPP0_IRQ_ACQUIRING);
	ret = zuma_dpp0_resetless_quiesce_irqs(zdev, true);
	if (ret)
		return ret;
	if (readl(zuma_dpp0.base + ZUMA_DPP_RDMA_ENABLE) !=
		ZUMA_DPP_RDMA_EXPECTED ||
	    readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_SWRST) ||
	    readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_OP_STATUS) ||
	    readl(zuma_decon0.base + ZUMA_DECON_SHD_REG_UP_REQ))
		return -EIO;

	*profile_started = true;
	zuma_dpp0_resetless_write_profile();
	if (zuma_dpp0_resetless_fixed_live_ready("post-write"))
		return -EIO;

	ret = zuma_dpp0_resetless_quiesce_irqs(zdev, false);
	if (ret)
		return ret;
	writel(0, zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ);
	writel(ZUMA_DPP_CORE_INHERITED_MASK,
	       zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_MASK);
	if (readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ) ||
	    readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_CON) ||
	    readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_MASK) !=
		ZUMA_DPP_CORE_INHERITED_MASK ||
	    readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_STATUS))
		return -EIO;

	zuma_dpp0_set_irq_owner_state(zdev, ZUMA_DPP0_IRQ_INHERITED);
	pr_info("zuma-display-handoff: resetless DPP0 fixed live initialization ready rcv=%u\n",
		rcv_num);
	return 0;
}

static int zuma_dpp0_resetless_fail_quiesce(struct zuma_drm *zdev)
{
	int ret;

	lockdep_assert_held(&zuma_display_mmio_lock);
	if (zdev->dpp_irq_routes_enabled) {
		disable_irq_nosync(zdev->dpp_core_irq);
		disable_irq_nosync(zdev->dpp_dma_irq);
		zdev->dpp_irq_routes_enabled = false;
		synchronize_irq(zdev->dpp_core_irq);
		synchronize_irq(zdev->dpp_dma_irq);
	}
	ret = zuma_dpp0_resetless_quiesce_irqs(zdev, false);
	zuma_dpp0_set_irq_owner_state(zdev, ZUMA_DPP0_IRQ_BROKEN);
	return ret;
}

static void
zuma_dpp0_irq_prepare_frame(struct zuma_drm *zdev,
			    struct zuma_drm_irq_proof *proof)
{
	reinit_completion(&zdev->dpp_dma_completion);
	reinit_completion(&zdev->dpp_core_completion);
	proof->dpp_dma_before = atomic64_read(&zdev->dpp_dma_irq_count);
	proof->dpp_core_before = atomic64_read(&zdev->dpp_core_irq_count);
	proof->dpp_irq_active = true;
	WRITE_ONCE(zdev->dpp_dma_status, 0);
	WRITE_ONCE(zdev->dpp_dma_control, 0);
	WRITE_ONCE(zdev->dpp_core_status, 0);
	WRITE_ONCE(zdev->dpp_core_control, 0);
	WRITE_ONCE(zdev->dpp_core_mask, 0);
	WRITE_ONCE(zdev->dpp_dma_config_error, 0);
	WRITE_ONCE(zdev->dpp_core_config_error, 0);
}

static int zuma_dpp0_irq_restore(struct zuma_drm *zdev)
{
	enum zuma_dpp0_irq_owner_state state;
	u32 core_status;
	u32 dma_enable;
	u32 dma_status;
	u32 value;
	bool restored;
	int ret = 0;

	lockdep_assert_held(&zuma_display_mmio_lock);
	state = zuma_dpp0_irq_owner_state(zdev);
	if (state == ZUMA_DPP0_IRQ_INHERITED)
		return 0;
	if (state == ZUMA_DPP0_IRQ_BROKEN)
		return -EIO;
	zuma_dpp0_set_irq_owner_state(zdev, ZUMA_DPP0_IRQ_RELEASING);

	if (zdev->dpp_irq_routes_enabled) {
		disable_irq_nosync(zdev->dpp_core_irq);
		disable_irq_nosync(zdev->dpp_dma_irq);
		zdev->dpp_irq_routes_enabled = false;
		synchronize_irq(zdev->dpp_core_irq);
		synchronize_irq(zdev->dpp_dma_irq);
	}

	value = readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ);
	dma_enable = value & ZUMA_DPP_RDMA_IRQ_ENABLE;
	dma_status = value & ZUMA_DPP_RDMA_ALL_IRQ_STATUS;
	if (value & ~(ZUMA_DPP_RDMA_ALL_IRQ_STATUS |
		      ZUMA_DPP_RDMA_CONTROL_MASK))
		ret = -EIO;
	if (dma_status & ZUMA_DPP_RDMA_CONFIG_ERR_IRQ)
		WRITE_ONCE(zdev->dpp_dma_config_error,
			   readl(zuma_dpp0.base +
				 ZUMA_DPP_RDMA_CONFIG_ERR_STATUS));
	if (dma_status) {
		atomic_or(dma_status, &zdev->dpp_dma_error_status);
		ret = -EIO;
	}
	writel(ZUMA_DPP_RDMA_MASKED_CONTROL | dma_enable | dma_status,
	       zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ);
	if (readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ) !=
	    (ZUMA_DPP_RDMA_MASKED_CONTROL | dma_enable))
		ret = -EIO;

	core_status = readl(zuma_dpp0_dpp.base +
			    ZUMA_DPP_CORE_IRQ_STATUS);
	if (core_status & ZUMA_DPP_CORE_CONFIG_ERR_IRQ)
		WRITE_ONCE(zdev->dpp_core_config_error,
			   readl(zuma_dpp0_dpp.base +
				 ZUMA_DPP_CORE_CONFIG_ERROR));
	if (core_status) {
		atomic_or(core_status, &zdev->dpp_core_error_status);
		ret = -EIO;
	}
	writel(ZUMA_DPP_CORE_MASKED_MASK,
	       zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_MASK);
	if (core_status & ZUMA_DPP_CORE_ALL_IRQ_STATUS)
		writel(core_status & ZUMA_DPP_CORE_ALL_IRQ_STATUS,
		       zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_STATUS);
	if (readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_MASK) !=
	    ZUMA_DPP_CORE_MASKED_MASK ||
	    readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_STATUS))
		ret = -EIO;

	writel(0, zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_CON);
	writel(ZUMA_DPP_RDMA_MASKED_CONTROL,
	       zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ);
	if (readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_CON) ||
	    readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ) !=
		ZUMA_DPP_RDMA_MASKED_CONTROL)
		ret = -EIO;

	value = readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ);
	dma_status = value & ZUMA_DPP_RDMA_ALL_IRQ_STATUS;
	if (dma_status & ZUMA_DPP_RDMA_CONFIG_ERR_IRQ)
		WRITE_ONCE(zdev->dpp_dma_config_error,
			   readl(zuma_dpp0.base +
				 ZUMA_DPP_RDMA_CONFIG_ERR_STATUS));
	if (dma_status) {
		atomic_or(dma_status, &zdev->dpp_dma_error_status);
		ret = -EIO;
		writel(ZUMA_DPP_RDMA_MASKED_CONTROL | dma_status,
		       zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ);
	}
	core_status = readl(zuma_dpp0_dpp.base +
			    ZUMA_DPP_CORE_IRQ_STATUS);
	if (core_status & ZUMA_DPP_CORE_CONFIG_ERR_IRQ)
		WRITE_ONCE(zdev->dpp_core_config_error,
			   readl(zuma_dpp0_dpp.base +
				 ZUMA_DPP_CORE_CONFIG_ERROR));
	if (core_status) {
		atomic_or(core_status, &zdev->dpp_core_error_status);
		ret = -EIO;
		writel(core_status & ZUMA_DPP_CORE_ALL_IRQ_STATUS,
		       zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_STATUS);
	}

	writel(0, zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ);
	writel(ZUMA_DPP_CORE_INHERITED_MASK,
	       zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_MASK);
	restored = !zuma_dpp0_irq_profile_ready(0, 0,
						ZUMA_DPP_CORE_INHERITED_MASK);
	if (!restored &&
	    zdev->replay_stage == ZUMA_DPP0_REPLAY_RESETLESS_INIT)
		restored = !zuma_dpp0_inherited_boot_profile_ready();
	zuma_dpp0_set_irq_owner_state(zdev, restored ?
				      ZUMA_DPP0_IRQ_INHERITED :
				      ZUMA_DPP0_IRQ_BROKEN);
	if (!restored)
		ret = -EIO;
	return ret;
}

static int
zuma_dpp0_irq_prepare(struct zuma_drm *zdev,
		      struct zuma_drm_irq_proof *proof,
		      bool resetless_pretrigger)
{
	u32 inherited_status;
	u32 inherited_value;

	lockdep_assert_held(&zuma_display_mmio_lock);
	if (!zuma_drm_irq_proof_is_armed(zdev) ||
	    zdev->dpp_irq_routes_enabled ||
	    zuma_dpp0_irq_owner_state(zdev) != ZUMA_DPP0_IRQ_INHERITED)
		return -EIO;
	if (resetless_pretrigger ?
	    zuma_dpp0_resetless_pretrigger_ready() :
	    zuma_dpp0_irq_inherited_ready())
		return -EIO;

	zuma_dpp0_irq_prepare_frame(zdev, proof);
	inherited_value = readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ);
	inherited_status = inherited_value & ZUMA_DPP_RDMA_ALL_IRQ_STATUS;
	if (inherited_status & ZUMA_DPP_RDMA_CONFIG_ERR_IRQ)
		WRITE_ONCE(zdev->dpp_dma_config_error,
			   readl(zuma_dpp0.base +
				 ZUMA_DPP_RDMA_CONFIG_ERR_STATUS));
	if ((inherited_status & ~ZUMA_DPP_RDMA_FRAME_DONE_IRQ) ||
	    (inherited_value & ~ZUMA_DPP_RDMA_ALL_IRQ_STATUS)) {
		WRITE_ONCE(zdev->dpp_dma_status, inherited_status);
		atomic_or(inherited_status, &zdev->dpp_dma_error_status);
		return -EIO;
	}
	zuma_dpp0_set_irq_owner_state(zdev, ZUMA_DPP0_IRQ_ACQUIRING);

	writel(ZUMA_DPP_RDMA_MASKED_CONTROL | inherited_status,
	       zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ);
	if (readl(zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ) !=
	    ZUMA_DPP_RDMA_MASKED_CONTROL)
		return -EIO;
	writel(ZUMA_DPP_CORE_MASKED_MASK,
	       zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_MASK);
	if (readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_MASK) !=
	    ZUMA_DPP_CORE_MASKED_MASK ||
	    readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_STATUS))
		return -EIO;

	writel(0, zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ);
	writel(ZUMA_DPP_CORE_OWNED_MASK,
	       zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_MASK);
	writel(ZUMA_DPP_RDMA_OWNED_CONTROL,
	       zuma_dpp0.base + ZUMA_DPP_RDMA_IRQ);
	writel(ZUMA_DPP_CORE_IRQ_ENABLE,
	       zuma_dpp0_dpp.base + ZUMA_DPP_CORE_IRQ_CON);
	if (resetless_pretrigger ?
	    zuma_dpp0_resetless_owned_pretrigger_ready() :
	    zuma_dpp0_irq_owned_ready())
		return -EIO;

	zdev->dpp_irq_routes_enabled = true;
	enable_irq(zdev->dpp_dma_irq);
	enable_irq(zdev->dpp_core_irq);
	pr_info("zuma-display-handoff: acquiring persistent DPP0 IRQs dma=%d dpp=%d rdma=%#x dpp-con=%#x dpp-mask=%#x\n",
		zdev->dpp_dma_irq, zdev->dpp_core_irq,
		(u32)ZUMA_DPP_RDMA_OWNED_CONTROL,
		(u32)ZUMA_DPP_CORE_IRQ_ENABLE,
		ZUMA_DPP_CORE_OWNED_MASK);
	return 0;
}

static void zuma_drm_dpp_irq_fault_work(struct work_struct *work)
{
	struct zuma_drm *zdev =
		container_of(work, struct zuma_drm, dpp_irq_fault_work);
	int ret;

	mutex_lock(&zuma_display_mmio_lock);
	zuma_display_set_hw_trigger(false);
	ret = zuma_dpp0_irq_restore(zdev);
	mutex_unlock(&zuma_display_mmio_lock);
	pr_err("zuma-display-handoff: DRM fault cleanup dma-status=%#x dma-errors=%#x dma-config=%#x dpp-status=%#x dpp-errors=%#x dpp-config=%#x restore=%s\n",
	       READ_ONCE(zdev->dpp_dma_status),
	       atomic_read(&zdev->dpp_dma_error_status),
	       READ_ONCE(zdev->dpp_dma_config_error),
	       READ_ONCE(zdev->dpp_core_status),
	       atomic_read(&zdev->dpp_core_error_status),
	       READ_ONCE(zdev->dpp_core_config_error), ret ? "failed" : "ok");
}

static int
zuma_dpp0_replay_prepare(struct zuma_drm *zdev, const char *operation,
			 u32 *replay_frame,
			 struct zuma_drm_irq_proof *proof,
			 bool *profile_started)
{
	enum zuma_dpp0_replay_stage stage = zdev->replay_stage;
	int ret;

	lockdep_assert_held(&zuma_display_mmio_lock);
	if (stage >= ZUMA_DPP0_REPLAY_DONE)
		return 0;

	*replay_frame = readl(zuma_decon0.base + ZUMA_DECON_FRAME_COUNT);
	if (readl(zuma_decon0.base + ZUMA_DECON_SHD_REG_UP_REQ))
		goto fail_profile;

	if (stage == ZUMA_DPP0_REPLAY_RESETLESS_INIT) {
		ret = zuma_dpp0_resetless_init(zdev, profile_started);
		if (!ret)
			ret = zuma_dpp0_irq_prepare(zdev, proof, true);
	} else {
		ret = -EIO;
	}
	if (ret)
		goto fail_profile;
	if (readl(zuma_decon0.base + ZUMA_DECON_FRAME_COUNT) != *replay_frame ||
	    readl(zuma_decon0.base + ZUMA_DECON_SHD_REG_UP_REQ))
		goto fail_profile;
	return 0;

fail_profile:
	pr_err("zuma-display-handoff: DPP0 stage %s %s pre-trigger profile failure frame=%#x poison=yes\n",
	       zuma_dpp0_replay_stage_name(stage), operation, *replay_frame);
	return -EIO;
}

static int
zuma_dpp0_replay_complete(struct zuma_drm *zdev, const char *operation,
			  u32 replay_frame, u32 frame_after,
			  const struct zuma_drm_irq_proof *proof,
			  bool release_after_frame,
			  bool profile_started)
{
	enum zuma_dpp0_replay_stage stage = zdev->replay_stage;

	lockdep_assert_held(&zuma_display_mmio_lock);
	if (stage >= ZUMA_DPP0_REPLAY_DONE)
		return 0;
	if (stage != ZUMA_DPP0_REPLAY_RESETLESS_INIT ||
	    !profile_started || frame_after != replay_frame + 1 ||
	    !proof->dpp_irq_active ||
	    proof->dpp_dma_after != proof->dpp_dma_before + 1 ||
	    proof->dpp_core_after != proof->dpp_core_before + 1 ||
	    READ_ONCE(zdev->dpp_dma_status) !=
		ZUMA_DPP_RDMA_FRAME_DONE_IRQ ||
	    READ_ONCE(zdev->dpp_core_status) !=
		ZUMA_DPP_CORE_FRAME_DONE_IRQ ||
	    READ_ONCE(zdev->dpp_dma_config_error) ||
	    READ_ONCE(zdev->dpp_core_config_error) ||
	    atomic_read(&zdev->dpp_dma_error_status) ||
	    atomic_read(&zdev->dpp_core_error_status))
		goto fail_profile;
	if (release_after_frame) {
		if (!proof->dpp_irq_released ||
		    zuma_dpp0_irq_owner_state(zdev) !=
			ZUMA_DPP0_IRQ_INHERITED ||
		    zuma_dpp0_irq_restored_ready())
			goto fail_profile;
	} else {
		if (proof->dpp_irq_released ||
		    zuma_dpp0_irq_owner_state(zdev) !=
			ZUMA_DPP0_IRQ_ACQUIRING ||
		    zuma_dpp0_irq_owned_ready())
			goto fail_profile;
		zuma_dpp0_set_irq_owner_state(zdev, ZUMA_DPP0_IRQ_OWNED);
	}

	zdev->replay_stage++;
	pr_info("zuma-display-handoff: resetless DPP0 fixed initialization and persistent IRQ takeover proven on %s frame=%#x->%#x dma=%llu->%llu status=%#x dpp=%llu->%llu status=%#x owner=%s next=%s\n",
		operation, replay_frame, frame_after,
		(unsigned long long)proof->dpp_dma_before,
		(unsigned long long)proof->dpp_dma_after,
		READ_ONCE(zdev->dpp_dma_status),
		(unsigned long long)proof->dpp_core_before,
		(unsigned long long)proof->dpp_core_after,
		READ_ONCE(zdev->dpp_core_status),
		release_after_frame ? "released" : "active",
		zuma_dpp0_replay_stage_name(zdev->replay_stage));
	return 0;

fail_profile:
	pr_err("zuma-display-handoff: DPP0 stage %s %s post-frame profile failure frame=%#x->%#x poison=yes\n",
	       zuma_dpp0_replay_stage_name(stage), operation, replay_frame,
	       frame_after);
	return -EIO;
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
	zuma_dpp0_preflight_snapshot(label);
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
	zuma_boot_buffer = pixels;

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

	if (zuma_drm_device &&
	    (!zuma_drm_irq_proof_is_armed(zuma_drm_device) ||
	     READ_ONCE(zuma_drm_update_failed)))
		return -EIO;
	*frame_before = readl(zuma_decon0.base + ZUMA_DECON_FRAME_COUNT);
	*frame_after = *frame_before;
	zuma_display_set_hw_trigger(true);
	ret = readl_poll_timeout_atomic(zuma_decon0.base +
			ZUMA_DECON_FRAME_COUNT, *frame_after,
			*frame_after != *frame_before, 1, 50000);
	zuma_display_set_hw_trigger(false);
	if (!ret && *frame_after != *frame_before + 1)
		ret = -EIO;
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

static bool zuma_display_update_ready_for_ctrl(u32 ctrl, u32 shadow_ctrl,
					       u32 int_en)
{
	u32 dpub, dpuf0, dpuf1;

	if (!zuma_display_domains_on(&dpub, &dpuf0, &dpuf1))
		return false;

	return readl(zuma_decon0.base + ZUMA_DECON_GLOBAL_CON) ==
			ZUMA_DECON_GLOBAL_EXPECTED &&
	       readl(zuma_decon0.base + ZUMA_DECON_TRIG_CON) ==
			ZUMA_DECON_TRIG_EXPECTED &&
	       !readl(zuma_decon0.base + ZUMA_DECON_SHD_REG_UP_REQ) &&
	       readl(zuma_decon0.base + ZUMA_DECON_INT_EN) == int_en &&
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
						  zuma_framebuffer_ctrl,
						  ZUMA_DECON_INT_QUIESCENT);
}

static bool zuma_dpp0_resetless_preflight_ready(struct zuma_drm *zdev)
{
	lockdep_assert_held(&zuma_display_mmio_lock);
	if (!zdev)
		return false;
	if (zdev->replay_stage != ZUMA_DPP0_REPLAY_RESETLESS_INIT)
		return true;

	return zuma_dpp0_irq_owner_state(zdev) == ZUMA_DPP0_IRQ_INHERITED &&
	       !zdev->dpp_irq_routes_enabled &&
	       !zuma_dpp0_inherited_boot_profile_ready() &&
	       !(readl(zuma_dpp0.base + ZUMA_DPP_RDMA_ENABLE) &
		 ZUMA_DPP_RDMA_BUSY) &&
	       !readl(zuma_dpp0_dpp.base + ZUMA_DPP_CORE_OP_STATUS);
}

static bool zuma_drm_scanout_update_ready(void)
{
	return zuma_boot_buffer && zuma_scanout_buffer &&
	       zuma_framebuffer_phys == ZUMA_SCANOUT_FB_BASE &&
	       !READ_ONCE(zuma_drm_update_failed) &&
	       zuma_dpp0_resetless_preflight_ready(zuma_drm_device) &&
	       (zuma_display_update_ready() ||
		zuma_display_update_ready_for_ctrl(zuma_framebuffer_ctrl,
						   zuma_framebuffer_ctrl,
						   ZUMA_DECON_INT_ACTIVE));
}

static int zuma_drm_clear_frame_pending(bool allow_stale)
{
	u32 pending;

	pending = readl(zuma_decon0.base + ZUMA_DECON_INT_PEND) &
		ZUMA_DECON_INT_FRAME_MASK;
	if (pending)
		writel(pending, zuma_decon0.base + ZUMA_DECON_INT_PEND);
	if (readl(zuma_decon0.base + ZUMA_DECON_INT_PEND) &
	    ZUMA_DECON_INT_FRAME_MASK)
		return -EIO;
	if (pending && !allow_stale)
		return -EIO;
	return 0;
}

static int zuma_drm_enable_irq_routes(struct zuma_drm *zdev)
{
	if (zdev->irq_routes_enabled ||
	    readl(zuma_decon0.base + ZUMA_DECON_INT_EN) !=
		ZUMA_DECON_INT_QUIESCENT)
		return -EIO;
	if (zuma_drm_clear_frame_pending(true))
		return -EIO;

	zdev->irq_routes_enabled = true;
	enable_irq(zdev->frame_start_irq);
	enable_irq(zdev->frame_done_irq);
	pr_info("zuma-display-handoff: enabled DECON frame IRQ routes with master gated by DRM vblank refs\n");
	return 0;
}

static int zuma_drm_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct zuma_drm *zdev = container_of(crtc, struct zuma_drm, crtc);
	u32 int_en;

	if (!zdev->irq_routes_enabled || READ_ONCE(zuma_drm_update_failed))
		return -EIO;

	int_en = readl(zuma_decon0.base + ZUMA_DECON_INT_EN);
	if (int_en == ZUMA_DECON_INT_ACTIVE)
		return 0;
	if (int_en != ZUMA_DECON_INT_QUIESCENT ||
	    zuma_drm_clear_frame_pending(true))
		return -EIO;

	writel(ZUMA_DECON_INT_ACTIVE,
	       zuma_decon0.base + ZUMA_DECON_INT_EN);
	return readl(zuma_decon0.base + ZUMA_DECON_INT_EN) ==
		ZUMA_DECON_INT_ACTIVE ? 0 : -EIO;
}

static void zuma_drm_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct zuma_drm *zdev = container_of(crtc, struct zuma_drm, crtc);
	u32 int_en;

	if (!zdev->irq_routes_enabled)
		return;

	int_en = readl(zuma_decon0.base + ZUMA_DECON_INT_EN);
	if (int_en == ZUMA_DECON_INT_QUIESCENT)
		return;
	if (int_en != ZUMA_DECON_INT_ACTIVE)
		goto fail;

	writel(ZUMA_DECON_INT_QUIESCENT,
	       zuma_decon0.base + ZUMA_DECON_INT_EN);
	if (readl(zuma_decon0.base + ZUMA_DECON_INT_EN) ==
	    ZUMA_DECON_INT_QUIESCENT)
		return;

fail:
	atomic_set(&zdev->irq_error, 1);
	WRITE_ONCE(zuma_drm_update_failed, true);
	queue_work(zdev->workqueue, &zdev->dpp_irq_fault_work);
	pr_err("zuma-display-handoff: failed to gate DECON frame interrupts\n");
}

static int zuma_drm_quiesce_irq_proof(struct zuma_drm *zdev,
				      bool allow_pending)
{
	zuma_drm_set_irq_proof_armed(zdev, false);
	if (zuma_dpp0_irq_handler_live(zdev) &&
	    zdev->dpp_irq_routes_enabled) {
		synchronize_irq(zdev->dpp_core_irq);
		synchronize_irq(zdev->dpp_dma_irq);
	}
	synchronize_irq(zdev->frame_start_irq);
	synchronize_irq(zdev->frame_done_irq);
	return zuma_drm_clear_frame_pending(allow_pending);
}

static int zuma_drm_prepare_irq_proof(struct zuma_drm *zdev,
				      struct zuma_drm_irq_proof *proof)
{
	enum zuma_dpp0_irq_owner_state owner_state;
	u32 int_en;
	int ret;

	int_en = readl(zuma_decon0.base + ZUMA_DECON_INT_EN);
	if (!zdev->irq_routes_enabled ||
	    (int_en != ZUMA_DECON_INT_QUIESCENT &&
	     int_en != ZUMA_DECON_INT_ACTIVE))
		return -EIO;

	ret = zuma_drm_quiesce_irq_proof(zdev, !zdev->irq_proven_once);
	if (ret || READ_ONCE(zuma_drm_update_failed))
		return ret ? ret : -EIO;

	reinit_completion(&zdev->frame_start_completion);
	reinit_completion(&zdev->frame_done_completion);
	proof->frame_start_before =
		atomic64_read(&zdev->frame_start_irq_count);
	proof->frame_done_before = atomic64_read(&zdev->frame_done_irq_count);
	proof->vblank_before = drm_crtc_vblank_count(&zdev->crtc);
	atomic_set(&zdev->irq_error, 0);

	owner_state = zuma_dpp0_irq_owner_state(zdev);
	if (owner_state == ZUMA_DPP0_IRQ_OWNED) {
		if (!zdev->dpp_irq_routes_enabled || zuma_dpp0_irq_owned_ready())
			return -EIO;
		zuma_dpp0_irq_prepare_frame(zdev, proof);
	} else if (owner_state != ZUMA_DPP0_IRQ_INHERITED) {
		return -EIO;
	}

	zuma_drm_set_irq_proof_armed(zdev, true);
	if (READ_ONCE(zuma_drm_update_failed)) {
		zuma_drm_quiesce_irq_proof(zdev, true);
		return -EIO;
	}
	return 0;
}

static int zuma_drm_wait_frame_irqs(struct zuma_drm *zdev,
				    struct zuma_drm_irq_proof *proof,
				    bool release_after_frame)
{
	enum zuma_dpp0_irq_owner_state owner_state;
	unsigned long deadline;
	unsigned long timeout;
	int cleanup_ret = 0;
	int close_ret;
	int ret = 0;

	deadline = jiffies + msecs_to_jiffies(ZUMA_DECON_IRQ_TIMEOUT_MS);
	timeout = time_before(jiffies, deadline) ? deadline - jiffies : 1;
	if (!wait_for_completion_timeout(&zdev->frame_start_completion,
					 timeout))
		ret = -ETIMEDOUT;
	if (!ret) {
		timeout = time_before(jiffies, deadline) ?
			deadline - jiffies : 1;
		if (!wait_for_completion_timeout(&zdev->frame_done_completion,
						 timeout))
			ret = -ETIMEDOUT;
	}
	if (!ret && proof->dpp_irq_active) {
		timeout = time_before(jiffies, deadline) ?
			deadline - jiffies : 1;
		if (!wait_for_completion_timeout(&zdev->dpp_dma_completion,
						 timeout))
			ret = -ETIMEDOUT;
	}
	if (!ret && proof->dpp_irq_active) {
		timeout = time_before(jiffies, deadline) ?
			deadline - jiffies : 1;
		if (!wait_for_completion_timeout(&zdev->dpp_core_completion,
						 timeout))
			ret = -ETIMEDOUT;
	}

	if (release_after_frame && proof->dpp_irq_active) {
		cleanup_ret = zuma_dpp0_irq_restore(zdev);
		proof->dpp_irq_released =
			zuma_dpp0_irq_owner_state(zdev) ==
			ZUMA_DPP0_IRQ_INHERITED;
	}
	close_ret = zuma_drm_quiesce_irq_proof(zdev, false);
	proof->frame_start_after =
		atomic64_read(&zdev->frame_start_irq_count);
	proof->frame_done_after = atomic64_read(&zdev->frame_done_irq_count);
	proof->vblank_after = drm_crtc_vblank_count(&zdev->crtc);
	proof->dpp_dma_after = atomic64_read(&zdev->dpp_dma_irq_count);
	proof->dpp_core_after = atomic64_read(&zdev->dpp_core_irq_count);
	if (!ret && cleanup_ret)
		ret = cleanup_ret;
	if (!ret && close_ret)
		ret = close_ret;
	if (!ret && (atomic_read(&zdev->irq_error) ||
		     READ_ONCE(zuma_drm_update_failed)))
		ret = -EIO;
	if (!ret &&
	    (proof->frame_start_after != proof->frame_start_before + 1 ||
	     proof->frame_done_after != proof->frame_done_before + 1 ||
	     proof->vblank_after != proof->vblank_before + 1))
		ret = -EIO;
	if (!ret && proof->dpp_irq_active &&
	    (proof->dpp_dma_after != proof->dpp_dma_before + 1 ||
	     proof->dpp_core_after != proof->dpp_core_before + 1 ||
	     READ_ONCE(zdev->dpp_dma_status) !=
			ZUMA_DPP_RDMA_FRAME_DONE_IRQ ||
	     READ_ONCE(zdev->dpp_core_status) !=
			ZUMA_DPP_CORE_FRAME_DONE_IRQ ||
	     READ_ONCE(zdev->dpp_dma_config_error) ||
	     READ_ONCE(zdev->dpp_core_config_error) ||
	     atomic_read(&zdev->dpp_dma_error_status) ||
	     atomic_read(&zdev->dpp_core_error_status)))
		ret = -EIO;

	owner_state = zuma_dpp0_irq_owner_state(zdev);
	if (!ret && proof->dpp_irq_active) {
		if (release_after_frame) {
			if (!proof->dpp_irq_released ||
			    owner_state != ZUMA_DPP0_IRQ_INHERITED ||
			    zuma_dpp0_irq_restored_ready())
				ret = -EIO;
		} else if ((owner_state != ZUMA_DPP0_IRQ_ACQUIRING &&
			    owner_state != ZUMA_DPP0_IRQ_OWNED) ||
			   zuma_dpp0_irq_owned_ready()) {
			ret = -EIO;
		}
	}
	if (!ret && readl(zuma_decon0.base + ZUMA_DECON_INT_EN) !=
	    ZUMA_DECON_INT_ACTIVE)
		ret = -EIO;
	if (!ret)
		zdev->irq_proven_once = true;
	return ret;
}

static int zuma_drm_finish_scanout_update(struct zuma_drm *zdev,
					  const char *operation,
					  bool release_after_frame)
{
	struct zuma_drm_irq_proof proof = {};
	enum zuma_dpp0_replay_stage replay_stage = zdev->replay_stage;
	u32 frame_before = 0, frame_after = 0;
	u32 replay_frame = 0;
	bool replay_pending = replay_stage < ZUMA_DPP0_REPLAY_DONE;
	bool replay_started = false;
	bool lifecycle_started = false;
	bool profile_started = false;
	bool trigger_issued = false;
	bool vblank_ref = false;
	int cleanup_ret;
	int restore_ret;
	int ret;

	arch_sync_dma_for_device(zuma_framebuffer_phys, ZUMA_HANDOFF_FB_SIZE,
				 DMA_TO_DEVICE);
	arch_sync_dma_flush();

	ret = zuma_drm_prepare_irq_proof(zdev, &proof);
	if (ret)
		goto out_abort;
	ret = drm_crtc_vblank_get(&zdev->crtc);
	if (ret)
		goto out_abort;
	vblank_ref = true;
	if (readl(zuma_decon0.base + ZUMA_DECON_INT_EN) !=
	    ZUMA_DECON_INT_ACTIVE) {
		ret = -EIO;
		goto out_abort;
	}

	if (replay_pending) {
		replay_started = true;
		ret = zuma_dpp0_replay_prepare(zdev, operation, &replay_frame,
					       &proof, &profile_started);
		if (ret)
			goto out_abort;
	} else if (zuma_dpp0_irq_owner_state(zdev) ==
		   ZUMA_DPP0_IRQ_INHERITED) {
		lifecycle_started = true;
		ret = zuma_dpp0_irq_prepare(zdev, &proof, false);
		if (ret)
			goto out_abort;
	}
	if (READ_ONCE(zuma_drm_update_failed)) {
		ret = -EIO;
		goto out_abort;
	}
	ret = zuma_display_request_active_window();
	if (ret)
		goto out_abort;
	trigger_issued = true;
	ret = zuma_display_trigger_frame(&frame_before, &frame_after);
	if (ret)
		goto out_abort;
	ret = zuma_drm_wait_frame_irqs(zdev, &proof, release_after_frame);
	if (ret)
		goto out_abort;
	if (!zuma_display_update_ready_for_ctrl(zuma_framebuffer_ctrl,
						zuma_framebuffer_ctrl,
						ZUMA_DECON_INT_ACTIVE)) {
		ret = -EIO;
		goto out_abort;
	}
	if (replay_pending) {
		if (frame_before != replay_frame) {
			ret = -EIO;
			goto out_abort;
		}
		ret = zuma_dpp0_replay_complete(zdev, operation, replay_frame,
						frame_after, &proof,
						release_after_frame,
						profile_started);
		if (ret)
			goto out_abort;
	} else if (lifecycle_started && !release_after_frame) {
		if (zuma_dpp0_irq_owner_state(zdev) !=
		    ZUMA_DPP0_IRQ_ACQUIRING) {
			ret = -EIO;
			goto out_abort;
		}
		zuma_dpp0_set_irq_owner_state(zdev, ZUMA_DPP0_IRQ_OWNED);
		pr_info("zuma-display-handoff: persistent DPP0 IRQ ownership reacquired on %s\n",
			operation);
	}

	zuma_drm_update_count++;
	if (zuma_drm_update_count <= 8) {
		pr_info("zuma-display-handoff: DRM %s %llu bytes=0..%zu frame=%#x->%#x irq-start=%llu->%llu irq-done=%llu->%llu vblank=%llu->%llu dpp-dma=%llu->%llu dpp=%llu->%llu owner=%u\n",
			operation, (unsigned long long)zuma_drm_update_count,
			(size_t)ZUMA_HANDOFF_FB_SIZE - 1,
			frame_before, frame_after,
			(unsigned long long)proof.frame_start_before,
			(unsigned long long)proof.frame_start_after,
			(unsigned long long)proof.frame_done_before,
			(unsigned long long)proof.frame_done_after,
			(unsigned long long)proof.vblank_before,
			(unsigned long long)proof.vblank_after,
			(unsigned long long)proof.dpp_dma_before,
			(unsigned long long)proof.dpp_dma_after,
			(unsigned long long)proof.dpp_core_before,
			(unsigned long long)proof.dpp_core_after,
			zuma_dpp0_irq_owner_state(zdev));
		zuma_dpp0_preflight_snapshot(operation);
	}

	/* The post-swap tail owns this vblank reference after success. */
	return 0;

out_abort:
	zuma_display_set_hw_trigger(false);
	cleanup_ret = zuma_drm_quiesce_irq_proof(zdev, true);
	if (profile_started)
		restore_ret = zuma_dpp0_resetless_fail_quiesce(zdev);
	else
		restore_ret = zuma_dpp0_irq_restore(zdev);
	if (!ret)
		ret = cleanup_ret ? cleanup_ret : restore_ret;
	if (vblank_ref)
		drm_crtc_vblank_put(&zdev->crtc);
	synchronize_irq(zdev->frame_start_irq);
	synchronize_irq(zdev->frame_done_irq);
	if (zuma_drm_clear_frame_pending(true) && !ret)
		ret = -EIO;
	WRITE_ONCE(zuma_drm_update_failed, true);
	if (replay_started || lifecycle_started)
		pr_err("zuma-display-handoff: DPP0 stage %s failed stage-unchanged profile-started=%s trigger-issued=%s outcome=%s owner=%u restore=%s poison=yes\n",
		       zuma_dpp0_replay_stage_name(replay_stage),
		       profile_started ? "yes" : "no",
		       trigger_issued ? "yes" : "no",
		       profile_started ? "fixed-state-uncertain" :
		       trigger_issued ? "frame-uncertain" :
		       "visible-frame-preserved",
		       zuma_dpp0_irq_owner_state(zdev),
		       restore_ret ? "failed" : "ok");
	pr_err("zuma-display-handoff: DRM %s failed closed: %d, frame=%#x irq-start=%llu->%llu irq-done=%llu->%llu vblank=%llu->%llu dpp-dma=%llu->%llu dpp=%llu->%llu\n",
	       operation, ret, frame_after,
	       (unsigned long long)proof.frame_start_before,
	       (unsigned long long)proof.frame_start_after,
	       (unsigned long long)proof.frame_done_before,
	       (unsigned long long)proof.frame_done_after,
	       (unsigned long long)proof.vblank_before,
	       (unsigned long long)proof.vblank_after,
	       (unsigned long long)proof.dpp_dma_before,
	       (unsigned long long)proof.dpp_dma_after,
	       (unsigned long long)proof.dpp_core_before,
	       (unsigned long long)proof.dpp_core_after);
	return ret;
}

static int zuma_drm_commit_shadow(struct drm_atomic_commit *state,
				  bool *fail_closed)
{
	struct zuma_drm *zdev = container_of(state->dev, struct zuma_drm, drm);
	struct drm_shadow_plane_state *shadow_plane_state;
	struct drm_plane_state *new_plane_state;
	struct drm_crtc_state *new_crtc_state;
	struct drm_framebuffer *fb;
	int ret;

	if (fail_closed)
		*fail_closed = false;
	new_plane_state =
		drm_atomic_get_new_plane_state(state, &zdev->primary_plane);
	if (!new_plane_state)
		return 0;

	if (!new_plane_state->crtc || !new_plane_state->fb)
		return 0;

	new_crtc_state =
		drm_atomic_get_new_crtc_state(state, new_plane_state->crtc);
	if (!new_crtc_state || !new_crtc_state->active)
		return 0;

	fb = new_plane_state->fb;
	shadow_plane_state = to_drm_shadow_plane_state(new_plane_state);
	ret = drm_gem_fb_begin_cpu_access(fb, DMA_FROM_DEVICE);
	if (ret)
		return ret;

	mutex_lock(&zuma_display_mmio_lock);
	if (!zuma_drm_scanout_update_ready()) {
		if (fail_closed)
			*fail_closed = true;
		ret = -EIO;
		goto out_unlock;
	}

	if (fail_closed)
		*fail_closed = true;
	iosys_map_memcpy_from(zuma_scanout_buffer,
			      &shadow_plane_state->data[0], 0,
			      ZUMA_HANDOFF_FB_SIZE);
	ret = zuma_drm_finish_scanout_update(zdev, "shadow update", false);

out_unlock:
	mutex_unlock(&zuma_display_mmio_lock);
	drm_gem_fb_end_cpu_access(fb, DMA_FROM_DEVICE);
	return ret;
}

static int zuma_drm_restore_boot_buffer(struct zuma_drm *zdev)
{
	int ret;

	mutex_lock(&zuma_display_mmio_lock);
	if (!zuma_drm_scanout_update_ready()) {
		ret = -EIO;
		goto out_unlock;
	}

	memcpy(zuma_scanout_buffer, zuma_boot_buffer,
	       ZUMA_HANDOFF_FB_SIZE);
	if (memcmp(zuma_scanout_buffer, zuma_boot_buffer,
		   ZUMA_HANDOFF_FB_SIZE)) {
		ret = -EIO;
		WRITE_ONCE(zuma_drm_update_failed, true);
		zuma_display_set_hw_trigger(false);
		if (zuma_dpp0_irq_restore(zdev))
			pr_err("zuma-display-handoff: DPP0 IRQ restore failed after boot copy failure\n");
		pr_err("zuma-display-handoff: DRM boot restore copy failed closed\n");
		goto out_unlock;
	}

	ret = zuma_drm_finish_scanout_update(zdev, "boot restore", true);

out_unlock:
	mutex_unlock(&zuma_display_mmio_lock);
	return ret;
}

static int zuma_drm_reboot_notifier(struct notifier_block *notifier,
				    unsigned long action, void *data)
{
	struct zuma_drm *zdev =
		container_of(notifier, struct zuma_drm, reboot_notifier);
	u32 int_en;
	int cleanup_ret;
	int ret = 0;

	mutex_lock(&zdev->commit_admission_lock);
	WRITE_ONCE(zuma_drm_update_failed, true);
	mutex_unlock(&zdev->commit_admission_lock);
	flush_workqueue(zdev->workqueue);

	mutex_lock(&zuma_display_mmio_lock);
	zuma_display_set_hw_trigger(false);
	ret = zuma_dpp0_irq_restore(zdev);
	int_en = readl(zuma_decon0.base + ZUMA_DECON_INT_EN);
	if (int_en == ZUMA_DECON_INT_ACTIVE) {
		writel(ZUMA_DECON_INT_QUIESCENT,
		       zuma_decon0.base + ZUMA_DECON_INT_EN);
		if (readl(zuma_decon0.base + ZUMA_DECON_INT_EN) !=
		    ZUMA_DECON_INT_QUIESCENT)
			ret = -EIO;
	} else if (int_en != ZUMA_DECON_INT_QUIESCENT) {
		ret = -EIO;
	}
	zuma_drm_set_irq_proof_armed(zdev, false);
	cleanup_ret = zuma_drm_clear_frame_pending(true);
	if (!ret)
		ret = cleanup_ret;
	if (zdev->irq_routes_enabled) {
		disable_irq_nosync(zdev->frame_done_irq);
		disable_irq_nosync(zdev->frame_start_irq);
		zdev->irq_routes_enabled = false;
		synchronize_irq(zdev->frame_done_irq);
		synchronize_irq(zdev->frame_start_irq);
	}
	mutex_unlock(&zuma_display_mmio_lock);

	cancel_work_sync(&zdev->dpp_irq_fault_work);
	flush_workqueue(zdev->workqueue);
	pr_info("zuma-display-handoff: reboot IRQ lifecycle shutdown restore=%s\n",
		ret ? "failed" : "ok");
	return NOTIFY_DONE;
}

static int zuma_drm_complete_vblank_event(struct drm_atomic_commit *state)
{
	struct zuma_drm *zdev = container_of(state->dev, struct zuma_drm, drm);
	struct drm_crtc_state *new_crtc_state;
	struct drm_pending_vblank_event *event;
	unsigned long flags;
	u64 sequence;
	u32 int_en;
	bool failed = false;

	new_crtc_state = drm_atomic_get_new_crtc_state(state, &zdev->crtc);
	if (WARN_ON(!new_crtc_state))
		failed = true;

	sequence = drm_crtc_vblank_count(&zdev->crtc);
	drm_crtc_vblank_put(&zdev->crtc);
	synchronize_irq(zdev->frame_start_irq);
	synchronize_irq(zdev->frame_done_irq);
	if (zuma_drm_clear_frame_pending(false))
		failed = true;
	int_en = readl(zuma_decon0.base + ZUMA_DECON_INT_EN);
	if (int_en != ZUMA_DECON_INT_QUIESCENT &&
	    int_en != ZUMA_DECON_INT_ACTIVE)
		failed = true;
	if (READ_ONCE(zuma_drm_update_failed))
		failed = true;
	if (failed)
		goto out_failed;

	spin_lock_irqsave(&state->dev->event_lock, flags);
	event = new_crtc_state->event;
	if (event) {
		drm_crtc_send_vblank_event(&zdev->crtc, event);
		new_crtc_state->event = NULL;
	}
	spin_unlock_irqrestore(&state->dev->event_lock, flags);

	if (zuma_drm_update_count <= 8)
		pr_info("zuma-display-handoff: DRM real vblank %s %llu sequence=%llu int-en=%#x\n",
			event ? "event" : "no-event completion",
			(unsigned long long)zuma_drm_update_count,
			(unsigned long long)sequence, int_en);
	return 0;

out_failed:
	WRITE_ONCE(zuma_drm_update_failed, true);
	queue_work(zdev->workqueue, &zdev->dpp_irq_fault_work);
	pr_err("zuma-display-handoff: DRM vblank event completion failed closed\n");
	return -EIO;
}

static void zuma_drm_cancel_vblank_event(struct drm_atomic_commit *state,
					 int error)
{
	struct zuma_drm *zdev = container_of(state->dev, struct zuma_drm, drm);
	struct drm_crtc_state *new_crtc_state;
	struct drm_pending_vblank_event *event = NULL;
	struct completion *completion;
	void (*completion_release)(struct completion *completion);
	unsigned long flags;

	if (WARN_ON(error >= 0))
		error = -EIO;
	new_crtc_state = drm_atomic_get_new_crtc_state(state, &zdev->crtc);
	spin_lock_irqsave(&state->dev->event_lock, flags);
	if (new_crtc_state) {
		event = new_crtc_state->event;
		new_crtc_state->event = NULL;
	}
	if (event && event->base.fence) {
		dma_fence_set_error(event->base.fence, error);
		dma_fence_signal(event->base.fence);
	}
	if (event && event->base.completion) {
		completion = event->base.completion;
		completion_release = event->base.completion_release;
		event->base.completion = NULL;
		event->base.completion_release = NULL;
		complete_all(completion);
		if (completion_release)
			completion_release(completion);
	}
	spin_unlock_irqrestore(&state->dev->event_lock, flags);

	if (event)
		drm_event_cancel_free(state->dev, &event->base);
}

static void zuma_drm_commit_tail(struct drm_atomic_commit *state,
				 int scanout_ret)
{
	struct drm_device *drm = state->dev;

	drm_atomic_helper_commit_modeset_disables(drm, state);
	drm_atomic_helper_commit_planes(drm, state, 0);
	drm_atomic_helper_commit_modeset_enables(drm, state);
	if (!scanout_ret)
		scanout_ret = zuma_drm_complete_vblank_event(state);
	if (scanout_ret)
		zuma_drm_cancel_vblank_event(state, scanout_ret);
	drm_atomic_helper_commit_hw_done(state);
	drm_atomic_helper_wait_for_flip_done(drm, state);
	drm_atomic_helper_cleanup_planes(drm, state);
	drm_atomic_helper_commit_cleanup_done(state);
}

static void zuma_drm_commit_work(struct work_struct *work)
{
	struct drm_atomic_commit *state =
		container_of(work, struct drm_atomic_commit, commit_work);
	struct zuma_drm *zdev =
		container_of(state->dev, struct zuma_drm, drm);
	bool fail_closed = false;
	int ret;

	ret = drm_atomic_helper_wait_for_fences(state->dev, state, false);
	drm_atomic_helper_wait_for_dependencies(state);
	if (!ret)
		ret = zuma_drm_commit_shadow(state, &fail_closed);
	if (ret && (fail_closed || READ_ONCE(zuma_drm_update_failed))) {
		WRITE_ONCE(zuma_drm_update_failed, true);
		mutex_lock(&zuma_display_mmio_lock);
		zuma_display_set_hw_trigger(false);
		if (zuma_dpp0_irq_restore(zdev))
			pr_err("zuma-display-handoff: DPP0 IRQ restore failed after asynchronous update failure\n");
		mutex_unlock(&zuma_display_mmio_lock);
		pr_err("zuma-display-handoff: asynchronous DRM update failed closed: %d; "
		       "flip event cancelled\n", ret);
	} else if (ret) {
		pr_err("zuma-display-handoff: asynchronous DRM update cancelled before framebuffer access: %d; flip event cancelled\n",
		       ret);
	}

	zuma_drm_commit_tail(state, ret);
	drm_atomic_commit_put(state);
}

static int zuma_drm_atomic_commit(struct drm_device *drm,
				  struct drm_atomic_commit *state,
				  bool nonblock)
{
	struct zuma_drm *zdev = container_of(drm, struct zuma_drm, drm);
	struct drm_plane_state *old_plane_state;
	struct drm_plane_state *new_plane_state;
	struct drm_crtc_state *new_crtc_state;
	bool restore_boot;
	bool update_scanout;
	int ret;

	old_plane_state =
		drm_atomic_get_old_plane_state(state, &zdev->primary_plane);
	new_plane_state =
		drm_atomic_get_new_plane_state(state, &zdev->primary_plane);
	restore_boot = old_plane_state && old_plane_state->fb &&
		new_plane_state && !new_plane_state->fb;
	update_scanout = restore_boot ||
		(new_plane_state && new_plane_state->crtc &&
		 new_plane_state->fb);
	if (!update_scanout)
		return -EOPNOTSUPP;
	if (READ_ONCE(zuma_drm_update_failed))
		return -EIO;

	new_crtc_state =
		drm_atomic_get_new_crtc_state(state, &zdev->crtc);
	if (nonblock &&
	    (!old_plane_state || !old_plane_state->crtc ||
	     !old_plane_state->fb || restore_boot || !new_plane_state ||
	     !new_plane_state->crtc || !new_plane_state->fb ||
	     !new_crtc_state || drm_atomic_crtc_needs_modeset(new_crtc_state)))
		return -EOPNOTSUPP;

	ret = drm_atomic_helper_setup_commit(state, nonblock);
	if (ret)
		return ret;

	if (nonblock)
		INIT_WORK(&state->commit_work, zuma_drm_commit_work);

	ret = drm_atomic_helper_prepare_planes(drm, state);
	if (ret)
		return ret;

	if (nonblock) {
		mutex_lock(&zdev->commit_admission_lock);
		if (READ_ONCE(zuma_drm_update_failed)) {
			ret = -EIO;
		} else {
			ret = drm_atomic_helper_swap_state(state, true);
			if (!ret) {
				drm_atomic_commit_get(state);
				WARN_ON(!queue_work(zdev->workqueue,
						    &state->commit_work));
			}
		}
		mutex_unlock(&zdev->commit_admission_lock);
		if (ret)
			goto out_unprepare;
		return 0;
	}

	ret = drm_atomic_helper_wait_for_fences(drm, state, true);
	if (ret)
		goto out_unprepare;
	drm_atomic_helper_wait_for_dependencies(state);

	if (restore_boot)
		ret = zuma_drm_restore_boot_buffer(zdev);
	else
		ret = zuma_drm_commit_shadow(state, NULL);
	if (ret)
		goto out_unprepare;

	ret = drm_atomic_helper_swap_state(state, false);
	if (WARN_ON(ret)) {
		drm_crtc_vblank_put(&zdev->crtc);
		mutex_lock(&zuma_display_mmio_lock);
		WRITE_ONCE(zuma_drm_update_failed, true);
		zuma_display_set_hw_trigger(false);
		if (zuma_dpp0_irq_restore(zdev))
			pr_err("zuma-display-handoff: DPP0 IRQ restore failed after state-swap failure\n");
		mutex_unlock(&zuma_display_mmio_lock);
		goto out_unprepare;
	}

	drm_atomic_commit_get(state);
	zuma_drm_commit_tail(state, 0);
	drm_atomic_commit_put(state);
	return 0;

out_unprepare:
	drm_atomic_helper_unprepare_planes(drm, state);
	return ret;
}

static bool __init zuma_flip_buffer_valid(void)
{
	phys_addr_t base = zuma_husky_flip_framebuffer_base;
	unsigned long pfn, end_pfn;

	if (base != ZUMA_SCANOUT_FB_BASE) {
		pr_err("zuma-display-handoff: refusing unexpected flip framebuffer base %pa\n",
		       &base);
		return false;
	}

	if (region_intersects(base, ZUMA_HANDOFF_FB_SIZE,
			      IORESOURCE_SYSTEM_RAM, IORES_DESC_NONE) !=
	    REGION_INTERSECTS) {
		pr_err("zuma-display-handoff: flip framebuffer %pa+%#zx is not System RAM\n",
		       &base, (size_t)ZUMA_HANDOFF_FB_SIZE);
		return false;
	}

	pfn = PHYS_PFN(base);
	end_pfn = PHYS_PFN(base + ZUMA_HANDOFF_FB_SIZE);
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

	if (!zuma_display_update_ready_for_ctrl(ctrl, shadow_ctrl,
						ZUMA_DECON_INT_QUIESCENT)) {
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
							ZUMA_HANDOFF_FB_CTRL,
							ZUMA_DECON_INT_QUIESCENT))
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
	const void *source = zuma_boot_buffer;
	void *destination;
	u32 frame_before = 0, frame_after = 0;
	u32 rollback_before = 0, rollback_after = 0;
	int rollback_ret;
	int ret;

	if (!zuma_display_update_ready()) {
		pr_err("zuma-display-handoff: base flip refused by inherited state\n");
		return false;
	}

	if (!source || !zuma_flip_buffer_valid())
		return false;

	destination = memremap(new_base, ZUMA_HANDOFF_FB_SIZE, MEMREMAP_WB);
	if (!destination) {
		pr_err("zuma-display-handoff: base flip destination mapping failed\n");
		return false;
	}

	memcpy(destination, source, ZUMA_HANDOFF_FB_SIZE);
	if (memcmp(destination, source, ZUMA_HANDOFF_FB_SIZE)) {
		pr_err("zuma-display-handoff: base flip copy verification failed\n");
		memunmap(destination);
		return false;
	}
	arch_sync_dma_for_device(new_base, ZUMA_HANDOFF_FB_SIZE, DMA_TO_DEVICE);
	arch_sync_dma_flush();

	ret = zuma_display_commit_base(new_base, &frame_before, &frame_after);
	if (!ret) {
		zuma_framebuffer_phys = new_base;
		if (zuma_display_update_ready()) {
			zuma_scanout_buffer = destination;
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
	memunmap(destination);
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

static void zuma_display_format_workfn(struct work_struct *work)
{
	bool publish_drm;

	mutex_lock(&zuma_display_mmio_lock);
	if (zuma_framebuffer_validated)
		zuma_framebuffer_validated = zuma_display_switch_to_bgrx();
	if (zuma_framebuffer_validated &&
	    (!zuma_boot_buffer || !zuma_scanout_buffer ||
	     zuma_framebuffer_phys != ZUMA_SCANOUT_FB_BASE)) {
		pr_err("zuma-display-handoff: DRM scanout mappings failed final validation\n");
		zuma_framebuffer_validated = false;
	} else if (zuma_framebuffer_validated &&
		   memcmp(zuma_scanout_buffer, zuma_boot_buffer,
			  ZUMA_HANDOFF_FB_SIZE)) {
		pr_err("zuma-display-handoff: DRM scanout differs from immutable boot buffer\n");
		zuma_framebuffer_validated = false;
	}
	publish_drm = zuma_framebuffer_validated;
	mutex_unlock(&zuma_display_mmio_lock);

	if (!publish_drm)
		return;

	mutex_lock(&zuma_drm_registration_lock);
	zuma_drm_scanout_ready = true;
	mutex_unlock(&zuma_drm_registration_lock);
	zuma_drm_try_register();
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

	if (!zuma_dpp0_dt_contract_valid())
		return 0;

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
		pr_info("zuma-display-handoff: reserved scanout framebuffer at %pa, size=%#zx\n",
			&zuma_husky_flip_framebuffer_base,
			(size_t)ZUMA_HANDOFF_FB_SIZE);
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
