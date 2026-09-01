// SPDX-License-Identifier: GPL-2.0-only
/*
 * Temporary watchdog bootloader handoff for Google Zuma/Husky.
 *
 * The shipping boot chain leaves the Zuma cluster watchdog pair running.
 * Until native Zuma clocks and watchdog support exist, stop those timers
 * using the shipping DT resources and Google's downstream stop sequence.
 */

#include <linux/bitops.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/printk.h>
#include <linux/string.h>

#define ZUMA_WTCON             0x00
#define ZUMA_WTDAT             0x04
#define ZUMA_WTCNT             0x08
#define ZUMA_WTCLRINT          0x0c

#define ZUMA_WTCON_RSTEN       BIT(0)
#define ZUMA_WTCON_INTEN       BIT(2)
#define ZUMA_WTCON_ENABLE      BIT(5)

#define ZUMA_CL0_WDT_BASE      0x10060000
#define ZUMA_CL1_WDT_BASE      0x10070000
#define ZUMA_WDT_MIN_SIZE      0x10

struct zuma_wdt_handoff {
	struct device_node *np;
	void __iomem *base;
	const char *name;
};

static int __init zuma_wdt_map(struct zuma_wdt_handoff *wdt,
			       const char *compatible,
			       resource_size_t expected_base)
{
	struct resource res;
	int ret;

	wdt->np = of_find_compatible_node(NULL, NULL, compatible);
	if (!wdt->np)
		return -ENODEV;

	ret = of_address_to_resource(wdt->np, 0, &res);
	if (ret)
		return ret;

	if (res.start != expected_base || resource_size(&res) < ZUMA_WDT_MIN_SIZE) {
		pr_err("zuma-wdt-handoff: refusing unexpected %s resource %pr\n",
		       wdt->name, &res);
		return -EINVAL;
	}

	wdt->base = ioremap(res.start, ZUMA_WDT_MIN_SIZE);
	if (!wdt->base)
		return -ENOMEM;

	return 0;
}

static void __init zuma_wdt_unmap(struct zuma_wdt_handoff *wdt)
{
	if (wdt->base)
		iounmap(wdt->base);
	of_node_put(wdt->np);
}

static void __init zuma_wdt_stop(struct zuma_wdt_handoff *wdt, u32 clear_mask)
{
	u32 before, after;

	before = readl(wdt->base + ZUMA_WTCON);
	after = before & ~clear_mask;
	writel(after, wdt->base + ZUMA_WTCON);
	writel(1, wdt->base + ZUMA_WTCLRINT);
	after = readl(wdt->base + ZUMA_WTCON);

	pr_info("zuma-wdt-handoff: %s WTCON %#x -> %#x, WTDAT %#x, WTCNT %#x\n",
		wdt->name, before, after,
		readl(wdt->base + ZUMA_WTDAT),
		readl(wdt->base + ZUMA_WTCNT));
}

static int __init zuma_watchdog_handoff_init(void)
{
	struct zuma_wdt_handoff cl0 = { .name = "cluster 0" };
	struct zuma_wdt_handoff cl1 = { .name = "cluster 1" };
	struct device_node *root;
	const char *model;
	int ret;

	root = of_find_node_by_path("/");
	if (!root)
		return 0;

	if (!of_device_is_compatible(root, "google,zuma") ||
	    of_property_read_string(root, "model", &model) ||
	    !strstr(model, "HUSKY")) {
		of_node_put(root);
		return 0;
	}
	of_node_put(root);

	ret = zuma_wdt_map(&cl0, "google,zuma-cl0-wdt", ZUMA_CL0_WDT_BASE);
	if (ret) {
		pr_err("zuma-wdt-handoff: cluster 0 map failed: %d\n", ret);
		goto out;
	}

	ret = zuma_wdt_map(&cl1, "google,zuma-cl1-wdt", ZUMA_CL1_WDT_BASE);
	if (ret) {
		pr_err("zuma-wdt-handoff: cluster 1 map failed: %d\n", ret);
		goto out;
	}

	/* Google's CL0 stop first disables the CL1 multistage watchdog. */
	zuma_wdt_stop(&cl1, ZUMA_WTCON_ENABLE | ZUMA_WTCON_INTEN);
	zuma_wdt_stop(&cl0, ZUMA_WTCON_ENABLE | ZUMA_WTCON_RSTEN);

	if ((readl(cl0.base + ZUMA_WTCON) | readl(cl1.base + ZUMA_WTCON)) &
	    ZUMA_WTCON_ENABLE)
		pr_err("zuma-wdt-handoff: a watchdog remains enabled\n");
	else
		pr_info("zuma-wdt-handoff: stopped powered bootloader watchdog pair\n");

out:
	zuma_wdt_unmap(&cl1);
	zuma_wdt_unmap(&cl0);
	return 0;
}
early_initcall(zuma_watchdog_handoff_init);
