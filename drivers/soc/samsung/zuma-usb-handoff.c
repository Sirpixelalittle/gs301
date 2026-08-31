// SPDX-License-Identifier: GPL-2.0-only
/*
 * Temporary USB bootloader-handoff fixups for Google Zuma/Husky.
 *
 * The Google bootloader requires the shipping DTB/DTBO, whose USB nodes
 * describe downstream-only clock, power-domain and IOMMU providers.  Remove
 * only those unavailable supplier links before platform devices are created.
 * The USB drivers still consume the shipping node addresses and compatibles.
 */

#include <linux/init.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/string.h>

static struct property zuma_usb_glue_handoff_property = {
	.name = "google,bootloader-usb-handoff",
	.length = 0,
};

static struct property zuma_usb_phy_handoff_property = {
	.name = "google,bootloader-usb-handoff",
	.length = 0,
};

static struct property zuma_usb_maximum_speed_property = {
	.name = "maximum-speed",
	.length = sizeof("high-speed"),
	.value = "high-speed",
};

static void __init zuma_usb_remove_property(struct device_node *np,
					    const char *name)
{
	struct property *prop;
	int ret;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return;

	ret = of_remove_property(np, prop);
	if (ret)
		pr_warn("zuma-usb-handoff: failed to remove %pOF/%s: %d\n",
			np, name, ret);
}

static int __init zuma_usb_handoff_fixup(void)
{
	struct device_node *root, *glue, *dwc3, *phy;
	const char *model;

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

	glue = of_find_node_by_path("/usb@11210000");
	dwc3 = of_find_node_by_path("/usb@11210000/dwc3");
	phy = of_find_node_by_path("/phy@11100000");
	if (!glue || !dwc3 || !phy) {
		pr_warn("zuma-usb-handoff: shipping USB nodes are incomplete\n");
		goto out;
	}

	/* These downstream suppliers do not exist in mainline yet. */
	zuma_usb_remove_property(glue, "clocks");
	zuma_usb_remove_property(glue, "power-domains");
	zuma_usb_remove_property(dwc3, "iommus");
	zuma_usb_remove_property(dwc3, "memory-region");
	zuma_usb_remove_property(phy, "clocks");
	zuma_usb_remove_property(phy, "power-domains");
	zuma_usb_remove_property(phy, "s2mpus");

	of_update_property(glue, &zuma_usb_glue_handoff_property);
	of_update_property(phy, &zuma_usb_phy_handoff_property);
	of_update_property(dwc3, &zuma_usb_maximum_speed_property);
	pr_info("zuma-usb-handoff: enabled guarded High-Speed peripheral handoff\n");

out:
	of_node_put(phy);
	of_node_put(dwc3);
	of_node_put(glue);
	return 0;
}
early_initcall(zuma_usb_handoff_fixup);
