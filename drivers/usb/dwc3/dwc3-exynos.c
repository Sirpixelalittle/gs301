// SPDX-License-Identifier: GPL-2.0
/*
 * dwc3-exynos.c - Samsung Exynos DWC3 Specific Glue layer
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Author: Anton Tikhomirov <av.tikhomirov@samsung.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#define DWC3_EXYNOS_MAX_CLOCKS	4

struct dwc3_exynos_driverdata {
	const char		*clk_names[DWC3_EXYNOS_MAX_CLOCKS];
	int			num_clks;
	int			suspend_clk_idx;
	bool			skip_regulators;
	bool			check_zuma_hsi0;
};

struct dwc3_exynos {
	struct device		*dev;

	const char		**clk_names;
	struct clk		*clks[DWC3_EXYNOS_MAX_CLOCKS];
	int			num_clks;
	int			suspend_clk_idx;
	bool			skip_regulators;

	struct regulator	*vdd33;
	struct regulator	*vdd10;
};

#define ZUMA_HSI0_STATUS	0x2a84

static int dwc3_exynos_check_zuma_hsi0(struct device *dev)
{
	struct regmap *pmu;
	unsigned int status;
	int ret;

	pmu = syscon_regmap_lookup_by_compatible("samsung,gs101-pmu");
	if (IS_ERR(pmu))
		return dev_err_probe(dev, PTR_ERR(pmu),
				     "failed to find Zuma PMU syscon\n");

	ret = regmap_read(pmu, ZUMA_HSI0_STATUS, &status);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read HSI0 status\n");

	if (!(status & BIT(0))) {
		dev_warn(dev, "HSI0 is off; refusing unsafe USB MMIO handoff\n");
		return -ENODEV;
	}

	dev_info(dev, "HSI0 is on; accepting guarded bootloader handoff\n");
	return 0;
}

static int dwc3_exynos_probe(struct platform_device *pdev)
{
	struct dwc3_exynos	*exynos;
	struct device		*dev = &pdev->dev;
	struct device_node	*node = dev->of_node;
	const struct dwc3_exynos_driverdata *driver_data;
	int			i, ret;

	exynos = devm_kzalloc(dev, sizeof(*exynos), GFP_KERNEL);
	if (!exynos)
		return -ENOMEM;

	driver_data = of_device_get_match_data(dev);
	exynos->dev = dev;
	exynos->num_clks = driver_data->num_clks;
	exynos->clk_names = (const char **)driver_data->clk_names;
	exynos->suspend_clk_idx = driver_data->suspend_clk_idx;
	exynos->skip_regulators = driver_data->skip_regulators;

	if (driver_data->check_zuma_hsi0) {
		if (!of_property_read_bool(node,
					   "google,bootloader-usb-handoff"))
			return -ENODEV;

		ret = dwc3_exynos_check_zuma_hsi0(dev);
		if (ret)
			return ret;
	}

	platform_set_drvdata(pdev, exynos);

	for (i = 0; i < exynos->num_clks; i++) {
		exynos->clks[i] = devm_clk_get(dev, exynos->clk_names[i]);
		if (IS_ERR(exynos->clks[i])) {
			dev_err(dev, "failed to get clock: %s\n",
				exynos->clk_names[i]);
			return PTR_ERR(exynos->clks[i]);
		}
	}

	for (i = 0; i < exynos->num_clks; i++) {
		ret = clk_prepare_enable(exynos->clks[i]);
		if (ret) {
			while (i-- > 0)
				clk_disable_unprepare(exynos->clks[i]);
			return ret;
		}
	}

	if (exynos->suspend_clk_idx >= 0)
		clk_prepare_enable(exynos->clks[exynos->suspend_clk_idx]);

	if (!exynos->skip_regulators) {
		exynos->vdd33 = devm_regulator_get(dev, "vdd33");
		if (IS_ERR(exynos->vdd33)) {
			ret = PTR_ERR(exynos->vdd33);
			goto vdd33_err;
		}
		ret = regulator_enable(exynos->vdd33);
		if (ret) {
			dev_err(dev, "Failed to enable VDD33 supply\n");
			goto vdd33_err;
		}

		exynos->vdd10 = devm_regulator_get(dev, "vdd10");
		if (IS_ERR(exynos->vdd10)) {
			ret = PTR_ERR(exynos->vdd10);
			goto vdd10_err;
		}
		ret = regulator_enable(exynos->vdd10);
		if (ret) {
			dev_err(dev, "Failed to enable VDD10 supply\n");
			goto vdd10_err;
		}
	}

	if (node) {
		ret = of_platform_populate(node, NULL, NULL, dev);
		if (ret) {
			dev_err(dev, "failed to add dwc3 core\n");
			goto populate_err;
		}
	} else {
		dev_err(dev, "no device node, failed to add dwc3 core\n");
		ret = -ENODEV;
		goto populate_err;
	}

	return 0;

populate_err:
	if (!exynos->skip_regulators)
		regulator_disable(exynos->vdd10);
vdd10_err:
	if (!exynos->skip_regulators)
		regulator_disable(exynos->vdd33);
vdd33_err:
	for (i = exynos->num_clks - 1; i >= 0; i--)
		clk_disable_unprepare(exynos->clks[i]);

	if (exynos->suspend_clk_idx >= 0)
		clk_disable_unprepare(exynos->clks[exynos->suspend_clk_idx]);

	return ret;
}

static void dwc3_exynos_remove(struct platform_device *pdev)
{
	struct dwc3_exynos	*exynos = platform_get_drvdata(pdev);
	int i;

	of_platform_depopulate(&pdev->dev);

	for (i = exynos->num_clks - 1; i >= 0; i--)
		clk_disable_unprepare(exynos->clks[i]);

	if (exynos->suspend_clk_idx >= 0)
		clk_disable_unprepare(exynos->clks[exynos->suspend_clk_idx]);

	if (!exynos->skip_regulators) {
		regulator_disable(exynos->vdd33);
		regulator_disable(exynos->vdd10);
	}
}

static const struct dwc3_exynos_driverdata zuma_handoff_drvdata = {
	.num_clks = 0,
	.suspend_clk_idx = -1,
	.skip_regulators = true,
	.check_zuma_hsi0 = true,
};

static const struct dwc3_exynos_driverdata exynos2200_drvdata = {
	.clk_names = { "link_aclk" },
	.num_clks = 1,
	.suspend_clk_idx = -1,
};

static const struct dwc3_exynos_driverdata exynos5250_drvdata = {
	.clk_names = { "usbdrd30" },
	.num_clks = 1,
	.suspend_clk_idx = -1,
};

static const struct dwc3_exynos_driverdata exynos5433_drvdata = {
	.clk_names = { "aclk", "susp_clk", "pipe_pclk", "phyclk" },
	.num_clks = 4,
	.suspend_clk_idx = 1,
};

static const struct dwc3_exynos_driverdata exynos7_drvdata = {
	.clk_names = { "usbdrd30", "usbdrd30_susp_clk", "usbdrd30_axius_clk" },
	.num_clks = 3,
	.suspend_clk_idx = 1,
};

static const struct dwc3_exynos_driverdata exynos7870_drvdata = {
	.clk_names = { "bus_early", "ref", "ctrl" },
	.num_clks = 3,
	.suspend_clk_idx = -1,
};

static const struct dwc3_exynos_driverdata exynos850_drvdata = {
	.clk_names = { "bus_early", "ref" },
	.num_clks = 2,
	.suspend_clk_idx = -1,
};

static const struct dwc3_exynos_driverdata gs101_drvdata = {
	.clk_names = { "bus_early", "susp_clk", "link_aclk", "link_pclk" },
	.num_clks = 4,
	.suspend_clk_idx = 1,
};

static const struct dwc3_exynos_driverdata exynosautov920_drvdata = {
	.clk_names = { "ref", "susp_clk"},
	.num_clks = 2,
	.suspend_clk_idx = 1,
};

static const struct of_device_id exynos_dwc3_match[] = {
	{
		.compatible = "samsung,exynos9-dwusb",
		.data = &zuma_handoff_drvdata,
	}, {
		.compatible = "samsung,exynos2200-dwusb3",
		.data = &exynos2200_drvdata,
	}, {
		.compatible = "samsung,exynos5250-dwusb3",
		.data = &exynos5250_drvdata,
	}, {
		.compatible = "samsung,exynos5433-dwusb3",
		.data = &exynos5433_drvdata,
	}, {
		.compatible = "samsung,exynos7-dwusb3",
		.data = &exynos7_drvdata,
	}, {
		.compatible = "samsung,exynos7870-dwusb3",
		.data = &exynos7870_drvdata,
	}, {
		.compatible = "samsung,exynos850-dwusb3",
		.data = &exynos850_drvdata,
	}, {
		.compatible = "samsung,exynosautov920-dwusb3",
		.data = &exynosautov920_drvdata,
	}, {
		.compatible = "google,gs101-dwusb3",
		.data = &gs101_drvdata,
	}, {
	}
};
MODULE_DEVICE_TABLE(of, exynos_dwc3_match);

static int dwc3_exynos_suspend(struct device *dev)
{
	struct dwc3_exynos *exynos = dev_get_drvdata(dev);
	int i;

	for (i = exynos->num_clks - 1; i >= 0; i--)
		clk_disable_unprepare(exynos->clks[i]);

	if (!exynos->skip_regulators) {
		regulator_disable(exynos->vdd33);
		regulator_disable(exynos->vdd10);
	}

	return 0;
}

static int dwc3_exynos_resume(struct device *dev)
{
	struct dwc3_exynos *exynos = dev_get_drvdata(dev);
	int i, ret;

	if (!exynos->skip_regulators) {
		ret = regulator_enable(exynos->vdd33);
		if (ret) {
			dev_err(dev, "Failed to enable VDD33 supply\n");
			return ret;
		}
		ret = regulator_enable(exynos->vdd10);
		if (ret) {
			dev_err(dev, "Failed to enable VDD10 supply\n");
			return ret;
		}
	}

	for (i = 0; i < exynos->num_clks; i++) {
		ret = clk_prepare_enable(exynos->clks[i]);
		if (ret) {
			while (i-- > 0)
				clk_disable_unprepare(exynos->clks[i]);
			return ret;
		}
	}

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(dwc3_exynos_dev_pm_ops,
				dwc3_exynos_suspend, dwc3_exynos_resume);

static struct platform_driver dwc3_exynos_driver = {
	.probe		= dwc3_exynos_probe,
	.remove		= dwc3_exynos_remove,
	.driver		= {
		.name	= "exynos-dwc3",
		.of_match_table = exynos_dwc3_match,
		.pm	= pm_sleep_ptr(&dwc3_exynos_dev_pm_ops),
	},
};

module_platform_driver(dwc3_exynos_driver);

MODULE_AUTHOR("Anton Tikhomirov <av.tikhomirov@samsung.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 Exynos Glue Layer");
