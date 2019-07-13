// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
 * Written by Jean-Jacques Hiblot  <jjhiblot@ti.com>
 */

#include <common.h>
#include <dm.h>
#include <dm/device.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <generic-phy.h>


struct usb_phy_generic_pdata {

        struct gpio_desc *phy_reset_gpio;
};


static int	nop_phy_reset(struct phy *phy)
{
	struct usb_phy_generic_pdata *platdata = dev_get_platdata(phy->dev);
	int ret;

	if (platdata->phy_reset_gpio == NULL)
		return 0;

	ret = dm_gpio_set_value(platdata->phy_reset_gpio, 1);
	if (ret < 0) {
		pr_err("dm_gpio_set_value(phy_reset, assert) failed: %d", ret);
		return ret;
	}

	udelay(2);

	ret = dm_gpio_set_value(platdata->phy_reset_gpio, 0);
	if (ret < 0) {
		pr_err("dm_gpio_set_value(phy_reset, deassert) failed: %d", ret);
		return ret;
	}

	ret = dm_gpio_set_value(platdata->phy_reset_gpio, 1);
	if (ret < 0) {
		pr_err("dm_gpio_set_value(phy_reset, assert) failed: %d", ret);
		return ret;
	}

	return 0;
}

static int nop_phy_probe(struct udevice *dev)
{
	//struct usb_phy_generic_pdata *platdata = dev_get_platdata(dev);


	return 0;
}

static int nop_phy_ofdata_to_platdata(struct udevice *dev)
{
	struct usb_phy_generic_pdata *platdata = dev_get_platdata(dev);
	int ret;

	ret = gpio_request_by_name(dev, "reset-gpios", 0,
			platdata->phy_reset_gpio,
			GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	if (ret)
		platdata->phy_reset_gpio = NULL;

	printf("request red = %d\n", ret);

	return 0;
}

static const struct udevice_id nop_phy_ids[] = {
	{ .compatible = "nop-phy" },
	{ .compatible = "usb-nop-xceiv" },
	{ }
};

static struct phy_ops nop_phy_ops = {
		.reset = nop_phy_reset,
};

U_BOOT_DRIVER(nop_phy) = {
	.name	= "nop_phy",
	.id	= UCLASS_PHY,
	.probe = nop_phy_probe,
	.of_match = nop_phy_ids,
	.ofdata_to_platdata = nop_phy_ofdata_to_platdata,
	.ops = &nop_phy_ops,
	.platdata_auto_alloc_size = sizeof(struct usb_phy_generic_pdata),
};
