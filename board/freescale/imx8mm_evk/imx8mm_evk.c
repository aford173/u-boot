// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 NXP
 */

#include <common.h>
#include <miiphy.h>
#include <netdev.h>

#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/io.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <cpu_func.h>
#include <sec_mipi_dsim.h>
#include <imx_mipi_dsi_bridge.h>
#include <mipi_dsi_panel.h>
#include <asm/mach-imx/video.h>

DECLARE_GLOBAL_DATA_PTR;

int dram_init(void)
{
	gd->ram_size = PHYS_SDRAM_SIZE;

	return 0;
}

#if IS_ENABLED(CONFIG_FEC_MXC)
static int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *gpr =
		(struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;

	/* Use 125M anatop REF_CLK1 for ENET1, not from external */
	clrsetbits_le32(&gpr->gpr[1], 0x2000, 0);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	/* enable rgmii rxc skew and phy mode select to RGMII copper */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x00);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x82ee);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);

	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}
#endif

int board_init(void)
{
	if (IS_ENABLED(CONFIG_FEC_MXC))
		setup_fec();

	return 0;
}

int board_mmc_get_env_dev(int devno)
{
	return devno;
}

#ifdef CONFIG_VIDEO_MXS

#define ADV7535_MAIN 0x3d
#define ADV7535_DSI_CEC 0x3c

static const struct sec_mipi_dsim_plat_data imx8mm_mipi_dsim_plat_data = {
	.version	= 0x1060200,
	.max_data_lanes = 4,
	.max_data_rate  = 1500000000ULL,
	.reg_base = MIPI_DSI_BASE_ADDR,
	.gpr_base = CSI_BASE_ADDR + 0x8000,
};

static int adv7535_i2c_reg_write(struct udevice *dev, uint addr, uint mask, uint data)
{
	uint8_t valb;
	int err;

	if (mask != 0xff) {
		err = dm_i2c_read(dev, addr, &valb, 1);
		if (err)
			return err;

		valb &= ~mask;
		valb |= data;
	} else {
		valb = data;
	}

	err = dm_i2c_write(dev, addr, &valb, 1);
	return err;
}

static int adv7535_i2c_reg_read(struct udevice *dev, uint8_t addr, uint8_t *data)
{
	uint8_t valb;
	int err;

	err = dm_i2c_read(dev, addr, &valb, 1);
	if (err)
		return err;

	*data = (int)valb;
	return 0;
}

static void adv7535_init(void)
{
	struct udevice *bus, *main_dev, *cec_dev;
	int i2c_bus = 1;
	int ret;
	uint8_t val;

	ret = uclass_get_device_by_seq(UCLASS_I2C, i2c_bus, &bus);
	if (ret) {
		printf("%s: No bus %d\n", __func__, i2c_bus);
		return;
	}

	ret = dm_i2c_probe(bus, ADV7535_MAIN, 0, &main_dev);
	if (ret) {
		printf("%s: Can't find device id=0x%x, on bus %d\n",
			__func__, ADV7535_MAIN, i2c_bus);
		return;
	}

	ret = dm_i2c_probe(bus, ADV7535_DSI_CEC, 0, &cec_dev);
	if (ret) {
		printf("%s: Can't find device id=0x%x, on bus %d\n",
			__func__, ADV7535_MAIN, i2c_bus);
		return;
	}

	adv7535_i2c_reg_read(main_dev, 0x00, &val);
	debug("Chip revision: 0x%x (expected: 0x14)\n", val);
	adv7535_i2c_reg_read(cec_dev, 0x00, &val);
	debug("Chip ID MSB: 0x%x (expected: 0x75)\n", val);
	adv7535_i2c_reg_read(cec_dev, 0x01, &val);
	debug("Chip ID LSB: 0x%x (expected: 0x33)\n", val);

	/* Power */
	adv7535_i2c_reg_write(main_dev, 0x41, 0xff, 0x10);
	/* Initialisation (Fixed) Registers */
	adv7535_i2c_reg_write(main_dev, 0x16, 0xff, 0x20);
	adv7535_i2c_reg_write(main_dev, 0x9A, 0xff, 0xE0);
	adv7535_i2c_reg_write(main_dev, 0xBA, 0xff, 0x70);
	adv7535_i2c_reg_write(main_dev, 0xDE, 0xff, 0x82);
	adv7535_i2c_reg_write(main_dev, 0xE4, 0xff, 0x40);
	adv7535_i2c_reg_write(main_dev, 0xE5, 0xff, 0x80);
	adv7535_i2c_reg_write(cec_dev, 0x15, 0xff, 0xD0);
	adv7535_i2c_reg_write(cec_dev, 0x17, 0xff, 0xD0);
	adv7535_i2c_reg_write(cec_dev, 0x24, 0xff, 0x20);
	adv7535_i2c_reg_write(cec_dev, 0x57, 0xff, 0x11);
	/* 4 x DSI Lanes */
	adv7535_i2c_reg_write(cec_dev, 0x1C, 0xff, 0x40);

	/* DSI Pixel Clock Divider */
	adv7535_i2c_reg_write(cec_dev, 0x16, 0xff, 0x18);

	/* Enable Internal Timing Generator */
	adv7535_i2c_reg_write(cec_dev, 0x27, 0xff, 0xCB);
	/* 1920 x 1080p 60Hz */
	adv7535_i2c_reg_write(cec_dev, 0x28, 0xff, 0x89); /* total width */
	adv7535_i2c_reg_write(cec_dev, 0x29, 0xff, 0x80); /* total width */
	adv7535_i2c_reg_write(cec_dev, 0x2A, 0xff, 0x02); /* hsync */
	adv7535_i2c_reg_write(cec_dev, 0x2B, 0xff, 0xC0); /* hsync */
	adv7535_i2c_reg_write(cec_dev, 0x2C, 0xff, 0x05); /* hfp */
	adv7535_i2c_reg_write(cec_dev, 0x2D, 0xff, 0x80); /* hfp */
	adv7535_i2c_reg_write(cec_dev, 0x2E, 0xff, 0x09); /* hbp */
	adv7535_i2c_reg_write(cec_dev, 0x2F, 0xff, 0x40); /* hbp */

	adv7535_i2c_reg_write(cec_dev, 0x30, 0xff, 0x46); /* total height */
	adv7535_i2c_reg_write(cec_dev, 0x31, 0xff, 0x50); /* total height */
	adv7535_i2c_reg_write(cec_dev, 0x32, 0xff, 0x00); /* vsync */
	adv7535_i2c_reg_write(cec_dev, 0x33, 0xff, 0x50); /* vsync */
	adv7535_i2c_reg_write(cec_dev, 0x34, 0xff, 0x00); /* vfp */
	adv7535_i2c_reg_write(cec_dev, 0x35, 0xff, 0x40); /* vfp */
	adv7535_i2c_reg_write(cec_dev, 0x36, 0xff, 0x02); /* vbp */
	adv7535_i2c_reg_write(cec_dev, 0x37, 0xff, 0x40); /* vbp */

	/* Reset Internal Timing Generator */
	adv7535_i2c_reg_write(cec_dev, 0x27, 0xff, 0xCB);
	adv7535_i2c_reg_write(cec_dev, 0x27, 0xff, 0x8B);
	adv7535_i2c_reg_write(cec_dev, 0x27, 0xff, 0xCB);

	/* HDMI Output */
	adv7535_i2c_reg_write(main_dev, 0xAF, 0xff, 0x16);
	/* AVI Infoframe - RGB - 16-9 Aspect Ratio */
	adv7535_i2c_reg_write(main_dev, 0x55, 0xff, 0x02);
	adv7535_i2c_reg_write(main_dev, 0x56, 0xff, 0x0);

	/*  GC Packet Enable */
	adv7535_i2c_reg_write(main_dev, 0x40, 0xff, 0x0);
	/*  GC Colour Depth - 24 Bit */
	adv7535_i2c_reg_write(main_dev, 0x4C, 0xff, 0x0);
	/*  Down Dither Output Colour Depth - 8 Bit (default) */
	adv7535_i2c_reg_write(main_dev, 0x49, 0xff, 0x00);

	/* set low refresh 1080p30 */
	adv7535_i2c_reg_write(main_dev, 0x4A, 0xff, 0x80); /*should be 0x80 for 1080p60 and 0x8c for 1080p30*/

	/* HDMI Output Enable */
	adv7535_i2c_reg_write(cec_dev, 0xbe, 0xff, 0x3c);
	adv7535_i2c_reg_write(cec_dev, 0x03, 0xff, 0x89);
}

#define DISPLAY_MIX_SFT_RSTN_CSR		0x00
#define DISPLAY_MIX_CLK_EN_CSR		0x04

   /* 'DISP_MIX_SFT_RSTN_CSR' bit fields */
#define BUS_RSTN_BLK_SYNC_SFT_EN	BIT(6)

   /* 'DISP_MIX_CLK_EN_CSR' bit fields */
#define LCDIF_PIXEL_CLK_SFT_EN		BIT(7)
#define LCDIF_APB_CLK_SFT_EN		BIT(6)

void disp_mix_bus_rstn_reset(ulong gpr_base, bool reset)
{
	if (!reset)
		/* release reset */
		setbits_le32(gpr_base + DISPLAY_MIX_SFT_RSTN_CSR, BUS_RSTN_BLK_SYNC_SFT_EN);
	else
		/* hold reset */
		clrbits_le32(gpr_base + DISPLAY_MIX_SFT_RSTN_CSR, BUS_RSTN_BLK_SYNC_SFT_EN);
}

void disp_mix_lcdif_clks_enable(ulong gpr_base, bool enable)
{
	if (enable)
		/* enable lcdif clks */
		setbits_le32(gpr_base + DISPLAY_MIX_CLK_EN_CSR, LCDIF_PIXEL_CLK_SFT_EN | LCDIF_APB_CLK_SFT_EN);
	else
		/* disable lcdif clks */
		clrbits_le32(gpr_base + DISPLAY_MIX_CLK_EN_CSR, LCDIF_PIXEL_CLK_SFT_EN | LCDIF_APB_CLK_SFT_EN);
}

struct mipi_dsi_client_dev adv7535_dev = {
	.channel	= 0,
	.lanes = 4,
	.format  = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_MODE_VIDEO_HSE,
	.name = "ADV7535",
};

struct mipi_dsi_client_dev rm67191_dev = {
	.channel	= 0,
	.lanes = 4,
	.format  = MIPI_DSI_FMT_RGB888,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_MODE_VIDEO_HSE,
};

#define FSL_SIP_GPC			0xC2000000
#define FSL_SIP_CONFIG_GPC_PM_DOMAIN	0x3
#define DISPMIX				9
#define MIPI				10

void do_enable_mipi2hdmi(struct display_info_t const *dev)
{
	gpio_request(IMX_GPIO_NR(1, 8), "DSI EN");
	gpio_direction_output(IMX_GPIO_NR(1, 8), 1);

	/* ADV7353 initialization */
	adv7535_init();

	/* enable the dispmix & mipi phy power domain */
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, DISPMIX, true, 0);
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, MIPI, true, 0);

	/* Put lcdif out of reset */
	disp_mix_bus_rstn_reset(imx8mm_mipi_dsim_plat_data.gpr_base, false);
	disp_mix_lcdif_clks_enable(imx8mm_mipi_dsim_plat_data.gpr_base, true);

	/* Setup mipi dsim */
	sec_mipi_dsim_setup(&imx8mm_mipi_dsim_plat_data);
	imx_mipi_dsi_bridge_attach(&adv7535_dev); /* attach adv7535 device */
}

void do_enable_mipi_led(struct display_info_t const *dev)
{
	gpio_request(IMX_GPIO_NR(1, 8), "DSI EN");
	gpio_direction_output(IMX_GPIO_NR(1, 8), 0);
	mdelay(100);
	gpio_direction_output(IMX_GPIO_NR(1, 8), 1);

	/* enable the dispmix & mipi phy power domain */
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, DISPMIX, true, 0);
	call_imx_sip(FSL_SIP_GPC, FSL_SIP_CONFIG_GPC_PM_DOMAIN, MIPI, true, 0);

	/* Put lcdif out of reset */
	disp_mix_bus_rstn_reset(imx8mm_mipi_dsim_plat_data.gpr_base, false);
	disp_mix_lcdif_clks_enable(imx8mm_mipi_dsim_plat_data.gpr_base, true);

	/* Setup mipi dsim */
	sec_mipi_dsim_setup(&imx8mm_mipi_dsim_plat_data);

	rm67191_init();
	rm67191_dev.name = displays[1].mode.name;
	imx_mipi_dsi_bridge_attach(&rm67191_dev); /* attach rm67191 device */
}

struct display_info_t const displays[] = {{
	.bus = LCDIF_BASE_ADDR,
	.addr = 0,
	.pixfmt = 24,
	.detect = NULL,
	.enable	= do_enable_mipi2hdmi,
	.mode	= {
		.name			= "MIPI2HDMI",
		.refresh		= 60,
		.xres			= 1920,
		.yres			= 1080,
		.pixclock		= 6734, /* 148500000 */
		.left_margin	= 148,
		.right_margin	= 88,
		.upper_margin	= 36,
		.lower_margin	= 4,
		.hsync_len		= 44,
		.vsync_len		= 5,
		.sync			= FB_SYNC_EXT,
		.vmode			= FB_VMODE_NONINTERLACED

} }, {
	.bus = LCDIF_BASE_ADDR,
	.addr = 0,
	.pixfmt = 24,
	.detect = NULL,
	.enable	= do_enable_mipi_led,
	.mode	= {
		.name			= "RM67191_OLED",
		.refresh		= 60,
		.xres			= 1080,
		.yres			= 1920,
		.pixclock		= 7575, /* 132000000 */
		.left_margin	= 34,
		.right_margin	= 20,
		.upper_margin	= 4,
		.lower_margin	= 10,
		.hsync_len		= 2,
		.vsync_len		= 2,
		.sync			= FB_SYNC_EXT,
		.vmode			= FB_VMODE_NONINTERLACED

} } };
size_t display_count = ARRAY_SIZE(displays);
#endif

int board_late_init(void)
{
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "EVK");
	env_set("board_rev", "iMX8MM");
#endif
	return 0;
}
