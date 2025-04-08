// SPDX-License-Identifier: GPL-2.0+
/* Copyright 2023 Logic PD, Inc dba Beacon EmbeddedWorks */

#include <common.h>
#include <init.h>
#include <miiphy.h>
#include <asm/arch/sys_proto.h>
#include "../../freescale/common/tcpc.h"

static void setup_fec(void)
{
	struct iomuxc_gpr_base_regs *gpr =
		(struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;

	/* Enable RGMII TX clk output */
	setbits_le32(&gpr->gpr[1], BIT(22));
}

#if IS_ENABLED(CONFIG_NET)
int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}
#endif

#ifdef CONFIG_USB_TCPC

/* Configure the incoming Power */
static int setup_typec(void)
{
	const struct tcpc_port_config power_port_config = {
		.i2c_bus = 2, /*i2c3*/
		.addr = 0x53,
		.port_type = TYPEC_PORT_UFP,
		.max_snk_mv = 9000,
		.max_snk_ma = 3000,
		.max_snk_mw = 45000,
		.op_snk_mv = 27000,
	};
	int ret;
	struct tcpc_port power_port;

	debug("tcpc_init power port\n");
	ret = tcpc_init(&power_port, power_port_config, NULL);

	if (ret) {
		printf("%s: tcpc power power init failed, err=%d\n",
		       __func__, ret);
		return ret;
	}
	return 0;
}

#endif /* CONFIG_USB_TCPC */

int board_init(void)
{
	int ret = 0;

#ifdef CONFIG_USB_TCPC
	setup_typec();

	/* Enable USB power default */
	imx8m_usb_power(0, true);
	imx8m_usb_power(1, true);
#endif

	if (CONFIG_IS_ENABLED(FEC_MXC))
		setup_fec();

	return ret;
}

#if defined(CONFIG_ENV_IS_IN_MMC)
int board_mmc_get_env_dev(int devno)
{
        return CONFIG_SYS_MMC_ENV_DEV;
}
#endif
