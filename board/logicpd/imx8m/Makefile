#
# Copyright 2020 Logic PD, Inc.
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += imx8m_logic.o
obj-y += ../../freescale/common/

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
obj-$(CONFIG_IMX8M_LPDDR4) += lpddr4_timing.o
endif
