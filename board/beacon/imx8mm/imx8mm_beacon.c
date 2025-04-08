// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2022 Logic PD, Inc. dba Beacon EmbeddedWorks
 */

#include <asm/global_data.h>

DECLARE_GLOBAL_DATA_PTR;

int board_init(void)
{
	return 0;
}

#if defined(CONFIG_ENV_IS_IN_MMC)
int board_mmc_get_env_dev(int devno)
{
        return CONFIG_SYS_MMC_ENV_DEV;
}
#endif
