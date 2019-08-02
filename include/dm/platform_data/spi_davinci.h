/*
 * Copyright (C) 2018 Jagan Teki <jagan@amarulasolutions.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __spi_davinci_h
#define __spi_davinci_h

#include <dma-uclass.h>

struct davinci_spi_platdata {
	struct davinci_spi_regs *regs;
	u8 num_cs;	   /* total no. of CS available */
	struct dma		dma_tx;
	struct dma		dma_rx;
};

#endif /* __spi_davinci_h */
