/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration file for the SAMA5D27 T2 Board.
 *
 * Copyright (C) 2021 JT Innovations Ltd
 *		      Tim Hardisty <timh@jti.uk.com>
*/

#ifndef __CONFIG_H
#define __CONFIG_H

#include "at91-sama5_common.h"



#define SAMA5D27_T2

#define CONFIG_ARCH_MISC_INIT

#undef CONFIG_SYS_AT91_MAIN_CLOCK
#define CONFIG_SYS_AT91_MAIN_CLOCK      24000000 /* from 24 MHz crystal */

/* SDRAM */
#define CONFIG_SYS_SDRAM_BASE		0x20000000
#define CONFIG_SYS_SDRAM_SIZE		0x4000000

#ifdef CONFIG_SPL_BUILD
#define CONFIG_SYS_INIT_SP_ADDR		0x218000
#else
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_SDRAM_BASE + 16 * 1024 - GENERATED_GBL_DATA_SIZE)
#endif

#define CONFIG_SYS_LOAD_ADDR		0x22000000 /* load address */

/* NAND flash */
#undef CONFIG_CMD_NAND

/* SPI flash */
#undef CONFIG_BOOTCOMMAND
#undef CONFIG_SD_BOOT
#undef CONFIG_BOOTARGS


#define CONFIG_SYS_MONITOR_LEN		(512 << 10)

#ifdef CONFIG_SD_BOOT
#define CONFIG_SYS_MMCSD_FS_BOOT_PARTITION	1
#define CONFIG_SPL_FS_LOAD_PAYLOAD_NAME		"u-boot.img"
#endif

#define CONFIG_CMD_BMP


#endif
