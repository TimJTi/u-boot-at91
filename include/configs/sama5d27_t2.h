/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration file for the SAMA5D27 T2 Board.
 *
 * Copyright (C) 2021 JT Innovations Ltd
 *		      Tim Hardisty <timh@jti.uk.com>
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define SAMA5D27_T2


#ifndef CONFIG_SPL_BUILD
#define CONFIG_SKIP_LOWLEVEL_INIT
#endif

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(4 * 1024 * 1024)

/* general purpose I/O */
#ifndef CONFIG_DM_GPIO
#define CONFIG_AT91_GPIO
#endif

#define CONFIG_ARCH_MISC_INIT

#define CONFIG_SYS_AT91_MAIN_CLOCK      24000000 /* from 24 MHz crystal */
#define CONFIG_SYS_AT91_SLOW_CLOCK      32768

/* SDRAM */
#define CONFIG_SYS_SDRAM_BASE		0x20000000
#undef CONFIG_SYS_SDRAM_SIZE
#define CONFIG_SYS_SDRAM_SIZE		0x8000000

#undef CONFIG_SPL_BUILD
#undef CONFIG_CMD_NAND

#define CONFIG_SYS_LOAD_ADDR		0x22000000 /* load address */

/* SPI flash */
//#undef CONFIG_BOOTCOMMAND
#undef CONFIG_SD_BOOT
#undef CONFIG_BOOTARGS

#define CONFIG_SYS_MONITOR_LEN		(512 << 10)

#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_SDRAM_BASE + 16 * 1024 - GENERATED_GBL_DATA_SIZE)
#define CONFIG_CMD_BMP

#undef CONFIG_BOOTCOMMAND
#define CONFIG_BOOTCOMMAND		\
  "echo Loading from flash...; "	\
  "lcdputs loading... ; " \
  "sf probe 0:1; " \
  "sf read 0x20008000 0xC0000 0x200000; "	\
  "cls;" \
  "go 0x20008000"

#endif
