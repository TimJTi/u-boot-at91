// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2017 Microchip Corporation
 *		      Wenyou.Yang <wenyou.yang@microchip.com>
 */

#include <common.h>
#include <debug_uart.h>
#include <dm.h>
#include <i2c.h>
#include <init.h>
#include <asm/io.h>
#include <asm/arch/at91_common.h>
#include <asm/arch/atmel_pio4.h>
#include <asm/arch/atmel_mpddrc.h>
#include <asm/arch/atmel_sdhci.h>
#include <asm/arch/clk.h>

#include <asm/arch/gpio.h>
#include <asm/arch/sama5d2.h>

#include <atmel_lcd.h>
#include <dm.h>
#include <version.h>
#include <video.h>
#include <video_console.h>
#include <vsprintf.h>
#include "jti_logo_8bpp.h"

void jtinnovations_logo_info(vidinfo_t *info);  

#if defined(CONFIG_ARCH_MISC_INIT)
int arch_misc_init(void)
{
	struct udevice *dev;
	int ret;
  uchar val;
  uchar buf[4];
  /* set up ACT8945A */
	ret = i2c_get_chip_for_busnum(0, 0x5B, 1, &dev);
	if (ret) {
		printf("Cannot find ACT8945A: %d\n", ret);
		return 0;  
  }
  else {
    /* set VDD_LED to 3V3 */
    val = 0x39;    
    dm_i2c_write(dev, 0x60, &val, 1);
    /* enable VDD_LED */
    val = 0x80;    
    dm_i2c_write(dev, 0x61, &val, 1);
    /*disable VDD for legacy GPS antenna*/
    val = 0;
    dm_i2c_write(dev, 0x65, &val, 1);
    
    mdelay(100);
    
    /* now set up and read ambient light sensor */
    ret = i2c_get_chip_for_busnum(0, 0x39, 1, &dev);
    if (ret) {
      printf("Cannot find ADPS9901: %d\n", ret);
      return 0;
    }
    else {
      /* disable */
      val = 0;
      dm_i2c_write(dev, 0x80, &val, 1);      
      /* ATIME default */
      val = 0xED;
      dm_i2c_write(dev, 0x81, &val, 1);
      /* PTIME default */
      val = 0xFF;
      dm_i2c_write(dev, 0x82, &val, 1);
      /* WTIME default */
      val = 0xFF;
      dm_i2c_write(dev, 0x83, &val, 1);    
      /* PPCOUNT default */
      val = 0x08;
      dm_i2c_write(dev, 0x8E, &val, 1);      
      /* CONTROL defaults */
      val = 0x20;
      dm_i2c_write(dev, 0x8F, &val, 1);
      /* enable prox and ALS */
      val = 0x07;
      dm_i2c_write(dev, 0x80, &val, 1);    
      
      /* wait to accumulate some data */
      mdelay(200);
      
      dm_i2c_read(dev, 0xB4, &buf[0], 2);

      unsigned short int als = (((unsigned short int) buf[1]<<8) + ((unsigned short int) buf[0]));
      if (als < 50) {
        env_set("bootcmd", "\
          echo Boot interrupted via ambient light sensor method; \
          setcurs 0 10; \
          lcdputs 'Boot interrupted, via light sensor.'; \
          setcurs 0 11;\
          sleep 1; \
          lcdputs 'Insert USB Flash drive with firmware to load'; \
          sleep 1;" );
          
      }
      else  {
        env_set("bootcmd", "\
          echo Trying to load from flash...; 	\
          lcdputs loading.; \
          sf probe 1:0; \
          sf read 0x20008000 0xC0000 0x200000; \
          cls; \
          go 0x20008000;");
      }
        
    }
  }
  
  return 0;    
}
#endif

DECLARE_GLOBAL_DATA_PTR;

static void board_usb_hw_init(void)
{
	struct udevice *dev;
	int ret;
  uchar val;
 
  
  /*Ensure power is not enabled; use as device port for now */
  atmel_pio4_set_pio_output(AT91_PIO_PORTC, 18, 0);
  //atmel_pio4_set_pio_output(AT91_PIO_PORTC, 18, 1); // ENABLED!!

  /*enable VBUS OK interrupt to detect connection*/
	ret = i2c_get_chip_for_busnum(0, 0x22, 1, &dev);
	if (ret) {
		printf("Cannot find FUSB302: %d\n", ret);
		//return 0;  
  }
  else {
    val = 0x07;
    /* power up the device */
    dm_i2c_write(dev, 0x0B, &val, 1);    
    /* enable VBUS detect interrupt */
    val = 0x80;
    dm_i2c_write(dev, 0x42, &val, 1);
#if 0      
    /* enable toggle functionality as a snk device */
  
    val = 0x25;
    dm_i2c_write(dev, 0x08, &val, 1);
#endif

    /* enable interrupts */
    val = 0x04;
    dm_i2c_write(dev, 0x06, &val, 1);
  }
  
}

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void)
{
#ifdef CONFIG_DM_VIDEO
	at91_video_show_board_info();
#endif
	return 0;
}
#endif

#ifdef CONFIG_DEBUG_UART_BOARD_INIT
static void board_uart1_hw_init(void)
{
	atmel_pio4_set_a_periph(AT91_PIO_PORTD, 2, ATMEL_PIO_PUEN_MASK);	/* URXD1 */
	atmel_pio4_set_a_periph(AT91_PIO_PORTD, 3, 0);	/* UTXD1 */

	at91_periph_clk_enable(ATMEL_ID_UART1);
}

void board_debug_uart_init(void)
{
	board_uart1_hw_init();
}
#endif

#ifdef CONFIG_BOARD_EARLY_INIT_F
int board_early_init_f(void)
{
#ifdef CONFIG_DEBUG_UART
	debug_uart_init();
#endif

	return 0;
}
#endif

int board_init(void)
{
 
	/* address of boot parameters */
	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;

#ifdef CONFIG_CMD_USB
	board_usb_hw_init();
#endif
	return 0;
}

int dram_init(void)
{
	gd->ram_size = get_ram_size((void *)CONFIG_SYS_SDRAM_BASE,
				    CONFIG_SYS_SDRAM_SIZE);
	return 0;
}

#define MAC24AA_MAC_OFFSET	0xfa

#ifdef CONFIG_MISC_INIT_R
int misc_init_r(void)
{
#ifdef CONFIG_I2C_EEPROM
	at91_set_ethaddr(MAC24AA_MAC_OFFSET);
#endif
	return 0;
}
#endif

/* SPL */
#ifdef CONFIG_SPL_BUILD
void spl_board_init(void)
{
}

static void ddrc_conf(struct atmel_mpddrc_config *ddrc)
{
	ddrc->md = (ATMEL_MPDDRC_MD_DBW_16_BITS | ATMEL_MPDDRC_MD_DDR2_SDRAM);

	ddrc->cr = (ATMEL_MPDDRC_CR_NC_COL_10 |
		    ATMEL_MPDDRC_CR_NR_ROW_13 |
		    ATMEL_MPDDRC_CR_CAS_DDR_CAS3 |
		    ATMEL_MPDDRC_CR_DIC_DS |
		    ATMEL_MPDDRC_CR_ZQ_LONG |
		    ATMEL_MPDDRC_CR_NB_4BANKS |
		    ATMEL_MPDDRC_CR_DECOD_INTERLEAVED |
		    ATMEL_MPDDRC_CR_UNAL_SUPPORTED);

	ddrc->rtr = 0x511;

	ddrc->tpr0 = ((7 << ATMEL_MPDDRC_TPR0_TRAS_OFFSET) |
		      (3 << ATMEL_MPDDRC_TPR0_TRCD_OFFSET) |
		      (3 << ATMEL_MPDDRC_TPR0_TWR_OFFSET) |
		      (9 << ATMEL_MPDDRC_TPR0_TRC_OFFSET) |
		      (3 << ATMEL_MPDDRC_TPR0_TRP_OFFSET) |
		      (4 << ATMEL_MPDDRC_TPR0_TRRD_OFFSET) |
		      (4 << ATMEL_MPDDRC_TPR0_TWTR_OFFSET) |
		      (2 << ATMEL_MPDDRC_TPR0_TMRD_OFFSET));

	ddrc->tpr1 = ((22 << ATMEL_MPDDRC_TPR1_TRFC_OFFSET) |
		      (23 << ATMEL_MPDDRC_TPR1_TXSNR_OFFSET) |
		      (200 << ATMEL_MPDDRC_TPR1_TXSRD_OFFSET) |
		      (3 << ATMEL_MPDDRC_TPR1_TXP_OFFSET));

	ddrc->tpr2 = ((2 << ATMEL_MPDDRC_TPR2_TXARD_OFFSET) |
		      (8 << ATMEL_MPDDRC_TPR2_TXARDS_OFFSET) |
		      (4 << ATMEL_MPDDRC_TPR2_TRPA_OFFSET) |
		      (4 << ATMEL_MPDDRC_TPR2_TRTP_OFFSET) |
		      (8 << ATMEL_MPDDRC_TPR2_TFAW_OFFSET));
}

void mem_init(void)
{
	struct at91_pmc *pmc = (struct at91_pmc *)ATMEL_BASE_PMC;
	struct atmel_mpddr *mpddrc = (struct atmel_mpddr *)ATMEL_BASE_MPDDRC;
	struct atmel_mpddrc_config ddrc_config;
	u32 reg;

	ddrc_conf(&ddrc_config);

	at91_periph_clk_enable(ATMEL_ID_MPDDRC);
	writel(AT91_PMC_DDR, &pmc->scer);

	reg = readl(&mpddrc->io_calibr);
	reg &= ~ATMEL_MPDDRC_IO_CALIBR_RDIV;
	reg |= ATMEL_MPDDRC_IO_CALIBR_DDR3_RZQ_55;
	reg &= ~ATMEL_MPDDRC_IO_CALIBR_TZQIO;
	reg |= ATMEL_MPDDRC_IO_CALIBR_TZQIO_(101);
	writel(reg, &mpddrc->io_calibr);

	writel(ATMEL_MPDDRC_RD_DATA_PATH_SHIFT_ONE_CYCLE,
	       &mpddrc->rd_data_path);

	ddr3_init(ATMEL_BASE_MPDDRC, ATMEL_BASE_DDRCS, &ddrc_config);

	writel(0x3, &mpddrc->cal_mr4);
	writel(64, &mpddrc->tim_cal);
}

void at91_pmc_init(void)
{
	u32 tmp;

	/*
	 * while coming from the ROM code, we run on PLLA @ 492 MHz / 164 MHz
	 * so we need to slow down and configure MCKR accordingly.
	 * This is why we have a special flavor of the switching function.
	 */
	tmp = AT91_PMC_MCKR_PLLADIV_2 |
	      AT91_PMC_MCKR_MDIV_3 |
	      AT91_PMC_MCKR_CSS_MAIN;
	at91_mck_init_down(tmp);

	tmp = AT91_PMC_PLLAR_29 |
	      AT91_PMC_PLLXR_PLLCOUNT(0x3f) |
	      AT91_PMC_PLLXR_MUL(40) |
	      AT91_PMC_PLLXR_DIV(1);
	at91_plla_init(tmp);

	tmp = AT91_PMC_MCKR_H32MXDIV |
	      AT91_PMC_MCKR_PLLADIV_2 |
	      AT91_PMC_MCKR_MDIV_3 |
	      AT91_PMC_MCKR_CSS_PLLA;
	at91_mck_init(tmp);
  

}
#endif

#ifdef CONFIG_DM_VIDEO

int at91_video_show_board_info(void)
{
	struct vidconsole_priv *priv;
	ulong dram_size;
	int i;
	u32 len = 0;
	char buf[255];
	char *corp = "JT Innovations Ltd.\n";
	char temp[32];
	struct udevice *dev, *con;
	const char *s;
	vidinfo_t logo_info;
	int ret;

	len += sprintf(&buf[len], "%s\n", U_BOOT_VERSION);
	memcpy(&buf[len], corp, strlen(corp));
	len += strlen(corp);
	len += sprintf(&buf[len], "%s CPU at %s MHz\n", get_cpu_name(),
			strmhz(temp, get_cpu_clk_rate()));

	dram_size = 0;
	for (i = 0; i < CONFIG_NR_DRAM_BANKS; i++)
		dram_size += gd->bd->bi_dram[i].size;
	len += sprintf(&buf[len], "%ld MB SDRAM\n", dram_size >> 20);

	ret = uclass_get_device(UCLASS_VIDEO, 0, &dev);
	if (ret)
		return ret;

	jtinnovations_logo_info(&logo_info);
	ret = video_bmp_display(dev, logo_info.logo_addr,
				logo_info.logo_x_offset,
				logo_info.logo_y_offset, false);
	if (ret)
		return ret;

	ret = uclass_get_device(UCLASS_VIDEO_CONSOLE, 0, &con);
	if (ret)
		return ret;

	priv = dev_get_uclass_priv(con);
	vidconsole_position_cursor(con, 0, (logo_info.logo_height +
				   priv->y_charsize - 1) / priv->y_charsize);
	for (s = buf, i = 0; i < len; s++, i++)
		vidconsole_put_char(con, *s);

	return 0;
}

void jtinnovations_logo_info(vidinfo_t *info)
{
	info->logo_width = JTINNOVATIONS_LOGO_8BPP_WIDTH;
	info->logo_height = JTINNOVATIONS_LOGO_8BPP_HEIGHT;
	info->logo_x_offset = JTINNOVATIONS_LOGO_8BPP_X_OFFSET;
	info->logo_y_offset = JTINNOVATIONS_LOGO_8BPP_X_OFFSET;
	info->logo_addr = (u_long)jtinnovations_logo_8bpp;
}
#endif

void at91_prepare_cpu_var(void)
{
	env_set("cpu", get_cpu_name());
}

