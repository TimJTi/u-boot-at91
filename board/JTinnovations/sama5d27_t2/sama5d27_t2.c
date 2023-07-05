// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2017 Microchip Corporation
 *		      Wenyou.Yang <wenyou.yang@microchip.com>
 */

#include <common.h>
#include <debug_uart.h>
//#include <fdtdec.h>
#include <init.h>
//#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/arch/at91_common.h>
#include <asm/arch/atmel_pio4.h>
//#include <asm/arch/atmel_mpddrc.h>
#include <asm/arch/clk.h>
#include <asm/arch/gpio.h>
#include <asm/arch/sama5d2.h>
#include <linux/delay.h>

#include <env.h>
#include <dm.h>
//#include <net.h>
#include <i2c.h>
#include <spi.h>
#include <spi_flash.h>
#include <atmel_lcd.h>
#include <version.h>
#include <video.h>
#include <video_console.h>
#include <vsprintf.h>
#include "jti_logo_8bpp.h"
//#include <asm/arch/atmel_usba_udc.h>

#define USE_ALS
#define ALS_PERCENT_DIFF 130
#define SHOW_MEM_INFO

DECLARE_GLOBAL_DATA_PTR;

void jtinnovations_logo_info(vidinfo_t *info);
int video_show_board_info(void);

static void board_usb_hw_init(void)
{
	atmel_pio4_set_pio_output(AT91_PIO_PORTC, 18, 0);
}

#if defined(CONFIG_ARCH_MISC_INIT)
int arch_misc_init(void)
{
	struct udevice *alsdev1, *alsdev2, *actdev;
	int ret;
  uchar val;
  unsigned int alsL, alsR;
  bool boot_intL = false;
  bool boot_intR = false;

  /* set up ACT8945A */
	ret = i2c_get_chip_for_busnum(0, 0x5B, 1, &actdev);
	if (ret) {
		printf("Cannot find ACT8945A: %d\n", ret);
		return 0;  
  }
  else
	{
    /* set VDD_LED to 3V3 */
    val = 0x39;    
    dm_i2c_write(actdev, 0x60, &val, 1);
    /* enable VDD_LED */
    val = 0x80;    
    dm_i2c_write(actdev, 0x61, &val, 1);
    /*disable VDD for legacy GPS antenna*/
    val = 0;
    dm_i2c_write(actdev, 0x65, &val, 1);

    /* enable vdd_fuse */
    val = 0x8A;    
    dm_i2c_write(actdev, 0x51, &val, 1);
		
	  /* GPS antenna volts ON */
    /* val = 0x8A;    
    dm_i2c_write(actdev, 0x65, &val, 1); */
    
    mdelay(150);

   
    /* now set up and read ambient light sensor */
#ifdef USE_ALS
		uchar buf[4];
    ret = i2c_get_chip_for_busnum(0, 0x53, 1, &alsdev1);
    if (ret)
		{
      printf("Cannot find Right ADPS9922: %d\n", ret);
      return 0;
    }
    ret = i2c_get_chip_for_busnum(1, 0x53, 1, &alsdev2);
    if (ret)
		{
      printf("Cannot find Left ADPS9922: %d\n", ret);
      return 0;
    }    
    else
      {
        val = 0x02; /* Enable ALS */
        dm_i2c_write(alsdev1, 0x0, &val, 1);
        dm_i2c_write(alsdev2, 0x0, &val, 1);
        mdelay(200);
        dm_i2c_read(alsdev1, 0x0d, &buf[0], 3);
        alsR = (((unsigned short int) buf[2] << 16) +
                ((unsigned short int) buf[1] << 8) +
                ((unsigned short int) buf[0]));        
        dm_i2c_read(alsdev2, 0x0d, &buf[0], 3);

        alsL = (((unsigned short int) buf[2] << 16) +
                ((unsigned short int) buf[1] << 8) +
                ((unsigned short int) buf[0]));
        //printf("als Left: %d\t Right %d\n", alsL, alsR);
        if ((alsL < 50) && 
           ((alsL * ALS_PERCENT_DIFF) < (ALS_PERCENT_DIFF * alsR)))
           {
              printf("Left button boot interrupt detected\n");
              boot_intL = true;
           }
        else if ((alsR < 50) &&
                ((alsR * ALS_PERCENT_DIFF) < (ALS_PERCENT_DIFF * alsL)))
          {
            printf("Right button boot interrupt detected\n");
            boot_intR = true;
          }

        if (boot_intR)
          {
            env_set("bootcmd", "\
            echo Boot interrupted via ambient light sensor method; \
            setcurs 0 10; \
            lcdputs 'Boot interrupted, via light sensor.'; \
            setcurs 0 11;\
            lcdputs 'This is where uBoot dfu etc will go';");
          }
        else  
			    {
            env_set("bootcmd", "\
              lcdputs 'loading main code'; \
              sf probe 1:0; \
              sf read 0x20008000 0xc0000 0x400000; \
              go 0x200082e0;");
          }
      }
#else
		env_set("bootcmd", "\
		echo Trying to load from flash...; 	\
		lcdputs loading.; \
		sf probe 1:0; \
		sf read 0x20008000 0xC0000 0x200000; \
		cls; \
		setcurs 0 0; \
		lcdputs 'Running nuttx now'; \
		go 0x20008040;");
    //<name> ram <offset> <size>  raw access to sf device
    //env_set("dfu_alt_info", "app raw 0 0x8000000;");
    //env_set("dfu_alt_info_ram", "dummy.bin ram 0x24000000 0x100;");
    //env_set("ethact", "usb_ether");

#endif		
  }
  
  return 0;    
}
#endif


#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void)
{
#ifdef CONFIG_DM_VIDEO
	video_show_board_info();
#endif
#if 0
#ifdef CONFIG_USB_GADGET
	int ret;
  struct udevice *dev;
	ret = uclass_get_device(UCLASS_USB_GADGET_GENERIC, 1, &dev);
	if (ret) {
		printf("%s: Cannot find USB device\n", __func__);
		}
#endif
#endif
#ifdef CONFIG_USB_ETHER
		usb_ether_init();
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
	return 0;
}
#endif

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = gd->bd->bi_dram[0].start + 0x100;

	board_usb_hw_init();
#ifdef CONFIG_USB_GADGET_ATMEL_USBA
	at91_udp_hw_init();
	usba_udc_probe(&pdata);
#endif

	return 0;
}

int dram_init_banksize(void)
{
	return fdtdec_setup_memory_banksize();
}

int dram_init(void)
{
	return fdtdec_setup_mem_size_base();
}


#ifdef CONFIG_MISC_INIT_R
int misc_init_r(void)
{
	return 0;
}
#endif

#ifdef CONFIG_DM_VIDEO

int video_show_board_info(void)
{
	struct vidconsole_priv *priv;

	int i;
	u32 len = 0;
	char buf[255];
	char *corp = "JT Innovations Ltd.\n";
	char temp[32];
	struct udevice *dev, *con;
	const char *s;
	vidinfo_t logo_info;
	int ret;

	if (ret)
		return ret;

	len += sprintf(&buf[len], "%s\n", U_BOOT_VERSION);
	memcpy(&buf[len], corp, strlen(corp));
	len += strlen(corp);
	len += sprintf(&buf[len], "%s CPU at %s MHz\n", get_cpu_name(),
			strmhz(temp, get_cpu_clk_rate()));

#ifdef SHOW_MEM_INFO
	ulong dram_size;
  struct spi_flash *flash1, *flash2;

	dram_size = 0;
	for (i = 0; i < CONFIG_NR_DRAM_BANKS; i++)
		dram_size += gd->bd->bi_dram[i].size;
	len += sprintf(&buf[len], "%ld MiB SDRAM", dram_size >> 20);

  flash1 = spi_flash_probe(1, 0, 66000000, SPI_MODE_0);
  if (flash1)  {
    len += sprintf(&buf[len], ", %dMiB program memory", flash1->size >> 20);
  }
  else
    printf("problem reading program flash\n");
  flash2 = spi_flash_probe(0, 1, 50000000, SPI_MODE_0);
  if (flash2)  {
    len += sprintf(&buf[len], ", %dMiB log memory", flash2->size >> 20);
  }
  else
    printf("problem reading flash\n");
#endif    
  len += sprintf(&buf[len], "\n");

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
		    ATMEL_MPDDRC_CR_NB_8BANKS |
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