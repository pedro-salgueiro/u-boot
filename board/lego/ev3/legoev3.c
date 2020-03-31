// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2016,2018 David Lechner <david@lechnology.com>
 *
 * Based on da850evm.c
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Based on da830evm.c. Original Copyrights follow:
 *
 * Copyright (C) 2009 Nick Thompson, GE Fanuc, Ltd. <nick.thompson@gefanuc.com>
 * Copyright (C) 2007 Sergey Kubushyn <ksi@koi8.net>
 */

#include <common.h>
#include <i2c.h>
#include <spi.h>
#include <spi_flash.h>
#include <asm/arch/hardware.h>
#include <asm/arch/pinmux_defs.h>
#include <asm/io.h>
#include <asm/arch/davinci_misc.h>
#include <linux/errno.h>
#include <hwconfig.h>
#include <asm/mach-types.h>
#include <asm/setup.h>

#include <mach/da850_lowlevel.h>
#include <mach/pll_defs.h>

#ifdef CONFIG_MMC_DAVINCI
#include <mmc.h>
#include <asm/arch/sdmmc_defs.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

#define EEPROM_I2C_ADDR		0x50
#define EEPROM_REV_OFFSET	0x3F00
#define EEPROM_BDADDR_OFFSET	0x3F06

const struct pinmux_resource pinmuxes[] = {
	PINMUX_ITEM(spi0_pins_base),
	PINMUX_ITEM(spi0_pins_scs0),
	PINMUX_ITEM(uart1_pins_txrx),
	PINMUX_ITEM(i2c0_pins),
	PINMUX_ITEM(mmc0_pins),
};

const int pinmuxes_size = ARRAY_SIZE(pinmuxes);

const struct lpsc_resource lpsc[] = {
	{ DAVINCI_LPSC_SPI0 },	/* Serial Flash */
	{ DAVINCI_LPSC_UART1 },	/* console */
	{ DAVINCI_LPSC_MMC_SD },
};

const int lpsc_size = ARRAY_SIZE(lpsc);

int board_early_init_f(void)
{
	/* enable the console UART */
	writel((DAVINCI_UART_PWREMU_MGMT_FREE | DAVINCI_UART_PWREMU_MGMT_URRST |
		DAVINCI_UART_PWREMU_MGMT_UTRST),
	       &davinci_uart1_ctrl_regs->pwremu_mgmt);

	/*
	 * Power on required peripherals
	 * ARM does not have access by default to PSC0 and PSC1
	 * assuming here that the DSP bootloader has set the IOPU
	 * such that PSC access is available to ARM
	 */
	if (da8xx_configure_lpsc_items(lpsc, ARRAY_SIZE(lpsc)))
		return 1;

	return 0;
}

int board_init(void)
{
	irq_init();

	/* arch number of the board */
	/* LEGO didn't register for a unique number and uses da850evm */
	gd->bd->bi_arch_number = MACH_TYPE_DAVINCI_DA850_EVM;

	/* address of boot parameters */
	gd->bd->bi_boot_params = LINUX_BOOT_PARAM_ADDR;

	/* setup the SUSPSRC for ARM to control emulation suspend */
	writel(readl(&davinci_syscfg_regs->suspsrc) &
	       ~(DAVINCI_SYSCFG_SUSPSRC_I2C |
		 DAVINCI_SYSCFG_SUSPSRC_SPI0 | DAVINCI_SYSCFG_SUSPSRC_TIMER0 |
		 DAVINCI_SYSCFG_SUSPSRC_UART1),
	       &davinci_syscfg_regs->suspsrc);

	/* configure pinmux settings */
	if (davinci_configure_pin_mux_items(pinmuxes, ARRAY_SIZE(pinmuxes)))
		return 1;

	return 0;
}

/*
 * PLL configuration
 */
#define CONFIG_SYS_DV_CLKMODE 0

static void da850_waitloop(unsigned long loopcnt)
{
	unsigned long i;

	for (i = 0; i < loopcnt; i++)
		asm(" NOP");
}

static void set_pll_rate(unsigned long pllm, unsigned long postdiv)

{
	/* effective values are register value + 1, so adjust */
	if (pllm)
		pllm--;

	if (postdiv)
		postdiv--;

	/* Unlock PLL registers. */
	clrbits_le32(&davinci_syscfg_regs->cfgchip0, PLL_MASTER_LOCK);

	/*
	 * Set PLLENSRC '0',bit 5, PLL Enable(PLLEN) selection is controlled
	 * through MMR
	 */
	clrbits_le32(&davinci_pllc0_regs->pllctl, PLLCTL_PLLENSRC);
	/* PLLCTL.EXTCLKSRC bit 9 should be left at 0 for Freon */
	clrbits_le32(&davinci_pllc0_regs->pllctl, PLLCTL_EXTCLKSRC);

	/* Set PLLEN=0 => PLL BYPASS MODE */
	clrbits_le32(&davinci_pllc0_regs->pllctl, PLLCTL_PLLEN);

	da850_waitloop(150);

	/*
	 * Select the Clock Mode bit 8 as External Clock or On Chip
	 * Oscilator
	 */
	dv_maskbits(&davinci_pllc0_regs->pllctl, ~PLLCTL_RES_9);
	setbits_le32(&davinci_pllc0_regs->pllctl,
		     (CONFIG_SYS_DV_CLKMODE << PLLCTL_CLOCK_MODE_SHIFT));

	/* Clear PLLRST bit to reset the PLL */
	clrbits_le32(&davinci_pllc0_regs->pllctl, PLLCTL_PLLRST);

	/* Disable the PLL output */
	setbits_le32(&davinci_pllc0_regs->pllctl, PLLCTL_PLLDIS);

	/* PLL initialization sequence */
	/*
	 * Power up the PLL- PWRDN bit set to 0 to bring the PLL out of
	 * power down bit
	 */
	clrbits_le32(&davinci_pllc0_regs->pllctl, PLLCTL_PLLPWRDN);

	/* Enable the PLL from Disable Mode PLLDIS bit to 0 */
	clrbits_le32(&davinci_pllc0_regs->pllctl, PLLCTL_PLLDIS);

	/* Program the required multiplier value in PLLM */
	writel(pllm, &davinci_pllc0_regs->pllm);

	/* program the postdiv */
	writel((PLL_POSTDEN | postdiv), &davinci_pllc0_regs->postdiv);

	/*
	 * Check for the GOSTAT bit in PLLSTAT to clear to 0 to indicate that
	 * no GO operation is currently in progress
	 */
	while ((readl(&davinci_pllc0_regs->pllstat) & PLLCMD_GOSTAT) == PLLCMD_GOSTAT)
		continue;

	/*
	 * Set the GOSET bit in PLLCMD to 1 to initiate a new divider
	 * transition.
	 */
	setbits_le32(&davinci_pllc0_regs->pllcmd, PLLCMD_GOSTAT);

	/*
	 * Wait for the GOSTAT bit in PLLSTAT to clear to 0
	 * (completion of phase alignment).
	 */
	while ((readl(&davinci_pllc0_regs->pllstat) & PLLCMD_GOSTAT) == PLLCMD_GOSTAT)
		continue;

	/* Wait for PLL to reset properly. See PLL spec for PLL reset time */
	da850_waitloop(200);

	/* Set the PLLRST bit in PLLCTL to 1 to bring the PLL out of reset */
	setbits_le32(&davinci_pllc0_regs->pllctl, PLLCTL_PLLRST);

	/* Wait for PLL to lock. See PLL spec for PLL lock time */
	da850_waitloop(2400);

	/*
	 * Set the PLLEN bit in PLLCTL to 1 to remove the PLL from bypass
	 * mode
	 */
	setbits_le32(&davinci_pllc0_regs->pllctl, PLLCTL_PLLEN);
}

/* change the PLL rate before handing off to Linux */
void board_quiesce_devices(void)
{
	const char *cpufreq_str;
	unsigned long cpufreq;

	cpufreq_str = env_get("cpufreq");
	if (!cpufreq_str)
		return;

	cpufreq = simple_strtoul(cpufreq_str, NULL, 10);

	if (cpufreq >= 456)
		set_pll_rate(19, 1); /* 456MHz */
	else if (cpufreq >= 432)
		set_pll_rate(18, 2); /* 432MHz */
	else if (cpufreq >= 408)
		set_pll_rate(17, 1); /* 408MHz */
	else if (cpufreq >= 384)
		set_pll_rate(16, 1); /* 384MHz */
	else if (cpufreq >= 372)
		set_pll_rate(31, 2); /* 372MHz */
	else if (cpufreq >= 360)
		set_pll_rate(15, 1); /* 360MHz */
	else if (cpufreq >= 348)
		set_pll_rate(29, 2); /* 348MHz */
	else if (cpufreq >= 336)
		set_pll_rate(14, 1); /* 336MHz */
	else if (cpufreq >= 324)
		set_pll_rate(27, 2); /* 324MHz */
	else if (cpufreq >= 312)
		set_pll_rate(13, 1); /* 312MHz */
}

static void set_serial_number(void)
{
	u8 buf[6];
	u8 board_rev;
	u32 offset;
	char serial_str[13] = { 0 };

	if (env_get("serial#"))
		return;

	if (i2c_read(EEPROM_I2C_ADDR, EEPROM_REV_OFFSET, 2, buf, 2)) {
		printf("\nBoard revision read failed!\n");
		return;
	}

	/*
	 * Board rev 3 has bdaddr at EEPROM_REV_OFFSET. Other revisions have
	 * checksum at EEPROM_REV_OFFSET+1 to detect this.
	 */
	if ((buf[0] ^ buf[1]) == 0xFF)
		board_rev = buf[0];
	else
		board_rev = 3;

	/* Board rev 3 has bdaddr where rev should be */
	offset = (board_rev == 3) ? EEPROM_REV_OFFSET : EEPROM_BDADDR_OFFSET;

	if (i2c_read(EEPROM_I2C_ADDR, offset, 2, buf, 6)) {
		printf("\nBoard bdaddr read failed!\n");
		return;
	}

	/* use bdaddr as board serial# */
	snprintf(serial_str, sizeof(serial_str), "%02X%02X%02X%02X%02X%02X",
		 buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

	env_set("serial#", serial_str);
}

int misc_init_r(void)
{
	set_serial_number();

	return 0;
}
