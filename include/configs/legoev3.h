/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2016,2018 David Lechner <david@lechnology.com>
 *
 * Based on da850evm.h
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Based on davinci_dvevm.h. Original Copyrights follow:
 *
 * Copyright (C) 2007 Sergey Kubushyn <ksi@koi8.net>
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * SoC Configuration
 */
#define CONFIG_SYS_EXCEPTION_VECTORS_HIGH
#define CONFIG_SYS_CLK_FREQ		clk_get(DAVINCI_ARM_CLKID)
#define CONFIG_SYS_OSCIN_FREQ		24000000
#define CONFIG_SYS_TIMERBASE		DAVINCI_TIMER0_BASE
#define CONFIG_SYS_HZ_CLOCK		clk_get(DAVINCI_AUXCLK_CLKID)
#define CONFIG_SKIP_LOWLEVEL_INIT

/*
 * Memory Info
 */
#define CONFIG_SYS_MALLOC_LEN	(0x10000 + 1*1024*1024) /* malloc() len */
#define PHYS_SDRAM_1		DAVINCI_DDR_EMIF_DATA_BASE /* DDR Start */
#define PHYS_SDRAM_1_SIZE	(64 << 20) /* SDRAM size 64MB */
#define CONFIG_MAX_RAM_BANK_SIZE (512 << 20) /* max size from SPRS586*/

/* memtest start addr */
#define CONFIG_SYS_MEMTEST_START	(PHYS_SDRAM_1 + 0x2000000)

/* memtest will be run on 16MB */
#define CONFIG_SYS_MEMTEST_END 	(PHYS_SDRAM_1 + 0x2000000 + 16*1024*1024)

#define CONFIG_NR_DRAM_BANKS	1 /* we have 1 bank of DRAM */

/*
 * Serial Driver info
 */
#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_CLK	clk_get(DAVINCI_UART2_CLKID)

#define CONFIG_SYS_SPI_CLK		clk_get(DAVINCI_SPI0_CLKID)

/*
 * I2C Configuration
 */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_DAVINCI_I2C_SPEED		400000
#define CONFIG_SYS_DAVINCI_I2C_SLAVE   10 /* Bogus, master-only in U-Boot */

/*
 * U-Boot general configuration
 */
#define CONFIG_BOOTFILE		"uImage" /* Boot file name */
#define CONFIG_SYS_CBSIZE	1024 /* Console I/O Buffer Size	*/
#define CONFIG_SYS_BARGSIZE	CONFIG_SYS_CBSIZE /* Boot Args Buffer Size */
#define CONFIG_SYS_LOAD_ADDR	(PHYS_SDRAM_1 + 0x700000)
#define CONFIG_MX_CYCLIC
#define CONFIG_MISC_INIT_R

/*
 * Linux Information
 */
#define LINUX_BOOT_PARAM_ADDR	(PHYS_SDRAM_1 + 0x100)
#define CONFIG_HWCONFIG		/* enable hwconfig */
#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_SETUP_INITRD_TAG
#define CONFIG_BOOTCOMMAND \
	"if mmc rescan; then " \
		"if run loadbootenv; then " \
			"echo Loaded env from ${bootenv};" \
			"run importbootenv;" \
		"fi;" \
		"if test -n $uenvcmd; then " \
			"echo Running uenvcmd...;" \
			"run uenvcmd;" \
		"fi;" \
		"if run loadbootscr; then " \
			"run bootscript;" \
		"fi;" \
		"if run loadimage; then " \
			"run mmcargs; " \
			"if run loadfdt; then " \
				"echo Using ${fdtfile}...;" \
				"run fdtfixup; " \
				"run fdtboot; "\
			"fi; " \
			"run mmcboot; " \
		"fi; " \
	"fi"
#define CONFIG_EXTRA_ENV_SETTINGS \
	"fdtfile=da850-lego-ev3.dtb\0" \
	"console=ttyS1,115200n8\0" \
	"bootenv=uEnv.txt\0" \
	"devtype=mmc\0" \
	"devnum=0\0" \
	"bootpart=1\0" \
	"distro_bootpart=2\0" \
	"prefix=/boot/\0" \
	"fdt_addr_r=0xc0600000\0" \
	"kernel_addr_r=0xc0007fc0\0" \
	"ramdisk_addr_r=0xc1180000\0" \
	"scriptaddr=0xc0700000\0" \
	"fwupdateboot=mw 0xFFFF1FFC 0x5555AAAA; reset\0" \
	"importbootenv=echo Importing environment...; " \
		"env import -t ${scriptaddr} ${filesize}\0" \
	"loadbootenv=load ${devtype} ${devnum}:${bootpart} ${scriptaddr} ${bootenv}\0" \
	"mmcargs=setenv bootargs console=${console} root=/dev/mmcblk0p2 rw " \
		"rootwait ${optargs}\0" \
	"mmcboot=bootm ${kernel_addr_r}\0" \
	"loadimage=fatload mmc 0 ${kernel_addr_r} uImage\0" \
	"loadfdt=fatload mmc 0 ${fdt_addr_r} ${fdtfile}\0" \
	"fdtfixup=fdt addr ${fdt_addr_r}; fdt resize; fdt chosen\0" \
	"fdtboot=bootm ${kernel_addr_r} - ${fdt_addr_r}\0" \
	"loadbootscr=load ${devtype} ${devnum}:${distro_bootpart} ${scriptaddr} ${prefix}boot.scr\0" \
	"bootscript=source ${scriptaddr}\0"

#ifdef CONFIG_CMD_BDI
#define CONFIG_CLOCKS
#endif

/* additions for new relocation code, must added to all boards */
#define CONFIG_SYS_SDRAM_BASE		0xc0000000

#define CONFIG_SYS_INIT_SP_ADDR		0x80010000

#include <asm/arch/hardware.h>

#endif /* __CONFIG_H */
