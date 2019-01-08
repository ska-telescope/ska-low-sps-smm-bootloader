/*
 * Copyright (C) 2018 Sanitas EG
 *
 * Author: Cristian Albanese <cristian.albanese@sanitaseg.com>
 *
 * Configuration settings for the Sanitas Ska-Management board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __SKA_MANAGEMENT_CONFIG_H
#define __SKA_MANAGEMENT_CONFIG_H

#include "mx6_common.h"
/*
#include <linux/sizes.h>
#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>
*/

#define CONFIG_MACH_TYPE        3980 // ???
#define CONFIG_CONSOLE_DEV      "ttymxc1"
#define CONFIG_BAUDRATE		115200
#define CONFIG_MMCROOT          "/dev/mmcblk1p2"
#define CONFIG_USE_BOOTSCRIPT

#define PHYS_SDRAM_SIZE		(1u * 1024 * 1024 * 1024) // 1 GB

#define CONFIG_MX6

#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_GPIO

#define CONFIG_MXC_UART_BASE    UART2_BASE
#define CONFIG_MXC_UART

#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
	#define CONFIG_MXC_OCOTP
#endif

/* MMC */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR	0

#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_SUPPORT_EMMC_BOOT
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION

/* SATA */
#define CONFIG_CMD_SATA
#define CONFIG_DWC_AHSATA
#define CONFIG_SYS_SATA_MAX_DEVICE	1
#define CONFIG_DWC_AHSATA_PORT_ID	0
#define CONFIG_DWC_AHSATA_BASE_ADDR	SATA_ARB_BASE_ADDR
#define CONFIG_LBA48
#define CONFIG_LIBATA

/* NETWORK */
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR	0
#undef CONFIG_PHY_LED_TXRX
#define CONFIG_PHYLIB
#define CONFIG_PHY_MARVELL

/* I2C */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1
#define CONFIG_SYS_I2C_MXC_I2C2
#define CONFIG_SYS_I2C_MXC_I2C3
#define CONFIG_SYS_I2C_SPEED		400000

#define CONFIG_CMD_EEPROM
#define CONFIG_SYS_I2C_EEPROM_ADDR              0x50 // U49
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN          1 // 1 byte Address
#define CONFIG_SYS_EEPROM_PAGE_WRITE_BITS       4 // 16 bytes Page
#define CONFIG_SYS_EEPROM_PAGE_WRITE_DELAY_MS   5 // 3ms typ
#define CONFIG_SYS_EEPROM_MAC_OFFSET			0xFA

/* Allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX              1

#define CONFIG_CMD_BMODE
#define CONFIG_CMD_BOOTZ
#define CONFIG_CMD_SETEXPR
#undef CONFIG_CMD_IMLS

#undef CONFIG_BOOTDELAY
#define CONFIG_BOOTDELAY		1

#define CONFIG_LOADADDR			0x12000000
#define CONFIG_SYS_TEXT_BASE	0x17800000

#define CONFIG_ARP_TIMEOUT     200UL

/* Miscellaneous configurable options */
#undef CONFIG_SYS_PROMPT
#define CONFIG_SYS_PROMPT              "8==> "
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2     "> "
#define CONFIG_AUTO_COMPLETE

#define CONFIG_SYS_MEMTEST_START	0x10000000
#define CONFIG_SYS_MEMTEST_END		0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH	0x10800000

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR

#define CONFIG_CMDLINE_EDITING
#define CONFIG_STACKSIZE		(128 * 1024)

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR		(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SIZE			(8 * 1024)

#define CONFIG_ENV_IS_IN_MMC
#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET		(6 * 64 * 1024)
#endif

#define CONFIG_OF_LIBFDT

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

#define CONFIG_SYS_FSL_USDHC_NUM	2
#if defined(CONFIG_ENV_IS_IN_MMC)
	#define CONFIG_SYS_MMC_ENV_DEV	0	/* SDHC1 */
#endif

/*
 * PCI express
 */
/* 
#define CONFIG_CMD_PCI
#ifdef CONFIG_CMD_PCI
	#define CONFIG_PCI
	#define CONFIG_PCI_PNP
	#define CONFIG_PCI_SCAN_SHOW
	#define CONFIG_PCIE_IMX
	#define CONFIG_PCIE_IMX_PERST_GPIO 
#endif
*/
#define CONFIG_CMD_PCI
#ifdef CONFIG_CMD_PCI
	#define CONFIG_PCI
	#define CONFIG_PCI_PNP
	#define CONFIG_PCI_SCAN_SHOW
	#define CONFIG_PCIE_IMX
	#define CONFIG_PCIE_IMX_PERST_GPIO	IMX_GPIO_NR(2, 28)
	#undef CONFIG_PCIE_IMX_POWER_GPIO
#endif

/* USB Configs */
#define CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_USB_MAX_CONTROLLER_COUNT	2
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0

/* Framebuffer */
#define CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IPUV3_CLK 260000000
/* #define CONFIG_IMX_HDMI */

#ifdef CONFIG_USE_BOOTSCRIPT
#ifdef CONFIG_CMD_SATA
	#define CONFIG_DRIVE_SATA "sata "
#else
	#define CONFIG_DRIVE_SATA
#endif

#ifdef CONFIG_CMD_MMC
	#define CONFIG_DRIVE_MMC "mmc "
#else
	#define CONFIG_DRIVE_MMC
#endif

#define CONFIG_DRIVE_TYPES CONFIG_DRIVE_MMC CONFIG_DRIVE_SATA

#define CONFIG_EXTRA_ENV_SETTINGS \
	"console=" CONFIG_CONSOLE_DEV "\0" \
	"init_bootargs=setenv bootargs console=${console},${baudrate} \0" \
	"fdt_addr=0x18000000\0" \
	"bootcmd=for dtype in " CONFIG_DRIVE_TYPES \
		"; do " \
			"for disk in 1 0 ; do ${dtype} dev ${disk} ;" \
				"for fs in fat ext2 ; do " \
					"${fs}load " \
						"${dtype} ${disk}:1 " \
						"10008000 " \
						"/ska_management.bootscript " \
						"&& source 10008000 ; " \
				"done ; " \
			"done ; " \
		"done; " \
		"echo ; echo ska_management.bootscript not found ; " \
		"setenv stdout serial\0" \
	"do_upgrade=for dtype in " CONFIG_DRIVE_TYPES \
		"; do " \
		"for disk in 1 0 ; do ${dtype} dev ${disk} ;" \
		     "for fs in fat ext2 ; do " \
				"${fs}load ${dtype} ${disk}:1 10008000 " \
					"/ska_management_upgrade.script " \
					"&& source 10008000 ; " \
			"done ; " \
		"done ; " \
	"done\0" 

#else

#error "It is supposed that you use the bootscript!!!"
#endif

#endif
