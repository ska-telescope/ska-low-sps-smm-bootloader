/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * Copyright 2018-2019 NXP
 *
 * Configuration settings for the Freescale i.MX6Q SabreSD board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __SKA_MANAGEMENT_CONFIG_H
#define __SKA_MANAGEMENT_CONFIG_H

#include "mx6_common.h"
#include "imx_env.h"

#undef CONFIG_LDO_BYPASS_CHECK

#define CONFIG_MACH_TYPE	3980
#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE	UART2_BASE
#define CONSOLE_DEV		"ttymxc1"


#define PHYS_SDRAM_SIZE		(1u * 1024 * 1024 * 1024) // 1 GB

#define CONFIG_IMX_THERMAL

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(16 * SZ_1M)




#ifdef CONFIG_CMD_SF
#define CONFIG_SF_DEFAULT_CS   0
#endif




/*BOOT and MMC CMD DEFINES*/
#define CONFIG_ENV_OVERWRITE
#ifdef CONFIG_CMD_FUSE
	#define CONFIG_MXC_OCOTP
#endif

#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_SUPPORT_EMMC_BOOT
#define CONFIG_SYS_FSL_USDHC_NUM	2
#define CONFIG_SYS_MMC_ENV_DEV		0
#define CONFIG_SYS_MMC_ENV_PART		1
#define CONFIG_MMCROOT			"/dev/mmcblk0p2"
#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC1_BASE_ADDR
#define CONFIG_SYS_SD_ENV_DEV	1
#define CONFIG_MMCROOT_SD		"/dev/mmcblk1p2"  	/* USDHC2 */
#undef is_boot_from_usb

/* FEC Config*/
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#ifdef CONFIG_DM_ETH
#define CONFIG_ETHPRIME			"eth0"
#else
#define CONFIG_ETHPRIME			"FEC"
#endif
#define CONFIG_FEC_MXC_PHYADDR		0

#define CONFIG_PHYLIB
#define CONFIG_PHY_MARVELL

/* USB Configs */
#ifdef CONFIG_CMD_USB
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0
#define CONFIG_USB_MAX_CONTROLLER_COUNT	1 /* Enabled USB controller number */
#endif




#ifdef CONFIG_MX6S
#define SYS_NOSMP "nosmp"
#else
#define SYS_NOSMP
#endif

#ifdef CONFIG_NAND_BOOT
#define MFG_NAND_PARTITION "mtdparts=8000000.nor:1m(boot),-(rootfs)\\;gpmi-nand:64m(nandboot),16m(nandkernel),16m(nanddtb),16m(nandtee),-(nandrootfs) "
#else
#define MFG_NAND_PARTITION ""
#endif

#define CONFIG_CMD_READ
#define CONFIG_SERIAL_TAG




#define CONFIG_MFG_ENV_SETTINGS \
	"mfgtool_args=setenv bootargs console=${console},${baudrate} " \
		"rdinit=/linuxrc " \
		"g_mass_storage.stall=0 g_mass_storage.removable=1 " \
		"g_mass_storage.file=/fat g_mass_storage.ro=1 " \
		"g_mass_storage.idVendor=0x066F g_mass_storage.idProduct=0x37FF "\
		"g_mass_storage.iSerialNumber=\"\" "\
		"\0" \
	"initrd_addr=0x12C00000\0" \
	"initrd_high=0xffffffff\0" \
	"echo START MFGTOOLS"		\
	"bootcmd_mfg=run mfgtool_args;bootz ${loadaddr} ${initrd_addr} ${fdt_addr};\0" \



#define CONFIG_EXTRA_ENV_SETTINGS \
	"script=boot.scr\0" \
	"image=zImage\0" \
	"console=" CONSOLE_DEV "\0" \
	"baudrate=115200\0" \
	"fdt_addr=0x18000000\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"fdt_file=ska-management.dtb\0" \
	"boot_fdt=yes\0" \
	"ip_dyn=yes\0" \
	"mmcdev="__stringify(CONFIG_SYS_MMC_ENV_DEV)"\0" \
	"mmcpart=1\0" \
	"mmcroot=" CONFIG_MMCROOT " rootwait rw\0" \
	"mmcautodetect=yes\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} " \
		"root=${mmcroot}\0" \
	"loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0" \
    "sdmmcroot=" CONFIG_MMCROOT_SD " rootwait rw\0" \
	"sddev="__stringify(CONFIG_SYS_SD_ENV_DEV)"\0" \
	"loadfdt_sd=fatload mmc ${sddev}:${mmcpart} ${fdt_addr} ${fdt_file}\0"\
	"sdargs=setenv bootargs console=${console},${baudrate} " \
		"root=${sdmmcroot}\0" \
    "sdloadimage=fatload mmc ${sddev}:${mmcpart} ${loadaddr} ${image}\0" \
	"sdboot=echo Booting from sd ...; " \
		"run sdargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt_sd; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"fi;\0" \
	"netargs=setenv bootargs console=${console},${baudrate} " \
		"root=/dev/nfs " \
	"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
		"netboot=echo Booting from net ...; " \
		"run netargs; " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${image}; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if ${get_cmd} ${fdt_addr} ${fdt_file}; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
        "fi;\0"

/*  use this define ! */

#define CONFIG_BOOTCOMMAND \
	   "echo U-BOOT from BOOTCOMMAND ...; " \
	   "if mmc dev ${sddev}; then " \
			"echo find SD ...; " \
			   "if run sdloadimage; then " \
       			   "echo run sdboot ...; " \
				   "run sdboot; " \
			   "else run netboot; " \
			   "fi; " \
	   "else " \
       	    "echo not find SD .. try mmcdev ...; " \
	        "mmc dev ${mmcdev};" \
            "if mmc rescan; then " \
       	    "echo find MMC ...; " \
				"if run loadimage; then " \
				    "run mmcboot; " \
			    "else run netboot; " \
			    "fi; " \
            "fi; " \
        "fi; " \



#define CONFIG_ARP_TIMEOUT     200UL

#define CONFIG_SYS_MEMTEST_START       0x10000000
#define CONFIG_SYS_MEMTEST_END         0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH     0x10800000

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS           1
#define PHYS_SDRAM                     MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE          PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* Environment organization */
#define CONFIG_ENV_SIZE			(8 * 1024)

#ifdef CONFIG_SATA
#define CONFIG_DWC_AHSATA
#define CONFIG_SYS_SATA_MAX_DEVICE	1
#define CONFIG_DWC_AHSATA_PORT_ID	0
#define CONFIG_DWC_AHSATA_BASE_ADDR	SATA_ARB_BASE_ADDR
#define CONFIG_LBA48
#define CONFIG_LIBATA
#endif

#ifdef CONFIG_CMD_SF
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS  0
#define CONFIG_SF_DEFAULT_SPEED 20000000
#define CONFIG_SF_DEFAULT_MODE (SPI_MODE_0)
#endif

#ifdef CONFIG_MTD_NOR_FLASH
#define CONFIG_SYS_FLASH_BASE           WEIM_ARB_BASE_ADDR
#define CONFIG_SYS_FLASH_SECT_SIZE      (128 * 1024)
#define CONFIG_SYS_MAX_FLASH_BANKS 1    /* max number of memory banks */
#define CONFIG_SYS_MAX_FLASH_SECT 256   /* max number of sectors on one chip */
#define CONFIG_SYS_FLASH_CFI            /* Flash memory is CFI compliant */
#define CONFIG_FLASH_CFI_DRIVER         /* Use drivers/cfi_flash.c */
#define CONFIG_SYS_FLASH_USE_BUFFER_WRITE /* Use buffered writes*/
#define CONFIG_SYS_FLASH_EMPTY_INFO
#define CONFIG_SYS_FLASH_CFI_WIDTH	FLASH_CFI_16BIT
#endif

#ifdef CONFIG_CMD_NAND
/* NAND flash command */
#define CONFIG_CMD_NAND_TRIMFFS

/* NAND stuff */
#define CONFIG_NAND_MXS
#define CONFIG_SYS_MAX_NAND_DEVICE     1
#define CONFIG_SYS_NAND_BASE           0x40000000
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_ONFI_DETECTION

/* DMA stuff, needed for GPMI/MXS NAND support */
#define CONFIG_APBH_DMA
#define CONFIG_APBH_DMA_BURST
#define CONFIG_APBH_DMA_BURST8
#endif

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET		(896 * 1024)
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_OFFSET              (896 * 1024)
#define CONFIG_ENV_SECT_SIZE           (64 * 1024)
#define CONFIG_ENV_SPI_BUS             CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS              CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE            CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ          CONFIG_SF_DEFAULT_SPEED
#elif defined(CONFIG_ENV_IS_IN_FLASH)
#undef CONFIG_ENV_SIZE
#define CONFIG_ENV_SIZE                        CONFIG_SYS_FLASH_SECT_SIZE
#define CONFIG_ENV_SECT_SIZE           CONFIG_SYS_FLASH_SECT_SIZE
#define CONFIG_ENV_OFFSET              (7 * CONFIG_SYS_FLASH_SECT_SIZE)
#elif defined(CONFIG_ENV_IS_IN_NAND)
#undef CONFIG_ENV_SIZE
#define CONFIG_ENV_OFFSET              (60 << 20)
#define CONFIG_ENV_SECT_SIZE           (128 << 10)
#define CONFIG_ENV_SIZE                        CONFIG_ENV_SECT_SIZE
#elif defined(CONFIG_ENV_IS_IN_SATA)
#define CONFIG_ENV_OFFSET		(896 * 1024)
#define CONFIG_SYS_SATA_ENV_DEV		0
#define CONFIG_SYS_DCACHE_OFF /* remove when sata driver support cache */
#endif

/* I2C Configs */
#ifndef CONFIG_DM_I2C
#define CONFIG_SYS_I2C
#endif
#ifdef CONFIG_CMD_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */
#define CONFIG_SYS_I2C_SPEED		400000
#endif

/* PMIC */
#ifndef CONFIG_DM_PMIC
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR 0x08
#endif

/* Framebuffer */

/*#define CONFIG_VIDEO*/
#define CONFIG_VIDEO_IPUV3
/*#define CONFIG_CFB_CONSOLE*/
/*#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE*/
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IPUV3_CLK 260000000
/* #define CONFIG_IMX_HDMI */






#if defined(CONFIG_ANDROID_SUPPORT)
#include "mx6sabreandroid_common.h"
#else
#define CONFIG_USBD_HS

#endif /* CONFIG_ANDROID_SUPPORT */
#endif                         /* __MX6QSABRE_COMMON_CONFIG_H */
