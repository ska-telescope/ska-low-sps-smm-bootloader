/*
 * Copyright (C) 2012-2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/mach-imx/spi.h>
#include <linux/errno.h>
#include <asm/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/video.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <input.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include "../common/pfuze.h"
#include <usb.h>
#include <usb/ehci-ci.h>
#include <asm/arch/mx6-ddr.h>
#if defined(CONFIG_MX6DL) && defined(CONFIG_MXC_EPDC)
#include <lcd.h>
#include <mxc_epdc_fb.h>
#endif
#ifdef CONFIG_SATA
#include <asm/mach-imx/sata.h>
#endif
#ifdef CONFIG_FSL_FASTBOOT
#include <fsl_fastboot.h>
#ifdef CONFIG_ANDROID_RECOVERY
#include <recovery.h>
#endif
#endif /*CONFIG_FSL_FASTBOOT*/


DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
		      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)


#define EPDC_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_SPEED_MED |	\
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define OTG_ID_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)


#define I2C_PMIC	1

#define I2C_PAD MUX_PAD_CTRL(I2C_PAD_CTRL)

#define DISP0_PWR_EN	IMX_GPIO_NR(1, 21)

#define KEY_VOL_UP	IMX_GPIO_NR(1, 4)

extern u32 reset_cause;

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

/*UART1*/
static iomux_v3_cfg_t const uart1_pads[] = {
	IOMUX_PADS(PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

/* UART 2 */
static iomux_v3_cfg_t const uart2_pads[] = {
	IOMUX_PADS(PAD_GPIO_7__UART2_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_GPIO_8__UART2_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};


/*Ethernet RGMII*/
static iomux_v3_cfg_t const enet_pads[] = {
		IOMUX_PADS(PAD_KEY_COL1__ENET_MDIO        | MUX_PAD_CTRL(ENET_PAD_CTRL)),
		IOMUX_PADS(PAD_KEY_COL2__ENET_MDC         | MUX_PAD_CTRL(ENET_PAD_CTRL)),
		IOMUX_PADS(PAD_GPIO_16__ENET_REF_CLK      | MUX_MODE_SION | MUX_PAD_CTRL(ENET_PAD_CTRL| PAD_CTL_SRE_FAST)),
		IOMUX_PADS(PAD_ENET_REF_CLK__ENET_TX_CLK  | MUX_PAD_CTRL(ENET_PAD_CTRL)),
		IOMUX_PADS(PAD_RGMII_TXC__RGMII_TXC       | MUX_PAD_CTRL(ENET_PAD_CTRL)),
		IOMUX_PADS(PAD_RGMII_TD0__RGMII_TD0       | MUX_PAD_CTRL(ENET_PAD_CTRL)),
		IOMUX_PADS(PAD_RGMII_TD1__RGMII_TD1       | MUX_PAD_CTRL(ENET_PAD_CTRL)),
		IOMUX_PADS(PAD_RGMII_TD2__RGMII_TD2       | MUX_PAD_CTRL(ENET_PAD_CTRL)),
		IOMUX_PADS(PAD_RGMII_TD3__RGMII_TD3       | MUX_PAD_CTRL(ENET_PAD_CTRL)),
		IOMUX_PADS(PAD_RGMII_TX_CTL__RGMII_TX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL)),
		IOMUX_PADS(PAD_RGMII_RXC__RGMII_RXC       | MUX_PAD_CTRL(ENET_PAD_CTRL)),
		IOMUX_PADS(PAD_RGMII_RD0__RGMII_RD0       | MUX_PAD_CTRL(ENET_PAD_CTRL)),
		IOMUX_PADS(PAD_RGMII_RD1__RGMII_RD1       | MUX_PAD_CTRL(ENET_PAD_CTRL)),
		IOMUX_PADS(PAD_RGMII_RD2__RGMII_RD2       | MUX_PAD_CTRL(ENET_PAD_CTRL)),
		IOMUX_PADS(PAD_RGMII_RD3__RGMII_RD3       | MUX_PAD_CTRL(ENET_PAD_CTRL)),
		IOMUX_PADS(PAD_RGMII_RX_CTL__RGMII_RX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL)),
};




/* eMMC */
static iomux_v3_cfg_t const usdhc1_pads[] = {
		IOMUX_PADS(PAD_SD1_CLK__SD1_CLK	  | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
		IOMUX_PADS(PAD_SD1_CMD__SD1_CMD	  | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
		IOMUX_PADS(PAD_SD1_DAT0__SD1_DATA0	  | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
		IOMUX_PADS(PAD_SD1_DAT1__SD1_DATA1	  | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
		IOMUX_PADS(PAD_SD1_DAT2__SD1_DATA2	  | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
		IOMUX_PADS(PAD_SD1_DAT3__SD1_DATA3	  | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
		IOMUX_PADS(PAD_NANDF_D0__SD1_DATA4	  | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
		IOMUX_PADS(PAD_NANDF_D1__SD1_DATA5	  | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
		IOMUX_PADS(PAD_NANDF_D2__SD1_DATA6	  | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
		IOMUX_PADS(PAD_NANDF_D3__SD1_DATA7	  | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
		IOMUX_PADS(PAD_GPIO_1__GPIO1_IO01	  | MUX_PAD_CTRL(NO_PAD_CTRL)), /* !CD */
		IOMUX_PADS(PAD_GPIO_9__GPIO1_IO09	  | MUX_PAD_CTRL(NO_PAD_CTRL)), /* !WP */
		IOMUX_PADS(PAD_NANDF_ALE__GPIO6_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL)),  /* !Reset */
};




/* Panel uSD Card */
static iomux_v3_cfg_t const usdhc2_pads[] = {
		IOMUX_PADS(PAD_SD2_CLK__SD2_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
		IOMUX_PADS(PAD_SD2_CMD__SD2_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
		IOMUX_PADS(PAD_SD2_DAT0__SD2_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
		IOMUX_PADS(PAD_SD2_DAT1__SD2_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
		IOMUX_PADS(PAD_SD2_DAT2__SD2_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
		IOMUX_PADS(PAD_SD2_DAT3__SD2_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
		IOMUX_PADS(PAD_GPIO_4__GPIO1_IO04	| MUX_PAD_CTRL(NO_PAD_CTRL)), /* !CD */
		IOMUX_PADS(PAD_GPIO_2__GPIO1_IO02	| MUX_PAD_CTRL(NO_PAD_CTRL)), /* !WP */
};


static iomux_v3_cfg_t const bl_pads[] = {
	IOMUX_PADS(PAD_SD1_DAT3__GPIO1_IO21 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};


static void fec_phy_reset(void)
{
	/* Reset AR8031 PHY */
	/*
	gpio_request(IMX_GPIO_NR(1, 25), "ENET PHY Reset");
	gpio_direction_output(IMX_GPIO_NR(1, 25) , 0);
	mdelay(10);
	gpio_set_value(IMX_GPIO_NR(1, 25), 1);
	udelay(100);
	*/
	printf("Reset PHY...\n");
	gpio_request(IMX_GPIO_NR(4, 29), "ENET PHY Reset");
	gpio_direction_output(IMX_GPIO_NR(4, 29) , 0);
	mdelay(10);
	gpio_set_value(IMX_GPIO_NR(4, 29), 1);
	udelay(100);
}

static void setup_iomux_enet(void)
{
	unsigned int reg;
	SETUP_IOMUX_PADS(enet_pads);
	fec_phy_reset();
	reg=readl(0x20E0004); //read pinmux IOMUXC_GPR1
	writel(reg|0x00200000,0x20E0004); //set ENET_CLK_SEL bit
}

#ifdef CONFIG_MXC_SPI
static iomux_v3_cfg_t const ecspi1_pads[] = {
	IOMUX_PADS(PAD_KEY_COL0__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_COL1__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_ROW0__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_ROW1__GPIO4_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static void setup_spi(void)
{
	SETUP_IOMUX_PADS(ecspi1_pads);
	gpio_request(IMX_GPIO_NR(4, 9), "ECSPI1 CS");
}

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 0) ? (IMX_GPIO_NR(4, 9)) : -1;
}
#endif

/*
static void enable_backlight(void)
{
	SETUP_IOMUX_PADS(bl_pads);
	gpio_request(DISP0_PWR_EN, "Display Power Enable");
	gpio_direction_output(DISP0_PWR_EN, 1);
}
*/



static void ethaddr_init(void)
{
	u8 mac[6];
	char mstr[20];

    int ret = 0;
    int retry=5;
    struct udevice *dev1;


	printf("Procedure: ethaddr_init...\n");

	ret = i2c_get_chip_for_busnum(0, 0x51,1, &dev1);
	if (ret) {
		printf("%s: Cannot find udev for a bus %d\n", __func__,0);
		return CMD_RET_FAILURE;
	}


	while(retry > 0)
	{
		if (dm_i2c_read(dev1, 0xfa, mac, 6))
		{	printf("i2c_read failed\n");
			retry--;
		}
		else
			break;
	}

    sprintf(mstr, "%0X:%0X:%0X:%0X:%0X:%0X",mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	printf("ethaddr set to: %s\n", mstr);
	env_set("ethaddr", mstr);


}


static void enable_backlight(void)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT;

	writel(reg, &iomux->gpr[2]);
}

static void enable_rgb(struct display_info_t const *dev)
{
	//SETUP_IOMUX_PADS(rgb_pads);
	enable_backlight();
}

static void enable_lvds(struct display_info_t const *dev)
{
	enable_backlight();
}


/* I2C1 */
static struct i2c_pads_info i2c1_pad_info = {
	.scl = {
		.i2c_mode = MX6_PAD_EIM_D21__I2C1_SCL | I2C_PAD,
		.gpio_mode = MX6_PAD_EIM_D21__GPIO3_IO21 | I2C_PAD,
		.gp = IMX_GPIO_NR(3, 21)
	},
	.sda = {
		.i2c_mode = MX6_PAD_EIM_D28__I2C1_SDA | I2C_PAD,
		.gpio_mode = MX6_PAD_EIM_D28__GPIO3_IO28 | I2C_PAD,
		.gp = IMX_GPIO_NR(3, 28)
	}
};


#ifdef CONFIG_PCIE_IMX
iomux_v3_cfg_t const pcie_pads[] = {
	IOMUX_PADS(PAD_EIM_D19__GPIO3_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL)),	/* POWER */
	IOMUX_PADS(PAD_GPIO_17__GPIO7_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL)),	/* RESET */
};

static void setup_pcie(void)
{
	SETUP_IOMUX_PADS(pcie_pads);
	gpio_request(CONFIG_PCIE_IMX_POWER_GPIO, "PCIE Power Enable");
	gpio_request(CONFIG_PCIE_IMX_PERST_GPIO, "PCIE Reset");
}
#endif

iomux_v3_cfg_t const di0_pads[] = {
	IOMUX_PADS(PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK),	/* DISP0_CLK */
	IOMUX_PADS(PAD_DI0_PIN2__IPU1_DI0_PIN02),		/* DISP0_HSYNC */
	IOMUX_PADS(PAD_DI0_PIN3__IPU1_DI0_PIN03),		/* DISP0_VSYNC */
};

static void setup_iomux_uart(void)
{
	SETUP_IOMUX_PADS(uart1_pads);
	SETUP_IOMUX_PADS(uart2_pads);
	printf("setup iomux \r\n");
}




#define CMC_SRS_TAMPER                    (1 << 31)
#define CMC_SRS_SECURITY                  (1 << 30)
#define CMC_SRS_TZWDG                     (1 << 29)
#define CMC_SRS_JTAG_RST                  (1 << 28)
#define CMC_SRS_CORE1                     (1 << 16)
#define CMC_SRS_LOCKUP                    (1 << 15)
#define CMC_SRS_SW                        (1 << 14)
#define CMC_SRS_WDG                       (1 << 13)
#define CMC_SRS_PIN_RESET                 (1 << 8)
#define CMC_SRS_WARM                      (1 << 4)
#define CMC_SRS_HVD                       (1 << 3)
#define CMC_SRS_LVD                       (1 << 2)
#define CMC_SRS_POR                       (1 << 1)
#define CMC_SRS_WUP                       (1 << 0)
static char * get_reset_cause_local()
{

	u32 cause;

	u32 *reg_srsr = (u32 *)(SRC_BASE_ADDR + 0x8);

	cause = readl(reg_srsr);


	//cause = readl(&src_regs->srsr);
	printf("Reset cause reg %x\n",cause);
	printf("Reset cause from ext variable: %x\n",reset_cause);
	switch (reset_cause) {
	case 0x00001:
	case 0x00011:
		return "POR";
	case 0x00004:
		return "CSU";
	case 0x00008:
		return "IPP USER";
	case 0x00010:
		return "WDOG";
	case 0x00020:
		return "JTAG HIGH-Z";
	case 0x00040:
		return "JTAG SW";
	case 0x00080:
		return "WDOG3";
	case 0x00100:
		return "TEMPSENSE";
	case 0x10000:
		return "WARM BOOT";
	default:
		return "unknown reset";
	}

}





static void setup_weim(void)
{

	struct weim  *pweim = (struct weim *)WEIM_BASE_ADDR;
	int reg;

	setbits_le32(CCM_CCGR6, MXC_CCM_CCGR6_EMI_SLOW_MASK); // Enable Clock
/*
	pweim->cs0gcr1 = 0x00610089; // Set registers for CS0
	pweim->cs0gcr2 = 0x00001000;
	pweim->cs0rcr1 = 0x1c022000;
	pweim->cs0rcr2 = 0x00000000;
	pweim->cs0wcr1 = 0x5c041041;
	pweim->cs0wcr2 = 0x00000000;
*/
	pweim->cs0gcr1 = 0x0061308F; // Set registers for CS0 BCD 4 SWR 1 SRD 1
	pweim->cs0gcr2 = 0x00001000;
	pweim->cs0rcr1 = 0x01012000;
	pweim->cs0rcr2 = 0x00000000;
	pweim->cs0wcr1 = 0x41041041;
	pweim->cs0wcr2 = 0x00000000;

	reg=readl(&pweim->wcr);

	writel((reg|0x26),&pweim->wcr);

}

#define FPGA_REGS_BA 0x8000000
struct fpga_regs{
	u32 build_info;
	u32 build_date;
	u32 fw_version;
};

struct switch_res{
	u32 sw_res_reg;
};

struct hw_rev{
	u32 hw_rev_reg;
};

struct boot_sel{
	u32 boot_sel_reg;
};

struct user_leds{
	u32 Led_User_K;
	u32 Led_User_A;
	u32 Led_Mode;
};

static void get_weim_info()
{
    int ret = 0;
    int retry = 5;
    u8 bootsel=0;
    u8 kern_part = 0;
    u8 fs_part = 0;
    struct udevice *dev1;
	u32 reg,reg1,reg2,reg3,hwrev;
	u32 hwrev_add=FPGA_REGS_BA+0x124;
	u32 sw_reset=FPGA_REGS_BA+0x700;
	u32 mcu_reset_reg=FPGA_REGS_BA+0x900;
	struct fpga_regs *pfpga_regs=(struct fpga_regs *)FPGA_REGS_BA;
	struct switch_res *pswitch_res=(struct switch_res *)(FPGA_REGS_BA+0x700);
	struct hw_rev *phw_rev=(struct hw_rev *)(FPGA_REGS_BA+0x124);
	struct boot_sel *pboot_sel=(struct boot_sel *)(FPGA_REGS_BA+0x10050);
	struct user_leds *puser_leds=(struct user_leds *)(FPGA_REGS_BA+0x400);

	//printf("Stop MCU\n");
	//writel(0x0,mcu_reset_reg);
	printf("Trying to read from EIM \n");
	reg=readl(&pfpga_regs->build_info);
	reg1=readl(&pfpga_regs->build_date);
	reg3=readl(&pfpga_regs->fw_version);
	printf( "FPGA_INFO %x %x\n",reg,reg1);
	printf("FW Ver: %x \n",reg3);
	hwrev=readl(&phw_rev->hw_rev_reg);
	printf("HW REV: %x \n",hwrev);
	/* add here SW1  reset sequence*/
	printf("Resetting switch... \n");
	writel(0x0,&pswitch_res->sw_res_reg);
	reg=readl(&pswitch_res->sw_res_reg);
	printf("SW_RESET Reg value %x\n",reg);
	mdelay(200);
	writel(0x1,&pswitch_res->sw_res_reg);
	mdelay(600);
	printf("Resetting complete \n");
	printf("Configuring SHDN Pin of UPS\n");
	gpio_request(IMX_GPIO_NR(4, 30), "UPS SHDN");
	gpio_direction_output(IMX_GPIO_NR(4, 30) , 0);
	mdelay(100);
	gpio_set_value(IMX_GPIO_NR(4, 30), 0);

	printf("Configuring BOOTSEL  \n");
	printf("fwver_check %x\n", (reg3 & 0xbe7a0000));
	if((u32)(reg3 & 0xbe7a0000)== 0xbe7a0000)
	{
		ret = i2c_get_chip_for_busnum(0, 0x50,1, &dev1);
		while(retry > 0)
		{
			if (dm_i2c_read(dev1, 0x70, &bootsel, 1))
			{	printf("i2c_read failed\n");
				retry--;
			}
			else
				break;
		}
		printf("BOOTSEL from EEP %x\n", bootsel);
	}
	else
	{
		reg=readl(&pboot_sel->boot_sel_reg);
		bootsel=reg&0xff;
		printf("BOOTSEL from CPLD %x\n", bootsel);
		printf("Setting Led_Mode to Red Heartbeat \n");
		writel(0x2000,&puser_leds->Led_Mode);
	}
	mdelay(100);
	kern_part=bootsel&0x1;
	fs_part=(bootsel&0x2)>>1;

	if( kern_part != 0 )
	{
		//gpio_set_value(IMX_GPIO_NR(5, 5), 1);
		printf("Set krn partition 1\n");
		env_set("krn_part", "1");
	}
	else
	{
		//gpio_set_value(IMX_GPIO_NR(5, 5), 0);
		printf("Set krn partition 0\n");
		env_set("krn_part", "0");
	}

	if( fs_part != 0 )
	{
		//gpio_set_value(IMX_GPIO_NR(5, 5), 1);
		printf("Set FS partition 1\n");
		env_set("fs_part", "1");
	}
	else
	{
		//gpio_set_value(IMX_GPIO_NR(5, 5), 0);
		printf("Set FS partition 0\n");
		env_set("fs_part", "0");
	}
	printf("Configuration Complete\n");

}


#if defined(CONFIG_MX6DL) && defined(CONFIG_MXC_EPDC)
static iomux_v3_cfg_t const epdc_enable_pads[] = {
	IOMUX_PADS(PAD_EIM_A16__EPDC_DATA00	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA10__EPDC_DATA01	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA12__EPDC_DATA02	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA11__EPDC_DATA03	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_LBA__EPDC_DATA04	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_EB2__EPDC_DATA05	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_CS0__EPDC_DATA06	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_RW__EPDC_DATA07	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_A21__EPDC_GDCLK	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_A22__EPDC_GDSP	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_A23__EPDC_GDOE	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_A24__EPDC_GDRL	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D31__EPDC_SDCLK_P	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D27__EPDC_SDOE	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA1__EPDC_SDLE	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_EB1__EPDC_SDSHR	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA2__EPDC_BDR0	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA4__EPDC_SDCE0	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA5__EPDC_SDCE1	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_DA6__EPDC_SDCE2	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
};

static iomux_v3_cfg_t const epdc_disable_pads[] = {
	IOMUX_PADS(PAD_EIM_A16__GPIO2_IO22),
	IOMUX_PADS(PAD_EIM_DA10__GPIO3_IO10),
	IOMUX_PADS(PAD_EIM_DA12__GPIO3_IO12),
	IOMUX_PADS(PAD_EIM_DA11__GPIO3_IO11),
	IOMUX_PADS(PAD_EIM_LBA__GPIO2_IO27),
	IOMUX_PADS(PAD_EIM_EB2__GPIO2_IO30),
	IOMUX_PADS(PAD_EIM_CS0__GPIO2_IO23),
	IOMUX_PADS(PAD_EIM_RW__GPIO2_IO26),
	IOMUX_PADS(PAD_EIM_A21__GPIO2_IO17),
	IOMUX_PADS(PAD_EIM_A22__GPIO2_IO16),
	IOMUX_PADS(PAD_EIM_A23__GPIO6_IO06),
	IOMUX_PADS(PAD_EIM_A24__GPIO5_IO04),
	IOMUX_PADS(PAD_EIM_D31__GPIO3_IO31),
	IOMUX_PADS(PAD_EIM_D27__GPIO3_IO27),
	IOMUX_PADS(PAD_EIM_DA1__GPIO3_IO01),
	IOMUX_PADS(PAD_EIM_EB1__GPIO2_IO29),
	IOMUX_PADS(PAD_EIM_DA2__GPIO3_IO02),
	IOMUX_PADS(PAD_EIM_DA4__GPIO3_IO04),
	IOMUX_PADS(PAD_EIM_DA5__GPIO3_IO05),
	IOMUX_PADS(PAD_EIM_DA6__GPIO3_IO06),
};
#endif

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC1_BASE_ADDR},
	{USDHC2_BASE_ADDR},
	/*{USDHC4_BASE_ADDR},*/
};


/*
#define USDHC2_CD_GPIO	IMX_GPIO_NR(2, 2)
#define USDHC3_CD_GPIO	IMX_GPIO_NR(2, 0)
*/
#define USDHC1_CD_GPIO	IMX_GPIO_NR(1, 1)
#define USDHC2_CD_GPIO	IMX_GPIO_NR(1, 4)



int board_mmc_get_env_dev(int devno)
{
	return devno - 1;
}

int mmc_map_to_kernel_blk(int devno)
{
	return devno + 1;
}

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	//case USDHC2_BASE_ADDR:
	case USDHC1_BASE_ADDR:
		//ret = !gpio_get_value(USDHC2_CD_GPIO);
		ret = gpio_get_value(USDHC1_CD_GPIO);
		break;
	case USDHC2_BASE_ADDR:
		ret = gpio_get_value(USDHC2_CD_GPIO)?0:1;
		break;
/*
	case USDHC3_BASE_ADDR:
		ret = !gpio_get_value(USDHC3_CD_GPIO);
		break;
	*/
	case USDHC4_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC4 is always present */
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
#ifndef CONFIG_SPL_BUILD
	int ret;
	int i;

	printf("init_mmc: start for\n");

	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
			switch (i) {
			case 0:
				SETUP_IOMUX_PADS(usdhc1_pads);
				gpio_request(USDHC1_CD_GPIO, "USDHC1 CD");
				gpio_direction_input(USDHC1_CD_GPIO);
				usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
				break;
			case 1:
				SETUP_IOMUX_PADS(usdhc2_pads);
				gpio_request(USDHC2_CD_GPIO, "USDHC2 CD");
				gpio_direction_input(USDHC2_CD_GPIO);
				usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
				break;
			default:
				printf("Warning: you configured more USDHC controllers"
				       "(%d) then supported by the board (%d)\n",
				       i + 1, CONFIG_SYS_FSL_USDHC_NUM);
				return -EINVAL;
			}


	ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret)
			return ret;
	}

	return 0;
#else
	struct src *psrc = (struct src *)SRC_BASE_ADDR;
	unsigned reg = readl(&psrc->sbmr1) >> 11;
	/*
	 * Upon reading BOOT_CFG register the following map is done:
	 * Bit 11 and 12 of BOOT_CFG register can determine the current
	 * mmc port
	 * 0x1                  SD1
	 * 0x2                  SD2
	 * 0x3                  SD4
	 */

	switch (reg & 0x3) {
	case 0x1:
		SETUP_IOMUX_PADS(usdhc2_pads);
		usdhc_cfg[0].esdhc_base = USDHC2_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	case 0x2:
		SETUP_IOMUX_PADS(usdhc3_pads);
		usdhc_cfg[0].esdhc_base = USDHC3_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	case 0x3:
		SETUP_IOMUX_PADS(usdhc4_pads);
		usdhc_cfg[0].esdhc_base = USDHC4_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	}

	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
#endif
}
#endif

static int ar8031_phy_fixup(struct phy_device *phydev)
{
	unsigned short val;

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	if (!is_mx6dqp()) {
		phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x7);
		phy_write(phydev, MDIO_DEVAD_NONE, 0xe, 0x8016);
		phy_write(phydev, MDIO_DEVAD_NONE, 0xd, 0x4007);

		val = phy_read(phydev, MDIO_DEVAD_NONE, 0xe);
		val &= 0xffe3;
		val |= 0x18;
		phy_write(phydev, MDIO_DEVAD_NONE, 0xe, val);
	}

	/* set the IO voltage to 1.8v */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	/* introduce tx clock delay */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x5);
	val = phy_read(phydev, MDIO_DEVAD_NONE, 0x1e);
	val |= 0x0100;
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, val);

	return 0;
}

#define EX_PHY_CFG
int board_phy_config(struct phy_device *phydev)
{
	/*ar8031_phy_fixup(phydev);*/

	/*
	if (phydev->drv->config)
		phydev->drv->config(phydev);
	*/
	char *reset_src;

	printf("NO BORD PHY CONFIG EXECUTION\n");
#ifdef EX_PHY_CFG
	printf("board_phy_config\n");
	if (phydev->drv->config)
	{
		phydev->drv->config(phydev);


		mdio_list_devices();
		reset_src=get_reset_cause_local();
		printf("Detected reset cause: %s\n",reset_src);

		//printf("Configuration for POR detected\n");
		phy_write(phydev, 0, 0x16, 0x8010);
		phy_write(phydev, 0, 0x0, 0x7);
		phy_write(phydev, 0, 0x1, 0x10);
		phy_write(phydev, 0, 0x1, 0xE03E);

		//configuration for SPF switch port
		phy_write(phydev, 9, 0x0, 0x9);
		/* P9*/
		phy_write(phydev,0x1c,25,0xF054);
		udelay(100);
		phy_write(phydev,0x1c,24,0x8124);
		udelay(100);
		phy_write(phydev,0x1c,25,0x400c);
		udelay(100);
		phy_write(phydev,0x1c,24,0x8524);
		udelay(100);
		phy_write(phydev,0x1c,25,0xF054);
		udelay(100);
		phy_write(phydev,0x1c,24,0x8124);
		udelay(100);
		phy_write(phydev,0x1c,25,0x4000);
		udelay(100);
		phy_write(phydev,0x1c,24,0x8524);
		/*Start configuring ports for traffic*/
		/*Clear power down bit and reset SERDES P9*/

		phy_write(phydev,0x1c,25,0x2000);
		udelay(100);
		phy_write(phydev,0x1c,24,0x8124);
		udelay(100);
		phy_write(phydev,0x1c,25,0xa040);
		udelay(100);
		phy_write(phydev,0x1c,24,0x8524);
		/*Fix 1000Base-X AN advertisement*/
		/*write45 4.2004.5 to 1*/
		/* ADDR 0x09*/
		phy_write(phydev,0x1c,25,0x2004);
		udelay(100);
		phy_write(phydev,0x1c,24,0x8124);
		udelay(100);
		phy_write(phydev,0x1c,25,0x20);
		udelay(100);
		phy_write(phydev,0x1c,24,0x8524);
		udelay(100);
		/*Enable Forwarding on ports:*/
		phy_write(phydev,9,4,0x007F);
		mdelay(300);
		//configuration for SPF switch port
		phy_write(phydev, 9, 0x0, 0x9);

		printf("board_phy_configured\n");
	}
#endif
	return 0;
}

/*
#define RAM_START_ADD 0x10000000 //indirizzo da modificare, prima bisogna verificare che non ci sia parte dell'uboot
#define size 1024*1024*4 		//4MB dimensione della memoria da testare
#define RAM_END_ADD (RAM_START_ADD + size)


void custom_ddr_test(void)
{
	unsigned int *add = (unsigned int *)RAM_START_ADD;
	unsigned int mem32;
	unsigned int testerr=0;
	int i;
	int k;
	unsigned int data;
	unsigned int pattern[6]={0x0,0xaaaa5555,0x5555aaaa,0xffffffff,0x12345678};
	printf("Start DDR TEST INFINITE LOOP\r\n");
	for(int k=0;k<6;k++)
	{
		printf("Pattern used %x\r\n",pattern[k]);
		mem32=RAM_START_ADD;
		for (i=0;i<(RAM_END_ADD-RAM_START_ADD)/4;i++)
		{
			mem32[i]=pattern[k];
		}
		for (i=0;i<(RAM_START_ADD-RAM_START_ADD)/4;i++)
		{
			if(mem32[i]!=pattern[k])
				{
					testerr=testerr+1;
					printf("Error detected at add %x, expected %x, read %x\n\r",i,pattern[k],mem32[i]);
					//break;
				}
		}
	}
}
*/

#if defined(CONFIG_MX6DL) && defined(CONFIG_MXC_EPDC)
vidinfo_t panel_info = {
	.vl_refresh = 85,
	.vl_col = 800,
	.vl_row = 600,
	.vl_pixclock = 26666667,
	.vl_left_margin = 8,
	.vl_right_margin = 100,
	.vl_upper_margin = 4,
	.vl_lower_margin = 8,
	.vl_hsync = 4,
	.vl_vsync = 1,
	.vl_sync = 0,
	.vl_mode = 0,
	.vl_flag = 0,
	.vl_bpix = 3,
	.cmap = 0,
};

struct epdc_timing_params panel_timings = {
	.vscan_holdoff = 4,
	.sdoed_width = 10,
	.sdoed_delay = 20,
	.sdoez_width = 10,
	.sdoez_delay = 20,
	.gdclk_hp_offs = 419,
	.gdsp_offs = 20,
	.gdoe_offs = 0,
	.gdclk_offs = 5,
	.num_ce = 1,
};

static iomux_v3_cfg_t const epdc_pwr_ctrl_pads[] = {
	IOMUX_PADS(PAD_EIM_A17__GPIO2_IO21	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D17__GPIO3_IO17	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D20__GPIO3_IO20	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_A18__GPIO2_IO20	| MUX_PAD_CTRL(EPDC_PAD_CTRL)),
};

static void setup_epdc_power(void)
{
	SETUP_IOMUX_PADS(epdc_pwr_ctrl_pads);

	/* Setup epdc voltage */

	/* EIM_A17 - GPIO2[21] for PWR_GOOD status */
	/* Set as input */
	gpio_request(IMX_GPIO_NR(2, 21), "EPDC PWRSTAT");
	gpio_direction_input(IMX_GPIO_NR(2, 21));

	/* EIM_D17 - GPIO3[17] for VCOM control */
	/* Set as output */
	gpio_request(IMX_GPIO_NR(3, 17), "EPDC VCOM0");
	gpio_direction_output(IMX_GPIO_NR(3, 17), 1);

	/* EIM_D20 - GPIO3[20] for EPD PMIC WAKEUP */
	/* Set as output */
	gpio_request(IMX_GPIO_NR(3, 20), "EPDC PWR WAKEUP");
	gpio_direction_output(IMX_GPIO_NR(3, 20), 1);

	/* EIM_A18 - GPIO2[20] for EPD PWR CTL0 */
	/* Set as output */
	gpio_request(IMX_GPIO_NR(2, 20), "EPDC PWR CTRL0");
	gpio_direction_output(IMX_GPIO_NR(2, 20), 1);
}

static void epdc_enable_pins(void)
{
	/* epdc iomux settings */
	SETUP_IOMUX_PADS(epdc_enable_pads);
}

static void epdc_disable_pins(void)
{
	/* Configure MUX settings for EPDC pins to GPIO */
	SETUP_IOMUX_PADS(epdc_disable_pads);
}

static void setup_epdc(void)
{
	unsigned int reg;
	struct mxc_ccm_reg *ccm_regs = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	/*** Set pixel clock rates for EPDC ***/

	/* EPDC AXI clk (IPU2_CLK) from PFD_400M, set to 396/2 = 198MHz */
	reg = readl(&ccm_regs->cscdr3);
	reg &= ~0x7C000;
	reg |= (1 << 16) | (1 << 14);
	writel(reg, &ccm_regs->cscdr3);

	/* EPDC AXI clk enable */
	reg = readl(&ccm_regs->CCGR3);
	reg |= 0x00C0;
	writel(reg, &ccm_regs->CCGR3);

	/* EPDC PIX clk (IPU2_DI1_CLK) from PLL5, set to 650/4/6 = ~27MHz */
	reg = readl(&ccm_regs->cscdr2);
	reg &= ~0x3FE00;
	reg |= (2 << 15) | (5 << 12);
	writel(reg, &ccm_regs->cscdr2);

	/* PLL5 enable (defaults to 650) */
	reg = readl(&ccm_regs->analog_pll_video);
	reg &= ~((1 << 16) | (1 << 12));
	reg |= (1 << 13);
	writel(reg, &ccm_regs->analog_pll_video);

	/* EPDC PIX clk enable */
	reg = readl(&ccm_regs->CCGR3);
	reg |= 0x0C00;
	writel(reg, &ccm_regs->CCGR3);

	panel_info.epdc_data.wv_modes.mode_init = 0;
	panel_info.epdc_data.wv_modes.mode_du = 1;
	panel_info.epdc_data.wv_modes.mode_gc4 = 3;
	panel_info.epdc_data.wv_modes.mode_gc8 = 2;
	panel_info.epdc_data.wv_modes.mode_gc16 = 2;
	panel_info.epdc_data.wv_modes.mode_gc32 = 2;

	panel_info.epdc_data.epdc_timings = panel_timings;

	setup_epdc_power();
}

void epdc_power_on(void)
{
	unsigned int reg;
	struct gpio_regs *gpio_regs = (struct gpio_regs *)GPIO2_BASE_ADDR;

	/* Set EPD_PWR_CTL0 to high - enable EINK_VDD (3.15) */
	gpio_set_value(IMX_GPIO_NR(2, 20), 1);
	udelay(1000);

	/* Enable epdc signal pin */
	epdc_enable_pins();

	/* Set PMIC Wakeup to high - enable Display power */
	gpio_set_value(IMX_GPIO_NR(3, 20), 1);

	/* Wait for PWRGOOD == 1 */
	while (1) {
		reg = readl(&gpio_regs->gpio_psr);
		if (!(reg & (1 << 21)))
			break;

		udelay(100);
	}

	/* Enable VCOM */
	gpio_set_value(IMX_GPIO_NR(3, 17), 1);

	udelay(500);
}

void epdc_power_off(void)
{
	/* Set PMIC Wakeup to low - disable Display power */
	gpio_set_value(IMX_GPIO_NR(3, 20), 0);

	/* Disable VCOM */
	gpio_set_value(IMX_GPIO_NR(3, 17), 0);

	epdc_disable_pins();

	/* Set EPD_PWR_CTL0 to low - disable EINK_VDD (3.15) */
	gpio_set_value(IMX_GPIO_NR(2, 20), 0);
}
#endif

#if defined(CONFIG_VIDEO_IPUV3)
static void disable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	int reg = readl(&iomux->gpr[2]);

	reg &= ~(IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
		 IOMUXC_GPR2_LVDS_CH1_MODE_MASK);

	writel(reg, &iomux->gpr[2]);
}

static void do_enable_hdmi(struct display_info_t const *dev)
{
	disable_lvds(dev);
	/*imx_enable_hdmi_phy();*/
}

struct display_info_t const displays[] = {};

size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	/* Setup HSYNC, VSYNC, DISP_CLK for debugging purposes */
	//SETUP_IOMUX_PADS(di0_pads);

	enable_ipu_clock();
	//imx_setup_hdmi();

	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg = readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      | (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV | MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     | IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     | IOMUXC_GPR2_LVDS_CH0_MODE_DISABLED
	     | IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS1_MUX_CTL_MASK
			| IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       << IOMUXC_GPR3_LVDS1_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}
#endif /* CONFIG_VIDEO_IPUV3 */

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

static void setup_fec(void)
{
	if (is_mx6dqp()) {
		int ret;

		/* select ENET MAC0 TX clock from PLL */
		imx_iomux_set_gpr_register(5, 9, 1, 1);
		ret = enable_fec_anatop_clock(0, ENET_125MHZ);
		if (ret)
		    printf("Error fec anatop clock settings!\n");
	}

	fec_phy_reset();
}

int board_eth_init(bd_t *bis)
{

	int ret = -ENODEV;
	char *env = NULL;
    printf("board_eth_init\n");
	setup_iomux_enet();


	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct anatop_regs *anatop = (struct anatop_regs *)ANATOP_BASE_ADDR;
	int reg;

	reg=readl(&anatop->pll_enet);
	printf("pllEnet reg1 = %x\n",reg);
	writel((0x000),&anatop->pll_enet);
	mdelay(100);
	writel((reg),&anatop->pll_enet);
	reg=readl(&anatop->pll_enet);
	printf("pllEnet reg2 = %x\n",reg);


	ret =cpu_eth_init(bis);
	printf("board_eth_init complete \n");

	return ret;
}

#ifdef CONFIG_USB_EHCI_MX6
#ifdef CONFIG_DM_USB
int board_ehci_hcd_init(int port)
{
	switch (port) {
	case 0:
		/*
		  * Set daisy chain for otg_pin_id on 6q.
		 *  For 6dl, this bit is reserved.
		 */
		imx_iomux_set_gpr_register(1, 13, 1, 0);
		break;
	case 1:
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return -EINVAL;
	}
	return 0;
}
#else
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)

static iomux_v3_cfg_t const usb_otg_pads[] = {
	IOMUX_PADS(PAD_EIM_D22__USB_OTG_PWR | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_ENET_RX_ER__USB_OTG_ID | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static iomux_v3_cfg_t const usb_hc1_pads[] = {
	IOMUX_PADS(PAD_ENET_TXD1__GPIO1_IO29 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	switch (port) {
	case 0:
		SETUP_IOMUX_PADS(usb_otg_pads);

		/*
		  * Set daisy chain for otg_pin_id on 6q.
		 *  For 6dl, this bit is reserved.
		 */
		imx_iomux_set_gpr_register(1, 13, 1, 0);

		usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
				 port * 4);

		setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);
		break;
	case 1:
		SETUP_IOMUX_PADS(usb_hc1_pads);
		gpio_request(IMX_GPIO_NR(1, 29), "USB HC1 Power Enable");
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return -EINVAL;
	}

	return 0;
}

int board_ehci_power(int port, int on)
{
	switch (port) {
	case 0:
		break;
	case 1:
		if (on)
			gpio_direction_output(IMX_GPIO_NR(1, 29), 1);
		else
			gpio_direction_output(IMX_GPIO_NR(1, 29), 0);
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return -EINVAL;
	}

	return 0;
}
#endif
#endif

int board_early_init_f(void)
{
	setup_iomux_uart();
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	printf("Board init\n");

#ifdef CONFIG_MXC_SPI
	printf("board_init: call setup_spi\n");
	setup_spi();
#endif

#ifdef CONFIG_CMD_I2C
	//setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	printf("board_init: call setup_i2c\n");
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c1_pad_info);
#endif
	printf("board_init: call setup_weim\n");
	setup_weim();

/*
#ifdef CONFIG_PCIE_IMX
	printf("board_init: call sesetup_pcie\n");
	setup_pcie();
#endif
*/
/*
#if defined(CONFIG_MX6DL) && defined(CONFIG_MXC_EPDC)
	printf("board_init: call setup_epdc\n");
	setup_epdc();
#endif
*/
/*
#ifdef CONFIG_SATA
	printf("board_init: call setup_sata\n");
	setup_sata();
#endif
*/
/*
#ifdef CONFIG_FEC_MXC
	printf("board_init: call setup_fec\n");
	setup_fec();
#endif
*/
	return 0;
}

#ifdef CONFIG_POWER
int power_init_board(void)
{
	struct pmic *pfuze;
	unsigned int reg;
	int ret;
	printf("power_init_board 1\n");
	pfuze = pfuze_common_init(I2C_PMIC);
	if (!pfuze)
		return -ENODEV;

	if (is_mx6dqp())
		ret = pfuze_mode_init(pfuze, APS_APS);
	else
		ret = pfuze_mode_init(pfuze, APS_PFM);

	if (ret < 0)
		return ret;
	/* VGEN3 and VGEN5 corrected on i.mx6qp board */
	if (!is_mx6dqp()) {
		/* Increase VGEN3 from 2.5 to 2.8V */
		pmic_reg_read(pfuze, PFUZE100_VGEN3VOL, &reg);
		reg &= ~LDO_VOL_MASK;
		reg |= LDOB_2_80V;
		pmic_reg_write(pfuze, PFUZE100_VGEN3VOL, reg);

		/* Increase VGEN5 from 2.8 to 3V */
		pmic_reg_read(pfuze, PFUZE100_VGEN5VOL, &reg);
		reg &= ~LDO_VOL_MASK;
		reg |= LDOB_3_00V;
		pmic_reg_write(pfuze, PFUZE100_VGEN5VOL, reg);
	}

	if (is_mx6dqp()) {
		/* set SW1C staby volatage 1.075V*/
		pmic_reg_read(pfuze, PFUZE100_SW1CSTBY, &reg);
		reg &= ~0x3f;
		reg |= 0x1f;
		pmic_reg_write(pfuze, PFUZE100_SW1CSTBY, reg);

		/* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
		pmic_reg_read(pfuze, PFUZE100_SW1CCONF, &reg);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(pfuze, PFUZE100_SW1CCONF, reg);

		/* set SW2/VDDARM staby volatage 0.975V*/
		pmic_reg_read(pfuze, PFUZE100_SW2STBY, &reg);
		reg &= ~0x3f;
		reg |= 0x17;
		pmic_reg_write(pfuze, PFUZE100_SW2STBY, reg);

		/* set SW2/VDDARM step ramp up time to from 16us to 4us/25mV */
		pmic_reg_read(pfuze, PFUZE100_SW2CONF, &reg);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(pfuze, PFUZE100_SW2CONF, reg);
	} else {
		/* set SW1AB staby volatage 0.975V*/
		pmic_reg_read(pfuze, PFUZE100_SW1ABSTBY, &reg);
		reg &= ~0x3f;
		reg |= 0x1b;
		pmic_reg_write(pfuze, PFUZE100_SW1ABSTBY, reg);

		/* set SW1AB/VDDARM step ramp up time from 16us to 4us/25mV */
		pmic_reg_read(pfuze, PFUZE100_SW1ABCONF, &reg);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(pfuze, PFUZE100_SW1ABCONF, reg);

		/* set SW1C staby volatage 0.975V*/
		pmic_reg_read(pfuze, PFUZE100_SW1CSTBY, &reg);
		reg &= ~0x3f;
		reg |= 0x1b;
		pmic_reg_write(pfuze, PFUZE100_SW1CSTBY, reg);

		/* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
		pmic_reg_read(pfuze, PFUZE100_SW1CCONF, &reg);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(pfuze, PFUZE100_SW1CCONF, reg);
	}

	return 0;
}

#elif defined(CONFIG_DM_PMIC_PFUZE100)
int power_init_board(void)
{
	struct udevice *dev;
	unsigned int reg;
	int ret;
	printf("power_init_board 2\n");
	dev = pfuze_common_init();
	if (!dev)
		return -ENODEV;

	if (is_mx6dqp())
		ret = pfuze_mode_init(dev, APS_APS);
	else
		ret = pfuze_mode_init(dev, APS_PFM);
	if (ret < 0)
		return ret;

	/* VGEN3 and VGEN5 corrected on i.mx6qp board */
	if (!is_mx6dqp()) {
		/* Increase VGEN3 from 2.5 to 2.8V */
		reg = pmic_reg_read(dev, PFUZE100_VGEN3VOL);
		reg &= ~LDO_VOL_MASK;
		reg |= LDOB_2_80V;
		pmic_reg_write(dev, PFUZE100_VGEN3VOL, reg);

		/* Increase VGEN5 from 2.8 to 3V */
		reg = pmic_reg_read(dev, PFUZE100_VGEN5VOL);
		reg &= ~LDO_VOL_MASK;
		reg |= LDOB_3_00V;
		pmic_reg_write(dev, PFUZE100_VGEN5VOL, reg);
	}

	if (is_mx6dqp()) {
		/* set SW1C staby volatage 1.075V*/
		reg = pmic_reg_read(dev, PFUZE100_SW1CSTBY);
		reg &= ~0x3f;
		reg |= 0x1f;
		pmic_reg_write(dev, PFUZE100_SW1CSTBY, reg);

		/* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
		reg = pmic_reg_read(dev, PFUZE100_SW1CCONF);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(dev, PFUZE100_SW1CCONF, reg);

		/* set SW2/VDDARM staby volatage 0.975V*/
		reg = pmic_reg_read(dev, PFUZE100_SW2STBY);
		reg &= ~0x3f;
		reg |= 0x17;
		pmic_reg_write(dev, PFUZE100_SW2STBY, reg);

		/* set SW2/VDDARM step ramp up time to from 16us to 4us/25mV */
		reg = pmic_reg_read(dev, PFUZE100_SW2CONF);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(dev, PFUZE100_SW2CONF, reg);
	} else {
		/* set SW1AB staby volatage 0.975V*/
		reg = pmic_reg_read(dev, PFUZE100_SW1ABSTBY);
		reg &= ~0x3f;
		reg |= 0x1b;
		pmic_reg_write(dev, PFUZE100_SW1ABSTBY, reg);

		/* set SW1AB/VDDARM step ramp up time from 16us to 4us/25mV */
		reg = pmic_reg_read(dev, PFUZE100_SW1ABCONF);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(dev, PFUZE100_SW1ABCONF, reg);

		/* set SW1C staby volatage 0.975V*/
		reg = pmic_reg_read(dev, PFUZE100_SW1CSTBY);
		reg &= ~0x3f;
		reg |= 0x1b;
		pmic_reg_write(dev, PFUZE100_SW1CSTBY, reg);

		/* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
		reg = pmic_reg_read(dev, PFUZE100_SW1CCONF);
		reg &= ~0xc0;
		reg |= 0x40;
		pmic_reg_write(dev, PFUZE100_SW1CCONF, reg);
	}

	return 0;
}
#endif

#ifdef CONFIG_LDO_BYPASS_CHECK
#ifdef CONFIG_POWER
void ldo_mode_set(int ldo_bypass)
{
	unsigned int value;
	int is_400M;
	unsigned char vddarm;
	struct pmic *p = pmic_get("PFUZE100");

	if (!p) {
		printf("No PMIC found!\n");
		return;
	}

	/* increase VDDARM/VDDSOC to support 1.2G chip */
	if (check_1_2G()) {
		ldo_bypass = 0;	/* ldo_enable on 1.2G chip */
		printf("1.2G chip, increase VDDARM_IN/VDDSOC_IN\n");
		if (is_mx6dqp()) {
			/* increase VDDARM to 1.425V */
			pmic_reg_read(p, PFUZE100_SW2VOL, &value);
			value &= ~0x3f;
			value |= 0x29;
			pmic_reg_write(p, PFUZE100_SW2VOL, value);
		} else {
			/* increase VDDARM to 1.425V */
			pmic_reg_read(p, PFUZE100_SW1ABVOL, &value);
			value &= ~0x3f;
			value |= 0x2d;
			pmic_reg_write(p, PFUZE100_SW1ABVOL, value);
		}
		/* increase VDDSOC to 1.425V */
		pmic_reg_read(p, PFUZE100_SW1CVOL, &value);
		value &= ~0x3f;
		value |= 0x2d;
		pmic_reg_write(p, PFUZE100_SW1CVOL, value);
	}
	/* switch to ldo_bypass mode , boot on 800Mhz */
	if (ldo_bypass) {
		prep_anatop_bypass();
		if (is_mx6dqp()) {
			/* decrease VDDARM for 400Mhz DQP:1.1V*/
			pmic_reg_read(p, PFUZE100_SW2VOL, &value);
			value &= ~0x3f;
			value |= 0x1c;
			pmic_reg_write(p, PFUZE100_SW2VOL, value);
		} else {
			/* decrease VDDARM for 400Mhz DQ:1.1V, DL:1.275V */
			pmic_reg_read(p, PFUZE100_SW1ABVOL, &value);
			value &= ~0x3f;
			if (is_mx6dl())
				value |= 0x27;
			else
				value |= 0x20;

			pmic_reg_write(p, PFUZE100_SW1ABVOL, value);
		}
		/* increase VDDSOC to 1.3V */
		pmic_reg_read(p, PFUZE100_SW1CVOL, &value);
		value &= ~0x3f;
		value |= 0x28;
		pmic_reg_write(p, PFUZE100_SW1CVOL, value);

		/*
		 * MX6Q/DQP:
		 * VDDARM:1.15V@800M; VDDSOC:1.175V@800M
		 * VDDARM:0.975V@400M; VDDSOC:1.175V@400M
		 * MX6DL:
		 * VDDARM:1.175V@800M; VDDSOC:1.175V@800M
		 * VDDARM:1.15V@400M; VDDSOC:1.175V@400M
		 */
		is_400M = set_anatop_bypass(2);
		if (is_mx6dqp()) {
			pmic_reg_read(p, PFUZE100_SW2VOL, &value);
			value &= ~0x3f;
			if (is_400M)
				value |= 0x17;
			else
				value |= 0x1e;
			pmic_reg_write(p, PFUZE100_SW2VOL, value);
		}

		if (is_400M) {
			if (is_mx6dl())
				vddarm = 0x22;
			else
				vddarm = 0x1b;
		} else {
			if (is_mx6dl())
				vddarm = 0x23;
			else
				vddarm = 0x22;
		}
		pmic_reg_read(p, PFUZE100_SW1ABVOL, &value);
		value &= ~0x3f;
		value |= vddarm;
		pmic_reg_write(p, PFUZE100_SW1ABVOL, value);

		/* decrease VDDSOC to 1.175V */
		pmic_reg_read(p, PFUZE100_SW1CVOL, &value);
		value &= ~0x3f;
		value |= 0x23;
		pmic_reg_write(p, PFUZE100_SW1CVOL, value);

		finish_anatop_bypass();
		printf("switch to ldo_bypass mode!\n");
	}
}
#elif defined(CONFIG_DM_PMIC_PFUZE100)
void ldo_mode_set(int ldo_bypass)
{
	int is_400M;
	unsigned char vddarm;
	struct udevice *dev;
	int ret;

	ret = pmic_get("pfuze100", &dev);
	if (ret == -ENODEV) {
		printf("No PMIC found!\n");
		return;
	}

	/* increase VDDARM/VDDSOC to support 1.2G chip */
	if (check_1_2G()) {
		ldo_bypass = 0; /* ldo_enable on 1.2G chip */
		printf("1.2G chip, increase VDDARM_IN/VDDSOC_IN\n");
		if (is_mx6dqp()) {
			/* increase VDDARM to 1.425V */
			pmic_clrsetbits(dev, PFUZE100_SW2VOL, 0x3f, 0x29);
		} else {
			/* increase VDDARM to 1.425V */
			pmic_clrsetbits(dev, PFUZE100_SW1ABVOL, 0x3f, 0x2d);
		}
		/* increase VDDSOC to 1.425V */
		pmic_clrsetbits(dev, PFUZE100_SW1CVOL, 0x3f, 0x2d);
	}
	/* switch to ldo_bypass mode , boot on 800Mhz */
	if (ldo_bypass) {
		prep_anatop_bypass();
		if (is_mx6dqp()) {
			/* decrease VDDARM for 400Mhz DQP:1.1V*/
			pmic_clrsetbits(dev, PFUZE100_SW2VOL, 0x3f, 0x1c);
		} else {
			/* decrease VDDARM for 400Mhz DQ:1.1V, DL:1.275V */
			if (is_mx6dl())
				pmic_clrsetbits(dev, PFUZE100_SW1ABVOL, 0x3f, 0x27);
			else
				pmic_clrsetbits(dev, PFUZE100_SW1ABVOL, 0x3f, 0x20);
		}
		/* increase VDDSOC to 1.3V */
		pmic_clrsetbits(dev, PFUZE100_SW1CVOL, 0x3f, 0x28);

		/*
		 * MX6Q/DQP:
		 * VDDARM:1.15V@800M; VDDSOC:1.175V@800M
		 * VDDARM:0.975V@400M; VDDSOC:1.175V@400M
		 * MX6DL:
		 * VDDARM:1.175V@800M; VDDSOC:1.175V@800M
		 * VDDARM:1.15V@400M; VDDSOC:1.175V@400M
		 */
		is_400M = set_anatop_bypass(2);
		if (is_mx6dqp()) {
			if (is_400M)
				pmic_clrsetbits(dev, PFUZE100_SW2VOL, 0x3f, 0x17);
			else
				pmic_clrsetbits(dev, PFUZE100_SW2VOL, 0x3f, 0x1e);
		}

		if (is_400M) {
			if (is_mx6dl())
				vddarm = 0x22;
			else
				vddarm = 0x1b;
		} else {
			if (is_mx6dl())
				vddarm = 0x23;
			else
				vddarm = 0x22;
		}
		pmic_clrsetbits(dev, PFUZE100_SW1ABVOL, 0x3f, vddarm);

		/* decrease VDDSOC to 1.175V */
		pmic_clrsetbits(dev, PFUZE100_SW1CVOL, 0x3f, 0x23);

		finish_anatop_bypass();
		printf("switch to ldo_bypass mode!\n");
	}
}
#endif
#endif

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd2",	 MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	/*{"sd3",	 MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},*/
	/* 8 bit bus width */
	/*{"emmc", MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)},*/
	{"emmc", MAKE_CFGVAL(0x60, 0x20, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

	env_set("tee", "no");
#ifdef CONFIG_IMX_OPTEE
	env_set("tee", "yes");
#endif

	//env_set("board_rev", "MX6QP");
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "SKA_MANAGEMENT");

	if (is_mx6dqp())
		env_set("board_rev", "MX6QP");
	else if (is_mx6dq())
		env_set("board_rev", "MX6Q");
	else if (is_mx6sdl())
		env_set("board_rev", "MX6DL");
#endif

	ethaddr_init();

	get_weim_info();


//#ifdef CONFIG_ENV_IS_IN_MMC
	//board_late_mmc_env_init();
	//env_set("mmcdev", "1");
	//env_set("mmcroot","/dev/mmcblk1p2 rootwait rw");
	env_set("bootcmd",CONFIG_BOOTCOMMAND);
//#endif
	printf("board_late_init complete\n");
	return 0;
}

int checkboard(void)
{


	    puts(" ____________________________________________________\n");
		puts("/                                                    \\\n");
		puts("|              --= Sanitas EG =--                    |\n");
		puts("|                                                    |\n");
		puts("|      _  _  _  _    _           _        _          |\n");
		puts("|    _(_)(_)(_)(_)_ (_)       _ (_)     _(_)_        |\n");
		puts("|   (_)          (_)(_)    _ (_)      _(_) (_)_      |\n");
		puts("|   (_)_  _  _  _   (_) _ (_)       _(_)     (_)_    |\n");
		puts("|     (_)(_)(_)(_)_ (_)(_) _       (_) _  _  _ (_)   |\n");
		puts("|    _           (_)(_)   (_) _    (_)(_)(_)(_)(_)   |\n");
		puts("|   (_)_  _  _  _(_)(_)      (_) _ (_)         (_)   |\n");
		puts("|     (_)(_)(_)(_)  (_)         (_)(_)         (_)   |\n");
		puts("|                                                    |\n");
		puts("|                                                    |\n");
		puts("|                                                    |\n");
		puts("|                                                    |\n");
		puts("|    ~~ More Power To Your Imagination! ~~           |\n");
		puts("\\___________________________________________________/\n");
		puts("       \\\n");
		puts("        \\   ^__^\n");
		puts("         \\  (oo)\\_______\n");
		puts("            (__)\\       )\\/\\\n");
		puts("             U  ||----w |\n");
		puts("                ||     ||\n");
	return 0;
}





#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY

#define GPIO_VOL_DN_KEY IMX_GPIO_NR(1, 5)
iomux_v3_cfg_t const recovery_key_pads[] = {
	IOMUX_PADS(PAD_GPIO_5__GPIO1_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

int is_recovery_key_pressing(void)
{
	int button_pressed = 0;

	/* Check Recovery Combo Button press or not. */
	SETUP_IOMUX_PADS(recovery_key_pads);

	gpio_request(GPIO_VOL_DN_KEY, "volume_dn_key");
	gpio_direction_input(GPIO_VOL_DN_KEY);

	if (gpio_get_value(GPIO_VOL_DN_KEY) == 0) { /* VOL_DN key is low assert */
		button_pressed = 1;
		printf("Recovery key pressed\n");
	}

	return  button_pressed;
}

#endif /*CONFIG_ANDROID_RECOVERY*/

#endif /*CONFIG_FSL_FASTBOOT*/

#ifdef CONFIG_SPL_BUILD
#include <asm/arch/mx6-ddr.h>
#include <spl.h>
#include <linux/libfdt.h>

#ifdef CONFIG_SPL_OS_BOOT
int spl_start_uboot(void)
{
	gpio_request(KEY_VOL_UP, "KEY Volume UP");
	gpio_direction_input(KEY_VOL_UP);

	/* Only enter in Falcon mode if KEY_VOL_UP is pressed */
	return gpio_get_value(KEY_VOL_UP);
}
#endif

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0x00C03F3F, &ccm->CCGR0);
	writel(0x0030FC03, &ccm->CCGR1);
	writel(0x0FFFC000, &ccm->CCGR2);
	writel(0x3FF00000, &ccm->CCGR3);
	writel(0x00FFF300, &ccm->CCGR4);
	writel(0x0F0000C3, &ccm->CCGR5);
	writel(0x000003FF, &ccm->CCGR6);
}

static int mx6q_dcd_table[] = {
	0x020e0798, 0x000C0000,
	0x020e0758, 0x00000000,
	0x020e0588, 0x00000030,
	0x020e0594, 0x00000030,
	0x020e056c, 0x00000030,
	0x020e0578, 0x00000030,
	0x020e074c, 0x00000030,
	0x020e057c, 0x00000030,
	0x020e058c, 0x00000000,
	0x020e059c, 0x00000030,
	0x020e05a0, 0x00000030,
	0x020e078c, 0x00000030,
	0x020e0750, 0x00020000,
	0x020e05a8, 0x00000030,
	0x020e05b0, 0x00000030,
	0x020e0524, 0x00000030,
	0x020e051c, 0x00000030,
	0x020e0518, 0x00000030,
	0x020e050c, 0x00000030,
	0x020e05b8, 0x00000030,
	0x020e05c0, 0x00000030,
	0x020e0774, 0x00020000,
	0x020e0784, 0x00000030,
	0x020e0788, 0x00000030,
	0x020e0794, 0x00000030,
	0x020e079c, 0x00000030,
	0x020e07a0, 0x00000030,
	0x020e07a4, 0x00000030,
	0x020e07a8, 0x00000030,
	0x020e0748, 0x00000030,
	0x020e05ac, 0x00000030,
	0x020e05b4, 0x00000030,
	0x020e0528, 0x00000030,
	0x020e0520, 0x00000030,
	0x020e0514, 0x00000030,
	0x020e0510, 0x00000030,
	0x020e05bc, 0x00000030,
	0x020e05c4, 0x00000030,
	0x021b0800, 0xa1390003,
	0x021b080c, 0x001F001F,
	0x021b0810, 0x001F001F,
	0x021b480c, 0x001F001F,
	0x021b4810, 0x001F001F,
	0x021b083c, 0x43270338,
	0x021b0840, 0x03200314,
	0x021b483c, 0x431A032F,
	0x021b4840, 0x03200263,
	0x021b0848, 0x4B434748,
	0x021b4848, 0x4445404C,
	0x021b0850, 0x38444542,
	0x021b4850, 0x4935493A,
	0x021b081c, 0x33333333,
	0x021b0820, 0x33333333,
	0x021b0824, 0x33333333,
	0x021b0828, 0x33333333,
	0x021b481c, 0x33333333,
	0x021b4820, 0x33333333,
	0x021b4824, 0x33333333,
	0x021b4828, 0x33333333,
	0x021b08b8, 0x00000800,
	0x021b48b8, 0x00000800,
	0x021b0004, 0x00020036,
	0x021b0008, 0x09444040,
	0x021b000c, 0x555A7975,
	0x021b0010, 0xFF538F64,
	0x021b0014, 0x01FF00DB,
	0x021b0018, 0x00001740,
	0x021b001c, 0x00008000,
	0x021b002c, 0x000026d2,
	0x021b0030, 0x005A1023,
	0x021b0040, 0x00000027,
	0x021b0000, 0x831A0000,
	0x021b001c, 0x04088032,
	0x021b001c, 0x00008033,
	0x021b001c, 0x00048031,
	0x021b001c, 0x09408030,
	0x021b001c, 0x04008040,
	0x021b0020, 0x00005800,
	0x021b0818, 0x00011117,
	0x021b4818, 0x00011117,
	0x021b0004, 0x00025576,
	0x021b0404, 0x00011006,
	0x021b001c, 0x00000000,
};

static int mx6qp_dcd_table[] = {
	0x020e0798, 0x000c0000,
	0x020e0758, 0x00000000,
	0x020e0588, 0x00000030,
	0x020e0594, 0x00000030,
	0x020e056c, 0x00000030,
	0x020e0578, 0x00000030,
	0x020e074c, 0x00000030,
	0x020e057c, 0x00000030,
	0x020e058c, 0x00000000,
	0x020e059c, 0x00000030,
	0x020e05a0, 0x00000030,
	0x020e078c, 0x00000030,
	0x020e0750, 0x00020000,
	0x020e05a8, 0x00000030,
	0x020e05b0, 0x00000030,
	0x020e0524, 0x00000030,
	0x020e051c, 0x00000030,
	0x020e0518, 0x00000030,
	0x020e050c, 0x00000030,
	0x020e05b8, 0x00000030,
	0x020e05c0, 0x00000030,
	0x020e0774, 0x00020000,
	0x020e0784, 0x00000030,
	0x020e0788, 0x00000030,
	0x020e0794, 0x00000030,
	0x020e079c, 0x00000030,
	0x020e07a0, 0x00000030,
	0x020e07a4, 0x00000030,
	0x020e07a8, 0x00000030,
	0x020e0748, 0x00000030,
	0x020e05ac, 0x00000030,
	0x020e05b4, 0x00000030,
	0x020e0528, 0x00000030,
	0x020e0520, 0x00000030,
	0x020e0514, 0x00000030,
	0x020e0510, 0x00000030,
	0x020e05bc, 0x00000030,
	0x020e05c4, 0x00000030,
	0x021b0800, 0xa1390003,
	0x021b080c, 0x001b001e,
	0x021b0810, 0x002e0029,
	0x021b480c, 0x001b002a,
	0x021b4810, 0x0019002c,
	0x021b083c, 0x43240334,
	0x021b0840, 0x0324031a,
	0x021b483c, 0x43340344,
	0x021b4840, 0x03280276,
	0x021b0848, 0x44383A3E,
	0x021b4848, 0x3C3C3846,
	0x021b0850, 0x2e303230,
	0x021b4850, 0x38283E34,
	0x021b081c, 0x33333333,
	0x021b0820, 0x33333333,
	0x021b0824, 0x33333333,
	0x021b0828, 0x33333333,
	0x021b481c, 0x33333333,
	0x021b4820, 0x33333333,
	0x021b4824, 0x33333333,
	0x021b4828, 0x33333333,
	0x021b08c0, 0x24912249,
	0x021b48c0, 0x24914289,
	0x021b08b8, 0x00000800,
	0x021b48b8, 0x00000800,
	0x021b0004, 0x00020036,
	0x021b0008, 0x24444040,
	0x021b000c, 0x555A7955,
	0x021b0010, 0xFF320F64,
	0x021b0014, 0x01ff00db,
	0x021b0018, 0x00001740,
	0x021b001c, 0x00008000,
	0x021b002c, 0x000026d2,
	0x021b0030, 0x005A1023,
	0x021b0040, 0x00000027,
	0x021b0400, 0x14420000,
	0x021b0000, 0x831A0000,
	0x021b0890, 0x00400C58,
	0x00bb0008, 0x00000000,
	0x00bb000c, 0x2891E41A,
	0x00bb0038, 0x00000564,
	0x00bb0014, 0x00000040,
	0x00bb0028, 0x00000020,
	0x00bb002c, 0x00000020,
	0x021b001c, 0x04088032,
	0x021b001c, 0x00008033,
	0x021b001c, 0x00048031,
	0x021b001c, 0x09408030,
	0x021b001c, 0x04008040,
	0x021b0020, 0x00005800,
	0x021b0818, 0x00011117,
	0x021b4818, 0x00011117,
	0x021b0004, 0x00025576,
	0x021b0404, 0x00011006,
	0x021b001c, 0x00000000,
};

static int mx6dl_dcd_table[] = {
	0x020e0774, 0x000C0000,
	0x020e0754, 0x00000000,
	0x020e04ac, 0x00000030,
	0x020e04b0, 0x00000030,
	0x020e0464, 0x00000030,
	0x020e0490, 0x00000030,
	0x020e074c, 0x00000030,
	0x020e0494, 0x00000030,
	0x020e04a0, 0x00000000,
	0x020e04b4, 0x00000030,
	0x020e04b8, 0x00000030,
	0x020e076c, 0x00000030,
	0x020e0750, 0x00020000,
	0x020e04bc, 0x00000030,
	0x020e04c0, 0x00000030,
	0x020e04c4, 0x00000030,
	0x020e04c8, 0x00000030,
	0x020e04cc, 0x00000030,
	0x020e04d0, 0x00000030,
	0x020e04d4, 0x00000030,
	0x020e04d8, 0x00000030,
	0x020e0760, 0x00020000,
	0x020e0764, 0x00000030,
	0x020e0770, 0x00000030,
	0x020e0778, 0x00000030,
	0x020e077c, 0x00000030,
	0x020e0780, 0x00000030,
	0x020e0784, 0x00000030,
	0x020e078c, 0x00000030,
	0x020e0748, 0x00000030,
	0x020e0470, 0x00000030,
	0x020e0474, 0x00000030,
	0x020e0478, 0x00000030,
	0x020e047c, 0x00000030,
	0x020e0480, 0x00000030,
	0x020e0484, 0x00000030,
	0x020e0488, 0x00000030,
	0x020e048c, 0x00000030,
	0x021b0800, 0xa1390003,
	0x021b080c, 0x001F001F,
	0x021b0810, 0x001F001F,
	0x021b480c, 0x001F001F,
	0x021b4810, 0x001F001F,
	0x021b083c, 0x4220021F,
	0x021b0840, 0x0207017E,
	0x021b483c, 0x4201020C,
	0x021b4840, 0x01660172,
	0x021b0848, 0x4A4D4E4D,
	0x021b4848, 0x4A4F5049,
	0x021b0850, 0x3F3C3D31,
	0x021b4850, 0x3238372B,
	0x021b081c, 0x33333333,
	0x021b0820, 0x33333333,
	0x021b0824, 0x33333333,
	0x021b0828, 0x33333333,
	0x021b481c, 0x33333333,
	0x021b4820, 0x33333333,
	0x021b4824, 0x33333333,
	0x021b4828, 0x33333333,
	0x021b08b8, 0x00000800,
	0x021b48b8, 0x00000800,
	0x021b0004, 0x0002002D,
	0x021b0008, 0x00333030,
	0x021b000c, 0x3F435313,
	0x021b0010, 0xB66E8B63,
	0x021b0014, 0x01FF00DB,
	0x021b0018, 0x00001740,
	0x021b001c, 0x00008000,
	0x021b002c, 0x000026d2,
	0x021b0030, 0x00431023,
	0x021b0040, 0x00000027,
	0x021b0000, 0x831A0000,
	0x021b001c, 0x04008032,
	0x021b001c, 0x00008033,
	0x021b001c, 0x00048031,
	0x021b001c, 0x05208030,
	0x021b001c, 0x04008040,
	0x021b0020, 0x00005800,
	0x021b0818, 0x00011117,
	0x021b4818, 0x00011117,
	0x021b0004, 0x0002556D,
	0x021b0404, 0x00011006,
	0x021b001c, 0x00000000,
};

static void ddr_init(int *table, int size)
{
	int i;

	for (i = 0; i < size / 2 ; i++)
		writel(table[2 * i + 1], table[2 * i]);
}

static void spl_dram_init(void)
{
	if (is_mx6dq())
		ddr_init(mx6q_dcd_table, ARRAY_SIZE(mx6q_dcd_table));
	else if (is_mx6dqp())
		ddr_init(mx6qp_dcd_table, ARRAY_SIZE(mx6qp_dcd_table));
	else if (is_mx6sdl())
		ddr_init(mx6dl_dcd_table, ARRAY_SIZE(mx6dl_dcd_table));
}

void board_init_f(ulong dummy)
{
	/* DDR initialization */
	spl_dram_init();

	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();
	gpr_init();

	/* iomux and setup of i2c */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}
#endif
