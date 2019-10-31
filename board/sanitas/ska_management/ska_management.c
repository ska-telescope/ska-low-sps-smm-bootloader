/*
 * Copyright (C) 2018 Sanitas EG
 *
 * Author: Cristian Albanese <cristian.albanese@sanitaseg.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/sata.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <phy.h>
#include <netdev.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <i2c.h>
#include <net.h>
#include <common.h>
#include <pci.h>


DECLARE_GLOBAL_DATA_PTR;

static char const *board_type = "uninitialized";

#define UART_PAD_CTRL         (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST | PAD_CTL_HYS               )
#define USDHC_PAD_CTRL        (PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm | PAD_CTL_SRE_FAST | PAD_CTL_HYS               )
//#define SPI_PAD_CTRL          (PAD_CTL_HYS         | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST                             )
#define I2C_PAD_CTRL          (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST | PAD_CTL_HYS | PAD_CTL_ODE )
#define EIM_OVERRIDE_PAD_CTRL (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST | PAD_CTL_HYS | PAD_CTL_ODE )
#define OUTPUT_40OHM_PAD_CTRL (                      PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm                                                )
#define ENET_PAD_CTRL         (PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |                    PAD_CTL_HYS               )
#define ENET_PAD_CTRL_CLK    ((PAD_CTL_PUS_100K_UP & ~PAD_CTL_PKE) | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define SPI_PAD_CTRL          (0x80000000 )

int skamngment_switch_config(void);

int dram_init(void)
{
	gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);

	return 0;
}

/*PCIE*/
iomux_v3_cfg_t const pcie_pads[] = {
	//MX6_PAD_EIM_D19__GPIO3_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL),	/* POWER */
	MX6_PAD_EIM_EB0__GPIO2_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL),	/* RESET */
};


/* UART 1 */
static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

/* UART 2 */
static iomux_v3_cfg_t const uart2_pads[] = {
	MX6_PAD_GPIO_7__UART2_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_GPIO_8__UART2_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

/* Ethernet RGMII */
static iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_KEY_COL1__ENET_MDIO        | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_KEY_COL2__ENET_MDC         | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_GPIO_16__ENET_REF_CLK      | MUX_MODE_SION | MUX_PAD_CTRL(ENET_PAD_CTRL| PAD_CTL_SRE_FAST),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK  | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__RGMII_TXC       | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0       | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1       | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__RGMII_TD2       | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__RGMII_TD3       | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RXC__RGMII_RXC       | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__RGMII_RD0       | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__RGMII_RD1       | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD2__RGMII_RD2       | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__RGMII_RD3       | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

/* eMMC */
static iomux_v3_cfg_t const usdhc1_pads[] = {
	MX6_PAD_SD1_CLK__SD1_CLK	  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__SD1_CMD	  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT0__SD1_DATA0	  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT1__SD1_DATA1	  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT2__SD1_DATA2	  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT3__SD1_DATA3	  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D0__SD1_DATA4	  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D1__SD1_DATA5	  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D2__SD1_DATA6	  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D3__SD1_DATA7	  | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_GPIO_1__GPIO1_IO01	  | MUX_PAD_CTRL(NO_PAD_CTRL), /* !CD */
	MX6_PAD_GPIO_9__GPIO1_IO09	  | MUX_PAD_CTRL(NO_PAD_CTRL), /* !WP */
	MX6_PAD_NANDF_ALE__GPIO6_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL)  /* !Reset */
};

/* Front Panel uSD Card */
static iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_SD2_CLK__SD2_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_CMD__SD2_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT0__SD2_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT1__SD2_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT2__SD2_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT3__SD2_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_GPIO_4__GPIO1_IO04	| MUX_PAD_CTRL(NO_PAD_CTRL), /* !CD */
	MX6_PAD_GPIO_2__GPIO1_IO02	| MUX_PAD_CTRL(NO_PAD_CTRL), /* !WP */
};

/* ECSPI1 */
static iomux_v3_cfg_t const ecspi1_pads[] = {
	MX6_PAD_DISP0_DAT20__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT21__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT22__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_KEY_ROW1__ECSPI1_SS0     | MUX_PAD_CTRL(NO_PAD_CTRL),
};


static iomux_v3_cfg_t const init_pads[] = {
	MX6_PAD_GPIO_0__CCM_CLKO1        | MUX_PAD_CTRL(OUTPUT_40OHM_PAD_CTRL),	/* SGTL5000 sys_mclk */
};

/* EIM Override */
static iomux_v3_cfg_t const eim_override_pads[] = {
	MX6_PAD_EIM_DA0__GPIO3_IO00 | MUX_PAD_CTRL(EIM_OVERRIDE_PAD_CTRL),	
};

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

/* I2C1 */
static struct i2c_pads_info i2c1_pad_info = {
	.scl = {
		.i2c_mode = MX6_PAD_EIM_D21__I2C1_SCL | PC,
		.gpio_mode = MX6_PAD_EIM_D21__GPIO3_IO21 | PC,
		.gp = IMX_GPIO_NR(3, 21)
	},
	.sda = {
		.i2c_mode = MX6_PAD_EIM_D28__I2C1_SDA | PC,
		.gpio_mode = MX6_PAD_EIM_D28__GPIO3_IO28 | PC,
		.gp = IMX_GPIO_NR(3, 28)
	}
};


static struct i2c_pads_info i2c3_pad_info = {
	.scl = {
		.i2c_mode = MX6_PAD_EIM_D17__I2C3_SCL | PC,
		.gpio_mode = MX6_PAD_EIM_D17__GPIO3_IO17 | PC,
		.gp = IMX_GPIO_NR(3, 17)
	},
	.sda = {
		.i2c_mode = MX6_PAD_EIM_D18__I2C3_SDA | PC,
		.gpio_mode = MX6_PAD_EIM_D18__GPIO3_IO18 | PC,
		.gp = IMX_GPIO_NR(3, 18)
	}
};





/*
static iomux_v3_cfg_t const lvds_pads[] = {
	MX6_PAD_DISP0_DAT8__PWM1_OUT | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT8__PWM2_OUT | MUX_PAD_CTRL(NO_PAD_CTRL),
};
*/
static void setup_iomux_enet(void)
{
	unsigned int reg;
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));
	//enable internal clk reference enet
	reg=readl(0x20E0004); //read pinmux IOMUXC_GPR1
	writel(reg|0x00200000,0x20E0004); //set ENET_CLK_SEL bit
}

static void setup_spi(void)
{
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
}

static void setup_pcie(void)
{
	imx_iomux_v3_setup_multiple_pads(pcie_pads, ARRAY_SIZE(pcie_pads));
}


static void setup_clockout(void) {
/*	int reg = readl(PMU_MISC2);
	reg &= ~(PMU_MISC2_LVDS2_CLK_SEL_MASK | PMU_MISC2_LVDS2_CLK_IBEN);
	reg |=  (PMU_MISC2_LVDS2_CLK_SEL_PCIE_REF | PMU_MISC2_LVDS2_CLK_OBEN);
	writel(reg, PMU_MISC2); */ //BACO
	writel(0x80000960, 0x020C8160);
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

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	imx_iomux_v3_setup_multiple_pads(uart2_pads, ARRAY_SIZE(uart2_pads));
}

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC1_BASE_ADDR},
	{USDHC2_BASE_ADDR},
};

#define USDHC1_CD_GPIO	IMX_GPIO_NR(1, 1)
#define USDHC2_CD_GPIO	IMX_GPIO_NR(1, 4)

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR: // Always true, as the eMMC cannot be removed.
		ret = gpio_get_value(USDHC1_CD_GPIO);
		break;
	case USDHC2_BASE_ADDR:
		ret = gpio_get_value(USDHC2_CD_GPIO) ? 0 : 1;
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	int i;

	/*
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    eMMC
	 * mmc1                    SD2
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
			gpio_direction_input(USDHC1_CD_GPIO);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
			gpio_direction_input(USDHC2_CD_GPIO);
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers (%d) then supported by the board (%d)\n", i + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return status;
		}

		status |= fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
	}

	return status;
}
#endif /* CONFIG_FSL_ESDHC */

int mx6_rgmii_rework(struct phy_device *phydev)
{
	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	mx6_rgmii_rework(phydev);

	unsigned short val;

#ifdef CONFIG_PHY_LED_TXRX
	printf("board_phy_config LED CONFIG\n");
	if (phydev->phy_id == 0x1410dd1) {
		/*
		 * Page 3, Register 16: LED[2:0] Function Control Register
		 * LED[0] (SPD:Amber) R16_3.3:0 to 0101: ot transmit
		 * LED[1] (LNK:Green) R16_3.7:4 to 0000: on receive
		 */
		phy_write(phydev, MDIO_DEVAD_NONE, 22, 3);
		val = phy_read(phydev, MDIO_DEVAD_NONE, 16);
		val &= 0xff00;
		val |= 0x0005;
		phy_write(phydev, MDIO_DEVAD_NONE, 16, val);
		phy_write(phydev, MDIO_DEVAD_NONE, 22, 0);
	}
#endif

	printf("board_phy_config\n");
	if (phydev->drv->config)
	{
		phydev->drv->config(phydev);
		
		printf("board_phy_configured\n");
		mdio_list_devices();
	}
	return 0;
}

#ifdef CONFIG_VIDEO_IPUV3
struct display_info_t {
	int    bus;
	int    addr;
	int    pixfmt;
	int    (*detect)(struct display_info_t const *dev);
	void   (*enable)(struct display_info_t const *dev);
	struct fb_videomode mode;
};

static int detect_hdmi(struct display_info_t const *dev)
{
	struct hdmi_regs *hdmi	= (struct hdmi_regs *)HDMI_ARB_BASE_ADDR;
	
	return readb(&hdmi->phy_stat0) & HDMI_DVI_STAT;
}


static void disable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	int reg = readl(&iomux->gpr[2]);
	reg &= ~(IOMUXC_GPR2_LVDS_CH0_MODE_MASK | IOMUXC_GPR2_LVDS_CH1_MODE_MASK);
	writel(reg, &iomux->gpr[2]);
}

static void do_enable_hdmi(struct display_info_t const *dev)
{
	disable_lvds(dev);
	//imx_enable_hdmi_phy(); // Workaround to avoid linux freeze
}

static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT;
	
	writel(reg, &iomux->gpr[2]);
}

static struct display_info_t const displays[] = {
};

/*
static struct display_info_t const displays[] = {
	{
		.bus	= -1,
		.addr	= 0,
		.pixfmt	= IPU_PIX_FMT_RGB24,
		.detect	= detect_hdmi,
		.enable	= do_enable_hdmi,
		.mode	= {
			.name           = "HDMI",
			.refresh        = 60,
			.xres           = 1024,
			.yres           = 768,
			.pixclock       = 15385,
			.left_margin    = 220,
			.right_margin   = 40,
			.upper_margin   = 21,
			.lower_margin   = 7,
			.hsync_len      = 60,
			.vsync_len      = 10,
			.sync           = FB_SYNC_EXT,
			.vmode          = FB_VMODE_NONINTERLACED
		}
	}
};
*/

int board_video_skip(void)
{
	int i;
	int ret;
	char const *panel = getenv("panel");
	
	if(ARRAY_SIZE(displays) == 0) {
		return -EINVAL;
	}
	
	if (!panel) {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			struct display_info_t const *dev = displays + i;
			if (dev->detect && dev->detect(dev)) {
				panel = dev->mode.name;
				printf("auto-detected panel %s\n", panel);
				break;
			}
		}
		if (!panel) {
			panel = displays[0].mode.name;
			printf("No panel detected: default to %s\n", panel);
			i = 0;
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			if (!strcmp(panel, displays[i].mode.name))
				break;
		}
	}
	
	if (i < ARRAY_SIZE(displays)) {
		ret = ipuv3_fb_init(&displays[i].mode, 0, displays[i].pixfmt);
		if (!ret) {
			displays[i].enable(displays+i);
			printf("Display: %s (%ux%u)\n",
			       displays[i].mode.name,
			       displays[i].mode.xres,
			       displays[i].mode.yres);
		} else
			printf("LCD %s cannot be configured: %d\n",
			       displays[i].mode.name, ret);
	} else {
		printf("unsupported panel %s\n", panel);
		return -EINVAL;
	}

	return 0;
}

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	/* Setup HSYNC, VSYNC, DISP_CLK for debugging purposes */
//	imx_iomux_v3_setup_multiple_pads(di0_pads, ARRAY_SIZE(di0_pads));

	enable_ipu_clock();
//	imx_setup_hdmi();

	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg = readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET) | (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
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

/* LVDS stuff */
static void setup_lvds_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	enable_ipu_clock();
	//imx_setup_hdmi(); // Workaround to avoid linux freeze

	/* Turn on LDB0 clocks */
	reg  = readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0 clk select to 011 */ /* Whatever it means */
	reg  = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg  = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg  = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0	<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	    | IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_HIGH
	    | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	    | IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	    | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	    | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	    | IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	    | IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
	    | IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
	
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS0_MUX_CTL_MASK | IOMUXC_GPR3_HDMI_MUX_CTL_MASK)) | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0 << IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}
#endif

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_eth_init(bd_t *bis)
{
	setup_iomux_enet();

	int ret = -ENODEV;
	char *env = NULL;
	
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct anatop_regs *anatop = (struct anatop_regs *)ANATOP_BASE_ADDR;
	int reg;
	
	reg=readl(&anatop->pll_enet);
	printf("pllEnet reg1 = %x\n",reg);
	//writel((reg|0x0003),&anatop->pll_enet);
	reg=readl(&anatop->pll_enet);
	printf("pllEnet reg2 = %x\n",reg);
		
	
/*
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	struct anatop_regs *anatop = (struct anatop_regs *)ANATOP_BASE_ADDR;

	int reg;
	
	reg = readl(&iomux->gpr[1]);
	printf("IOMUXC_GPR1: 0x%08x ->", reg);
	reg |= IOMUXC_GPR1_ENET_CLK_SEL_MASK;
	writel(reg, &iomux->gpr[1]);
	printf("0x%08x\n", reg);
	
	writel(BM_ANADIG_PLL_ENET_DIV_SELECT, &anatop->pll_enet_clr);
	writel(0x3, &anatop->pll_enet_set);

	writel(BM_ANADIG_PLL_ENET_BYPASS | BM_ANADIG_PLL_ENET_POWERDOWN, &anatop->pll_enet_clr);
	writel(BM_ANADIG_PLL_ENET_ENABLE, &anatop->pll_enet_set);

	mdelay(30);
*/
	ret = cpu_eth_init(bis);
	
	/*
	env = getenv("fec_addr");
	if (env) {
		printf("Configuring MAC Address... ");
		uchar mac_address[6];
		eth_getenv_enetaddr("fec_addr", mac_address);
		
		struct eth_device *dev = eth_get_dev();

		if(dev) {
			printf(" 1 ");
			if(dev->write_hwaddr) {
				printf(" 2 ");
				dev->write_hwaddr(dev);
			}
		}
	}
	*/
	
	return ret;
}

int board_early_init_f(void)
{
	setup_iomux_uart();
	
	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));
	
#if defined(CONFIG_VIDEO_IPUV3)
	//setup_lvds_display();
	setup_display(); // Workaround to avoid linux freeze
#endif

	setup_clockout(); // Clock Output?

	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_CMD_PCI
	/*reset and init pcie */
	setup_pcie();
#endif	

#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif

#ifdef CONFIG_CMD_I2C
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c1_pad_info);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c3_pad_info);
#endif

#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif

#ifdef CONFIG_CMD_MEMORY
	setup_weim();
#endif

	

	/* Enable ANATOP */
	//enable_fec_anatop_clock(ENET_50MHz);

	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	{"sd2",	 MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)}, /* 4 bit bus width */
	{"emmc", MAKE_CFGVAL(0x60, 0x20, 0x00, 0x00)}, /* 4 bit bus width */ // TODO Verify!
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_SYS_EEPROM_MAC_OFFSET
	mac_read_from_eeprom();
#endif


	int cpurev = get_cpu_rev();
	setenv("cpu", get_imx_type((cpurev & 0xFF000) >> 12));

	if (0 == getenv("board")) {
		setenv("board", board_type);
	}

	return 0;
}

int checkboard(void)
{
	board_type = "ska-management";

	puts(" ___________________________________________________________________________________\n");
	puts("/                                                                                   \\\n");
	puts("|                                  --= Sanitas EG =--                                |\n");
	puts("|                                                                                    |\n");
	puts("|                                                                                    |\n");
	puts("|       _______  __________  _______   _______   _________    /\\       _______      |\n");
	puts("|   |  |       | \\        /           |       |      |      /  \\     |   |   |  |  |\n");
	puts("|   |  |       |  \\      /   _______  |       |      |     /    \\    |   |   |  |  |\n");
	puts("|   |  |       |   \\    /             |       |      |    /      \\   |   |   |  |  |\n");
	puts("|   |  |       |    \\  /     _______  |       |      |   /________\\  |   |   |  |  |\n");
	puts("|                    \\/                                                             |\n");
	puts("|                                                                                    |\n");
	puts("|                                                                                    |\n");
	puts("|                                     /                                              |\n");
	puts("|                                    /                                               |\n");
	puts("|                                   /                                                |\n");
	puts("|                                  /___|__                                           |\n");
	puts("|                                      |                                             |\n");
	puts("|                                                                                    |\n");
	puts("|               __________                                                           |\n");
	puts("|              |               |        /         /\                                 |\n");
	puts("|              |               |      /          /  \                                |\n");
	puts("|              |               |    /           /    \                               |\n");
	puts("|               _________      |  /            /      \                              |\n");
	puts("|                        |     |/             /        \                             |\n");
	puts("|                        |     |\            /==========\                            |\n");
	puts("|                        |     |  \         /            \                           |\n");
	puts("|                        |     |    \      /              \                          |\n");
	puts("|               _________|     |      \   /                \                         |\n");
	puts("|                                                                                    |\n");
	puts("|                                                                                    |\n");
	puts("|                        ~~ More Power To Your Imagination! ~~                       |\n");
	puts("\\___________________________________________________________________________________/\n");
	puts("       \\\n");
	puts("        \\   ^__^\n");
	puts("         \\  (oo)\\_______\n");
	puts("            (__)\\       )\\/\\\n");
	puts("             U  ||----w |\n");
	puts("                ||     ||\n");

	return 0;}

#ifdef CONFIG_SYS_EEPROM_MAC_OFFSET
int mac_read_from_eeprom(void)
{
	int retry=0;
	uchar buf[32];
	uchar str[32];
	printf("Reading MAC Address from EEPROM@0x%02x: ", CONFIG_SYS_I2C_EEPROM_ADDR);

	while(retry<10)
	{	
		if (eeprom_read(CONFIG_SYS_I2C_EEPROM_ADDR, CONFIG_SYS_EEPROM_MAC_OFFSET, buf, 6)) {
			
			retry++;
		} else {
			printf("%02X:%02X:%02X:%02X:%02X:%02X - ", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

			eth_setenv_enetaddr("ethaddr", buf);
			eth_setenv_enetaddr("fec_addr", buf);
			break;
		}
	}
	if (retry==10)
		printf("Can't load MAC address from eeprom!\n");
	return 0;
}
#endif	/* CONFIG_SYS_EEPROM_MAC_OFFSET */


int skamngment_switch_config(void)
{
	miiphy_init();
	miiphy_set_current_dev("FEC");
	

}
