// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host controller driver for Freescale i.MX6 SoCs
 *
 * Copyright (C) 2013 Kosagi
 *		http://www.kosagi.com
 *
 * Author: Sean Cross <xobs@kosagi.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/mfd/syscon/imx7-iomuxc-gpr.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/reset.h>
#include <linux/of_address.h>

#include "pcie-designware.h"

#define to_imx6_pcie(x)	dev_get_drvdata((x)->dev)

enum imx6_pcie_variants {
	IMX6Q,
	IMX6SX,
	IMX6QP,
	IMX7D,
	IMX8MQ,
};

struct imx6_pcie {
	struct dw_pcie		*pci;
	u32			ctrl_id;
	u32			cpu_base;
	u32			hard_wired;
	int			clkreq_gpio;
	int			dis_gpio;
	int			power_on_gpio;
	int			reset_gpio;
	bool			gpio_active_high;
	struct clk		*pcie_bus;
	struct clk		*pcie_phy;
	struct clk		*pcie_inbound_axi;
	struct clk		*pcie;
	struct regmap		*iomuxc_gpr;
	struct reset_control	*pciephy_reset;
	struct reset_control	*apps_reset;
	enum imx6_pcie_variants variant;
	u32			hsio_cfg;
	u32			tx_deemph_gen1;
	u32			tx_deemph_gen2_3p5db;
	u32			tx_deemph_gen2_6db;
	u32			tx_swing_full;
	u32			tx_swing_low;
	int			link_gen;
	int			clk2_gen;
	void __iomem		*phy_base;
	struct regmap		*ccm_base;
	struct regmap		*reg_src;
	struct regmap		*reg_gpc;
	struct regulator	*vpcie;
	struct regulator	*epdev_on;
	struct regulator	*pcie_bus_regulator;
};

/* Parameters for the waiting for PCIe PHY PLL to lock on i.MX7 */
#define PHY_PLL_LOCK_WAIT_MAX_RETRIES	2000
#define PHY_PLL_LOCK_WAIT_USLEEP_MIN	50
#define PHY_PLL_LOCK_WAIT_USLEEP_MAX	200

/* PCIe Root Complex registers (memory-mapped) */
#define PCIE_RC_LCR				0x7c
#define PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN1	0x1
#define PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN2	0x2
#define PCIE_RC_LCR_MAX_LINK_SPEEDS_MASK	0xf

#define PCIE_RC_LCSR				0x80

/* PCIe Port Logic registers (memory-mapped) */
#define PL_OFFSET 0x700
#define PCIE_PL_PFLR (PL_OFFSET + 0x08)
#define PCIE_PL_PFLR_LINK_STATE_MASK		(0x3f << 16)
#define PCIE_PL_PFLR_FORCE_LINK			(1 << 15)
#define PCIE_PHY_DEBUG_R0 (PL_OFFSET + 0x28)
#define PCIE_PHY_DEBUG_R1 (PL_OFFSET + 0x2c)
#define PCIE_PHY_DEBUG_R1_XMLH_LINK_IN_TRAINING	(1 << 29)
#define PCIE_PHY_DEBUG_R1_XMLH_LINK_UP		(1 << 4)

#define PCIE_PHY_CTRL (PL_OFFSET + 0x114)
#define PCIE_PHY_CTRL_DATA_LOC 0
#define PCIE_PHY_CTRL_CAP_ADR_LOC 16
#define PCIE_PHY_CTRL_CAP_DAT_LOC 17
#define PCIE_PHY_CTRL_WR_LOC 18
#define PCIE_PHY_CTRL_RD_LOC 19

#define PCIE_PHY_STAT (PL_OFFSET + 0x110)
#define PCIE_PHY_STAT_ACK_LOC 16

#define PCIE_LINK_WIDTH_SPEED_CONTROL	0x80C
#define PORT_LOGIC_SPEED_CHANGE		(0x1 << 17)

/* PHY registers (not memory-mapped) */
#define PCIE_PHY_RX_ASIC_OUT 0x100D
#define PCIE_PHY_RX_ASIC_OUT_VALID	(1 << 0)

#define PHY_RX_OVRD_IN_LO 0x1005
#define PHY_RX_OVRD_IN_LO_RX_DATA_EN (1 << 5)
#define PHY_RX_OVRD_IN_LO_RX_PLL_EN (1 << 3)

#define IMX8MQ_SRC_PCIEPHY_RCR_OFFSET	       0x2C
#define IMX8MQ_SRC_PCIE2PHY_RCR_OFFSET	       0x48
#define IMX8MQ_PCIEPHY_PWR_ON_RST	       BIT(0)
#define IMX8MQ_PCIEPHY_G_RST		       BIT(1)
#define IMX8MQ_PCIEPHY_BTN		       BIT(2)
#define IMX8MQ_PCIEPHY_PERST		       BIT(3)
#define IMX8MQ_PCIE_CTRL_APPS_EN	       BIT(6)
#define IMX8MQ_PCIE_CTRL_APPS_TURNOFF	       BIT(11)

#define IMX8MQ_GPC_PGC_CPU_0_1_MAPPING_OFFSET  0xEC
#define IMX8MQ_GPC_PU_PGC_SW_PUP_REQ_OFFSET    0xF8
#define IMX8MQ_GPC_PU_PGC_SW_PDN_REQ_OFFSET    0x104
#define IMX8MQ_GPC_PGC_PCIE_CTRL_OFFSET		       0xC40
#define IMX8MQ_GPC_PGC_PCIE2_CTRL_OFFSET       0xF00
#define IMX8MQ_GPC_PGC_PCIE_A53_DOMAIN	       BIT(3)
#define IMX8MQ_GPC_PU_PGC_PCIE_SW_PWR_REQ      BIT(1)
#define IMX8MQ_GPC_PGC_PCIE2_BIT_OFFSET		       12
#define IMX8MQ_GPC_PCG_PCIE_CTRL_PCR	       BIT(0)
#define IMX8MQ_GPR_PCIE_REF_USE_PAD	       BIT(9)

/* iMX8 CCM registers */
#define IMX8MQ_CCM_ANALOG_PLLOUT_MONITOR_CFG_OFFSET	0x74
#define IMX8MQ_CCM_PLLOUT_MONITOR_CKE			BIT(4)
#define IMX8MQ_CCM_PLLOUT_MONITOR_CLK_SEL(x)		((x) & 0xF)
#define IMX8MQ_CCM_PLLOUT_MONITOR_CLK_SEL_CLK2		\
					IMX8MQ_CCM_PLLOUT_MONITOR_CLK_SEL(0xB)
#define IMX8MQ_CCM_PLLOUT_MONITOR_CLK_SEL_MSK		\
					IMX8MQ_CCM_PLLOUT_MONITOR_CLK_SEL(0xF)

#define IMX8MQ_CCM_ANALOG_SCCG_PLLOUT_DIV_CFG_OFFSET	0x7C
#define IMX8MQ_CCM_SYSTEM_PLL1_DIV_VAL(x)		((((x) - 1) & 7) <<  0)
#define IMX8MQ_CCM_SYSTEM_PLL1_DIV_MSK			\
					IMX8MQ_CCM_SYSTEM_PLL1_DIV_VAL(8)

static int pcie_phy_poll_ack(struct imx6_pcie *imx6_pcie, int exp_val)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	u32 val;
	u32 max_iterations = 10;
	u32 wait_counter = 0;

	do {
		val = dw_pcie_readl_dbi(pci, PCIE_PHY_STAT);
		val = (val >> PCIE_PHY_STAT_ACK_LOC) & 0x1;
		wait_counter++;

		if (val == exp_val)
			return 0;

		udelay(1);
	} while (wait_counter < max_iterations);

	return -ETIMEDOUT;
}

static int pcie_phy_wait_ack(struct imx6_pcie *imx6_pcie, int addr)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	u32 val;
	int ret;

	val = addr << PCIE_PHY_CTRL_DATA_LOC;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, val);

	val |= (0x1 << PCIE_PHY_CTRL_CAP_ADR_LOC);
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, val);

	ret = pcie_phy_poll_ack(imx6_pcie, 1);
	if (ret)
		return ret;

	val = addr << PCIE_PHY_CTRL_DATA_LOC;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, val);

	return pcie_phy_poll_ack(imx6_pcie, 0);
}

/* Read from the 16-bit PCIe PHY control registers (not memory-mapped) */
static int pcie_phy_read(struct imx6_pcie *imx6_pcie, int addr, int *data)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	u32 val, phy_ctl;
	int ret;

	ret = pcie_phy_wait_ack(imx6_pcie, addr);
	if (ret)
		return ret;

	/* assert Read signal */
	phy_ctl = 0x1 << PCIE_PHY_CTRL_RD_LOC;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, phy_ctl);

	ret = pcie_phy_poll_ack(imx6_pcie, 1);
	if (ret)
		return ret;

	val = dw_pcie_readl_dbi(pci, PCIE_PHY_STAT);
	*data = val & 0xffff;

	/* deassert Read signal */
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, 0x00);

	return pcie_phy_poll_ack(imx6_pcie, 0);
}

static int pcie_phy_write(struct imx6_pcie *imx6_pcie, int addr, int data)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	u32 var;
	int ret;

	/* write addr */
	/* cap addr */
	ret = pcie_phy_wait_ack(imx6_pcie, addr);
	if (ret)
		return ret;

	var = data << PCIE_PHY_CTRL_DATA_LOC;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	/* capture data */
	var |= (0x1 << PCIE_PHY_CTRL_CAP_DAT_LOC);
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	ret = pcie_phy_poll_ack(imx6_pcie, 1);
	if (ret)
		return ret;

	/* deassert cap data */
	var = data << PCIE_PHY_CTRL_DATA_LOC;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	/* wait for ack de-assertion */
	ret = pcie_phy_poll_ack(imx6_pcie, 0);
	if (ret)
		return ret;

	/* assert wr signal */
	var = 0x1 << PCIE_PHY_CTRL_WR_LOC;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	/* wait for ack */
	ret = pcie_phy_poll_ack(imx6_pcie, 1);
	if (ret)
		return ret;

	/* deassert wr signal */
	var = data << PCIE_PHY_CTRL_DATA_LOC;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	/* wait for ack de-assertion */
	ret = pcie_phy_poll_ack(imx6_pcie, 0);
	if (ret)
		return ret;

	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, 0x0);

	return 0;
}

static void imx6_pcie_reset_phy(struct imx6_pcie *imx6_pcie)
{
	u32 tmp;

	pcie_phy_read(imx6_pcie, PHY_RX_OVRD_IN_LO, &tmp);
	tmp |= (PHY_RX_OVRD_IN_LO_RX_DATA_EN |
		PHY_RX_OVRD_IN_LO_RX_PLL_EN);
	pcie_phy_write(imx6_pcie, PHY_RX_OVRD_IN_LO, tmp);

	usleep_range(2000, 3000);

	pcie_phy_read(imx6_pcie, PHY_RX_OVRD_IN_LO, &tmp);
	tmp &= ~(PHY_RX_OVRD_IN_LO_RX_DATA_EN |
		  PHY_RX_OVRD_IN_LO_RX_PLL_EN);
	pcie_phy_write(imx6_pcie, PHY_RX_OVRD_IN_LO, tmp);
}

#ifdef CONFIG_ARM
/*  Added for PCI abort handling */
static int imx6q_pcie_abort_handler(unsigned long addr,
		unsigned int fsr, struct pt_regs *regs)
{
	unsigned long pc = instruction_pointer(regs);
	unsigned long instr = *(unsigned long *)pc;
	int reg = (instr >> 12) & 15;

	/*
	 * If the instruction being executed was a read,
	 * make it look like it read all-ones.
	 */
	if ((instr & 0x0c100000) == 0x04100000) {
		unsigned long val;

		if (instr & 0x00400000)
			val = 255;
		else
			val = -1;

		regs->uregs[reg] = val;
		regs->ARM_pc += 4;
		return 0;
	}

	if ((instr & 0x0e100090) == 0x00100090) {
		regs->uregs[reg] = -1;
		regs->ARM_pc += 4;
		return 0;
	}

	return 1;
}
#endif

static void imx6_pcie_assert_core_reset(struct imx6_pcie *imx6_pcie)
{
	struct device *dev = imx6_pcie->pci->dev;
	u32 val;

	switch (imx6_pcie->variant) {
	case IMX7D:
		reset_control_assert(imx6_pcie->pciephy_reset);
		reset_control_assert(imx6_pcie->apps_reset);
		break;
	case IMX6SX:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6SX_GPR12_PCIE_TEST_POWERDOWN,
				   IMX6SX_GPR12_PCIE_TEST_POWERDOWN);
		/* Force PCIe PHY reset */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR5,
				   IMX6SX_GPR5_PCIE_BTNRST_RESET,
				   IMX6SX_GPR5_PCIE_BTNRST_RESET);
		break;
	case IMX6QP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_SW_RST,
				   IMX6Q_GPR1_PCIE_SW_RST);
		break;
	case IMX6Q:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_TEST_PD, 1 << 18);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_REF_CLK_EN, 0 << 16);
		break;
	case IMX8MQ:
	       if (imx6_pcie->ctrl_id == 0)
		       val = IMX8MQ_SRC_PCIEPHY_RCR_OFFSET;
	       else
		       val = IMX8MQ_SRC_PCIE2PHY_RCR_OFFSET;
	       /* Do RSTs */
	       regmap_update_bits(imx6_pcie->reg_src, val,
			       IMX8MQ_PCIEPHY_BTN,
			       IMX8MQ_PCIEPHY_BTN);
	       regmap_update_bits(imx6_pcie->reg_src, val,
			       IMX8MQ_PCIEPHY_G_RST,
			       IMX8MQ_PCIEPHY_G_RST);
	}



	if (imx6_pcie->vpcie && regulator_is_enabled(imx6_pcie->vpcie) > 0) {
		int ret = regulator_disable(imx6_pcie->vpcie);

		if (ret)
			dev_err(dev, "failed to disable vpcie regulator: %d\n",
				ret);
	}
}

static int imx6_pcie_enable_ref_clk(struct imx6_pcie *imx6_pcie)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;
	int ret = 0;

	switch (imx6_pcie->variant) {
	case IMX6SX:
		ret = clk_prepare_enable(imx6_pcie->pcie_inbound_axi);
		if (ret) {
			dev_err(dev, "unable to enable pcie_axi clock\n");
			break;
		}

		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6SX_GPR12_PCIE_TEST_POWERDOWN, 0);
		break;
	case IMX6QP:		/* FALLTHROUGH */
	case IMX6Q:
		/* power up core phy and enable ref clock */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_TEST_PD, 0 << 18);
		/*
		 * the async reset input need ref clock to sync internally,
		 * when the ref clock comes after reset, internal synced
		 * reset time is too short, cannot meet the requirement.
		 * add one ~10us delay here.
		 */
		udelay(10);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_REF_CLK_EN, 1 << 16);
		break;
	case IMX7D:
	case IMX8MQ:
		break;
	}

	return ret;
}

static void imx7d_pcie_wait_for_phy_pll_lock(struct imx6_pcie *imx6_pcie)
{
	u32 val;
	unsigned int retries;
	struct device *dev = imx6_pcie->pci->dev;

	for (retries = 0; retries < PHY_PLL_LOCK_WAIT_MAX_RETRIES; retries++) {
		regmap_read(imx6_pcie->iomuxc_gpr, IOMUXC_GPR22, &val);

		if (val & IMX7D_GPR22_PCIE_PHY_PLL_LOCKED)
			return;

		usleep_range(PHY_PLL_LOCK_WAIT_USLEEP_MIN,
			     PHY_PLL_LOCK_WAIT_USLEEP_MAX);
	}

	dev_err(dev, "PCIe PLL lock timeout\n");
}

static void imx6_pcie_deassert_core_reset(struct imx6_pcie *imx6_pcie)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;
	int ret;
	u32 val;

	if (imx6_pcie->vpcie && !regulator_is_enabled(imx6_pcie->vpcie)) {
		ret = regulator_enable(imx6_pcie->vpcie);
		if (ret) {
			dev_err(dev, "failed to enable vpcie regulator: %d\n",
				ret);
			return;
		}
	}

	ret = clk_prepare_enable(imx6_pcie->pcie_phy);
	if (ret) {
		dev_err(dev, "unable to enable pcie_phy clock\n");
		goto err_pcie_phy;
	}

	ret = clk_prepare_enable(imx6_pcie->pcie_bus);
	if (ret) {
		dev_err(dev, "unable to enable pcie_bus clock\n");
		goto err_pcie_bus;
	}

	ret = clk_prepare_enable(imx6_pcie->pcie);
	if (ret) {
		dev_err(dev, "unable to enable pcie clock\n");
		goto err_pcie;
	}

	ret = imx6_pcie_enable_ref_clk(imx6_pcie);
	if (ret) {
		dev_err(dev, "unable to enable pcie ref clock\n");
		goto err_ref_clk;
	}

	/* allow the clocks to stabilize */
	usleep_range(200, 500);

	/* Some boards don't have PCIe reset GPIO. */
	if (gpio_is_valid(imx6_pcie->reset_gpio)) {
		gpio_set_value_cansleep(imx6_pcie->reset_gpio,
					imx6_pcie->gpio_active_high);
		msleep(100);
		gpio_set_value_cansleep(imx6_pcie->reset_gpio,
					!imx6_pcie->gpio_active_high);
	}

	switch (imx6_pcie->variant) {
	case IMX7D:
		reset_control_deassert(imx6_pcie->pciephy_reset);
		imx7d_pcie_wait_for_phy_pll_lock(imx6_pcie);
		break;
	case IMX6SX:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR5,
				   IMX6SX_GPR5_PCIE_BTNRST_RESET, 0);
		break;
	case IMX6QP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_SW_RST, 0);

		usleep_range(200, 500);
		break;
	case IMX6Q:		/* Nothing to do */
		break;

	case IMX8MQ:
	  /* wait for more than 10us to release phy g_rst and btnrst */
	  udelay(10);
	  if (imx6_pcie->ctrl_id == 0)
	    val = IMX8MQ_SRC_PCIEPHY_RCR_OFFSET;
	  else
	    val = IMX8MQ_SRC_PCIE2PHY_RCR_OFFSET;
	  regmap_update_bits(imx6_pcie->reg_src, val,
			     IMX8MQ_PCIEPHY_BTN, 0);
	  regmap_update_bits(imx6_pcie->reg_src, val,
			     IMX8MQ_PCIEPHY_G_RST, 0);
	  regmap_update_bits(imx6_pcie->reg_src, val,
			     IMX8MQ_PCIE_CTRL_APPS_EN, 0);
	  break;
	}

	return;

err_ref_clk:
	clk_disable_unprepare(imx6_pcie->pcie);
err_pcie:
	clk_disable_unprepare(imx6_pcie->pcie_bus);
err_pcie_bus:
	clk_disable_unprepare(imx6_pcie->pcie_phy);
err_pcie_phy:
	if (imx6_pcie->vpcie && regulator_is_enabled(imx6_pcie->vpcie) > 0) {
		ret = regulator_disable(imx6_pcie->vpcie);
		if (ret)
			dev_err(dev, "failed to disable vpcie regulator: %d\n",
				ret);
	}
}

static void imx6_pcie_phy_pwr_up(struct imx6_pcie *imx6_pcie)
{
	u32 val, offset;
	unsigned long timeout = jiffies + msecs_to_jiffies(500);

	if (imx6_pcie->variant != IMX8MQ)
		return;
	/*
	 * Power up PHY.
	 * pcie phy ref clock select by gpr configuration.
	 * 1? external osc : internal pll
	 */

	if (imx6_pcie->ctrl_id == 0)
		offset = 0;
	else
		offset = IMX8MQ_GPC_PGC_PCIE2_BIT_OFFSET;

	regmap_update_bits(imx6_pcie->reg_gpc,
			IMX8MQ_GPC_PGC_CPU_0_1_MAPPING_OFFSET,
			IMX8MQ_GPC_PGC_PCIE_A53_DOMAIN << offset,
			IMX8MQ_GPC_PGC_PCIE_A53_DOMAIN << offset);
	regmap_update_bits(imx6_pcie->reg_gpc,
			IMX8MQ_GPC_PU_PGC_SW_PUP_REQ_OFFSET,
			IMX8MQ_GPC_PU_PGC_PCIE_SW_PWR_REQ << offset,
			IMX8MQ_GPC_PU_PGC_PCIE_SW_PWR_REQ << offset);

	regmap_read(imx6_pcie->reg_gpc,
			IMX8MQ_GPC_PU_PGC_SW_PUP_REQ_OFFSET,
			&val);
	while (val & (IMX8MQ_GPC_PU_PGC_PCIE_SW_PWR_REQ << offset)) {
		regmap_read(imx6_pcie->reg_gpc,
				IMX8MQ_GPC_PU_PGC_SW_PUP_REQ_OFFSET,
				&val);
		if (time_after(jiffies, timeout)) {
			pr_err("CAN NOT PWR UP PCIE%d PHY!\n",
					imx6_pcie->ctrl_id);
			break;
		}
	}
	udelay(1);
}


static void imx6_pcie_init_phy(struct imx6_pcie *imx6_pcie)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	u32 val;

	switch (imx6_pcie->variant) {
	case IMX8MQ:
	  imx6_pcie_phy_pwr_up(imx6_pcie);

	  if (imx6_pcie->ctrl_id == 0)
	    val = IOMUXC_GPR14;
	  else
	    val = IOMUXC_GPR16;

	  /* Use internal clk if we should generate PCIe clock on CLK2 */
	  regmap_update_bits(imx6_pcie->iomuxc_gpr, val,
			     IMX8MQ_GPR_PCIE_REF_USE_PAD,
			     imx6_pcie->clk2_gen ?
			     0 : IMX8MQ_GPR_PCIE_REF_USE_PAD);

	case IMX7D:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX7D_GPR12_PCIE_PHY_REFCLK_SEL, 0);
		break;
	case IMX6SX:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6SX_GPR12_PCIE_RX_EQ_MASK,
				   IMX6SX_GPR12_PCIE_RX_EQ_2);
		/* FALLTHROUGH */
	default:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6Q_GPR12_PCIE_CTL_2, 0 << 10);

		/* configure constant input signal to the pcie ctrl and phy */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6Q_GPR12_LOS_LEVEL, 9 << 4);

		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_DEEMPH_GEN1,
				   imx6_pcie->tx_deemph_gen1 << 0);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_DEEMPH_GEN2_3P5DB,
				   imx6_pcie->tx_deemph_gen2_3p5db << 6);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_DEEMPH_GEN2_6DB,
				   imx6_pcie->tx_deemph_gen2_6db << 12);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_SWING_FULL,
				   imx6_pcie->tx_swing_full << 18);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_SWING_LOW,
				   imx6_pcie->tx_swing_low << 25);
		break;
	}

	if (imx6_pcie->variant != IMX8MQ) {
	  regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
			IMX6Q_GPR12_DEVICE_TYPE, PCI_EXP_TYPE_ROOT_PORT << 12);
	}

	if (imx6_pcie->pcie_bus_regulator != NULL) {
	  if (regulator_enable(imx6_pcie->pcie_bus_regulator))
	    dev_err(pci->dev, "failed to enable pcie regulator.\n");
	}

	/* configure the device type */
	if (IS_ENABLED(CONFIG_EP_MODE_IN_EP_RC_SYS)) {
	  if (unlikely(imx6_pcie->ctrl_id))
	    /* iMX8MQ second PCIE */
	    regmap_update_bits(imx6_pcie->iomuxc_gpr,
			       IOMUXC_GPR12,
			       IMX6Q_GPR12_DEVICE_TYPE >> 4,
			       PCI_EXP_TYPE_ROOT_PORT << 8);
	  else
	    regmap_update_bits(imx6_pcie->iomuxc_gpr,
			       IOMUXC_GPR12,
			       IMX6Q_GPR12_DEVICE_TYPE,
			       PCI_EXP_TYPE_ROOT_PORT << 12);
	}
}

static int imx6_pcie_wait_for_link(struct imx6_pcie *imx6_pcie)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;

	/* check if the link is up or not */
	if (!dw_pcie_wait_for_link(pci))
		return 0;

	dev_dbg(dev, "DEBUG_R0: 0x%08x, DEBUG_R1: 0x%08x\n",
		dw_pcie_readl_dbi(pci, PCIE_PHY_DEBUG_R0),
		dw_pcie_readl_dbi(pci, PCIE_PHY_DEBUG_R1));
	return -ETIMEDOUT;
}

static int imx6_pcie_wait_for_speed_change(struct imx6_pcie *imx6_pcie)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;
	u32 tmp;
	unsigned int retries;

	for (retries = 0; retries < 200; retries++) {
		tmp = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
		/* Test if the speed change finished. */
		if (!(tmp & PORT_LOGIC_SPEED_CHANGE))
			return 0;
		usleep_range(100, 1000);
	}

	dev_err(dev, "Speed change timeout\n");
	return -EINVAL;
}

static int imx6_pcie_establish_link(struct imx6_pcie *imx6_pcie)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;
	u32 tmp;
	int ret;

	/*
	 * Force Gen1 operation when starting the link.	 In case the link is
	 * started in Gen2 mode, there is a possibility the devices on the
	 * bus will not be detected at all.  This happens with PCIe switches.
	 */
	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_LCR);
	tmp &= ~PCIE_RC_LCR_MAX_LINK_SPEEDS_MASK;
	tmp |= PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN1;
	dw_pcie_writel_dbi(pci, PCIE_RC_LCR, tmp);

	/* Start LTSSM. */
	if (imx6_pcie->variant == IMX7D)
		reset_control_deassert(imx6_pcie->apps_reset);
	else
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6Q_GPR12_PCIE_CTL_2, 1 << 10);

	ret = imx6_pcie_wait_for_link(imx6_pcie);
	if (ret)
		goto err_reset_phy;

	if (imx6_pcie->link_gen == 2) {
		/* Allow Gen2 mode after the link is up. */
		tmp = dw_pcie_readl_dbi(pci, PCIE_RC_LCR);
		tmp &= ~PCIE_RC_LCR_MAX_LINK_SPEEDS_MASK;
		tmp |= PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN2;
		dw_pcie_writel_dbi(pci, PCIE_RC_LCR, tmp);

		/*
		 * Start Directed Speed Change so the best possible
		 * speed both link partners support can be negotiated.
		 */
		tmp = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
		tmp |= PORT_LOGIC_SPEED_CHANGE;
		dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, tmp);

		if (imx6_pcie->variant != IMX7D) {
			/*
			 * On i.MX7, DIRECT_SPEED_CHANGE behaves differently
			 * from i.MX6 family when no link speed transition
			 * occurs and we go Gen1 -> yep, Gen1. The difference
			 * is that, in such case, it will not be cleared by HW
			 * which will cause the following code to report false
			 * failure.
			 */

			ret = imx6_pcie_wait_for_speed_change(imx6_pcie);
			if (ret) {
				dev_err(dev, "Failed to bring link up!\n");
				goto err_reset_phy;
			}
		}

		/* Make sure link training is finished as well! */
		ret = imx6_pcie_wait_for_link(imx6_pcie);
		if (ret) {
			dev_err(dev, "Failed to bring link up!\n");
			goto err_reset_phy;
		}
	} else {
		dev_info(dev, "Link: Gen2 disabled\n");
	}

	tmp = dw_pcie_readl_dbi(pci, PCIE_RC_LCSR);
	dev_info(dev, "Link up, Gen%i\n", (tmp >> 16) & 0xf);
	return 0;

err_reset_phy:
	dev_dbg(dev, "PHY DEBUG_R0=0x%08x DEBUG_R1=0x%08x\n",
		dw_pcie_readl_dbi(pci, PCIE_PHY_DEBUG_R0),
		dw_pcie_readl_dbi(pci, PCIE_PHY_DEBUG_R1));
	imx6_pcie_reset_phy(imx6_pcie);
	return ret;
}

static int imx6_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct imx6_pcie *imx6_pcie = to_imx6_pcie(pci);

	imx6_pcie_assert_core_reset(imx6_pcie);
	imx6_pcie_init_phy(imx6_pcie);
	imx6_pcie_deassert_core_reset(imx6_pcie);
	dw_pcie_setup_rc(pp);
	imx6_pcie_establish_link(imx6_pcie);

	if (IS_ENABLED(CONFIG_PCI_MSI))
		dw_pcie_msi_init(pp);

	return 0;
}

static int imx6_pcie_link_up(struct dw_pcie *pci)
{
	return dw_pcie_readl_dbi(pci, PCIE_PHY_DEBUG_R1) &
			PCIE_PHY_DEBUG_R1_XMLH_LINK_UP;
}

static const struct dw_pcie_host_ops imx6_pcie_host_ops = {
	.host_init = imx6_pcie_host_init,
};

static int imx6_add_pcie_port(struct imx6_pcie *imx6_pcie,
			      struct platform_device *pdev)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	struct pcie_port *pp = &pci->pp;
	struct device *dev = &pdev->dev;
	int ret;

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pp->msi_irq = platform_get_irq_byname(pdev, "msi");
		if (pp->msi_irq <= 0) {
			dev_err(dev, "failed to get MSI irq\n");
			return -ENODEV;
		}
	}

	pp->root_bus_nr = -1;
	pp->ops = &imx6_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static const struct dw_pcie_ops dw_pcie_ops = {
	.link_up = imx6_pcie_link_up,
};

static int imx6_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci;
	struct imx6_pcie *imx6_pcie;
	struct device_node *np;
	struct resource *dbi_base;
	struct device_node *node = dev->of_node;
	int ret;

	imx6_pcie = devm_kzalloc(dev, sizeof(*imx6_pcie), GFP_KERNEL);
	if (!imx6_pcie)
		return -ENOMEM;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pci->dev = dev;
	pci->ops = &dw_pcie_ops;

	imx6_pcie->pci = pci;
	imx6_pcie->variant =
		(enum imx6_pcie_variants)of_device_get_match_data(dev);

	if (of_property_read_u32(node, "hsio-cfg", &imx6_pcie->hsio_cfg))
		imx6_pcie->hsio_cfg = 0;

	if (of_property_read_u32(node, "ctrl-id", &imx6_pcie->ctrl_id))
		imx6_pcie->ctrl_id = 0;

	if (of_property_read_u32(node, "cpu-base-addr", &imx6_pcie->cpu_base))
		imx6_pcie->cpu_base = 0;
	if (of_property_read_u32(node, "hard-wired", &imx6_pcie->hard_wired))
		imx6_pcie->hard_wired = 0;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx-pcie-phy");
	if (np != NULL) {
		imx6_pcie->phy_base = of_iomap(np, 0);
		WARN_ON(!imx6_pcie->phy_base);
	} else {
		imx6_pcie->phy_base = NULL;
	}


	dbi_base = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pci->dbi_base = devm_ioremap_resource(dev, dbi_base);
	if (IS_ERR(pci->dbi_base))
		return PTR_ERR(pci->dbi_base);

	/* Fetch GPIOs */
	imx6_pcie->clkreq_gpio = of_get_named_gpio(node, "clkreq-gpio", 0);
	if (gpio_is_valid(imx6_pcie->clkreq_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev, imx6_pcie->clkreq_gpio,
					    GPIOF_OUT_INIT_LOW, "PCIe CLKREQ");
		if (ret) {
			dev_err(&pdev->dev, "unable to get clkreq gpio\n");
			return ret;
		}
	}
	imx6_pcie->dis_gpio = of_get_named_gpio(node, "disable-gpio", 0);
	if (gpio_is_valid(imx6_pcie->dis_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev, imx6_pcie->dis_gpio,
					    GPIOF_OUT_INIT_HIGH, "PCIe DIS");
		if (ret) {
			dev_err(&pdev->dev, "unable to get disable gpio\n");
			return ret;
		}
	} else if (imx6_pcie->dis_gpio == -EPROBE_DEFER) {
		return imx6_pcie->dis_gpio;
	}

	imx6_pcie->power_on_gpio = of_get_named_gpio(node, "power-on-gpio", 0);
	if (gpio_is_valid(imx6_pcie->power_on_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev,
					    imx6_pcie->power_on_gpio,
					    GPIOF_OUT_INIT_LOW,
					    "PCIe power enable");
		if (ret) {
			dev_err(&pdev->dev, "unable to get power-on gpio\n");
			return ret;
		}
	} else if (imx6_pcie->power_on_gpio == -EPROBE_DEFER) {
		return imx6_pcie->power_on_gpio;
	}

	imx6_pcie->reset_gpio = of_get_named_gpio(node, "reset-gpio", 0);
	imx6_pcie->gpio_active_high = of_property_read_bool(node,
						"reset-gpio-active-high");
	if (gpio_is_valid(imx6_pcie->reset_gpio)) {
		ret = devm_gpio_request_one(dev, imx6_pcie->reset_gpio,
				imx6_pcie->gpio_active_high ?
					GPIOF_OUT_INIT_HIGH :
					GPIOF_OUT_INIT_LOW,
				"PCIe reset");
		if (ret) {
			dev_err(dev, "unable to get reset gpio\n");
			return ret;
		}
	} else if (imx6_pcie->reset_gpio == -EPROBE_DEFER) {
		return imx6_pcie->reset_gpio;
	}

	imx6_pcie->epdev_on = devm_regulator_get(&pdev->dev, "epdev_on");
	if (IS_ERR(imx6_pcie->epdev_on)) {
		if (PTR_ERR(imx6_pcie->epdev_on) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_info(dev, "no ep regulator found\n");
		imx6_pcie->epdev_on = NULL;
	} else {
		ret = regulator_enable(imx6_pcie->epdev_on);
		if (ret)
			dev_err(dev, "failed to enable the epdev_on regulator\n");
	}

	/* Fetch clocks */
	imx6_pcie->pcie_phy = devm_clk_get(dev, "pcie_phy");
	if (IS_ERR(imx6_pcie->pcie_phy)) {
		dev_err(dev, "pcie_phy clock source missing or invalid\n");
		return PTR_ERR(imx6_pcie->pcie_phy);
	}

	imx6_pcie->pcie_bus = devm_clk_get(dev, "pcie_bus");
	if (IS_ERR(imx6_pcie->pcie_bus)) {
		dev_err(dev, "pcie_bus clock source missing or invalid\n");
		return PTR_ERR(imx6_pcie->pcie_bus);
	}

	imx6_pcie->pcie = devm_clk_get(dev, "pcie");
	if (IS_ERR(imx6_pcie->pcie)) {
		dev_err(dev, "pcie clock source missing or invalid\n");
		return PTR_ERR(imx6_pcie->pcie);
	}

	switch (imx6_pcie->variant) {
	case IMX6SX:
		imx6_pcie->pcie_inbound_axi = devm_clk_get(dev,
							   "pcie_inbound_axi");
		if (IS_ERR(imx6_pcie->pcie_inbound_axi)) {
			dev_err(dev, "pcie_inbound_axi clock missing or invalid\n");
			return PTR_ERR(imx6_pcie->pcie_inbound_axi);
		}
		break;
	case IMX7D:
		imx6_pcie->pciephy_reset = devm_reset_control_get_exclusive(dev,
									    "pciephy");
		if (IS_ERR(imx6_pcie->pciephy_reset)) {
			dev_err(dev, "Failed to get PCIEPHY reset control\n");
			return PTR_ERR(imx6_pcie->pciephy_reset);
		}

		imx6_pcie->apps_reset = devm_reset_control_get_exclusive(dev,
									 "apps");
		if (IS_ERR(imx6_pcie->apps_reset)) {
			dev_err(dev, "Failed to get PCIE APPS reset control\n");
			return PTR_ERR(imx6_pcie->apps_reset);
		}
		break;
	default:
		break;
	}

	/* Grab GPR config register range */
	if (imx6_pcie->variant == IMX8MQ) {
	  imx6_pcie->iomuxc_gpr =
	    syscon_regmap_lookup_by_compatible
	    ("fsl,imx8mq-iomuxc-gpr");
	  imx6_pcie->reg_src =
	    syscon_regmap_lookup_by_compatible("fsl,imx8mq-src");
	  if (IS_ERR(imx6_pcie->reg_src)) {
	    dev_err(&pdev->dev,
		    "imx8mq pcie phy src missing or invalid\n");
	    return PTR_ERR(imx6_pcie->reg_src);
	  }
	  imx6_pcie->reg_gpc =
	    syscon_regmap_lookup_by_compatible("fsl,imx8mq-gpc");
	  if (IS_ERR(imx6_pcie->reg_gpc)) {
	    dev_err(&pdev->dev,
		    "imx8mq pcie phy src missing or invalid\n");
	    return PTR_ERR(imx6_pcie->reg_gpc);
	  }
	  imx6_pcie->clk2_gen = of_property_read_bool(node, "clk2-gen");
	  if (imx6_pcie->clk2_gen) {
	    /* Generate 100MHz PCIe clock on CLK2_P/N */
	    imx6_pcie->ccm_base =
	      syscon_regmap_lookup_by_compatible(
						 "fsl,imx8mq-anatop");
	    if (IS_ERR(imx6_pcie->ccm_base)) {
	      dev_err(&pdev->dev,
		      "imx8mq  ccm src missing or invalid\n");
	      return PTR_ERR(imx6_pcie->ccm_base);
	    }

	    regmap_update_bits(imx6_pcie->ccm_base,
			       IMX8MQ_CCM_ANALOG_PLLOUT_MONITOR_CFG_OFFSET,
			       IMX8MQ_CCM_PLLOUT_MONITOR_CKE |
			       IMX8MQ_CCM_PLLOUT_MONITOR_CLK_SEL_MSK,
			       IMX8MQ_CCM_PLLOUT_MONITOR_CKE |
			       IMX8MQ_CCM_PLLOUT_MONITOR_CLK_SEL_CLK2);

	    regmap_update_bits(imx6_pcie->ccm_base,
			       IMX8MQ_CCM_ANALOG_SCCG_PLLOUT_DIV_CFG_OFFSET,
			       IMX8MQ_CCM_SYSTEM_PLL1_DIV_MSK,
			       IMX8MQ_CCM_SYSTEM_PLL1_DIV_VAL(8));
	  }
	} else {
	  imx6_pcie->iomuxc_gpr =
	    syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	  if (IS_ERR(imx6_pcie->iomuxc_gpr)) {
	    dev_err(dev, "unable to find iomuxc registers\n");
	    return PTR_ERR(imx6_pcie->iomuxc_gpr);
	  }
	}

	/* Grab PCIe PHY Tx Settings */
	if (of_property_read_u32(node, "fsl,tx-deemph-gen1",
				 &imx6_pcie->tx_deemph_gen1))
		imx6_pcie->tx_deemph_gen1 = 0;

	if (of_property_read_u32(node, "fsl,tx-deemph-gen2-3p5db",
				 &imx6_pcie->tx_deemph_gen2_3p5db))
		imx6_pcie->tx_deemph_gen2_3p5db = 0;

	if (of_property_read_u32(node, "fsl,tx-deemph-gen2-6db",
				 &imx6_pcie->tx_deemph_gen2_6db))
		imx6_pcie->tx_deemph_gen2_6db = 20;

	if (of_property_read_u32(node, "fsl,tx-swing-full",
				 &imx6_pcie->tx_swing_full))
		imx6_pcie->tx_swing_full = 127;

	if (of_property_read_u32(node, "fsl,tx-swing-low",
				 &imx6_pcie->tx_swing_low))
		imx6_pcie->tx_swing_low = 127;

	/* Limit link speed */
	ret = of_property_read_u32(node, "fsl,max-link-speed",
				   &imx6_pcie->link_gen);
	if (ret)
		imx6_pcie->link_gen = 1;

	imx6_pcie->vpcie = devm_regulator_get_optional(&pdev->dev, "vpcie");
	if (IS_ERR(imx6_pcie->vpcie)) {
		if (PTR_ERR(imx6_pcie->vpcie) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		imx6_pcie->vpcie = NULL;
	}

	platform_set_drvdata(pdev, imx6_pcie);

	ret = imx6_add_pcie_port(imx6_pcie, pdev);
	if (ret < 0)
		return ret;

	printk("Probed imx8 pcie\n");
	return 0;
}

static void imx6_pcie_shutdown(struct platform_device *pdev)
{
	struct imx6_pcie *imx6_pcie = platform_get_drvdata(pdev);

	/* bring down link, so bootloader gets clean state in case of reboot */
	imx6_pcie_assert_core_reset(imx6_pcie);
}

static const struct of_device_id imx6_pcie_of_match[] = {
	{ .compatible = "fsl,imx6q-pcie",  .data = (void *)IMX6Q,  },
	{ .compatible = "fsl,imx6sx-pcie", .data = (void *)IMX6SX, },
	{ .compatible = "fsl,imx6qp-pcie", .data = (void *)IMX6QP, },
	{ .compatible = "fsl,imx7d-pcie",  .data = (void *)IMX7D,  },
	{ .compatible = "fsl,imx8mq-pcie", .data = (void *)IMX8MQ, },
	{},
};

static struct platform_driver imx6_pcie_driver = {
	.driver = {
		.name	= "imx6q-pcie",
		.of_match_table = imx6_pcie_of_match,
		.suppress_bind_attrs = true,
	},
	.probe	  = imx6_pcie_probe,
	.shutdown = imx6_pcie_shutdown,
};

static int __init imx6_pcie_init(void)
{
#ifdef CONFIG_ARM
	/*
	 * Since probe() can be deferred we need to make sure that
	 * hook_fault_code is not called after __init memory is freed
	 * by kernel and since imx6q_pcie_abort_handler() is a no-op,
	 * we can install the handler here without risking it
	 * accessing some uninitialized driver state.
	 */
	hook_fault_code(8, imx6q_pcie_abort_handler, SIGBUS, 0,
			"external abort on non-linefetch");
#endif

	return platform_driver_register(&imx6_pcie_driver);
}
device_initcall(imx6_pcie_init);
