// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek RTL8197F PCIe root complex.
 *
 * The SoC exposes a single-lane root port with a companion Wi-Fi endpoint
 * (RTL8822BE on the ipTIME A2004MU) hard-wired to it. Bring-up was validated on
 * hardware by poking the registers below directly: the link reaches LTSSM L0 and
 * the endpoint answers config reads with 10ec:b822.
 *
 * Config space is split into two windows. The root port's own registers live at
 * the "rc" window and only tolerate 32-bit accesses, so both windows are driven
 * through the PCI core's 32-bit accessors. The endpoint sits on bus 1 in its own
 * window, indexed by function. Nothing deeper than bus 1 exists on this SoC.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/pci.h>
#include <linux/platform_device.h>

/* System register block. */
#define SYS_STRAP		0x008
#define SYS_STRAP_PCIE_40MHZ	BIT(24)
#define SYS_CLK_MANAGE		0x010
#define SYS_CLK_PCIE		(BIT(12) | BIT(13) | BIT(18))
#define SYS_CLK_PCIE_ACTIVE	BIT(14)
#define SYS_ENABLE		0x050
#define SYS_ENABLE_PERST	BIT(1)		/* endpoint PERST#, active low */
#define SYS_PCIE_PHY		0x100
#define SYS_PCIE_PHY_MDIO_RST	BIT(0)
#define SYS_PCIE_PHY_LOAD_DONE	BIT(1)
#define SYS_PCIE_PHY_ON		BIT(3)

/*
 * The SoC interrupt controller shares the system register window. It routes
 * each source to a CPU interrupt line; the PCIe source is GISR bit 21 and the
 * BSP gives it CPU IP5. IRR2 covers sources 16..23 in 4-bit fields, so source
 * 21 lands in bits 23:20. This mirrors what the ethernet driver does for the
 * switch core on IP4 -- neither needs the (still unwired) ictl cascade.
 */
#define ICTL_GIMR		0x3000
#define ICTL_GIMR_PCIE_IE	BIT(21)
#define ICTL_IRR2		0x3010
#define ICTL_IRR2_PCIE_MASK	(0xfu << 20)
#define ICTL_IRR2_PCIE_IP	(5u << 20)

/* Root-complex extended registers. */
#define PCIE_EXT_MDIO		0x00
#define PCIE_MDIO_WRITE		BIT(0)
#define PCIE_MDIO_REG_SHIFT	8
#define PCIE_MDIO_DATA_SHIFT	16
#define PCIE_EXT_PWRCR		0x08
#define PCIE_PWRCR_LTSSM_EN	BIT(0)
#define PCIE_PWRCR_PHY_RST	BIT(7)

/* Root-complex config space. */
#define PCIE_RC_LINK_STATUS	0x728
#define PCIE_LTSSM_MASK		0x1f
#define PCIE_LTSSM_L0		0x11

#define PCIE_EP_FUNC_SHIFT	12
#define PCIE_LINK_WAIT_MS	100

struct rtl8197f_pcie {
	void __iomem *rc_cfg;
	void __iomem *rc_ext;
	void __iomem *ep_cfg;
	void __iomem *sysreg;
	struct device *dev;
};

static void __iomem *rtl8197f_pcie_map_bus(struct pci_bus *bus,
					   unsigned int devfn, int where)
{
	struct rtl8197f_pcie *pcie = bus->sysdata;

	/* Root port: one device, one function. */
	if (bus->number == 0) {
		if (PCI_SLOT(devfn) || PCI_FUNC(devfn))
			return NULL;
		return pcie->rc_cfg + where;
	}

	/* The single downstream device. */
	if (bus->number == 1) {
		if (PCI_SLOT(devfn))
			return NULL;
		return pcie->ep_cfg + (PCI_FUNC(devfn) << PCIE_EP_FUNC_SHIFT) +
		       where;
	}

	return NULL;
}

static struct pci_ops rtl8197f_pcie_ops = {
	.map_bus = rtl8197f_pcie_map_bus,
	.read	 = pci_generic_config_read32,
	.write	 = pci_generic_config_write32,
};

static void rtl8197f_pcie_phy_write(struct rtl8197f_pcie *pcie, u8 reg, u16 val)
{
	writel(((u32)val << PCIE_MDIO_DATA_SHIFT) |
	       ((u32)(reg & 0x1f) << PCIE_MDIO_REG_SHIFT) | PCIE_MDIO_WRITE,
	       pcie->rc_ext + PCIE_EXT_MDIO);
	udelay(100);
}

/* Release the PHY from reset with the link trainer enabled. */
static void rtl8197f_pcie_phy_reset(struct rtl8197f_pcie *pcie)
{
	writel(PCIE_PWRCR_LTSSM_EN, pcie->rc_ext + PCIE_EXT_PWRCR);
	writel(PCIE_PWRCR_LTSSM_EN | PCIE_PWRCR_PHY_RST,
	       pcie->rc_ext + PCIE_EXT_PWRCR);
}

static void rtl8197f_pcie_phy_setup(struct rtl8197f_pcie *pcie)
{
	/* The reference clock the board strapped decides the PHY tuning. */
	if (readl(pcie->sysreg + SYS_STRAP) & SYS_STRAP_PCIE_40MHZ) {
		rtl8197f_pcie_phy_write(pcie, 0x0f, 0x12f6);
		rtl8197f_pcie_phy_write(pcie, 0x00, 0x0071);
		rtl8197f_pcie_phy_write(pcie, 0x06, 0x1ac1);
	} else {
		rtl8197f_pcie_phy_write(pcie, 0x00, 0x0071);
		rtl8197f_pcie_phy_write(pcie, 0x06, 0x18c1);
	}
}

static int rtl8197f_pcie_wait_link(struct rtl8197f_pcie *pcie)
{
	unsigned int i;
	u32 val;

	for (i = 0; i < PCIE_LINK_WAIT_MS / 10; i++) {
		val = readl(pcie->rc_cfg + PCIE_RC_LINK_STATUS);
		if ((val & PCIE_LTSSM_MASK) == PCIE_LTSSM_L0)
			return 0;
		msleep(10);
	}

	dev_err(pcie->dev, "link training failed, LTSSM 0x%02x\n",
		val & PCIE_LTSSM_MASK);
	return -ETIMEDOUT;
}

static int rtl8197f_pcie_hw_init(struct rtl8197f_pcie *pcie)
{
	u32 val;

	/* Ungate the PCIe block, then mark port 0 active. */
	val = readl(pcie->sysreg + SYS_CLK_MANAGE);
	writel(val | SYS_CLK_PCIE, pcie->sysreg + SYS_CLK_MANAGE);
	val = readl(pcie->sysreg + SYS_CLK_MANAGE);
	writel(val | SYS_CLK_PCIE_ACTIVE, pcie->sysreg + SYS_CLK_MANAGE);
	usleep_range(10000, 11000);

	/* MDIO block out of reset, then latch the parameter set. */
	writel(SYS_PCIE_PHY_ON, pcie->sysreg + SYS_PCIE_PHY);
	writel(SYS_PCIE_PHY_ON | SYS_PCIE_PHY_MDIO_RST,
	       pcie->sysreg + SYS_PCIE_PHY);
	writel(SYS_PCIE_PHY_ON | SYS_PCIE_PHY_MDIO_RST | SYS_PCIE_PHY_LOAD_DONE,
	       pcie->sysreg + SYS_PCIE_PHY);
	usleep_range(10000, 11000);

	/*
	 * The PHY has to come out of reset once before it will accept the
	 * tuning writes, and again afterwards for them to take effect.
	 */
	rtl8197f_pcie_phy_reset(pcie);
	usleep_range(10000, 11000);
	rtl8197f_pcie_phy_setup(pcie);
	usleep_range(10000, 11000);
	rtl8197f_pcie_phy_reset(pcie);

	/* PERST# low long enough to satisfy the endpoint's power-on timing. */
	val = readl(pcie->sysreg + SYS_ENABLE);
	writel(val & ~SYS_ENABLE_PERST, pcie->sysreg + SYS_ENABLE);
	msleep(300);
	writel(val | SYS_ENABLE_PERST, pcie->sysreg + SYS_ENABLE);

	return rtl8197f_pcie_wait_link(pcie);
}

/* Point the PCIe source at its CPU interrupt line and unmask it. */
static void rtl8197f_pcie_irq_route(struct rtl8197f_pcie *pcie)
{
	u32 val;

	val = readl(pcie->sysreg + ICTL_IRR2);
	val = (val & ~ICTL_IRR2_PCIE_MASK) | ICTL_IRR2_PCIE_IP;
	writel(val, pcie->sysreg + ICTL_IRR2);

	writel(readl(pcie->sysreg + ICTL_GIMR) | ICTL_GIMR_PCIE_IE,
	       pcie->sysreg + ICTL_GIMR);
}

static void __iomem *rtl8197f_pcie_map(struct platform_device *pdev,
				       unsigned int idx)
{
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, idx);
	if (!res)
		return NULL;

	/*
	 * Plain ioremap rather than devm_ioremap_resource(): the system
	 * register window is shared with the ethernet driver, which maps it the
	 * same way, so neither may claim it exclusively.
	 */
	return devm_ioremap(&pdev->dev, res->start, resource_size(res));
}

static int rtl8197f_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pci_host_bridge *bridge;
	struct rtl8197f_pcie *pcie;
	int ret;

	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*pcie));
	if (!bridge)
		return -ENOMEM;

	pcie = pci_host_bridge_priv(bridge);
	pcie->dev = dev;

	pcie->rc_cfg = rtl8197f_pcie_map(pdev, 0);
	pcie->rc_ext = rtl8197f_pcie_map(pdev, 1);
	pcie->ep_cfg = rtl8197f_pcie_map(pdev, 2);
	pcie->sysreg = rtl8197f_pcie_map(pdev, 3);
	if (!pcie->rc_cfg || !pcie->rc_ext || !pcie->ep_cfg || !pcie->sysreg)
		return -ENOMEM;

	ret = rtl8197f_pcie_hw_init(pcie);
	if (ret)
		return ret;

	rtl8197f_pcie_irq_route(pcie);

	bridge->sysdata = pcie;
	bridge->ops = &rtl8197f_pcie_ops;

	return pci_host_probe(bridge);
}

static const struct of_device_id rtl8197f_pcie_of_match[] = {
	{ .compatible = "realtek,rtl8197f-pcie" },
	{ },
};

static struct platform_driver rtl8197f_pcie_driver = {
	.probe = rtl8197f_pcie_probe,
	.driver = {
		.name = "rtl8197f-pcie",
		.of_match_table = rtl8197f_pcie_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(rtl8197f_pcie_driver);
