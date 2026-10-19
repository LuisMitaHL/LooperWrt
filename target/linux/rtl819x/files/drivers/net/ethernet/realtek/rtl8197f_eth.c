// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek RTL8197F on-SoC CPU network engine ("swNic") driver.
 *
 * Reimplemented from the RTL8197F SDK register-level facts (drivers/net/rtl819x):
 * a descriptor-ring DMA engine at CPU_IFACE_BASE 0x18010000 that hangs off the
 * on-chip 5-port switch. RX/TX run under NAPI driven by the switch-core
 * interrupt (a plain CPU IP -- see the ictl comment below), with a slow timer
 * as a safety net and a full polled fallback when the DT carries no interrupt.
 * It registers a single eth0 bound to the CPU port; the switch does the egress
 * lookup (hwlkup+bridge) so frames flood to the LAN ports.
 */

#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/inetdevice.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ip.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/timer.h>
#include <net/dsa.h>
#include <net/flow_offload.h>
#include <net/pkt_cls.h>

/* CPU interface registers (offsets from the mapped base = phys 0x18010000). */
/* The swNic DMA engine wants descriptor-ring addresses in the uncached
 * window: SDK writes (ring | UNCACHE_MASK) to CPURPDCR/CPUTPDCR. */
#define UNCACHE_MASK	0x20000000
#define CPUICR		0x000	/* interface control */
#define CPURPDCR(i)	(0x004 + ((i) << 2))	/* Rx ring i base / cdp */
#define CPUTPDCR(i)	(0x020 + ((i) << 2))	/* Tx ring i base / cdp */
#define CPUIIMR		0x028	/* interrupt mask (SDK asicregs.h CPUIIMR) */
#define CPUIISR		0x02c	/* interrupt status (SDK asicregs.h CPUIISR) */

/*
 * CPUIIMR/CPUIISR share a bit layout. CPUIISR is write-1-to-clear, so an
 * interrupt is acked by writing the value just read back.
 */
#define II_RX_DONE_ALL		(0x3fu << 3)	/* Rx ring 0..5 packet done */
#define II_TX_DONE_ALL		(0x3u << 9)	/* Tx ring 0..1 packet done */
#define II_PKTHDR_RUNOUT_ALL	(0x3fu << 17)	/* Rx ring 0..5 descriptor runout */
#define II_NAPI_SOURCES		(II_RX_DONE_ALL | II_TX_DONE_ALL | \
				 II_PKTHDR_RUNOUT_ALL)
#define DMA_CR0		0x03c
#define DMA_CR1		0x040
#define TXRINGCR	0x078
#define DMA_CR4		0x0a0
#define CPUICR1		0x0a4

/* System register block @ phys 0x18000000 (KSEG1 0xb8000000). */
#define SYSREG_BASE_PHYS	0x18000000
#define SYSREG_SIZE		0x4000	/* covers SYS 0x800/0x850 + GPIO block @0x3500 */
#define CLK_MANAGE		0x010		/* 0xb8000010 */
#define CLK_MANAGE_SWITCH	(1u << 11)	/* switch/NIC core clock */
#define CLK_MANAGE_LX2_CLK	(1u << 19)
#define CLK_MANAGE_LX2_ARB	(1u << 20)

/*
 * SoC interrupt controller (ictl), phys 0x18003000 -- inside the sysreg
 * mapping above, at +0x3000. The Realtek BSP header (mach-rtl8197f/bspchip.h)
 * documents it as a router: GISR/GIMR hold one bit per source, and the IRRn
 * registers assign each source a CPU IP number in a 4-bit field. GISR bit 15
 * is SW_IP, the switch core (our NIC), and IRR1 covers sources [15:8] with
 * source 15 in the top nibble. The BSP routes the switch core to CPU IP4
 * (BSP_SWCORE_IRQ = BSP_IRQ_CPU_BASE + 4), so it never uses the ictl's own
 * cascade -- which is why this works while the cascade is still unwired. The
 * CP0 timer is set up the same way in arch/mips/rtl819x/setup.c (IRR5 -> IP7).
 */
#define ICTL_GIMR		0x3000
#define ICTL_IRR1		0x300c
#define ICTL_GIMR_SW_IE		(1u << 15)
#define SWCORE_CPU_IP		4		/* must match the DT interrupts = <4> */
#define ICTL_IRR1_SRC15_MASK	0xf0000000u
#define ICTL_IRR1_SRC15_IP	(SWCORE_CPU_IP << 28)

/* Switch core block @ phys 0x1b800000 (KSEG1 0xbb800000). */
#define SWCORE_BASE_PHYS	0x1b800000
#define SWCORE_SIZE		0x8000
#define SW_CVIDR		0x4200		/* SWMISC + 0x00: chip version id */
#define SW_SIRR			0x4204		/* SWMISC + 0x04: system init/reset */
#define SIRR_TRXRDY		(1u << 0)	/* start normal TX and RX */

/* Per-port config (within the swcore mapping). */
#define SW_PCRP(n)		(0x4104 + ((n) * 4))	/* PCRAM+0x4+n*4: port cfg */
#define PCR_ENABLE_PHYIF	(1u << 0)		/* enable PHY interface */
#define PCR_FORCE_DUPLEX	(1u << 18)
#define PCR_FORCE_SPEED_100	(1u << 19)
#define PCR_FORCE_LINK		(1u << 23)
#define PCR_EN_FORCE_MODE	(1u << 25)
#define PCR_FORCE_UP	(PCR_EN_FORCE_MODE | PCR_FORCE_LINK | \
			 PCR_FORCE_SPEED_100 | PCR_FORCE_DUPLEX)
#define BMCR_FORCE_100_FD	0x2100	/* 100M(13) full(8), autoneg off */

#define SW_PSRP(n)		(0x4128 + ((n) * 4))	/* PCRAM+0x28+n*4: port status */
#define SW_MIB_RXPKT(n)		(0x1000 + 0x84 + ((n) * 0x80))	/* per-port rx pkts */
#define SW_P0GMIICR		0x414c
#define P0GMIICR_CPU_TAG	(1u << 25)
#define P0GMIICR_TX_CPU_TAG	(1u << 26)

/* MDC/MDIO controller for the internal PHYs (SWMACCR @ swcore+0x4000). */
#define SW_MDCIOCR		0x4004		/* command */
#define SW_MDCIOSR		0x4008		/* status/result */
#define MDC_CMD_WRITE		(1u << 31)	/* 1=write, 0=read */
#define MDC_PHYADD_SH		24		/* PHY address [28:24] */
#define MDC_REGADD_SH		16		/* PHY register [20:16] */
#define MDC_STATUS_BUSY		(1u << 31)	/* MDCIOSR: 1=in progress */

#define MII_BMCR		0x00
#define MII_BMSR		0x01
#define MII_ADVERTISE		0x04
#define BMCR_RESET_ANEG		0x9000		/* BMCR reset(15) + aneg enable(12) */
#define BMCR_ANEG_RESTART	0x1200		/* aneg enable(12) + restart(9) */
#define ADVERTISE_10_100_PAUSE	0x0de1		/* 10/100 h/f + pause + CSMA */
#define BMSR_LINK		(1u << 2)
#define RTL_NPORTS		5

/*
 * Switch-core init registers (offsets from the swcore base 0x1b800000), matching
 * the clean-room hackpascal RE865X driver (lede-rtl8196c, branch realtek). The
 * rtl865x switch core is shared with the 8197F; register bases verified equal.
 */
#define SW_RESET		0x4204	/* MISC+0x04 (also SIRR): reset/trx-ready */
#define SW_RESET_FULL		(1u << 2)
#define SW_TRX_READY		(1u << 0)
#define SW_MIB_CONTROL		0x1000
#define SW_MIB_ALL_RESTART	0x0000ffff	/* restart all per-port counters */
#define SW_PORTCFG(p)		(0x4104 + ((p) * 4))	/* == SW_PCRP(p) */
#define SW_PORTCFG_PHYIF	(1u << 0)
#define SW_PORTCFG_MAC_RESET_L	(1u << 3)
#define SW_PORTCFG_EXTPHY_SH	26
#define SW_FRAME_FWD_CFG	0x4428	/* ALE+0x28 */
#define SW_UNICAST_TO_CPU	(1u << 1)
#define SW_MULTICAST_TO_CPU	(1u << 0)
#define SW_VLAN_CONTROL_0	0x4a00
#define SW_PORT_VLAN_CONTROL_0	0x4a08
#define SW_VLAN_INGRESS_ALL	0x1ff
#define SW_TABLE_CONTROL_0	0x4418	/* ALE+0x18 */
#define SW_TABLE_CONTROL_1	0x441c	/* ALE+0x1c */
#define SW_PORT_NETIF_MAP	0x4420	/* 3-bit netif index per port */
#define SW_LAN_DECISION_SH	16	/* [17:16], 0=by-vlan */
#define SW_LAN_DECISION_MASK	0x3
#define SW_UNKNOWN_VLAN_TO_CPU	(1u << 15)
#define SW_UNKNOWN_NAPT_TO_CPU	(1u << 14)
#define SW_TLU_STOP		(1u << 18)
#define SW_TLU_STOPPED		(1u << 19)
#define SW_ENHANCED_HASH1	(1u << 13)
#define SW_L4_4WAY_HASH		(1u << 9)
#define SW_MODULE_SW_CONTROL	0x4410	/* ALE+0x10 */
#define SW_L2_ENGINE_ENABLE	(1u << 0)
#define SW_L3_ENGINE_ENABLE	(1u << 1)
#define SW_L4_ENGINE_ENABLE	(1u << 2)
#define SW_MAC_CONFIG		0x4000	/* GLOBAL_PORT_CTRL+0x00 */
#define SW_CHECKSUM_CONTROL	0x4048
#define SW_L3_CHECKSUM_RECALC	BIT(4)
#define SW_L4_CHECKSUM_RECALC	BIT(5)
#define SW_MAC_CTRL1		0x5100	/* EXT_MAC: CPU-port MAC control */
#define SW_CMAC_CLK_SEL		(1u << 0)
#define SW_CMAC_LATPKT_EN	(1u << 5)
#define SW_CF_RXIPG_MASK	0xf	/* [3:0] */
#define SW_SELIPG_SH		18	/* [19:18] */
#define SW_SELIPG_MASK		0x3
#define SW_SELIPG_11		2
#define SW_PORTCFG_FORCED_MODE	(1u << 25)
/* Table access (VLAN LUT) */
#define SW_TBL_ACC_CONTROL	0x4d00
#define SW_TBL_ACC_ADDRESS	0x4d08
#define SW_TBL_ACC_DATA(i)	(0x4d20 + ((i) * 4))
#define SW_TBL_CMD_FORCE	(1u << 1)	/* cmd = force write */
#define SW_TBL_ACTION		(1u << 0)

#define RTL_TBL_L2		0
#define RTL_TBL_ARP		1
#define RTL_TBL_ROUTE		2
#define RTL_TBL_NETIF		4
#define RTL_TBL_EXTIP		5
#define RTL_TBL_VLAN		6
#define RTL_TBL_NAPT		9
#define RTL_TBL_NEXTHOP		13
#define RTL_TBL_ADDR(type, index)	(0xbb000000u | ((type) << 16) | \
					 ((index) << 5))
#define RTL_LAN_VID		9
#define RTL_WAN_VID		8

#define RTL_HWNAT_MAX_FLOWS	64
#define RTL_HWNAT_MAX_PENDING	32
#define RTL_HWNAT_MAX_SAVED	144
#define RTL_HWNAT_PENDING_TTL	(5 * HZ)
#define RTL_HS_WORDS		23

/* CPUICR bits */
#define TXCMD		(1u << 31)
#define RXCMD		(1u << 30)
#define BUSBURST_128W	(2u << 28)
#define MBUF_2048B	(4u << 24)
#define TXFD		(1u << 23)
#define CPUICR_ENABLE	(TXCMD | RXCMD | BUSBURST_128W | MBUF_2048B)

#define DMA_CR4_TX0_TAIL_AWARE	(1u << 0)
#define TX_RING0_EN		(1u << 0)
#define TX_RING1_EN		(1u << 1)
#define CPUICR1_TXRX_DIV_LX	(1u << 0)
#define CPUICR1_LITTLE_ENDIAN	(1u << 1)
#define CPUICR1_TSO_ID		(1u << 4)
#define CPUICR1_NEW_DESC	(1u << 8)

/* Descriptor opts1 bits */
#define D_OWN		(1u << 0)	/* owned by SWCORE (hw) */
#define D_WRAP		(1u << 1)	/* end of ring */
#define D_LS		(1u << 2)	/* last segment */
#define D_FS		(1u << 3)	/* first segment */
#define D_HWLKUP	(1u << 4)
#define D_BRIDGE	(1u << 5)
#define TD_TYPE_SH	29
#define TD_TYPE_IPV6	7
#define TD_TYPE_TCP	5
#define TD_TYPE_UDP	6
#define TD_PHLEN_SH	6		/* tx: packet length, [22:6] */
#define TD_PHLEN_MASK	0x1ffff
#define TD_MLEN_SH	15		/* tx opts2: mbuf length */
#define RD_LEN_MASK	0x3fff		/* rx opts2: received length (incl CRC) */
#define RD_EXTSIZE_SH	16		/* rx opts1: buffer size, [31:16] */
#define RD_SPA_SH	13		/* rx opts4: source port [15:13] */
#define RD_SPA_MASK	0x7
#define RD_IPV6		BIT(9)
#define RD_IPV4		BIT(8)
#define RD_L3CS_OK	BIT(31)
#define RD_L4CS_OK	BIT(30)
#define TD_L3CS		BIT(20)
#define TD_L4CS		BIT(19)
#define TD_IPV6		BIT(18)
#define TD_IPV4		BIT(17)
#define TD_IPV4_FIRST	BIT(16)

#define RTL8_4_TAG_LEN	8
#define RTL8_4_ETHERTYPE	0x8899
#define RTL8_4_PROTOCOL		0x04
#define RTL8_4_RX_MASK		0x07ff

#define RTL_RING_SIZE	128
#define RTL_AUX_RX_RING_SIZE	2
#define RTL_TX_RING1_SIZE	2
#define RTL_BUF_SIZE	2048
#define RTL_TXCAP_SIZE	96
/*
 * The RTL8197F swNic has 6 hardware RX pkthdr rings (CPURPDCR0..5). The switch
 * delivers CPU-bound frames to whichever ring its priority/queue map selects,
 * so ALL six must point at valid buffers -- an unconfigured ring keeps the
 * bootloader's garbage base and the DMA engine then corrupts kernel memory when
 * a frame lands on it (observed as an unrelated-process maple-tree Oops).
 * Matches the SDK New_swNic_init loop over NEW_NIC_MAX_RX_DESC_RING.
 */
#define NUM_RX_RING	6
#define RTL_POLL_MS	1
/*
 * Safety-net poll used in interrupt mode. It only has to catch a lost
 * interrupt, so it is slow enough to cost nothing when the IRQ works.
 */
#define RTL_SAFETY_MS	1000

/* 6 x u32 = 24-byte hardware descriptor (opts1, addr, opts2..opts5). */
struct rtl_desc {
	u32 opts1;
	u32 addr;
	u32 opts2;
	u32 opts3;
	u32 opts4;
	u32 opts5;
};

static inline unsigned int rtl_rx_ring_size(unsigned int ring)
{
	return ring ? RTL_AUX_RX_RING_SIZE : RTL_RING_SIZE;
}

struct rtl_ft_rule {
	struct ethhdr eth;
	__be32 orig_src_addr;
	__be32 orig_dst_addr;
	__be32 src_addr;
	__be32 dst_addr;
	__be16 orig_src_port;
	__be16 orig_dst_port;
	__be16 src_port;
	__be16 dst_port;
	u8 ip_proto;
	u8 ingress_port;
	u8 egress_port;
	__be32 ingress_local_addr;
};

struct rtl_hwnat_pending {
	bool used;
	unsigned long cookie;
	unsigned long expires;
	struct rtl_ft_rule rule;
};

struct rtl_hwnat_flow {
	bool used;
	unsigned long cookie[2];
	u16 out_index;
	u16 in_index;
	u32 saved_out[8];
	u32 saved_in[8];
};

struct rtl_hwnat_saved {
	bool used;
	u8 type;
	u16 index;
	u32 data[8];
};

struct rtl_eth {
	void __iomem *base;
	void __iomem *sysreg;	/* 0x18000000: CLK_MANAGE */
	void __iomem *swcore;	/* 0x1b800000: SIRR/CVIDR */
	void __iomem *tbl;	/* 0x1b000000: direct ASIC table reads */
	struct net_device *ndev;
	struct device *dev;
	struct napi_struct napi;
	struct timer_list poll_timer;
	int irq;		/* swcore IRQ, or <0 when running polled */
	bool irq_mode;		/* true once the IRQ is claimed and unmasked */

	struct rtl_desc *rx_ring[NUM_RX_RING];
	dma_addr_t rx_ring_dma[NUM_RX_RING];
	struct sk_buff *rx_skb[NUM_RX_RING][RTL_RING_SIZE];
	dma_addr_t rx_buf_dma[NUM_RX_RING][RTL_RING_SIZE];
	unsigned int rx_head[NUM_RX_RING];

	struct rtl_desc *tx_ring;
	dma_addr_t tx_ring_dma;
	struct rtl_desc *tx_ring1;
	dma_addr_t tx_ring1_dma;
	struct sk_buff *tx_skb[RTL_RING_SIZE];
	dma_addr_t tx_buf_dma[RTL_RING_SIZE];
	unsigned int tx_head;	/* next to fill */
	unsigned int tx_tail;	/* next to reap */
	spinlock_t tx_lock;
	struct mutex tbl_lock;
	bool hwnat_prepared;
	u32 hwnat_saved_vlan[2][8];
	u32 hwnat_saved_netif[8];
	u16 hwnat_saved_ext_vlan[2][2];
	u16 hwnat_saved_ext_misc[5];
	bool hwnat_ext_saved;
	bool hwnat_active;
	u32 hwnat_saved_swtcr1;
	u32 hwnat_saved_swtcr0;
	u32 hwnat_saved_pvcr[4];
	u32 hwnat_saved_cscr;
	u32 hwnat_lan_net;
	__be32 hwnat_lan_ip;
	__be32 hwnat_ext_ip;
	u8 hwnat_wan_mac[ETH_ALEN];
	u8 hwnat_gateway_mac[ETH_ALEN];
	u8 hwnat_lan_mac[ETH_ALEN];
	unsigned int hwnat_flow_count;
	unsigned int hwnat_saved_count;
	struct rtl_hwnat_pending hwnat_pending[RTL_HWNAT_MAX_PENDING];
	struct rtl_hwnat_flow hwnat_flow[RTL_HWNAT_MAX_FLOWS];
	struct rtl_hwnat_saved hwnat_saved[RTL_HWNAT_MAX_SAVED];
	u32 hsb_watch[RTL_HS_WORDS];
	u32 hsa_watch[RTL_HS_WORDS];
	u32 hsb_watch_sip;
	u32 hsb_watch_dip;
	u8 hsb_watch_state;
	u8 txcap[RTL_TXCAP_SIZE];
	unsigned int txcap_len;
	unsigned int txcap_seq;
	u32 rx_last_opts4;
	u32 rx_last_opts5;
	u8 rx_last_spa;
	unsigned long napi_polls;
	unsigned long napi_budget_exhausted;
	unsigned long rx_ring_packets[NUM_RX_RING];
	unsigned long rx_csum_good;
	unsigned long rx_csum_none;
	unsigned long tx_csum_good;
	unsigned long tx_csum_help;
	unsigned long tx_stops;
	unsigned long tx_wakes;
	unsigned long ft_binds;
	unsigned long ft_unbinds;
	unsigned long ft_replace;
	unsigned long ft_destroy;
	unsigned long ft_stats;
	unsigned long ft_parsed;
	unsigned long ft_rejected;

	unsigned long next_link_log;	/* jiffies: throttle link diag */
	u16 link_seen;			/* bitmask of phyaddrs last seen linked */
	u16 mdio_last;			/* last value read via the mdio sysfs poke */
	u32 reg_last;			/* last value read via the reg sysfs poke */
	u16 smi_sck;			/* GPIO id (port<<8|pin) for ext-switch SMI clock */
	u16 smi_sda;			/* GPIO id for ext-switch SMI data */
	bool dma_on;			/* CPU-NIC DMA engine enabled (runtime-gated) */
};

static inline u32 rtl_rd(struct rtl_eth *p, u32 reg)
{
	return readl(p->base + reg);
}

static inline void rtl_wr(struct rtl_eth *p, u32 reg, u32 val)
{
	writel(val, p->base + reg);
}

static int rtl_alloc_rx_buf(struct rtl_eth *p, unsigned int ring, unsigned int i)
{
	struct sk_buff *skb;
	dma_addr_t dma;

	skb = netdev_alloc_skb(p->ndev, RTL_BUF_SIZE);
	if (!skb)
		return -ENOMEM;

	dma = dma_map_single(p->dev, skb->data, RTL_BUF_SIZE, DMA_FROM_DEVICE);
	if (dma_mapping_error(p->dev, dma)) {
		dev_kfree_skb_any(skb);
		return -ENOMEM;
	}

	p->rx_skb[ring][i] = skb;
	p->rx_buf_dma[ring][i] = dma;
	p->rx_ring[ring][i].addr = (u32)dma;
	p->rx_ring[ring][i].opts1 = D_OWN | (RTL_BUF_SIZE << RD_EXTSIZE_SH) |
			      (i == rtl_rx_ring_size(ring) - 1 ? D_WRAP : 0);
	return 0;
}

static void rtl_free_rings(struct rtl_eth *p)
{
	unsigned int r, i;

	for (r = 0; r < NUM_RX_RING; r++) {
		for (i = 0; i < rtl_rx_ring_size(r); i++) {
			if (p->rx_skb[r][i]) {
				dma_unmap_single(p->dev, p->rx_buf_dma[r][i],
						 RTL_BUF_SIZE, DMA_FROM_DEVICE);
				dev_kfree_skb_any(p->rx_skb[r][i]);
				p->rx_skb[r][i] = NULL;
			}
		}
	}
	for (i = 0; i < RTL_RING_SIZE; i++) {
		if (p->tx_skb[i]) {
			dma_unmap_single(p->dev, p->tx_buf_dma[i],
					 p->tx_skb[i]->len, DMA_TO_DEVICE);
			dev_kfree_skb_any(p->tx_skb[i]);
			p->tx_skb[i] = NULL;
		}
	}
}

static int rtl_init_rings(struct rtl_eth *p)
{
	unsigned int r, i;
	int ret;

	p->tx_head = 0;
	p->tx_tail = 0;

	memset(p->tx_ring, 0, RTL_RING_SIZE * sizeof(struct rtl_desc));
	p->tx_ring[RTL_RING_SIZE - 1].opts1 = D_WRAP;
	memset(p->tx_ring1, 0, RTL_TX_RING1_SIZE * sizeof(struct rtl_desc));
	p->tx_ring1[RTL_TX_RING1_SIZE - 1].opts1 = D_WRAP;

	for (r = 0; r < NUM_RX_RING; r++) {
		p->rx_head[r] = 0;
		memset(p->rx_ring[r], 0,
		       rtl_rx_ring_size(r) * sizeof(struct rtl_desc));
		for (i = 0; i < rtl_rx_ring_size(r); i++) {
			ret = rtl_alloc_rx_buf(p, r, i);
			if (ret) {
				rtl_free_rings(p);
				return ret;
			}
		}
	}

	/*
	 * Program the ring bases with UNCACHE_MASK (bit 29) set, exactly as the
	 * SDK's New_swNic_init does:
	 *   New_rxDescRing_base = ((u32)ring) | UNCACHE_MASK; REG32(CPURPDCR)=...
	 * The swNic DMA engine wants the uncached-window form of the descriptor
	 * ring address; writing the bare physical (bit 29 clear) makes the engine
	 * fetch descriptors from the wrong place -- it never clears our OWN bits,
	 * so RX never advances (rx_cdp stuck at the bare physical base, rx=0).
	 * All six RX rings must be armed (see NUM_RX_RING) or a frame delivered to
	 * an unconfigured ring corrupts memory.
	 */
	rtl_wr(p, CPUTPDCR(0), (u32)p->tx_ring_dma | UNCACHE_MASK);
	rtl_wr(p, CPUTPDCR(1), (u32)p->tx_ring1_dma | UNCACHE_MASK);
	for (r = 0; r < NUM_RX_RING; r++)
		rtl_wr(p, CPURPDCR(r), (u32)p->rx_ring_dma[r] | UNCACHE_MASK);
	rtl_wr(p, DMA_CR1, (RTL_RING_SIZE - 1) * sizeof(struct rtl_desc));
	rtl_wr(p, DMA_CR4, DMA_CR4_TX0_TAIL_AWARE);
	rtl_wr(p, TXRINGCR, TX_RING0_EN | TX_RING1_EN);
	return 0;
}

static void rtl_tx_reap(struct rtl_eth *p)
{
	spin_lock(&p->tx_lock);
	while (p->tx_tail != p->tx_head) {
		struct rtl_desc *d = &p->tx_ring[p->tx_tail];

		if (d->opts1 & D_OWN)	/* still owned by hw */
			break;
		if (p->tx_skb[p->tx_tail]) {
			dma_unmap_single(p->dev, p->tx_buf_dma[p->tx_tail],
					 p->tx_skb[p->tx_tail]->len,
					 DMA_TO_DEVICE);
			dev_consume_skb_any(p->tx_skb[p->tx_tail]);
			p->tx_skb[p->tx_tail] = NULL;
		}
		p->tx_tail = (p->tx_tail + 1) % RTL_RING_SIZE;
	}
	spin_unlock(&p->tx_lock);

	if (netif_queue_stopped(p->ndev)) {
		p->tx_wakes++;
		netif_wake_queue(p->ndev);
	}
}

static void rtl_link_log(struct rtl_eth *p);

static int rtl_rx_poll(struct napi_struct *napi, int budget)
{
	struct rtl_eth *p = container_of(napi, struct rtl_eth, napi);
	int done = 0;
	int r;

	p->napi_polls++;
	rtl_tx_reap(p);
	rtl_link_log(p);

	/*
	 * Drain the RX rings high->low, matching the SDK handler
	 * (for ring_idx = MAX-1 .. 0). CPU-bound frames land on whichever ring
	 * the switch's priority/queue map picked, so all six must be serviced.
	 */
	for (r = NUM_RX_RING - 1; r >= 0 && done < budget; r--) {
		while (done < budget) {
			unsigned int h = p->rx_head[r];
			struct rtl_desc *d = &p->rx_ring[r][h];
			struct sk_buff *skb = p->rx_skb[r][h];
			unsigned int len;
			u32 opts4, opts5;

			if (skb) {
				/* Armed slot: if hw still owns it, ring is drained. */
				if (d->opts1 & D_OWN)
					break;
				len = (d->opts2 & RD_LEN_MASK);
				opts4 = d->opts4;
				opts5 = d->opts5;
				p->rx_last_opts4 = opts4;
				p->rx_last_opts5 = opts5;
				p->rx_last_spa = (d->opts4 >> RD_SPA_SH) &
						 RD_SPA_MASK;
				dma_unmap_single(p->dev, p->rx_buf_dma[r][h],
						 RTL_BUF_SIZE, DMA_FROM_DEVICE);
				if (len > 4 && len <= RTL_BUF_SIZE) {
					len -= 4;	/* strip CRC */
					skb_put(skb, len);
					/*
					 * RTL8197F CPU-tag hardware removes the native
					 * RTL8367 4-byte tag and reports its source port
					 * in opts4.SPA. Recreate the standard RTL8_4 RX
					 * tag expected by the DSA core.
					 */
					skb_push(skb, RTL8_4_TAG_LEN);
					memmove(skb->data, skb->data + RTL8_4_TAG_LEN,
						2 * ETH_ALEN);
					skb->data[12] = RTL8_4_ETHERTYPE >> 8;
					skb->data[13] = RTL8_4_ETHERTYPE & 0xff;
					skb->data[14] = RTL8_4_PROTOCOL;
					skb->data[15] = 0;
					skb->data[16] = 0;
					skb->data[17] = 0;
					skb->data[18] = 0;
					skb->data[19] = p->rx_last_spa;
					len += RTL8_4_TAG_LEN;
					skb->protocol = eth_type_trans(skb, p->ndev);
					if ((p->ndev->features & NETIF_F_RXCSUM) &&
					    (((opts4 & RD_IPV4) &&
					      (opts5 & (RD_L3CS_OK | RD_L4CS_OK)) ==
					      (RD_L3CS_OK | RD_L4CS_OK)) ||
					     ((opts4 & RD_IPV6) &&
					      (opts5 & RD_L4CS_OK)))) {
						skb->ip_summed = CHECKSUM_UNNECESSARY;
						p->rx_csum_good++;
					} else {
						p->rx_csum_none++;
					}
					p->ndev->stats.rx_packets++;
					p->ndev->stats.rx_bytes += len;
					p->rx_ring_packets[r]++;
					napi_gro_receive(napi, skb);
				} else {
					p->ndev->stats.rx_errors++;
					dev_kfree_skb_any(skb);
				}
				p->rx_skb[r][h] = NULL;
			}
			/*
			 * (Re)arm this slot. skb==NULL here means either we just
			 * consumed a packet or a previous refill failed; either way
			 * retry the allocation. On failure stop and retry next poll --
			 * never advance past an unarmed slot (that would desync
			 * rx_head from the hw and later deref a NULL skb).
			 */
			if (rtl_alloc_rx_buf(p, r, h)) {
				p->ndev->stats.rx_dropped++;
				break;
			}
			p->rx_head[r] = (h + 1) % rtl_rx_ring_size(r);
			done++;
		}
	}

	if (done < budget) {
		napi_complete_done(napi, done);
		if (p->irq_mode) {
			/*
			 * Re-arm the NIC sources masked by the ISR. The safety
			 * timer below still runs, so a lost interrupt costs one
			 * extra second of latency rather than a stalled ring.
			 */
			rtl_wr(p, CPUIIMR, II_NAPI_SOURCES);
			mod_timer(&p->poll_timer,
				  jiffies + msecs_to_jiffies(RTL_SAFETY_MS));
		} else {
			mod_timer(&p->poll_timer,
				  jiffies + msecs_to_jiffies(RTL_POLL_MS));
		}
	} else {
		p->napi_budget_exhausted++;
	}
	return done;
}

/*
 * Switch-core interrupt (CPU IP4). Ack what is pending, mask the NIC sources
 * that NAPI owns, and let rtl_rx_poll() re-arm them when it drains the rings.
 */
static irqreturn_t rtl_isr(int irq, void *dev_id)
{
	struct rtl_eth *p = dev_id;
	u32 status = rtl_rd(p, CPUIISR);

	if (!status)
		return IRQ_NONE;

	rtl_wr(p, CPUIISR, status);	/* write-1-to-clear */
	rtl_wr(p, CPUIIMR, 0);
	napi_schedule(&p->napi);
	return IRQ_HANDLED;
}

/*
 * Silence the switch core at both the NIC mask and the ictl, and wait for any
 * in-flight handler. Must run before the rings are torn down, or a late
 * interrupt schedules NAPI against freed state.
 */
static void rtl_irq_mask(struct rtl_eth *p)
{
	if (!p->irq_mode)
		return;

	writel(readl(p->sysreg + ICTL_GIMR) & ~ICTL_GIMR_SW_IE,
	       p->sysreg + ICTL_GIMR);
	rtl_wr(p, CPUIIMR, 0);
	p->irq_mode = false;
	synchronize_irq(p->irq);
}

static void rtl_poll_timer(struct timer_list *t)
{
	struct rtl_eth *p = timer_container_of(p, t, poll_timer);

	napi_schedule(&p->napi);
}

static netdev_tx_t rtl_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct rtl_eth *p = netdev_priv(ndev);
	unsigned int idx, next, len;
	u16 port_mask = 1;
	u32 tx_opts1 = 0, tx_opts3 = 0;
	struct rtl_desc *d;
	dma_addr_t dma;

	if (!p->dma_on) {	/* engine runtime-gated off: drop, never poke ring */
		dev_kfree_skb_any(skb);
		ndev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	/* Preserve the exact frame handed to DMA.  Restrict capture to the
	 * RTL8_4-tagged ARP used by the wired bring-up so background traffic
	 * cannot overwrite the diagnostic before userspace reads it.
	 */
	if (skb->len >= 22 &&
	    skb->data[12] == 0x88 && skb->data[13] == 0x99 &&
	    skb->data[20] == 0x08 && skb->data[21] == 0x06) {
		p->txcap_len = min_t(unsigned int, skb->len, RTL_TXCAP_SIZE);
		memcpy(p->txcap, skb->data, p->txcap_len);
		p->txcap_seq++;
	}

	/*
	 * DSA supplies an RTL8_4 tag containing the destination user-port mask.
	 * The vendor RTL8197F+RTL8367R path instead uses the swNic descriptor
	 * port mask and lets P0GMIICR TX CPU-tag hardware emit the native 4-byte
	 * wire tag. Preserve DSA's port selection, but remove its software tag
	 * before handing the frame to DMA.
	 */
	if (likely(skb->len >= ETH_HLEN + RTL8_4_TAG_LEN &&
		   skb->data[12] == (RTL8_4_ETHERTYPE >> 8) &&
		   skb->data[13] == (RTL8_4_ETHERTYPE & 0xff) &&
		   skb->data[14] == RTL8_4_PROTOCOL)) {
		port_mask = ((u16)skb->data[18] << 8) | skb->data[19];
		port_mask &= RTL8_4_RX_MASK;
		if (!port_mask) {
			dev_kfree_skb_any(skb);
			ndev->stats.tx_dropped++;
			return NETDEV_TX_OK;
		}

		memmove(skb->data + RTL8_4_TAG_LEN, skb->data, 2 * ETH_ALEN);
		skb_pull(skb, RTL8_4_TAG_LEN);
	}

	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		unsigned int l3 = ETH_HLEN;
		u16 proto = 0;

		if (skb->len >= ETH_HLEN)
			proto = ((u16)skb->data[12] << 8) | skb->data[13];
		if ((proto == ETH_P_8021Q || proto == ETH_P_8021AD) &&
		    skb->len >= ETH_HLEN + VLAN_HLEN) {
			proto = ((u16)skb->data[16] << 8) | skb->data[17];
			l3 += VLAN_HLEN;
		}
		if (proto == ETH_P_IP && skb->len >= l3 + sizeof(struct iphdr) &&
		    (skb->data[l3 + 9] == IPPROTO_TCP ||
		     skb->data[l3 + 9] == IPPROTO_UDP)) {
			tx_opts1 = (skb->data[l3 + 9] == IPPROTO_TCP ?
				    TD_TYPE_TCP : TD_TYPE_UDP) << TD_TYPE_SH;
			tx_opts3 = TD_L3CS | TD_L4CS | TD_IPV4 | TD_IPV4_FIRST;
			p->tx_csum_good++;
		} else if (proto == ETH_P_IPV6) {
			tx_opts1 = TD_TYPE_IPV6 << TD_TYPE_SH;
			tx_opts3 = TD_L4CS | TD_IPV6;
			p->tx_csum_good++;
		} else {
			if (skb_checksum_help(skb)) {
				dev_kfree_skb_any(skb);
				ndev->stats.tx_dropped++;
				return NETDEV_TX_OK;
			}
			p->tx_csum_help++;
		}
	}

	len = skb->len;
	if (len < 60)
		len = 64;		/* min frame; hw pads */
	else
		len += 4;		/* + CRC */

	dma = dma_map_single(p->dev, skb->data, skb->len, DMA_TO_DEVICE);
	if (dma_mapping_error(p->dev, dma)) {
		dev_kfree_skb_any(skb);
		ndev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	spin_lock(&p->tx_lock);
	idx = p->tx_head;
	next = (idx + 1) % RTL_RING_SIZE;
	if (next == p->tx_tail) {		/* ring full */
		p->tx_stops++;
		netif_stop_queue(ndev);
		spin_unlock(&p->tx_lock);
		dma_unmap_single(p->dev, dma, skb->len, DMA_TO_DEVICE);
		return NETDEV_TX_BUSY;
	}

	d = &p->tx_ring[idx];
	p->tx_skb[idx] = skb;
	p->tx_buf_dma[idx] = dma;

	d->addr = (u32)dma;
	d->opts2 = (len & TD_PHLEN_MASK) << TD_MLEN_SH;
	d->opts3 = tx_opts3;
	d->opts4 = (u32)port_mask << 24;
	d->opts5 = 0;
	d->opts1 = (idx == RTL_RING_SIZE - 1 ? D_WRAP : 0) | D_FS | D_LS |
		   tx_opts1 | ((len & TD_PHLEN_MASK) << TD_PHLEN_SH);
	wmb();			/* fields visible before OWN handover */
	d->opts1 |= D_OWN;
	p->tx_head = next;
	spin_unlock(&p->tx_lock);

	rtl_wr(p, CPUICR, rtl_rd(p, CPUICR) | TXFD);	/* kick */

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;
	return NETDEV_TX_OK;
}

/* Internal-PHY MDIO via the SWMACCR MDC/MDIO controller. */
static int rtl_mdio_wait(struct rtl_eth *p)
{
	unsigned int t = 10000;

	while (t--) {
		if (!(readl(p->swcore + SW_MDCIOSR) & MDC_STATUS_BUSY))
			return 0;
		udelay(1);
	}
	return -ETIMEDOUT;
}

static int rtl_mdio_read(struct rtl_eth *p, unsigned int phy, unsigned int reg)
{
	if (rtl_mdio_wait(p))
		return -ETIMEDOUT;
	writel((phy << MDC_PHYADD_SH) | (reg << MDC_REGADD_SH),
	       p->swcore + SW_MDCIOCR);
	if (rtl_mdio_wait(p))
		return -ETIMEDOUT;
	return readl(p->swcore + SW_MDCIOSR) & 0xffff;
}

static void rtl_mdio_write(struct rtl_eth *p, unsigned int phy,
			   unsigned int reg, u16 val)
{
	if (rtl_mdio_wait(p))
		return;
	writel(MDC_CMD_WRITE | (phy << MDC_PHYADD_SH) |
	       (reg << MDC_REGADD_SH) | val, p->swcore + SW_MDCIOCR);
	rtl_mdio_wait(p);
}

/* Poll every internal PHY's link bit; log transitions so we can map physical
 * ports to PHY addresses by moving the cable. */
static void rtl_link_log(struct rtl_eth *p)
{
	unsigned int i;
	u32 cdp, hw_idx;
	u8 link = 0;

	if (time_before(jiffies, p->next_link_log))
		return;
	p->next_link_log = jiffies + HZ;	/* re-check link ~1x/sec */

	/* PHY link bitmap (read BMSR twice: link bit is latch-low). Bit N == PHY
	 * addr N; the real internal GPHYs are at addr 1..4 (addr 0 has none). */
	for (i = 0; i < RTL_NPORTS; i++) {
		rtl_mdio_read(p, i, MII_BMSR);
		if (rtl_mdio_read(p, i, MII_BMSR) & BMSR_LINK)
			link |= BIT(i);
	}

	/* Edge-triggered: stay quiet unless the link bitmap changed (plug/unplug
	 * a cable -> one line). link_seen holds the last-logged bitmap. */
	if (link == (u8)p->link_seen)
		return;
	p->link_seen = link;

	/* switch MIB per-port RX packet counters (do frames enter the switch?) +
	 * RX engine progress (does the switch forward to the CPU?). */
	cdp = rtl_rd(p, CPURPDCR(0));
	hw_idx = (cdp - (u32)p->rx_ring_dma[0]) / sizeof(struct rtl_desc);

	netdev_info(p->ndev,
		    "LINK CHANGE link=0x%02x MIBrx=%u,%u,%u,%u,%u hw_idx=%u rx_pkts=%lu\n",
		    link,
		    readl(p->swcore + SW_MIB_RXPKT(0)),
		    readl(p->swcore + SW_MIB_RXPKT(1)),
		    readl(p->swcore + SW_MIB_RXPKT(2)),
		    readl(p->swcore + SW_MIB_RXPKT(3)),
		    readl(p->swcore + SW_MIB_RXPKT(4)),
		    hw_idx, p->ndev->stats.rx_packets);
}

/* Paged internal-PHY register RMW (page select via reg 31; reg 30 for page>=31),
 * matching the clean-room rtl865x_phy_page_reg_rmw. */
static void rtl_phy_page_rmw(struct rtl_eth *p, int addr, int page, int reg,
			     u16 clear, u16 set)
{
	u16 v = 0;

	if (page >= 31) {
		rtl_mdio_write(p, addr, 31, 7);
		rtl_mdio_write(p, addr, 30, page);
	} else {
		rtl_mdio_write(p, addr, 31, page);
	}
	if (clear != 0xffff)
		v = rtl_mdio_read(p, addr, reg) & ~clear;
	rtl_mdio_write(p, addr, reg, v | set);
	rtl_mdio_write(p, addr, 31, 0);
}

/*
 * Internal-GPHY analog init. Ported from the clean-room rtl8196c_revb_fixup
 * (same 001c:c8xx internal-PHY family); the 8197F's own analog patch lives in
 * the bootloader and is lost when we (re-)clock the switch. EXPERIMENTAL: if the
 * link still does not come up, an 8197F-specific patch is required.
 */
static void __maybe_unused rtl_phy_analog_init(struct rtl_eth *p)
{
	unsigned int i;

	for (i = 0; i < RTL_NPORTS; i++)
		writel(readl(p->swcore + SW_PORTCFG(i)) | SW_PORTCFG_FORCED_MODE,
		       p->swcore + SW_PORTCFG(i));

	for (i = 0; i < RTL_NPORTS; i++) {
		rtl_phy_page_rmw(p, i, 1, 17, 7 << 10, 7 << 10);
		rtl_phy_page_rmw(p, i, 4, 24, 0xff, 0xf3);
		rtl_phy_page_rmw(p, i, 4, 16, 1 << 3, 1 << 3);
		rtl_phy_page_rmw(p, i, 1, 19, 7 << 11, 2 << 11);
		rtl_phy_page_rmw(p, i, 1, 23, 7 << 6, 4 << 6);
		rtl_phy_page_rmw(p, i, 1, 18, 7 << 3, 6 << 3);
	}

	writel((readl(p->swcore + SW_MAC_CONFIG) & ~SW_CF_RXIPG_MASK) | 0x5,
	       p->swcore + SW_MAC_CONFIG);
	writel((readl(p->swcore + SW_MAC_CONFIG) &
		~(SW_SELIPG_MASK << SW_SELIPG_SH)) |
	       (SW_SELIPG_11 << SW_SELIPG_SH), p->swcore + SW_MAC_CONFIG);

	for (i = 0; i < RTL_NPORTS; i++) {
		rtl_phy_page_rmw(p, i, 0, 21, 0xff, 0x32);
		rtl_phy_page_rmw(p, i, 0, 22, 7 << 4, 5 << 4);
		rtl_phy_page_rmw(p, i, 0, 0, 1 << 9, 1 << 9);	/* aneg */
		rtl_phy_page_rmw(p, i, 1, 17, 3 << 1, 3 << 1);
		rtl_phy_page_rmw(p, i, 1, 18, 0xffff, 0x9004);
		rtl_phy_page_rmw(p, i, 4, 26, 0xfff0, 0xff80);
		rtl_phy_page_rmw(p, i, 0, 21, 1 << 14, 1 << 14);
	}

	for (i = 0; i < RTL_NPORTS; i++)
		writel(readl(p->swcore + SW_PORTCFG(i)) & ~SW_PORTCFG_FORCED_MODE,
		       p->swcore + SW_PORTCFG(i));

	/* disable 10M power saving (reg 0x18 bit 15) */
	for (i = 0; i < RTL_NPORTS; i++) {
		int v = rtl_mdio_read(p, i, 0x18);

		if (v != 0xffff)
			rtl_mdio_write(p, i, 0x18, v & ~(1 << 15));
	}
}

/*
 * Bring up the on-SoC switch core (clean-room rtl865x_reset recipe from the
 * hackpascal RE865X driver). Without this the switch never forwards LAN frames
 * to the CPU port (the L2 engine is off and "to CPU" forwarding is disabled),
 * which is why the CPU RX ring never advanced.
 */
static void __maybe_unused rtl_switch_init(struct rtl_eth *p)
{
	unsigned int i;

	/*
	 * NOTE: intentionally NO full switch reset (SW_RESET_FULL). The 8197F
	 * internal GPHYs are analog-initialised by the bootloader; a full reset
	 * loses that (the vendor re-applies a per-SoC PHY analog patch after
	 * reset, which we do not have for 8197F) and the link never comes up.
	 * We only (re)apply the forwarding/L2 config on top of the existing state.
	 */

	/* reset MIB counters */
	writel(SW_MIB_ALL_RESTART, p->swcore + SW_MIB_CONTROL);

	/* per LAN port: toggle the MAC reset and enable the PHY interface */
	for (i = 0; i < RTL_NPORTS; i++) {
		u32 v = readl(p->swcore + SW_PORTCFG(i));

		writel(v & ~SW_PORTCFG_MAC_RESET_L, p->swcore + SW_PORTCFG(i));
		writel((v & ~(0x1fu << SW_PORTCFG_EXTPHY_SH)) |
		       (i << SW_PORTCFG_EXTPHY_SH) |
		       SW_PORTCFG_PHYIF | SW_PORTCFG_MAC_RESET_L,
		       p->swcore + SW_PORTCFG(i));
	}

	/* forward unknown-unicast + multicast/broadcast to the CPU port */
	writel(readl(p->swcore + SW_FRAME_FWD_CFG) |
	       SW_UNICAST_TO_CPU | SW_MULTICAST_TO_CPU,
	       p->swcore + SW_FRAME_FWD_CFG);

	/* VLAN ingress filter on all ports */
	writel(SW_VLAN_INGRESS_ALL, p->swcore + SW_VLAN_CONTROL_0);

	/* LAN decision by VLAN + trap unknown-VLAN/NAPT frames to the CPU */
	{
		u32 v = readl(p->swcore + SW_TABLE_CONTROL_0);

		v &= ~(SW_LAN_DECISION_MASK << SW_LAN_DECISION_SH);
		v |= SW_UNKNOWN_VLAN_TO_CPU | SW_UNKNOWN_NAPT_TO_CPU;
		writel(v, p->swcore + SW_TABLE_CONTROL_0);
	}

	/* enable the L2 lookup engine (no forwarding happens without this) */
	writel(readl(p->swcore + SW_MODULE_SW_CONTROL) | SW_L2_ENGINE_ENABLE,
	       p->swcore + SW_MODULE_SW_CONTROL);

	/*
	 * REAL boot switch/GPHY init, reverse-engineered from the stock a2004m_ml
	 * kernel via a purpose-built MIPS emulator that followed the boot NIC-init
	 * call graph ("Probing RTL819X NIC" -> fn 0x807f7acc). The earlier leads
	 * (SYS 0x64|=0x1f, page-2 analog, LX clocks) were NOT on the boot path.
	 *   - fn 0x802281b0: switch port config + SYS 0x18000850 control.
	 *   - fn 0x80228c3c: internal-GPHY paged (page 1) init + PHY reset/autoneg.
	 */

	/* switch port init (stock fn 0x802281b0) */
	{
		unsigned int pr;

		for (pr = 0; pr < RTL_NPORTS; pr++) {
			u32 v = readl(p->swcore + SW_PORTCFG(pr));

			v = (v & 0x83ffdff6) | 0x14002009;
			writel(v, p->swcore + SW_PORTCFG(pr));
			v = readl(p->swcore + SW_PORTCFG(pr));
			v = (v & 0xfd03ffff) | 0x02940000;
			writel(v, p->swcore + SW_PORTCFG(pr));
		}
		writel(readl(p->swcore + 0x414c) & 0xfe7fffff, p->swcore + 0x414c);
		writel((readl(p->swcore + 0x414c) & 0xfff3ffe8) | 0x000c0005,
		       p->swcore + 0x414c);
		writel(readl(p->swcore + 0x414c) | 0x06000000, p->swcore + 0x414c);
		writel(readl(p->swcore + 0x414c) | 0x40, p->swcore + 0x414c);
		writel(readl(p->swcore + 0x4058) | 1, p->swcore + 0x4058);
		writel(readl(p->swcore + 0x4100) | 1, p->swcore + 0x4100);
		writel(readl(p->swcore + SW_MAC_CONFIG) | 0x1000,
		       p->swcore + SW_MAC_CONFIG);
		/* SYS 0x18000850 control -- never programmed before */
		writel((readl(p->sysreg + 0x850) & 0x01bfffff) | 0xd8000000,
		       p->sysreg + 0x850);
	}

	/*
	 * FULL internal-GPHY analog calibration (stock fn 0x80228c3c), captured
	 * VERBATIM by emulating the boot NIC-init and logging every MDIO write:
	 * a 32-entry paged (page 0/1/2) analog-register sequence. Our earlier
	 * 3-register fragment was wildly incomplete -- THIS is the real init.
	 * {page, reg, value}; applied in order to each internal GPHY.
	 */
	{
		static const struct { u8 pg, reg; u16 val; } gphy_seq[] = {
			{1, 0x1d, 0x0002}, {1, 0x1d, 0x0004},
			{0, 0x00, 0x8000},			/* PHY reset */
			{1, 0x17, 0x7000}, {1, 0x12, 0x0050},
			{1, 0x10, 0xe000}, {1, 0x11, 0x3000}, {1, 0x10, 0x0000},
			{0, 0x17, 0x0100}, {0, 0x18, 0x0000},
			{2, 0x14, 0x0000}, {2, 0x10, 0x0c00}, {2, 0x1a, 0x0027},
			{2, 0x15, 0x8000}, {2, 0x18, 0x0200}, {2, 0x14, 0x0c44},
			{2, 0x1a, 0x0200}, {2, 0x19, 0x0658}, {2, 0x11, 0x0064},
			{2, 0x12, 0x00c5}, {2, 0x13, 0x010f}, {2, 0x10, 0x00b2},
			{2, 0x15, 0x01c8}, {2, 0x16, 0x011f}, {2, 0x17, 0x0064},
			{2, 0x18, 0x0034}, {2, 0x1a, 0x7000}, {2, 0x13, 0x6000},
			{2, 0x12, 0x2000}, {0, 0x18, 0x8000}, {2, 0x19, 0x0758},
		};
		static const u8 gphy_addr[] = { 1, 2, 3, 4, 8 };
		unsigned int n, k;

		for (n = 0; n < ARRAY_SIZE(gphy_addr); n++) {
			u8 a = gphy_addr[n];
			u8 cur = 0xff;

			for (k = 0; k < ARRAY_SIZE(gphy_seq); k++) {
				if (gphy_seq[k].pg != cur) {
					cur = gphy_seq[k].pg;
					rtl_mdio_write(p, a, 31, cur);
				}
				rtl_mdio_write(p, a, gphy_seq[k].reg,
					       gphy_seq[k].val);
			}
			rtl_mdio_write(p, a, 31, 0x0000);	/* page 0 */
		}
		mdelay(20);
		for (n = 0; n < ARRAY_SIZE(gphy_addr); n++) {
			u8 a = gphy_addr[n];

			rtl_mdio_write(p, a, MII_ADVERTISE, ADVERTISE_10_100_PAUSE);
			rtl_mdio_write(p, a, MII_BMCR, BMCR_ANEG_RESTART);
		}
	}
	(void)i;
}

static void rtl_dma_enable(struct rtl_eth *p);

/*
 * read-only diagnostic: dump the on-SoC switch port-config / GMIICR block + key
 * SYS/strap regs, to trace whether the bootloader configures port5(RGMII)/ext
 * ports and whether OpenWrt init later clears them. Called at the earliest
 * initcall (bootloader state) and around eth probe / rtl_open.
 */
static void rtl_regdump(const char *when, void __iomem *sw, void __iomem *sys)
{
	int i;

	pr_info("### a2004 regdump [%s] ###\n", when);
	for (i = 0x4104; i <= 0x4124; i += 4)
		pr_info("###  swcore %04x = 0x%08x%s\n", i, readl(sw + i),
			i == 0x4118 ? "  PCRP5(port5/MII)" :
			i == 0x411c ? "  PCRP6(extPort0/RGMII)" :
			i == 0x4120 ? "  PCRP7(extPort1/CPU)" : "");
	pr_info("###  P0GMIICR(414c)=0x%08x P5GMIICR(4150)=0x%08x MAC_CFG(4000)=0x%08x CVIDR(4200)=0x%08x\n",
		readl(sw + 0x414c), readl(sw + 0x4150),
		readl(sw + 0x4000), readl(sw + 0x4200));
	pr_info("###  SYS STRAP(0008)=0x%08x SYS0850=0x%08x CLK_MANAGE(0010)=0x%08x\n",
		readl(sys + 0x0008), readl(sys + 0x0850), readl(sys + 0x0010));
}

static int __init rtl_early_regdump(void)
{
	void __iomem *sw = ioremap(0x1b800000, 0x8000);
	void __iomem *sys = ioremap(0x18000000, 0x4000);

	if (sw && sys)
		rtl_regdump("EARLY-initcall (bootloader/pre-driver state)", sw, sys);
	if (sw)
		iounmap(sw);
	if (sys)
		iounmap(sys);
	return 0;
}
early_initcall(rtl_early_regdump);

static void rtl_apply_a(struct rtl_eth *p);
static void rtl_apply_fwd(struct rtl_eth *p);
static int rtl_hwnat_prepare(struct rtl_eth *p);
static void rtl_hwnat_off(struct rtl_eth *p);
static int rtl_hwnat_replace(struct rtl_eth *p, unsigned long cookie,
			     const struct rtl_ft_rule *rule);
static int rtl_hwnat_destroy(struct rtl_eth *p, unsigned long cookie);
static int rtl_hwnat_stats(struct rtl_eth *p, struct flow_cls_offload *cls);

static int rtl_open(struct net_device *ndev)
{
	struct rtl_eth *p = netdev_priv(ndev);

	rtl_regdump("OPEN-start (before clk/dma)", p->swcore, p->sysreg);
	napi_enable(&p->napi);

	/*
	 * Enable the switch/NIC core clock (bit 11 of CLK_MANAGE) -- idempotent;
	 * the bootloader already left it on because it uses the switch + external
	 * RTL8367 for its own LAN recovery.
	 *
	 * Deliberately DO NOT reset or reconfigure the on-SoC switch here: the
	 * bootloader has already set it up to bridge the CPU port <-> the RGMII
	 * port that the external RTL8367 sits behind, which is exactly the path
	 * DSA needs. Our old SW_RESET_FULL + GPHY-oriented rtl_switch_init tore
	 * that bridge down (frames reached the RTL8367 but never the CPU NIC).
	 */
	writel(readl(p->sysreg + CLK_MANAGE) | CLK_MANAGE_SWITCH,
	       p->sysreg + CLK_MANAGE);
	mdelay(10);

	rtl_apply_a(p);
	rtl_apply_fwd(p);

	netdev_info(ndev, "up: CVIDR=0x%08x FWD=0x%08x MSC=0x%08x\n",
		    readl(p->swcore + SW_CVIDR),
		    readl(p->swcore + SW_FRAME_FWD_CFG),
		    readl(p->swcore + SW_MODULE_SW_CONTROL));

	netif_start_queue(ndev);
	timer_setup(&p->poll_timer, rtl_poll_timer, 0);
	/* As the DSA conduit we must actually move frames: start the CPU-NIC
	 * DMA engine now (it is stable). RX/TX rings + poll timer come up here;
	 * the switch (on-SoC) bridges the CPU port to the RGMII port that the
	 * external RTL8367 (managed by the rtl8365mb DSA driver) sits behind. */
	rtl_dma_enable(p);
	rtl_regdump("OPEN-end (after clk/dma)", p->swcore, p->sysreg);
	return 0;
}

static int rtl_stop(struct net_device *ndev)
{
	struct rtl_eth *p = netdev_priv(ndev);

	netif_stop_queue(ndev);
	mutex_lock(&p->tbl_lock);
	rtl_hwnat_off(p);
	mutex_unlock(&p->tbl_lock);
	p->dma_on = false;
	rtl_irq_mask(p);
	timer_delete_sync(&p->poll_timer);
	napi_disable(&p->napi);
	rtl_wr(p, CPUICR, rtl_rd(p, CPUICR) & ~(TXCMD | RXCMD));
	rtl_free_rings(p);
	return 0;
}

static LIST_HEAD(rtl_ft_block_cb_list);

static int rtl_ft_dsa_port(struct net_device *dev, struct net_device *conduit)
{
	struct dsa_port *dp;

	dp = dsa_port_from_netdev(dev);
	if (IS_ERR(dp) || dsa_port_to_conduit(dp) != conduit || dp->index >= RTL_NPORTS)
		return -EOPNOTSUPP;

	return dp->index;
}

static __be32 rtl_ft_local_addr(struct net_device *dev)
{
	struct in_ifaddr *ifa;
	struct in_device *in_dev;
	__be32 addr = 0;

	rcu_read_lock();
	dev = netdev_master_upper_dev_get_rcu(dev) ?: dev;
	in_dev = __in_dev_get_rcu(dev);
	if (in_dev)
		in_dev_for_each_ifa_rcu(ifa, in_dev) {
			addr = ifa->ifa_local;
			break;
		}
	rcu_read_unlock();
	return addr;
}

static int rtl_ft_mangle_ports(const struct flow_action_entry *act,
			       struct rtl_ft_rule *data)
{
	u32 val = ntohl(act->mangle.val);

	switch (act->mangle.offset) {
	case 0:
		if (act->mangle.mask == ~htonl(0xffff))
			data->dst_port = cpu_to_be16(val);
		else if (act->mangle.mask == ~htonl(0xffff0000))
			data->src_port = cpu_to_be16(val >> 16);
		else
			return -EOPNOTSUPP;
		break;
	case 2:
		if (act->mangle.mask != ~htonl(0xffff))
			return -EOPNOTSUPP;
		data->dst_port = cpu_to_be16(val);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int rtl_ft_mangle_ipv4(const struct flow_action_entry *act,
			      struct rtl_ft_rule *data)
{
	__be32 *dest;

	switch (act->mangle.offset) {
	case offsetof(struct iphdr, saddr):
		dest = &data->src_addr;
		break;
	case offsetof(struct iphdr, daddr):
		dest = &data->dst_addr;
		break;
	default:
		return -EOPNOTSUPP;
	}
	memcpy(dest, &act->mangle.val, sizeof(*dest));
	return 0;
}

static int rtl_ft_mangle_eth(const struct flow_action_entry *act,
			     struct ethhdr *eth)
{
	void *dest = (u8 *)eth + act->mangle.offset;
	const void *src = &act->mangle.val;

	if (act->mangle.offset > 8)
		return -EOPNOTSUPP;
	if (act->mangle.mask == 0xffff) {
		src += 2;
		dest += 2;
	}
	memcpy(dest, src, act->mangle.mask ? 2 : 4);
	return 0;
}

static int rtl_ft_parse(struct rtl_eth *p, struct flow_cls_offload *cls,
			struct rtl_ft_rule *data)
{
	struct flow_rule *rule = flow_cls_offload_flow_rule(cls);
	struct flow_action_entry *act;
	struct flow_match_control control;
	struct flow_match_ipv4_addrs addrs;
	struct flow_match_basic basic;
	struct flow_match_ports ports;
	struct flow_match_meta meta;
	struct net_device *dev;
	int i, err;

	data->egress_port = RTL_NPORTS;

	if (!flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_META) ||
	    !flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_CONTROL) ||
	    !flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_BASIC) ||
	    !flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_IPV4_ADDRS) ||
	    !flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_PORTS))
		return -EOPNOTSUPP;

	flow_rule_match_control(rule, &control);
	if (control.key->addr_type != FLOW_DISSECTOR_KEY_IPV4_ADDRS ||
	    flow_rule_has_control_flags(control.mask->flags, cls->common.extack))
		return -EOPNOTSUPP;

	flow_rule_match_basic(rule, &basic);
	if (basic.key->ip_proto != IPPROTO_TCP &&
	    basic.key->ip_proto != IPPROTO_UDP)
		return -EOPNOTSUPP;
	data->ip_proto = basic.key->ip_proto;

	flow_rule_match_meta(rule, &meta);
	dev = __dev_get_by_index(&init_net, meta.key->ingress_ifindex);
	if (!dev)
		return -ENODEV;
	err = rtl_ft_dsa_port(dev, p->ndev);
	if (err < 0)
		return err;
	data->ingress_port = err;
	data->ingress_local_addr = rtl_ft_local_addr(dev);

	flow_rule_match_ipv4_addrs(rule, &addrs);
	data->orig_src_addr = data->src_addr = addrs.key->src;
	data->orig_dst_addr = data->dst_addr = addrs.key->dst;
	flow_rule_match_ports(rule, &ports);
	data->orig_src_port = data->src_port = ports.key->src;
	data->orig_dst_port = data->dst_port = ports.key->dst;

	flow_action_for_each(i, act, &rule->action) {
		switch (act->id) {
		case FLOW_ACTION_MANGLE:
			switch (act->mangle.htype) {
			case FLOW_ACT_MANGLE_HDR_TYPE_ETH:
				err = rtl_ft_mangle_eth(act, &data->eth);
				break;
			case FLOW_ACT_MANGLE_HDR_TYPE_IP4:
				err = rtl_ft_mangle_ipv4(act, data);
				break;
			case FLOW_ACT_MANGLE_HDR_TYPE_TCP:
				if (data->ip_proto != IPPROTO_TCP)
					return -EOPNOTSUPP;
				err = rtl_ft_mangle_ports(act, data);
				break;
			case FLOW_ACT_MANGLE_HDR_TYPE_UDP:
				if (data->ip_proto != IPPROTO_UDP)
					return -EOPNOTSUPP;
				err = rtl_ft_mangle_ports(act, data);
				break;
			default:
				err = -EOPNOTSUPP;
			}
			if (err)
				return err;
			break;
		case FLOW_ACTION_REDIRECT:
			err = rtl_ft_dsa_port(act->dev, p->ndev);
			if (err < 0)
				return err;
			data->egress_port = err;
			break;
		case FLOW_ACTION_CSUM:
			break;
		default:
			return -EOPNOTSUPP;
		}
	}

	if (data->egress_port >= RTL_NPORTS ||
	    !is_valid_ether_addr(data->eth.h_source) ||
	    !is_valid_ether_addr(data->eth.h_dest))
		return -EINVAL;

	return 0;
}

static int rtl_setup_ft_cb(enum tc_setup_type type, void *type_data,
			   void *cb_priv)
{
	struct net_device *ndev = cb_priv;
	struct rtl_eth *p = netdev_priv(ndev);
	struct flow_cls_offload *cls = type_data;

	if (type != TC_SETUP_CLSFLOWER)
		return -EOPNOTSUPP;

	switch (cls->command) {
	case FLOW_CLS_REPLACE:
	{
		struct rtl_ft_rule rule = {};
		int err;

		p->ft_replace++;
		err = rtl_ft_parse(p, cls, &rule);
		if (err) {
			p->ft_rejected++;
			return err;
		}
		p->ft_parsed++;
		return rtl_hwnat_replace(p, cls->cookie, &rule);
	}
	case FLOW_CLS_DESTROY:
		p->ft_destroy++;
		return rtl_hwnat_destroy(p, cls->cookie);
	case FLOW_CLS_STATS:
		p->ft_stats++;
		return rtl_hwnat_stats(p, cls);
	default:
		return -EOPNOTSUPP;
	}
}

static int rtl_setup_tc(struct net_device *ndev, enum tc_setup_type type,
			void *type_data)
{
	struct rtl_eth *p = netdev_priv(ndev);
	struct flow_block_offload *f = type_data;
	struct flow_block_cb *block_cb;

	if (type != TC_SETUP_FT)
		return -EOPNOTSUPP;
	if (f->binder_type != FLOW_BLOCK_BINDER_TYPE_CLSACT_INGRESS)
		return -EOPNOTSUPP;

	f->driver_block_list = &rtl_ft_block_cb_list;
	switch (f->command) {
	case FLOW_BLOCK_BIND:
		block_cb = flow_block_cb_lookup(f->block, rtl_setup_ft_cb, ndev);
		if (block_cb) {
			flow_block_cb_incref(block_cb);
		} else {
			block_cb = flow_block_cb_alloc(rtl_setup_ft_cb, ndev, ndev,
						     NULL);
			if (IS_ERR(block_cb))
				return PTR_ERR(block_cb);
			flow_block_cb_incref(block_cb);
			flow_block_cb_add(block_cb, f);
			list_add_tail(&block_cb->driver_list,
				      &rtl_ft_block_cb_list);
		}
		p->ft_binds++;
		return 0;
	case FLOW_BLOCK_UNBIND:
		block_cb = flow_block_cb_lookup(f->block, rtl_setup_ft_cb, ndev);
		if (!block_cb)
			return -ENOENT;
		if (!flow_block_cb_decref(block_cb)) {
			flow_block_cb_remove(block_cb, f);
			list_del(&block_cb->driver_list);
		}
		p->ft_unbinds++;
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static const struct net_device_ops rtl_netdev_ops = {
	.ndo_open = rtl_open,
	.ndo_stop = rtl_stop,
	.ndo_start_xmit = rtl_start_xmit,
	.ndo_setup_tc = rtl_setup_tc,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_validate_addr = eth_validate_addr,
};

/*
 * On-demand PHY status, read live via MDIO (no PHY reset). Read it any time
 * with:  cat /sys/devices/platform/18010000.ethernet/phy
 * Reports BMSR, current link (latch cleared by the double read) and
 * autoneg-done for each internal GPHY (addr 1..4).
 */
static ssize_t phy_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rtl_eth *p = netdev_priv(ndev);
	int a, bmsr, n = 0;

	for (a = 1; a <= 4; a++) {
		rtl_mdio_read(p, a, MII_BMSR);		/* clear latch */
		bmsr = rtl_mdio_read(p, a, MII_BMSR) & 0xffff;
		n += sprintf(buf + n,
			     "phy%d bmsr=0x%04x link=%d aneg_done=%d\n",
			     a, bmsr, !!(bmsr & BMSR_LINK), !!(bmsr & 0x20));
	}
	return n;
}
static DEVICE_ATTR_RO(phy);

/*
 * Interactive MDIO poke (no rebuild needed to experiment):
 *   echo "PHY REG"     > .../mdio ; cat .../mdio   -> read  (PHY,REG decimal)
 *   echo "PHY REG VAL" > .../mdio                  -> write (VAL hex, no 0x)
 * e.g.  echo "1 1"     -> read  phy1 BMSR
 *       echo "1 0 1200"-> write phy1 BMCR = autoneg-restart
 * Results also go to dmesg.
 */
static ssize_t mdio_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rtl_eth *p = netdev_priv(ndev);

	return sprintf(buf, "0x%04x\n", p->mdio_last);
}

static ssize_t mdio_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t len)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rtl_eth *p = netdev_priv(ndev);
	unsigned int phy, reg, val;
	int n;

	n = sscanf(buf, "%u %u %x", &phy, &reg, &val);
	if (n < 2 || phy > 31 || reg > 31)
		return -EINVAL;

	if (n == 2) {
		p->mdio_last = rtl_mdio_read(p, phy, reg) & 0xffff;
		dev_info(dev, "mdio rd phy%u reg%u = 0x%04x\n",
			 phy, reg, p->mdio_last);
	} else {
		rtl_mdio_write(p, phy, reg, val);
		dev_info(dev, "mdio wr phy%u reg%u <= 0x%04x\n", phy, reg, val);
	}
	return len;
}
static DEVICE_ATTR_RW(mdio);

/*
 * Interactive raw SoC register poke (32-bit), for replaying the bootcode's
 * switch/GPHY bring-up live:
 *   echo "ADDR"     > .../reg ; cat .../reg   -> read  (ADDR hex phys)
 *   echo "ADDR VAL" > .../reg                 -> write (both hex)
 * e.g.  echo 1800005c > reg ; cat reg   (SYS_GPHY_CTRL)
 *       echo "1b804104 427f0038" > reg  (port0 config, force mode)
 * Addresses inside the driver's already-mapped windows reuse those maps;
 * anything else is ioremap'd for the single access.
 */
static void __iomem *rtl_resolve(struct rtl_eth *p, u32 addr,
				 void __iomem **tofree)
{
	*tofree = NULL;
	if (addr >= SYSREG_BASE_PHYS && addr < SYSREG_BASE_PHYS + SYSREG_SIZE)
		return p->sysreg + (addr - SYSREG_BASE_PHYS);
	if (addr >= 0x18010000 && addr < 0x18010200)
		return p->base + (addr - 0x18010000);
	if (addr >= SWCORE_BASE_PHYS && addr < SWCORE_BASE_PHYS + SWCORE_SIZE)
		return p->swcore + (addr - SWCORE_BASE_PHYS);
	*tofree = ioremap(addr, 4);
	return *tofree;
}

static ssize_t reg_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rtl_eth *p = netdev_priv(ndev);

	return sprintf(buf, "0x%08x\n", p->reg_last);
}

static ssize_t reg_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t len)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rtl_eth *p = netdev_priv(ndev);
	void __iomem *m, *tofree;
	unsigned int addr, val;
	int n;

	n = sscanf(buf, "%x %x", &addr, &val);
	if (n < 1 || (addr & 3))
		return -EINVAL;

	m = rtl_resolve(p, addr, &tofree);
	if (!m)
		return -ENOMEM;

	if (n == 1) {
		p->reg_last = readl(m);
		dev_info(dev, "reg rd 0x%08x = 0x%08x\n", addr, p->reg_last);
	} else {
		writel(val, m);
		dev_info(dev, "reg wr 0x%08x <= 0x%08x\n", addr, val);
	}
	if (tofree)
		iounmap(tofree);
	return len;
}
static DEVICE_ATTR_RW(reg);

/* ------------------------------------------------------------------ *
 * External RTL8367 switch: GPIO-bit-banged SMI (2-wire).
 *
 * The A2004MU's physical LAN/WAN ports are on an EXTERNAL RTL8367R
 * (chip id 0x6367), reached over RGMII + a software SMI clocked on two
 * GPIO pins.  Their exact pin numbers are compiled into the stock kernel;
 * we discover them empirically: sweep GPIO-configured pins in ports A-D,
 * read RTL8367 CHIP_NUMBER (reg 0x1300 -> 0x6367) for each candidate pair.
 *
 * GPIO block lives at 0x18003500 (inside the sysreg window):
 *   ports A-D: CNR +0x00, DIR +0x08, DAT +0x0C ; bit = port*8 + pin
 *   ports E-H: CNR +0x1C, DIR +0x24, DAT +0x28 ; bit = (port-4)*8 + pin
 *   CNR bit 0 = GPIO function ; DIR 1 = output.  gpioId = port<<8 | pin.
 * ------------------------------------------------------------------ */
#define GPIO_BLK 0x3500
static const u8 gp_cnr[2] = { 0x00, 0x1c };
static const u8 gp_dir[2] = { 0x08, 0x24 };
static const u8 gp_dat[2] = { 0x0c, 0x28 };
#define GP_BLK(id) (((id) >> 8) < 4 ? 0 : 1)
#define GP_BIT(id) (((((id) >> 8) & 3) * 8) + ((id) & 7))

static inline void gp_wb(struct rtl_eth *p, u8 off, u32 bit, int set)
{
	void __iomem *r = p->sysreg + GPIO_BLK + off;
	u32 v = readl(r);

	if (set)
		v |= (1u << bit);
	else
		v &= ~(1u << bit);
	writel(v, r);
}
static void gp_out(struct rtl_eth *p, u16 id)		/* GPIO mux, dir=out */
{
	gp_wb(p, gp_cnr[GP_BLK(id)], GP_BIT(id), 0);
	gp_wb(p, gp_dir[GP_BLK(id)], GP_BIT(id), 1);
}
static void gp_in(struct rtl_eth *p, u16 id)		/* GPIO mux, dir=in */
{
	gp_wb(p, gp_cnr[GP_BLK(id)], GP_BIT(id), 0);
	gp_wb(p, gp_dir[GP_BLK(id)], GP_BIT(id), 0);
}
static void gp_set(struct rtl_eth *p, u16 id, int v)
{
	gp_wb(p, gp_dat[GP_BLK(id)], GP_BIT(id), v);
}
static int gp_get(struct rtl_eth *p, u16 id)
{
	return (readl(p->sysreg + GPIO_BLK + gp_dat[GP_BLK(id)])
		>> GP_BIT(id)) & 1;
}

#define SMI_DLY() udelay(3)
#define SCK (p->smi_sck)
#define SDA (p->smi_sda)

static void smi_start(struct rtl_eth *p)
{
	gp_out(p, SDA); gp_out(p, SCK);
	gp_set(p, SCK, 0); gp_set(p, SDA, 1); SMI_DLY();
	gp_set(p, SCK, 1); SMI_DLY(); gp_set(p, SCK, 0); SMI_DLY();
	gp_set(p, SCK, 1); SMI_DLY(); gp_set(p, SDA, 0); SMI_DLY();
	gp_set(p, SCK, 0); SMI_DLY(); gp_set(p, SDA, 1);
}
static void smi_stop(struct rtl_eth *p)
{
	SMI_DLY(); gp_set(p, SDA, 0); gp_set(p, SCK, 1);
	SMI_DLY(); gp_set(p, SDA, 1); SMI_DLY(); gp_set(p, SCK, 1);
	SMI_DLY(); gp_set(p, SCK, 0); SMI_DLY(); gp_set(p, SCK, 1);
	SMI_DLY(); gp_set(p, SCK, 0); SMI_DLY(); gp_set(p, SCK, 1);
	gp_in(p, SDA); gp_in(p, SCK);
}
static void smi_wbit(struct rtl_eth *p, u16 sig, int len)
{
	for (; len > 0; len--) {
		SMI_DLY();
		gp_set(p, SDA, (sig >> (len - 1)) & 1); SMI_DLY();
		gp_set(p, SCK, 1); SMI_DLY(); gp_set(p, SCK, 0);
	}
}
static void smi_rbit(struct rtl_eth *p, int len, u32 *rd)
{
	u32 u;

	gp_in(p, SDA);
	for (*rd = 0; len > 0; len--) {
		SMI_DLY(); gp_set(p, SCK, 1); SMI_DLY();
		u = gp_get(p, SDA); gp_set(p, SCK, 0);
		*rd |= (u << (len - 1));
	}
	gp_out(p, SDA);
}
/* Read a 16-bit RTL8367 register (bit-bang SMI).  Returns 0 if every
 * transaction phase was ACKed by a device (i.e. a switch is really there). */
static int smi_read_reg(struct rtl_eth *p, u32 addr, u32 *val)
{
	u32 ack, raw;
	int con, ret = 0;

	smi_start(p);
	smi_wbit(p, 0xB, 4);			/* CTRL_CODE (RTL8367) */
	smi_wbit(p, 0x4, 3);
	smi_wbit(p, 0x1, 1);			/* issue READ */
	con = 0; do { con++; smi_rbit(p, 1, &ack); } while (ack && con < 5);
	if (ack) ret = -1;
	smi_wbit(p, addr & 0xff, 8);		/* reg_addr[7:0] */
	con = 0; do { con++; smi_rbit(p, 1, &ack); } while (ack && con < 5);
	if (ack) ret = -1;
	smi_wbit(p, (addr >> 8) & 0xff, 8);	/* reg_addr[15:8] */
	con = 0; do { con++; smi_rbit(p, 1, &ack); } while (ack && con < 5);
	if (ack) ret = -1;
	smi_rbit(p, 8, &raw); *val = raw & 0xff;		/* data[7:0] */
	smi_wbit(p, 0x00, 1);					/* CPU ACK */
	smi_rbit(p, 8, &raw); *val |= (raw & 0xff) << 8;	/* data[15:8] */
	smi_wbit(p, 0x01, 1);					/* CPU NACK */
	smi_stop(p);
	return ret;
}

/* Write a 16-bit RTL8367 register (bit-bang SMI). */
static int smi_write_reg(struct rtl_eth *p, u32 addr, u32 data)
{
	u32 ack;
	int con, ret = 0;

	smi_start(p);
	smi_wbit(p, 0xB, 4);
	smi_wbit(p, 0x4, 3);
	smi_wbit(p, 0x0, 1);			/* issue WRITE */
	con = 0; do { con++; smi_rbit(p, 1, &ack); } while (ack && con < 5);
	if (ack) ret = -1;
	smi_wbit(p, addr & 0xff, 8);
	con = 0; do { con++; smi_rbit(p, 1, &ack); } while (ack && con < 5);
	if (ack) ret = -1;
	smi_wbit(p, (addr >> 8) & 0xff, 8);
	con = 0; do { con++; smi_rbit(p, 1, &ack); } while (ack && con < 5);
	if (ack) ret = -1;
	smi_wbit(p, data & 0xff, 8);		/* data[7:0] */
	con = 0; do { con++; smi_rbit(p, 1, &ack); } while (ack && con < 5);
	if (ack) ret = -1;
	smi_wbit(p, (data >> 8) & 0xff, 8);	/* data[15:8] */
	con = 0; do { con++; smi_rbit(p, 1, &ack); } while (ack && con < 5);
	if (ack) ret = -1;
	smi_stop(p);
	return ret;
}

#define RTL8367_TBL_CTRL	0x0500
#define RTL8367_TBL_ADDR	0x0501
#define RTL8367_TBL_WRDATA	0x0510
#define RTL8367_TBL_RDDATA	0x0520
#define RTL8367_PORT_MISC(port)	(0x000e + ((port) << 5))
#define RTL8367_VLAN_EGRESS_MODE	0x0030

static int rtl8367_vlan_read(struct rtl_eth *p, u16 vid, u16 data[2])
{
	u32 val;
	int err;

	err = smi_write_reg(p, RTL8367_TBL_ADDR, vid);
	if (!err)
		err = smi_write_reg(p, RTL8367_TBL_CTRL, 0x0003);
	if (!err)
		err = smi_read_reg(p, RTL8367_TBL_RDDATA, &val);
	if (!err)
		data[0] = val;
	if (!err)
		err = smi_read_reg(p, RTL8367_TBL_RDDATA + 1, &val);
	if (!err)
		data[1] = val;
	return err;
}

static int rtl8367_vlan_write(struct rtl_eth *p, u16 vid, const u16 data[2])
{
	int err;

	err = smi_write_reg(p, RTL8367_TBL_WRDATA, data[0]);
	if (!err)
		err = smi_write_reg(p, RTL8367_TBL_WRDATA + 1, data[1]);
	if (!err)
		err = smi_write_reg(p, RTL8367_TBL_ADDR, vid);
	if (!err)
		err = smi_write_reg(p, RTL8367_TBL_CTRL, 0x000b);
	return err;
}

static void rtl_hwnat_ext_restore(struct rtl_eth *p)
{
	int port;

	if (!p->hwnat_ext_saved)
		return;
	rtl8367_vlan_write(p, RTL_WAN_VID, p->hwnat_saved_ext_vlan[0]);
	rtl8367_vlan_write(p, RTL_LAN_VID, p->hwnat_saved_ext_vlan[1]);
	for (port = 0; port < 5; port++)
		smi_write_reg(p, RTL8367_PORT_MISC(port),
			      p->hwnat_saved_ext_misc[port]);
	p->hwnat_ext_saved = false;
}

static int rtl_hwnat_ext_prepare(struct rtl_eth *p)
{
	static const u16 vlan[2][2] = {
		{ 0x1050, 0x4001 }, /* VID8: CPU6 + WAN4, untag WAN4, FID1 */
		{ 0x0f4f, 0x4000 }, /* VID9: CPU6 + LAN0..3, untag LAN0..3 */
	};
	u16 check[2];
	u32 val;
	int port, err;

	err = rtl8367_vlan_read(p, RTL_WAN_VID,
				p->hwnat_saved_ext_vlan[0]);
	if (!err)
		err = rtl8367_vlan_read(p, RTL_LAN_VID,
					p->hwnat_saved_ext_vlan[1]);
	for (port = 0; !err && port < 5; port++) {
		err = smi_read_reg(p, RTL8367_PORT_MISC(port), &val);
		p->hwnat_saved_ext_misc[port] = val;
	}
	if (err)
		return err;
	p->hwnat_ext_saved = true;

	err = rtl8367_vlan_write(p, RTL_WAN_VID, vlan[0]);
	if (!err)
		err = rtl8367_vlan_write(p, RTL_LAN_VID, vlan[1]);
	for (port = 0; !err && port < 5; port++)
		err = smi_write_reg(p, RTL8367_PORT_MISC(port),
				    p->hwnat_saved_ext_misc[port] &
				    ~RTL8367_VLAN_EGRESS_MODE);
	if (!err)
		err = rtl8367_vlan_read(p, RTL_WAN_VID, check);
	if (!err && memcmp(check, vlan[0], sizeof(check)))
		err = -EIO;
	if (!err)
		err = rtl8367_vlan_read(p, RTL_LAN_VID, check);
	if (!err && memcmp(check, vlan[1], sizeof(check)))
		err = -EIO;
	for (port = 0; !err && port < 5; port++) {
		err = smi_read_reg(p, RTL8367_PORT_MISC(port), &val);
		if (!err && (val & RTL8367_VLAN_EGRESS_MODE))
			err = -EIO;
	}
	if (err)
		rtl_hwnat_ext_restore(p);
	return err;
}

static void smi_dump_gpio(struct rtl_eth *p, struct device *dev)
{
	dev_info(dev, "GPIO A-D: CNR=%08x DIR=%08x DAT=%08x\n",
		 readl(p->sysreg + GPIO_BLK + 0x00),
		 readl(p->sysreg + GPIO_BLK + 0x08),
		 readl(p->sysreg + GPIO_BLK + 0x0c));
	dev_info(dev, "GPIO E-H: CNR=%08x DIR=%08x DAT=%08x\n",
		 readl(p->sysreg + GPIO_BLK + 0x1c),
		 readl(p->sysreg + GPIO_BLK + 0x24),
		 readl(p->sysreg + GPIO_BLK + 0x28));
}

/* commands via  echo ... > .../smi :
 *   dump                    - print GPIO A-H control/dir/data
 *   read  <sck> <sda> <reg> - one SMI read (pins are linear 0-63, reg hex)
 *   sweep                   - try every GPIO-configured A-D pin pair,
 *                             report any that returns 0x6367 at reg 0x1300
 *   id                      - read RTL8367 chip number/version/mode
 *   ports                   - read per-port link status (reg 0x1352+port)
 *   sr <reg>                - read one switch register (hex)
 *   sw <reg> <val>          - write one switch register (both hex)
 * (sr/sw/id/ports use the discovered pins: SCK=pin18 SDA=pin19)
 */
static ssize_t smi_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t len)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rtl_eth *p = netdev_priv(ndev);
	unsigned int a, b, r;
	u32 val;

	if (!strncmp(buf, "dump", 4)) {
		smi_dump_gpio(p, dev);
		return len;
	}
	if (sscanf(buf, "read %u %u %x", &a, &b, &r) == 3) {
		p->smi_sck = ((a / 8) << 8) | (a % 8);
		p->smi_sda = ((b / 8) << 8) | (b % 8);
		val = 0xdead;
		r = smi_read_reg(p, r, &val);
		dev_info(dev, "SMI sck=%u sda=%u reg -> 0x%04x (ack=%s)\n",
			 a, b, val, r ? "no" : "yes");
		return len;
	}
	if (!strncmp(buf, "sweep", 5)) {
		u32 sv_cnr, sv_dir, sv_dat;
		int cand[32], nc = 0, i, j, pin, hits = 0;
		u32 cnr = readl(p->sysreg + GPIO_BLK + 0x00);

		smi_dump_gpio(p, dev);
		sv_cnr = cnr;
		sv_dir = readl(p->sysreg + GPIO_BLK + 0x08);
		sv_dat = readl(p->sysreg + GPIO_BLK + 0x0c);

		/* candidates: ports A-D pins currently muxed as GPIO (CNR bit 0) */
		for (pin = 0; pin < 32; pin++)
			if (!((cnr >> pin) & 1))
				cand[nc++] = pin;
		dev_info(dev, "SMI sweep: %d GPIO-mode pins in A-D\n", nc);

		for (i = 0; i < nc && hits < 4; i++) {
			for (j = 0; j < nc; j++) {
				if (i == j)
					continue;
				p->smi_sck = ((cand[i] / 8) << 8) | (cand[i] % 8);
				p->smi_sda = ((cand[j] / 8) << 8) | (cand[j] % 8);
				val = 0;
				smi_read_reg(p, 0x1300, &val);
				if (val == 0x6367) {
					dev_info(dev, "*** RTL8367 FOUND: SCK=pin%d SDA=pin%d (reg0x1300=0x%04x) ***\n",
						 cand[i], cand[j], val);
					hits++;
				}
			}
		}
		/* restore ports A-D exactly as the bootloader left them */
		writel(sv_cnr, p->sysreg + GPIO_BLK + 0x00);
		writel(sv_dir, p->sysreg + GPIO_BLK + 0x08);
		writel(sv_dat, p->sysreg + GPIO_BLK + 0x0c);
		if (!hits)
			dev_info(dev, "SMI sweep: no 0x6367 among A-D GPIO pins\n");
		return len;
	}
	if (!strncmp(buf, "id", 2)) {
		u32 num = 0, ver = 0, mode = 0;

		smi_read_reg(p, 0x1300, &num);
		smi_read_reg(p, 0x1301, &ver);
		smi_read_reg(p, 0x1302, &mode);
		dev_info(dev, "RTL8367 chip_num=0x%04x ver=0x%04x mode=0x%04x\n",
			 num, ver, mode);
		return len;
	}
	if (!strncmp(buf, "ports", 5)) {
		int pt;

		for (pt = 0; pt <= 6; pt++) {
			u32 st = 0;

			smi_read_reg(p, 0x1352 + pt, &st);
			dev_info(dev, "port%d status=0x%04x link=%d speed=%d duplex=%d\n",
				 pt, st, !!(st & 0x10), st & 3, !!(st & 4));
		}
		return len;
	}
	if (sscanf(buf, "sr %x", &r) == 1) {
		val = 0;
		a = smi_read_reg(p, r, &val);
		dev_info(dev, "SMI switch reg 0x%04x = 0x%04x (ack=%s)\n",
			 r, val, a ? "no" : "yes");
		return len;
	}
	if (sscanf(buf, "sw %x %x", &r, &val) == 2) {
		a = smi_write_reg(p, r, val);
		dev_info(dev, "SMI switch reg 0x%04x <= 0x%04x (ack=%s)\n",
			 r, val, a ? "no" : "yes");
		return len;
	}
	if (!strncmp(buf, "rgmiisweep", 10)) {
		/* Empirically find the ext-port RGMII delay (RTL8367B_EXT_RGMXF_REG
		 * 0x1306/0x1307: TXDELAY bit3, RXDELAY bits0-2) that lets frames
		 * actually cross to the SoC.  Run a continuous ping on the LAN while
		 * this sweeps; a delay that receives anything is reported. */
		int d;
		unsigned long b;

		dev_info(dev, "RGMII delay sweep starting (ping the router now)\n");
		for (d = 0; d < 16; d++) {
			b = ndev->stats.rx_packets + ndev->stats.rx_errors;
			smi_write_reg(p, 0x1306, d);
			smi_write_reg(p, 0x1307, d);
			mdelay(900);
			if (ndev->stats.rx_packets + ndev->stats.rx_errors > b)
				dev_info(dev, "*** RGMII delay=0x%x GOT FRAMES rx=%lu err=%lu ***\n",
					 d, ndev->stats.rx_packets,
					 ndev->stats.rx_errors);
		}
		dev_info(dev, "RGMII delay sweep done\n");
		return len;
	}
	return -EINVAL;
}
static DEVICE_ATTR_WO(smi);

/* ------------------------------------------------------------------ *
 * Runtime-gated CPU-NIC DMA engine.  Kept OFF at boot (a bad enable
 * used to scribble kernel memory); enable/disable/inspect at runtime so
 * a misfire only needs a reboot, never bricks the boot.  Now that the
 * datapath is understood (frames flow LAN -> ext RTL8367 -> RGMII ->
 * on-SoC switch CPU port -> this DMA ring), turn it on and watch RX/TX.
 *   echo on   > .../dma   - init rings, start engine + switch TRX, arm poll
 *   echo off  > .../dma   - stop engine, free rings
 *   echo stat > .../dma   - dump engine/ring/counter state
 * ------------------------------------------------------------------ */
static void rtl_dma_enable(struct rtl_eth *p)
{
	if (p->dma_on)
		return;

	/*
	 * RTL8197F splits NIC Tx/Rx over two Lexra buses when CPUICR1 bit 0 is
	 * set.  The vendor SDK enables the LX2 clock and arbiter before setting
	 * that bit; without them Rx works but Tx descriptors never complete.
	 */
	writel(readl(p->sysreg + CLK_MANAGE) |
	       CLK_MANAGE_LX2_CLK | CLK_MANAGE_LX2_ARB,
	       p->sysreg + CLK_MANAGE);
	writel(readl(p->swcore + SW_MAC_CTRL1) |
	       SW_CMAC_CLK_SEL | SW_CMAC_LATPKT_EN,
	       p->swcore + SW_MAC_CTRL1);

	if (rtl_init_rings(p)) {
		netdev_err(p->ndev, "DMA enable: ring alloc failed\n");
		return;
	}
	rtl_wr(p, CPUICR1, rtl_rd(p, CPUICR1) |
	       CPUICR1_TXRX_DIV_LX | CPUICR1_LITTLE_ENDIAN |
	       CPUICR1_TSO_ID | CPUICR1_NEW_DESC);
	rtl_wr(p, CPUICR, CPUICR_ENABLE);
	/*
	 * Updating CPUICR's burst-size field resets RTL8197F HiFifoMark to
	 * 0x57.  Match rtl865x_start() by restoring both marks afterwards.
	 */
	rtl_wr(p, DMA_CR0, (rtl_rd(p, DMA_CR0) & ~0xffffu) | 0xa0a0);
	/* start normal TX/RX in the switch core (SIRR TRXRDY). */
	writel(readl(p->swcore + SW_SIRR) | SIRR_TRXRDY, p->swcore + SW_SIRR);
	p->dma_on = true;

	/*
	 * Bring up interrupt-driven RX only once the rings are armed. The ictl
	 * source stays masked until the NIC's own mask is programmed, so a
	 * status bit left over from the bootloader cannot storm us on IP4.
	 */
	if (p->irq >= 0) {
		u32 irr1;

		rtl_wr(p, CPUIISR, rtl_rd(p, CPUIISR));
		rtl_wr(p, CPUIIMR, II_NAPI_SOURCES);

		irr1 = readl(p->sysreg + ICTL_IRR1);
		irr1 = (irr1 & ~ICTL_IRR1_SRC15_MASK) | ICTL_IRR1_SRC15_IP;
		writel(irr1, p->sysreg + ICTL_IRR1);
		writel(readl(p->sysreg + ICTL_GIMR) | ICTL_GIMR_SW_IE,
		       p->sysreg + ICTL_GIMR);
		p->irq_mode = true;
	}
	mod_timer(&p->poll_timer, jiffies + msecs_to_jiffies(p->irq_mode ?
							     RTL_SAFETY_MS :
							     RTL_POLL_MS));
	netdev_info(p->ndev,
		    "DMA ENABLED (%s): CPUICR=0x%08x rxbase=0x%08x txbase=0x%08x\n",
		    p->irq_mode ? "irq" : "polled",
		    rtl_rd(p, CPUICR), rtl_rd(p, CPURPDCR(0)),
		    rtl_rd(p, CPUTPDCR(0)));
}
static void rtl_dma_disable(struct rtl_eth *p)
{
	if (!p->dma_on)
		return;
	p->dma_on = false;
	rtl_irq_mask(p);
	timer_delete_sync(&p->poll_timer);
	rtl_wr(p, CPUICR, rtl_rd(p, CPUICR) & ~(TXCMD | RXCMD));
	rtl_free_rings(p);
	netdev_info(p->ndev, "DMA DISABLED\n");
}
/*
 * Verbatim stock on-SoC switch init, captured from the vendor kernel boot via
 * the MIPS emulator (swcore-relative offset, value).  Replayed to reproduce the
 * exact ALE/VLAN-table/forwarding/MAC config that makes frames from the RGMII
 * port (-> external RTL8367) reach the CPU port.  Entry 0 is SW_RESET_FULL.
 */
static const struct { u16 off; u32 val; } stock_swinit[] = {
	{ 0x4204, 0x00000004 }, { 0x4504, 0x01b201c4 }, { 0x4508, 0x01800190 },
	{ 0x4104, 0x00000000 }, { 0x4108, 0x00000000 }, { 0x410c, 0x00000000 },
	{ 0x4110, 0x00000000 }, { 0x4114, 0x00000000 }, { 0x7014, 0x00000000 },
	{ 0x4704, 0x00010001 }, { 0x4708, 0x00010001 }, { 0x470c, 0x00010001 },
	{ 0x4710, 0x00000000 }, { 0x457c, 0x00fc00ff }, { 0x4580, 0x00fc00ff },
	{ 0x4584, 0x00fc00ff }, { 0x45dc, 0x00000000 }, { 0x45dc, 0x00000001 },
	{ 0x45dc, 0x00000000 }, { 0x7014, 0x00000000 }, { 0x4704, 0x00000000 },
	{ 0x4708, 0x00000000 }, { 0x470c, 0x00000000 }, { 0x4710, 0x00000000 },
	{ 0x4104, 0x00000000 }, { 0x4108, 0x00000000 }, { 0x410c, 0x00000000 },
	{ 0x4110, 0x00000000 }, { 0x4114, 0x00000000 }, { 0x4104, 0x02000000 },
	{ 0x4108, 0x02000000 }, { 0x410c, 0x02000000 }, { 0x4110, 0x02000000 },
	{ 0x4114, 0x02000000 }, { 0x4000, 0x00000480 }, { 0x4104, 0x00000000 },
	{ 0x4108, 0x00000000 }, { 0x410c, 0x00000000 }, { 0x4110, 0x00000000 },
	{ 0x4114, 0x00000000 }, { 0x441c, 0x00000400 }, { 0x441c, 0x00000c00 },
	{ 0x4410, 0x00000000 }, { 0x4410, 0x00000000 }, { 0x4410, 0x00000000 },
	{ 0x4400, 0x00000003 }, { 0x4418, 0x00040000 }, { 0x4d20, 0x00008000 },
	{ 0x4d24, 0x00000000 }, { 0x4d28, 0x00000001 }, { 0x4d2c, 0x00000000 },
	{ 0x4d30, 0x00000000 }, { 0x4d08, 0xbb040000 }, { 0x4d00, 0x00000009 },
	{ 0x4418, 0x00000000 }, { 0x4418, 0x00040000 }, { 0x4d20, 0x00008000 },
	{ 0x4d08, 0xbb040020 }, { 0x4d00, 0x00000009 }, { 0x4418, 0x00000000 },
	{ 0x4418, 0x00040000 }, { 0x4d20, 0x00008000 }, { 0x4d08, 0xbb040040 },
	{ 0x4d00, 0x00000009 }, { 0x4418, 0x00000000 }, { 0x4418, 0x00040000 },
	{ 0x4d20, 0x00008000 }, { 0x4d08, 0xbb040060 }, { 0x4d00, 0x00000009 },
	{ 0x4418, 0x00000000 }, { 0x4418, 0x00040000 }, { 0x4d20, 0x00008000 },
	{ 0x4d08, 0xbb040080 }, { 0x4d00, 0x00000009 }, { 0x4418, 0x00000000 },
	{ 0x4234, 0x00000000 }, { 0x4234, 0x00000024 }, { 0x4418, 0x00040000 },
	{ 0x4d20, 0xbb800000 }, { 0x4d08, 0xbb0c0000 }, { 0x4d00, 0x00000009 },
	{ 0x4418, 0x00000000 }, { 0x4418, 0x00040000 }, { 0x4d20, 0xbb800000 },
	{ 0x4d08, 0xbb0c0020 }, { 0x4d00, 0x00000009 }, { 0x4418, 0x00000000 },
	{ 0x4418, 0x00040000 }, { 0x4d20, 0xbb800000 }, { 0x4d08, 0xbb0c0040 },
	{ 0x4d00, 0x00000009 }, { 0x4418, 0x00000000 }, { 0x4418, 0x00040000 },
	{ 0x4d20, 0xbb800000 }, { 0x4d08, 0xbb0c0060 }, { 0x4d00, 0x00000009 },
	{ 0x4418, 0x00000000 }, { 0x4418, 0x00040000 }, { 0x4d20, 0xbb800000 },
	{ 0x4d08, 0xbb0c0080 }, { 0x4d00, 0x00000009 }, { 0x4418, 0x00000000 },
	{ 0x4418, 0x00040000 }, { 0x4d20, 0x5a5a5a5a }, { 0x4d08, 0xbb000000 },
	{ 0x4d00, 0x00000009 }, { 0x4418, 0x00000000 }, { 0x4418, 0x00040000 },
	{ 0x4d20, 0x5a5a5a5a }, { 0x4d08, 0xbb0e0000 }, { 0x4d00, 0x00000009 },
	{ 0x4418, 0x00000000 }, { 0x4410, 0x00000000 }, { 0x4408, 0x00000000 },
	{ 0x4300, 0x00200000 }, { 0x1000, 0x0007ffff }, { 0x4418, 0x00000000 },
	{ 0x4a00, 0x00000000 }, { 0x4418, 0x00000000 }, { 0x4418, 0x00004000 },
	{ 0x4418, 0x00004000 }, { 0x4428, 0x00000000 }, { 0x4428, 0x00000001 },
	{ 0x4048, 0x00000000 }, { 0x4048, 0x00000000 }, { 0x4048, 0x00000000 },
	{ 0x4048, 0x00000010 }, { 0x4048, 0x00000030 }, { 0x4a1c, 0x00000000 },
	{ 0x4a20, 0x00000000 }, { 0x4a24, 0x00000000 }, { 0x4a28, 0x00000000 },
	{ 0x4a2c, 0x00000000 }, { 0x4104, 0x00000030 }, { 0x4108, 0x00000030 },
	{ 0x410c, 0x00000030 }, { 0x4110, 0x00000030 }, { 0x4114, 0x00000030 },
	{ 0x4704, 0x00000000 }, { 0x4710, 0x00000000 }, { 0x48b0, 0x00000000 },
	{ 0x4704, 0x00000000 }, { 0x4710, 0x00000000 }, { 0x48bc, 0x00000000 },
	{ 0x4708, 0x00000000 }, { 0x4710, 0x00000000 }, { 0x48c8, 0x00000000 },
	{ 0x4708, 0x00000000 }, { 0x4710, 0x00000000 }, { 0x48d4, 0x00000000 },
	{ 0x470c, 0x00000000 }, { 0x4710, 0x00000000 }, { 0x470c, 0x00000000 },
	{ 0x4710, 0x00000000 }, { 0x4104, 0x00030030 }, { 0x4108, 0x00030030 },
	{ 0x410c, 0x00030030 }, { 0x4110, 0x00030030 }, { 0x4114, 0x00030030 },
};
static void rtl_stock_swinit(struct rtl_eth *p)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(stock_swinit); i++) {
		writel(stock_swinit[i].val, p->swcore + stock_swinit[i].off);
		if (i == 0)
			mdelay(50);	/* let SW_RESET_FULL settle */
		else
			udelay(20);	/* let indirect table ops complete */
	}
	/* start normal TX/RX (SIRR TRXRDY), clearing any residual reset bit. */
	writel(SIRR_TRXRDY, p->swcore + SW_SIRR);
	netdev_info(p->ndev, "stock swinit replayed (%u writes)\n",
		    (unsigned int)ARRAY_SIZE(stock_swinit));
}

static ssize_t dma_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t len)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rtl_eth *p = netdev_priv(ndev);

	if (!strncmp(buf, "on", 2)) {
		rtl_dma_enable(p);
	} else if (!strncmp(buf, "swinit", 6)) {
		/* replay the exact stock on-SoC switch init, then (re)start DMA */
		bool was_on = p->dma_on;

		rtl_dma_disable(p);
		rtl_stock_swinit(p);
		if (was_on)
			rtl_dma_enable(p);
	} else if (!strncmp(buf, "off", 3)) {
		rtl_dma_disable(p);
	} else if (!strncmp(buf, "stat", 4)) {
		/* Per-ring cdp so we can see WHICH of the 6 RX rings the switch
		 * delivers CPU-bound frames to (cdp advances off its base). */
		netdev_info(ndev,
			    "dma_on=%d CPUICR=0x%08x rx=%lu tx=%lu txdrop=%lu rxerr=%lu rxdrop=%lu\n",
			    p->dma_on, rtl_rd(p, CPUICR),
			    ndev->stats.rx_packets, ndev->stats.tx_packets,
			    ndev->stats.tx_dropped, ndev->stats.rx_errors,
			    ndev->stats.rx_dropped);
		netdev_info(ndev,
			    "rx_cdp[0..5]=%08x %08x %08x %08x %08x %08x base[0]=%08x head[0]=%u\n",
			    rtl_rd(p, CPURPDCR(0)), rtl_rd(p, CPURPDCR(1)),
			    rtl_rd(p, CPURPDCR(2)), rtl_rd(p, CPURPDCR(3)),
			    rtl_rd(p, CPURPDCR(4)), rtl_rd(p, CPURPDCR(5)),
			    (u32)p->rx_ring_dma[0], p->rx_head[0]);
	} else {
		return -EINVAL;
	}
	return len;
}
static DEVICE_ATTR_WO(dma);

/*
 * read-only packet-path probe (segments 3+4). CORRECT on-SoC switch MIB map from
 * SDK rtl865xc_asicregs.h: MIB_COUNTER_BASE=SWCORE+0x1000, per-port stride 0x80,
 * IN counters at +0x100 (ifInOctets +0x00, ifInUcastPkts +0x08,
 * etherStatsBroadcastPkts +0x40, etherStatsDropEvents +0x48), OUT at +0x800
 * (ifOutOctets +0x00). The old SW_MIB_RXPKT(0x1084) map was a GLOBAL counter
 * (RX_PKTS_1), which is why it read 0 even under known TX. Do NOT write
 * MIB_CONTROL(0x1000) here -- 0x7ffff = ALL_COUNTER_RESTART (zeros counters).
 * cat before and after the ARP burst; delta computed off-board.
 */
static ssize_t probe_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rtl_eth *p = netdev_priv(ndev);
	int len = 0, port, i;

	len += sysfs_emit_at(buf, len,
		"[SEG3 on-SoC RTL8197F switch MIB base=0x1b801000 stride=0x80]\n");
	for (port = 0; port <= 8; port++) {
		u32 in = 0x1100 + port * 0x80;	/* IN block  */
		u32 out = 0x1800 + port * 0x80;	/* OUT block */

		len += sysfs_emit_at(buf, len,
			"P%d iOct=%u iUc=%u iBc=%u iDrop=%u oOct=%u | FCS=%u Sym=%u rxDV=%u\n", port,
			readl(p->swcore + in + 0x00),	/* ifInOctets */
			readl(p->swcore + in + 0x08),	/* ifInUcastPkts */
			readl(p->swcore + in + 0x40),	/* etherStatsBroadcastPkts */
			readl(p->swcore + in + 0x48),	/* etherStatsDropEvents */
			readl(p->swcore + out + 0x00),	/* ifOutOctets */
			readl(p->swcore + in + 0x4c),	/* dot3StatsFCSErrors */
			readl(p->swcore + in + 0x50),	/* dot3StatsSymbolErrors */
			readl(p->swcore + in + 0x5c));	/* inRxDvcnt (RX_DV assertions) */
	}
	len += sysfs_emit_at(buf, len,
		"[SEG4 swNic] CPUICR=0x%08x RPDCR0=0x%08x IISR=0x%08x rx=%lu rxerr=%lu dma_on=%d rx_opts4=0x%08x rx_opts5=0x%08x spa=%u\n",
		rtl_rd(p, CPUICR), rtl_rd(p, CPURPDCR(0)), rtl_rd(p, CPUIISR),
		ndev->stats.rx_packets, ndev->stats.rx_errors, p->dma_on,
		p->rx_last_opts4, p->rx_last_opts5, p->rx_last_spa);
	len += sysfs_emit_at(buf, len,
		"napi polls=%lu budget=%lu rxr=%lu/%lu/%lu/%lu/%lu/%lu tx_stop=%lu tx_wake=%lu rx_csum=%lu/%lu tx_csum=%lu/%lu\n",
		p->napi_polls, p->napi_budget_exhausted,
		p->rx_ring_packets[0], p->rx_ring_packets[1],
		p->rx_ring_packets[2], p->rx_ring_packets[3],
		p->rx_ring_packets[4], p->rx_ring_packets[5],
		p->tx_stops, p->tx_wakes, p->rx_csum_good,
		p->rx_csum_none, p->tx_csum_good, p->tx_csum_help);
	len += sysfs_emit_at(buf, len,
		"flowtable bind=%lu unbind=%lu replace=%lu destroy=%lu stats=%lu parsed=%lu rejected=%lu\n",
		p->ft_binds, p->ft_unbinds, p->ft_replace,
		p->ft_destroy, p->ft_stats, p->ft_parsed, p->ft_rejected);
	len += sysfs_emit_at(buf, len, "rxd0 OWN:");
	for (i = 0; i < 8; i++)
		len += sysfs_emit_at(buf, len, " %d:%d(%08x)", i,
			!!(p->rx_ring[0][i].opts1 & D_OWN), p->rx_ring[0][i].opts1);
	len += sysfs_emit_at(buf, len, "\n");
	len += sysfs_emit_at(buf, len,
			     "TPDCR0=0x%08x TPDCR1=0x%08x txd OWN:",
			     rtl_rd(p, CPUTPDCR(0)), rtl_rd(p, CPUTPDCR(1)));
	for (i = 0; i < 4; i++)
		len += sysfs_emit_at(buf, len, " %d:%d(%08x)", i,
			!!(p->tx_ring[i].opts1 & D_OWN), p->tx_ring[i].opts1);
	len += sysfs_emit_at(buf, len, "\n");
	/* on-SoC switch port config/status (SDK: PCRP=0x4104+n*4, PSRP=0x4128+n*4,
	 * P0GMIICR=0x414c, P5GMIICR=0x4150). PSRP bit4=LinkUp, [1:0]=speed. */
	len += sysfs_emit_at(buf, len, "[SEG5 on-SoC ports] P0GMIICR=0x%08x P5GMIICR=0x%08x\n",
		readl(p->swcore + 0x414c), readl(p->swcore + 0x4150));
	for (i = 0; i <= 7; i++)
		len += sysfs_emit_at(buf, len, "P%d PCRP=0x%08x PSRP=0x%08x link=%d spd=%d\n", i,
			readl(p->swcore + 0x4104 + i * 4),
			readl(p->swcore + 0x4128 + i * 4),
			!!(readl(p->swcore + 0x4128 + i * 4) & 0x10),
			readl(p->swcore + 0x4128 + i * 4) & 3);
	return len;
}
static DEVICE_ATTR_RO(probe);

/*
 * A-only diagnostic: apply the stock port0 RGMII/CPU-tag bring-up (fn 0x802281b0)
 * verbatim RMW sequence, in exact stock order, dumping the 6 key regs before/after.
 * Reboot-recoverable. `echo go > applyA`. Forwarding tables are NOT touched here.
 */
static void rtl_apply_a(struct rtl_eth *p)
{
	void __iomem *sw = p->swcore, *sys = p->sysreg;
	u32 v;

	pr_info("### applyA BEFORE MACCR(4000)=0x%08x MACCR1(4058)=0x%08x PITCR(4100)=0x%08x PCRP0(4104)=0x%08x P0GMIICR(414c)=0x%08x SYS850=0x%08x\n",
		readl(sw + 0x4000), readl(sw + 0x4058), readl(sw + 0x4100),
		readl(sw + 0x4104), readl(sw + 0x414c), readl(sys + 0x850));

	/* 1. PCRP0 force-link 1000full (two-step RMW) */
	v = readl(sw + 0x4104); writel((v & 0x83ffdff6) | 0x14002009, sw + 0x4104);
	v = readl(sw + 0x4104); writel((v & 0xfd03ffff) | 0x02940000, sw + 0x4104);
	/* 2. P0GMIICR mode/timing (clear GMAC, set RCOMP/TCOMP/RGTXC) */
	v = readl(sw + 0x414c); writel(v & 0xfe7fffff, sw + 0x414c);
	v = readl(sw + 0x414c); writel((v & 0xfff3ffe8) | 0x000c0005, sw + 0x414c);
	/* 3. Match the vendor RTL8197F+RTL8367R path: translate between the
	 * RTL8367 native 4-byte CPU tag and swNic descriptor port metadata.
	 */
	v = readl(sw + SW_P0GMIICR);
	writel(v | P0GMIICR_CPU_TAG | P0GMIICR_TX_CPU_TAG,
	       sw + SW_P0GMIICR);
	/* 4. MACCR1 PORT0_ROUTER_MODE (bit0) */
	v = readl(sw + 0x4058); writel(v | 0x1, sw + 0x4058);
	/* 5. PITCR Port0 external interface GMII/MII/RGMII (bit0) */
	v = readl(sw + 0x4100); writel(v | 0x1, sw + 0x4100);
	/* 6. MACCR bit12 (idempotent) */
	v = readl(sw + 0x4000); writel(v | 0x1000, sw + 0x4000);
	/* 7. P0GMIICR Conf_done (bit6) -- LAST, after all port0 config */
	v = readl(sw + 0x414c); writel(v | 0x40, sw + 0x414c);
	/* 8. SYS 0x850 pinmux/RGMII */
	v = readl(sys + 0x850); writel((v & 0x01bfffff) | 0xd8000000, sys + 0x850);

	pr_info("### applyA AFTER  MACCR(4000)=0x%08x MACCR1(4058)=0x%08x PITCR(4100)=0x%08x PCRP0(4104)=0x%08x P0GMIICR(414c)=0x%08x SYS850=0x%08x\n",
		readl(sw + 0x4000), readl(sw + 0x4058), readl(sw + 0x4100),
		readl(sw + 0x4104), readl(sw + 0x414c), readl(sys + 0x850));
}

static ssize_t applyA_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t len)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rtl_eth *p = netdev_priv(ndev);

	if (strncmp(buf, "go", 2))
		return -EINVAL;

	rtl_apply_a(p);
	return len;
}
static DEVICE_ATTR_WO(applyA);

/* indirect ASIC table access (SWTAA=0x4d08 addr, data 0x4d20+, SWTACR=0x4d00
 * cmd; CMD_FORCE|ACTION_START=0x09 to write, 0x01 to read; poll ACTION_DONE=bit0;
 * SWTCR0=0x4418 STOP_TLU=bit18 held during programming). */
static int rtl_tbl_wait(void __iomem *sw)
{
	int i;

	for (i = 0; i < 2000 && (readl(sw + SW_TBL_ACC_CONTROL) & SW_TBL_ACTION); i++)
		udelay(1);

	return (readl(sw + SW_TBL_ACC_CONTROL) & SW_TBL_ACTION) ? -ETIMEDOUT : 0;
}

static int rtl_tbl_wr(void __iomem *sw, u32 swtaa, const u32 *d, int n)
{
	int i;

	for (i = 0; i < n; i++)
		writel(d[i], sw + SW_TBL_ACC_DATA(i));
	writel(swtaa, sw + SW_TBL_ACC_ADDRESS);
	writel(0x09, sw + SW_TBL_ACC_CONTROL);

	return rtl_tbl_wait(sw);
}

static int rtl_tbl_rd(void __iomem *tbl, u32 swtaa, u32 *d, int n)
{
	int i;
	for (i = 0; i < n; i++)
		d[i] = readl(tbl + (swtaa & 0xfffff) + i * 4);

	return 0;
}

static int rtl_tlu_stop(void __iomem *sw, u32 *saved)
{
	int i;

	*saved = readl(sw + SW_TABLE_CONTROL_0);
	writel(*saved | SW_TLU_STOP, sw + SW_TABLE_CONTROL_0);
	for (i = 0; i < 2000 &&
	     !(readl(sw + SW_TABLE_CONTROL_0) & SW_TLU_STOPPED); i++)
		udelay(1);

	return (readl(sw + SW_TABLE_CONTROL_0) & SW_TLU_STOPPED) ? 0 : -ETIMEDOUT;
}

static void rtl_tlu_start(void __iomem *sw, u32 saved)
{
	writel(saved, sw + SW_TABLE_CONTROL_0);
}

static void rtl_pack_netif(u32 *entry, u16 vid, const u8 *mac, bool route)
{
	u32 mac47_19, mac18_0;
	u16 mtu = 1536;

	memset(entry, 0, sizeof(u32) * 8);
	mac47_19 = ((u32)mac[0] << 21) | ((u32)mac[1] << 13) |
		    ((u32)mac[2] << 5) | ((u32)mac[3] >> 3);
	mac18_0 = ((u32)(mac[3] & 7) << 16) | ((u32)mac[4] << 8) | mac[5];
	entry[0] = 1 | ((u32)vid << 1) | (mac18_0 << 13);
	entry[1] = (mac47_19 & 0x1fffffff) | (route ? BIT(29) : 0);
	entry[2] = BIT(31);
	entry[3] = 3 | ((u32)mtu << 2) | ((u32)mtu << 17);
}

static u16 rtl_hwnat_hash(bool tcp, u32 src_addr, u16 src_port,
			  u32 dst_addr, u16 dst_port, bool verify)
{
	u8 proto = tcp | (verify ? 2 : 0);

	if (!dst_addr && !dst_port && !(proto & 2))
		return (src_addr & 0x3ff) ^ ((src_addr >> 10) & 0x3ff) ^
		       ((src_addr >> 20) & 0x3ff) ^ (src_port & 0x3ff) ^
		       (((proto & 1) << 8) | ((src_addr >> 24) & 0xc0) |
			((src_port >> 10) & 0x3f));

	return (src_port & 0x3ff) ^
	       (((src_port & 0xfc00) >> 10) | ((src_addr & 0xf) << 6)) ^
	       ((src_addr >> 4) & 0x3ff) ^ ((src_addr >> 14) & 0x3ff) ^
	       (((src_addr >> 24) & 0xff) | ((proto & 1) << 8) |
		((dst_port & 1) << 9)) ^ ((dst_port >> 1) & 0x3ff) ^
	       (((dst_port >> 11) & 0x1f) | ((dst_addr & 0x1f) << 5)) ^
	       ((dst_addr >> 5) & 0x3ff) ^ ((dst_addr >> 15) & 0x3ff) ^
	       ((dst_addr >> 25) & 0x7f);
}

static void rtl_hwnat_pack_l2(u32 *entry, const u8 *mac, u8 port, u8 fid)
{
	memset(entry, 0, sizeof(u32) * 8);
	entry[0] = ((u32)mac[1] << 24) | ((u32)mac[2] << 16) |
		   ((u32)mac[3] << 8) | mac[4];
	entry[1] = mac[0] | ((u32)BIT(port) << 8) | BIT(18) |
		   (3u << 19) | BIT(22) | ((u32)fid << 23);
}

static void rtl_hwnat_pack_cpu_route(u32 *entry, __be32 addr)
{
	memset(entry, 0, sizeof(u32) * 8);
	entry[0] = ntohl(addr);
	entry[1] = 31 | BIT(5) | (4u << 6) | BIT(9);
}

static void rtl_hwnat_pack_napt(u32 *entry, u32 int_ip, u16 int_port,
				u8 offset, u8 sel_ip, u16 sel_e,
				bool tcp, bool outbound)
{
	memset(entry, 0, sizeof(u32) * 8);
	entry[0] = int_ip;
	entry[1] = BIT(0) | BIT(1) | (30u << 2) | ((u32)offset << 8) |
		   BIT(14) | BIT(16) | ((u32)sel_ip << 17) |
		   ((u32)sel_e << 21);
	entry[2] = int_port | ((outbound ? 3u : 2u) << 16) |
		   ((u32)tcp << 19);
}

static bool rtl_hsb_ipv4_match(const u32 *hsb, u32 sip, u32 dip)
{
	u32 got_sip = hsb[1] >> 2 | (hsb[2] & 3) << 30;
	u32 got_dip = hsb[2] >> 18 | (hsb[3] & 0x3ffff) << 14;

	return got_sip == sip && got_dip == dip;
}

static bool rtl_hwnat_selftest(void)
{
	u32 entry[8], hsb[4] = {};

	if (rtl_hwnat_hash(true, 0xc0a80164, 12345, 0x01010101,
			    443, false) != 692 ||
	    rtl_hwnat_hash(true, 0x01010101, 443, 0xcb007105,
			    40000, false) != 912 ||
	    rtl_hwnat_hash(true, 0x01010101, 443, 0, 0, true) != 238)
		return false;
	rtl_hwnat_pack_napt(entry, 0xc0a80164, 12345, 0x10, 0, 0x9c,
			      true, true);
	if (entry[0] != 0xc0a80164 || entry[1] != 0x1381507b ||
	    entry[2] != 0x000b3039)
		return false;
	rtl_hwnat_pack_cpu_route(entry, (__force __be32)0x0101a8c0);
	if (entry[0] != 0xc0a80101 || entry[1] != 0x0000033f)
		return false;
	hsb[1] = (0xc0a80102 & 0x3fffffff) << 2;
	hsb[2] = (0xc0a80102 >> 30) | ((0x01010101 & 0x3fff) << 18);
	hsb[3] = 0x01010101 >> 14;
	return rtl_hsb_ipv4_match(hsb, 0xc0a80102, 0x01010101);
}

static struct rtl_hwnat_saved *
rtl_hwnat_saved_find(struct rtl_eth *p, u8 type, u16 index)
{
	unsigned int i;

	for (i = 0; i < p->hwnat_saved_count; i++)
		if (p->hwnat_saved[i].used && p->hwnat_saved[i].type == type &&
		    p->hwnat_saved[i].index == index)
			return &p->hwnat_saved[i];
	return NULL;
}

/* Caller holds tbl_lock and has stopped the TLU. */
static int rtl_hwnat_base_write(struct rtl_eth *p, u8 type, u16 index,
				const u32 *entry)
{
	struct rtl_hwnat_saved *saved;
	u32 check[8];
	int err;

	saved = rtl_hwnat_saved_find(p, type, index);
	if (!saved) {
		if (p->hwnat_saved_count >= RTL_HWNAT_MAX_SAVED)
			return -ENOSPC;
		saved = &p->hwnat_saved[p->hwnat_saved_count++];
		saved->used = true;
		saved->type = type;
		saved->index = index;
		rtl_tbl_rd(p->tbl, RTL_TBL_ADDR(type, index), saved->data, 8);
	}

	err = rtl_tbl_wr(p->swcore, RTL_TBL_ADDR(type, index), entry, 8);
	if (err)
		return err;
	rtl_tbl_rd(p->tbl, RTL_TBL_ADDR(type, index), check, 8);
	return memcmp(check, entry, sizeof(check)) ? -EIO : 0;
}

static void rtl_hwnat_base_restore(struct rtl_eth *p)
{
	while (p->hwnat_saved_count) {
		struct rtl_hwnat_saved *saved =
			&p->hwnat_saved[--p->hwnat_saved_count];

		rtl_tbl_wr(p->swcore, RTL_TBL_ADDR(saved->type, saved->index),
			   saved->data, 8);
		memset(saved, 0, sizeof(*saved));
	}
}

static bool rtl_hwnat_l2_matches(const u32 *entry, const u8 *mac, u8 fid)
{
	return entry[0] == (((u32)mac[1] << 24) | ((u32)mac[2] << 16) |
			    ((u32)mac[3] << 8) | mac[4]) &&
	       (entry[1] & 0xff) == mac[0] && ((entry[1] >> 23) & 3) == fid;
}

static int rtl_hwnat_l2_add(struct rtl_eth *p, const u8 *mac, u8 port, u8 fid)
{
	static const u8 fid_hash[] = { 0x00, 0x0f, 0xf0, 0xff };
	u32 entry[8], old[8];
	u8 row = mac[0] ^ mac[1] ^ mac[2] ^ mac[3] ^ mac[4] ^ mac[5] ^
		 fid_hash[fid];
	int empty = -1, col, index, err;

	rtl_hwnat_pack_l2(entry, mac, port, fid);
	for (col = 0; col < 4; col++) {
		index = (row << 2) | col;
		rtl_tbl_rd(p->tbl, RTL_TBL_ADDR(RTL_TBL_L2, index), old, 8);
		if (rtl_hwnat_l2_matches(old, mac, fid))
			return rtl_hwnat_base_write(p, RTL_TBL_L2, index, entry) ?:
			       index;
		if (empty < 0 && !memchr_inv(old, 0, sizeof(old)))
			empty = index;
	}
	if (empty < 0)
		return -ENOSPC;
	err = rtl_hwnat_base_write(p, RTL_TBL_L2, empty, entry);
	return err ?: empty;
}

static int rtl_hwnat_base_start(struct rtl_eth *p,
				const struct rtl_ft_rule *out,
				const struct rtl_ft_rule *in)
{
	u32 netif[8], route[8] = {}, next_hop[8] = {}, extip[8] = {};
	u32 swtcr0, lan_net;
	int gateway, err;

	lan_net = ntohl(out->orig_src_addr) & 0xffffff00;
	if (!out->ingress_local_addr)
		return -EADDRNOTAVAIL;
	if (p->hwnat_active)
		return p->hwnat_lan_net == lan_net &&
		       p->hwnat_lan_ip == out->ingress_local_addr &&
		       p->hwnat_ext_ip == out->src_addr &&
		       ether_addr_equal(p->hwnat_wan_mac, out->eth.h_source) &&
		       ether_addr_equal(p->hwnat_gateway_mac, out->eth.h_dest) &&
		       ether_addr_equal(p->hwnat_lan_mac, in->eth.h_source) ? 0 :
		       -EOPNOTSUPP;

	err = rtl_hwnat_prepare(p);
	if (err)
		return err;
	err = rtl_tlu_stop(p->swcore, &swtcr0);
	if (err)
		return err;

	p->hwnat_saved_count = 0;
	p->hwnat_saved_swtcr0 = swtcr0;
	p->hwnat_saved_swtcr1 = readl(p->swcore + SW_TABLE_CONTROL_1);
	p->hwnat_saved_cscr = readl(p->swcore + SW_CHECKSUM_CONTROL);
	p->hwnat_saved_pvcr[0] = readl(p->swcore + SW_PORT_VLAN_CONTROL_0);
	p->hwnat_saved_pvcr[1] = readl(p->swcore + SW_PORT_VLAN_CONTROL_0 + 4);
	p->hwnat_saved_pvcr[2] = readl(p->swcore + SW_PORT_VLAN_CONTROL_0 + 8);
	p->hwnat_saved_pvcr[3] = readl(p->swcore + SW_PORT_VLAN_CONTROL_0 + 16);
	gateway = rtl_hwnat_l2_add(p, out->eth.h_dest, RTL_NPORTS - 1, 1);
	if (gateway < 0) {
		err = gateway;
		goto rollback;
	}

	rtl_pack_netif(netif, RTL_LAN_VID, in->eth.h_source, true);
	err = rtl_hwnat_base_write(p, RTL_TBL_NETIF, 0, netif);
	if (err)
		goto rollback;
	rtl_pack_netif(netif, RTL_WAN_VID, out->eth.h_source, true);
	err = rtl_hwnat_base_write(p, RTL_TBL_NETIF, 1, netif);
	if (err)
		goto rollback;

	/* Routing entries are priority ordered, longest prefix first. Keep the
	 * router's own LAN address on the CPU before accelerating the subnet. */
	rtl_hwnat_pack_cpu_route(route, out->ingress_local_addr);
	err = rtl_hwnat_base_write(p, RTL_TBL_ROUTE, 0, route);
	if (err)
		goto rollback;
	memset(route, 0, sizeof(route));
	route[0] = lan_net;
	route[1] = 23 | BIT(5) | (2u << 6) | BIT(9) | (31u << 20);
	err = rtl_hwnat_base_write(p, RTL_TBL_ROUTE, 1, route);
	if (err)
		goto rollback;
	memset(route, 0, sizeof(route));
	route[1] = BIT(5) | (5u << 6);
	err = rtl_hwnat_base_write(p, RTL_TBL_ROUTE, 7, route);
	if (err)
		goto rollback;

	next_hop[0] = (1u << 5) | ((u32)gateway << 11);
	err = rtl_hwnat_base_write(p, RTL_TBL_NEXTHOP, 0, next_hop);
	if (!err)
		err = rtl_hwnat_base_write(p, RTL_TBL_NEXTHOP, 1, next_hop);
	if (err)
		goto rollback;

	extip[1] = ntohl(out->src_addr);
	extip[2] = BIT(0);
	err = rtl_hwnat_base_write(p, RTL_TBL_EXTIP, 0, extip);
	if (err)
		goto rollback;

	writel((p->hwnat_saved_swtcr1 | SW_ENHANCED_HASH1) &
	       ~SW_L4_4WAY_HASH, p->swcore + SW_TABLE_CONTROL_1);
	writel(p->hwnat_saved_cscr | SW_L3_CHECKSUM_RECALC |
	       SW_L4_CHECKSUM_RECALC, p->swcore + SW_CHECKSUM_CONTROL);
	p->hwnat_lan_net = lan_net;
	p->hwnat_lan_ip = out->ingress_local_addr;
	p->hwnat_ext_ip = out->src_addr;
	ether_addr_copy(p->hwnat_wan_mac, out->eth.h_source);
	ether_addr_copy(p->hwnat_gateway_mac, out->eth.h_dest);
	ether_addr_copy(p->hwnat_lan_mac, in->eth.h_source);
	writel((p->hwnat_saved_pvcr[0] & ~0x0fff0fffu) |
	       RTL_LAN_VID | (RTL_LAN_VID << 16),
	       p->swcore + SW_PORT_VLAN_CONTROL_0);
	writel((p->hwnat_saved_pvcr[1] & ~0x0fff0fffu) |
	       RTL_LAN_VID | (RTL_LAN_VID << 16),
	       p->swcore + SW_PORT_VLAN_CONTROL_0 + 4);
	writel((p->hwnat_saved_pvcr[2] & ~0xfffu) | RTL_WAN_VID,
	       p->swcore + SW_PORT_VLAN_CONTROL_0 + 8);
	writel((p->hwnat_saved_pvcr[3] & ~0xfffu) | RTL_LAN_VID,
	       p->swcore + SW_PORT_VLAN_CONTROL_0 + 16);
	p->hwnat_active = true;
	rtl_tlu_start(p->swcore,
		      swtcr0 & ~(SW_LAN_DECISION_MASK << SW_LAN_DECISION_SH));
	return 0;

rollback:
	rtl_hwnat_base_restore(p);
	writel(p->hwnat_saved_swtcr1, p->swcore + SW_TABLE_CONTROL_1);
	writel(p->hwnat_saved_cscr, p->swcore + SW_CHECKSUM_CONTROL);
	writel(p->hwnat_saved_pvcr[0], p->swcore + SW_PORT_VLAN_CONTROL_0);
	writel(p->hwnat_saved_pvcr[1], p->swcore + SW_PORT_VLAN_CONTROL_0 + 4);
	writel(p->hwnat_saved_pvcr[2], p->swcore + SW_PORT_VLAN_CONTROL_0 + 8);
	writel(p->hwnat_saved_pvcr[3], p->swcore + SW_PORT_VLAN_CONTROL_0 + 16);
	rtl_tlu_start(p->swcore, swtcr0);
	return err;
}

/* Caller holds tbl_lock and has stopped the TLU. */
static u32 rtl_hwnat_base_stop(struct rtl_eth *p)
{
	writel(readl(p->swcore + SW_MODULE_SW_CONTROL) &
	       ~(SW_L3_ENGINE_ENABLE | SW_L4_ENGINE_ENABLE),
	       p->swcore + SW_MODULE_SW_CONTROL);
	rtl_hwnat_base_restore(p);
	writel(p->hwnat_saved_swtcr1, p->swcore + SW_TABLE_CONTROL_1);
	writel(p->hwnat_saved_cscr, p->swcore + SW_CHECKSUM_CONTROL);
	writel(p->hwnat_saved_pvcr[0], p->swcore + SW_PORT_VLAN_CONTROL_0);
	writel(p->hwnat_saved_pvcr[1], p->swcore + SW_PORT_VLAN_CONTROL_0 + 4);
	writel(p->hwnat_saved_pvcr[2], p->swcore + SW_PORT_VLAN_CONTROL_0 + 8);
	writel(p->hwnat_saved_pvcr[3], p->swcore + SW_PORT_VLAN_CONTROL_0 + 16);
	p->hwnat_active = false;
	p->hwnat_flow_count = 0;
	return p->hwnat_saved_swtcr0;
}

static bool rtl_hwnat_is_outbound(const struct rtl_ft_rule *r)
{
	return r->ingress_port < RTL_NPORTS - 1 &&
	       r->egress_port == RTL_NPORTS - 1 &&
	       (r->orig_src_addr != r->src_addr ||
		r->orig_src_port != r->src_port) &&
	       r->orig_dst_addr == r->dst_addr &&
	       r->orig_dst_port == r->dst_port;
}

static bool rtl_hwnat_is_reply(const struct rtl_ft_rule *r)
{
	return r->ingress_port == RTL_NPORTS - 1 &&
	       r->egress_port < RTL_NPORTS - 1 &&
	       r->orig_src_addr == r->src_addr &&
	       r->orig_src_port == r->src_port &&
	       (r->orig_dst_addr != r->dst_addr ||
		r->orig_dst_port != r->dst_port);
}

static bool rtl_hwnat_pair(const struct rtl_ft_rule *out,
			   const struct rtl_ft_rule *in)
{
	return out->ip_proto == in->ip_proto &&
	       out->orig_dst_addr == in->orig_src_addr &&
	       out->orig_dst_port == in->orig_src_port &&
	       out->src_addr == in->orig_dst_addr &&
	       out->src_port == in->orig_dst_port &&
	       out->orig_src_addr == in->dst_addr &&
	       out->orig_src_port == in->dst_port &&
	       out->orig_dst_addr == in->src_addr &&
	       out->orig_dst_port == in->src_port;
}

static int rtl_hwnat_flow_install(struct rtl_eth *p, unsigned long out_cookie,
				  unsigned long in_cookie,
				  const struct rtl_ft_rule *out,
				  const struct rtl_ft_rule *in)
{
	struct rtl_hwnat_flow *flow = NULL;
	u32 out_entry[8], in_entry[8], check[8], arp[8] = {}, swtcr0;
	u32 int_ip = ntohl(out->orig_src_addr);
	u32 rem_ip = ntohl(out->orig_dst_addr);
	u32 ext_ip = ntohl(out->src_addr);
	u16 int_port = ntohs(out->orig_src_port);
	u16 rem_port = ntohs(out->orig_dst_port);
	u16 ext_port = ntohs(out->src_port);
	bool tcp = out->ip_proto == IPPROTO_TCP;
	u16 out_index, in_index, verify;
	unsigned int i, arp_index;
	bool wrote_out = false, wrote_in = false;
	int client, err;

	if (!int_port || !rem_port || !ext_port)
		return -EOPNOTSUPP;
	for (i = 0; i < RTL_HWNAT_MAX_FLOWS; i++)
		if (!p->hwnat_flow[i].used) {
			flow = &p->hwnat_flow[i];
			break;
		}
	if (!flow)
		return -ENOSPC;

	err = rtl_hwnat_base_start(p, out, in);
	if (err)
		return err;
	out_index = rtl_hwnat_hash(tcp, int_ip, int_port, rem_ip, rem_port,
				     false);
	in_index = rtl_hwnat_hash(tcp, rem_ip, rem_port, ext_ip, ext_port,
				    false);
	verify = rtl_hwnat_hash(tcp, rem_ip, rem_port, 0, 0, true);
	if (out_index == in_index) {
		err = -EADDRINUSE;
		goto stop_empty_base;
	}

	err = rtl_tlu_stop(p->swcore, &swtcr0);
	if (err)
		goto stop_empty_base;
	client = rtl_hwnat_l2_add(p, in->eth.h_dest, in->egress_port, 0);
	if (client < 0) {
		err = client;
		goto out_start;
	}
	arp_index = ntohl(out->orig_src_addr) & 0xff;
	arp[0] = BIT(0) | ((u32)client << 1) | (31u << 11);
	err = rtl_hwnat_base_write(p, RTL_TBL_ARP, arp_index, arp);
	if (err)
		goto out_start;

	rtl_tbl_rd(p->tbl, RTL_TBL_ADDR(RTL_TBL_NAPT, out_index),
		   flow->saved_out, 8);
	rtl_tbl_rd(p->tbl, RTL_TBL_ADDR(RTL_TBL_NAPT, in_index),
		   flow->saved_in, 8);
	if (memchr_inv(flow->saved_out, 0, sizeof(flow->saved_out)) ||
	    memchr_inv(flow->saved_in, 0, sizeof(flow->saved_in))) {
		err = -EADDRINUSE;
		goto out_start;
	}
	rtl_hwnat_pack_napt(out_entry, int_ip, int_port, ext_port >> 10, 0,
			      ext_port & 0x3ff, tcp, true);
	rtl_hwnat_pack_napt(in_entry, int_ip, int_port, ext_port & 0x3f,
			      (ext_port & 0x3ff) >> 6, verify, tcp, false);
	err = rtl_tbl_wr(p->swcore, RTL_TBL_ADDR(RTL_TBL_NAPT, out_index),
			 out_entry, 8);
	if (err)
		goto out_start;
	wrote_out = true;
	err = rtl_tbl_wr(p->swcore, RTL_TBL_ADDR(RTL_TBL_NAPT, in_index),
			 in_entry, 8);
	if (err)
		goto rollback;
	wrote_in = true;
	rtl_tbl_rd(p->tbl, RTL_TBL_ADDR(RTL_TBL_NAPT, out_index), check, 8);
	if (memcmp(check, out_entry, sizeof(check))) {
		err = -EIO;
		goto rollback;
	}
	rtl_tbl_rd(p->tbl, RTL_TBL_ADDR(RTL_TBL_NAPT, in_index), check, 8);
	if (memcmp(check, in_entry, sizeof(check))) {
		err = -EIO;
		goto rollback;
	}

	flow->used = true;
	flow->cookie[0] = out_cookie;
	flow->cookie[1] = in_cookie;
	flow->out_index = out_index;
	flow->in_index = in_index;
	p->hwnat_flow_count++;
	writel(readl(p->swcore + SW_MODULE_SW_CONTROL) |
	       SW_L2_ENGINE_ENABLE | SW_L3_ENGINE_ENABLE | SW_L4_ENGINE_ENABLE,
	       p->swcore + SW_MODULE_SW_CONTROL);
	rtl_tlu_start(p->swcore, swtcr0);
	return 0;

rollback:
	if (wrote_in)
		rtl_tbl_wr(p->swcore, RTL_TBL_ADDR(RTL_TBL_NAPT, in_index),
			   flow->saved_in, 8);
	if (wrote_out)
		rtl_tbl_wr(p->swcore, RTL_TBL_ADDR(RTL_TBL_NAPT, out_index),
			   flow->saved_out, 8);
out_start:
	if (!p->hwnat_flow_count)
		swtcr0 = rtl_hwnat_base_stop(p);
	rtl_tlu_start(p->swcore, swtcr0);
	return err;

stop_empty_base:
	if (!p->hwnat_flow_count && p->hwnat_active &&
	    !rtl_tlu_stop(p->swcore, &swtcr0)) {
		swtcr0 = rtl_hwnat_base_stop(p);
		rtl_tlu_start(p->swcore, swtcr0);
	}
	return err;
}

static int rtl_hwnat_replace(struct rtl_eth *p, unsigned long cookie,
			     const struct rtl_ft_rule *rule)
{
	struct rtl_hwnat_pending *free = NULL;
	unsigned int i;
	int err = -EOPNOTSUPP;

	mutex_lock(&p->tbl_lock);
	for (i = 0; i < RTL_HWNAT_MAX_PENDING; i++) {
		struct rtl_hwnat_pending *pending = &p->hwnat_pending[i];

		if (pending->used && time_after(jiffies, pending->expires))
			pending->used = false;
		if (!pending->used && !free)
			free = pending;
	}

	if (rtl_hwnat_is_outbound(rule)) {
		if (!free) {
			err = -ENOSPC;
			goto out;
		}
		free->used = true;
		free->cookie = cookie;
		free->expires = jiffies + RTL_HWNAT_PENDING_TTL;
		free->rule = *rule;
		goto out;
	}
	if (!rtl_hwnat_is_reply(rule))
		goto out;

	for (i = 0; i < RTL_HWNAT_MAX_PENDING; i++) {
		struct rtl_hwnat_pending *pending = &p->hwnat_pending[i];

		if (!pending->used || !rtl_hwnat_pair(&pending->rule, rule))
			continue;
		err = rtl_hwnat_flow_install(p, pending->cookie, cookie,
					      &pending->rule, rule);
		pending->used = false;
		break;
	}
out:
	mutex_unlock(&p->tbl_lock);
	return err;
}

static int rtl_hwnat_destroy(struct rtl_eth *p, unsigned long cookie)
{
	u32 swtcr0;
	unsigned int i;
	int ret = -EOPNOTSUPP;

	mutex_lock(&p->tbl_lock);
	for (i = 0; i < RTL_HWNAT_MAX_PENDING; i++)
		if (p->hwnat_pending[i].used &&
		    p->hwnat_pending[i].cookie == cookie) {
			p->hwnat_pending[i].used = false;
			ret = 0;
		}
	for (i = 0; i < RTL_HWNAT_MAX_FLOWS; i++) {
		struct rtl_hwnat_flow *flow = &p->hwnat_flow[i];

		if (!flow->used || (flow->cookie[0] != cookie &&
				   flow->cookie[1] != cookie))
			continue;
		if (rtl_tlu_stop(p->swcore, &swtcr0)) {
			ret = -ETIMEDOUT;
			break;
		}
		rtl_tbl_wr(p->swcore, RTL_TBL_ADDR(RTL_TBL_NAPT,
						     flow->out_index),
			   flow->saved_out, 8);
		rtl_tbl_wr(p->swcore, RTL_TBL_ADDR(RTL_TBL_NAPT,
						     flow->in_index),
			   flow->saved_in, 8);
		flow->used = false;
		p->hwnat_flow_count--;
		if (!p->hwnat_flow_count)
			swtcr0 = rtl_hwnat_base_stop(p);
		rtl_tlu_start(p->swcore, swtcr0);
		ret = 0;
		break;
	}
	mutex_unlock(&p->tbl_lock);
	return ret;
}

static int rtl_hwnat_stats(struct rtl_eth *p, struct flow_cls_offload *cls)
{
	u32 entry[8];
	unsigned int i;
	int ret = -EOPNOTSUPP;

	mutex_lock(&p->tbl_lock);
	for (i = 0; i < RTL_HWNAT_MAX_FLOWS; i++) {
		struct rtl_hwnat_flow *flow = &p->hwnat_flow[i];

		if (!flow->used || (flow->cookie[0] != cls->cookie &&
				   flow->cookie[1] != cls->cookie))
			continue;
		rtl_tbl_rd(p->tbl, RTL_TBL_ADDR(RTL_TBL_NAPT, flow->out_index),
			   entry, 8);
		if ((entry[1] & BIT(0)) && ((entry[1] >> 2) & 0x3f))
			flow_stats_update(&cls->stats, 0, 0, 0, jiffies,
					  FLOW_ACTION_HW_STATS_DELAYED);
		ret = 0;
		break;
	}
	mutex_unlock(&p->tbl_lock);
	return ret;
}

/*
 * applyFwd: VLAN 1 (member=Port0+CPU Ext2, both untagged), Port0 PVID=1,
 * netif idx0 valid (vid=1, MAC=eth0, enHWRoute=0).  RTL8197F maps the logical
 * CPU port to hardware extension-port bit 8; without it, VLAN-classified ingress
 * cannot reach swNic.  Full readback. `echo go`.
 */
static void rtl_apply_fwd(struct rtl_eth *p)
{
	void __iomem *sw = p->swcore;
	const u8 *mac = p->ndev->dev_addr;
	u32 vlan[5] = { 0 }, netif[8] = { 0 };
	u32 swtcr0;
	void __iomem *tbl;
	u16 mtu = 1536;
	int err;

	/* member=BIT0|BIT8 (0x101), egressUntag=(BIT0|BIT8)<<9 (0x20200) */
	vlan[0] = 0x00020301;

	/* netif idx0: valid, vid=1, MAC=eth0, enHWRoute=0. */
	rtl_pack_netif(netif, 1, mac, false);

	/* STOP_TLU and WAIT for STOP_TLU_READY (bit19) before the first table access
	 * -- this was the bug: without the wait the first (VLAN) write was dropped. */
	mutex_lock(&p->tbl_lock);
	err = rtl_tlu_stop(sw, &swtcr0);
	if (err)
		goto out_unlock;
	err = rtl_tbl_wr(sw, RTL_TBL_ADDR(RTL_TBL_VLAN, 1), vlan, 3);
	if (!err)
		err = rtl_tbl_wr(sw, RTL_TBL_ADDR(RTL_TBL_NETIF, 0), netif, 5);
	rtl_tlu_start(sw, swtcr0);
out_unlock:
	mutex_unlock(&p->tbl_lock);
	if (err) {
		dev_err(p->dev, "applyFwd table access failed: %d\n", err);
		return;
	}

	/* Port0 PVID=1 (PVCR0 bits[11:0], port0 even) */
	writel((readl(sw + 0x4a08) & ~0xfffu) | 1u, sw + 0x4a08);
	/* Match the proven SDK-style ingress state: no per-port VLAN ingress
	 * filtering, and send multicast/broadcast traffic to the CPU path. */
	writel(0, sw + SW_VLAN_CONTROL_0);
	writel(readl(sw + SW_FRAME_FWD_CFG) | SW_MULTICAST_TO_CPU,
	       sw + SW_FRAME_FWD_CFG);

	/* ---- readback: tables are memory-mapped at SWTABLE 0x1b000000 (direct read) ---- */
	pr_info("### applyFwd MAC=%pM mtu=%u\n", mac, mtu);
	pr_info("### applyFwd WROTE vlan0=0x%08x netif=[0x%08x 0x%08x 0x%08x 0x%08x]\n",
		vlan[0], netif[0], netif[1], netif[2], netif[3]);
	tbl = ioremap(0x1b000000, 0x100000);
	if (tbl) {
		pr_info("### applyFwd RB VLAN vid1 w0=0x%08x (expect 0x20301)\n",
			readl(tbl + 0x60020));
		pr_info("### applyFwd RB NETIF idx0 = 0x%08x 0x%08x 0x%08x 0x%08x\n",
			readl(tbl + 0x40000), readl(tbl + 0x40004),
			readl(tbl + 0x40008), readl(tbl + 0x4000c));
		iounmap(tbl);
	}
	pr_info("### applyFwd PVCR0(4a08)=0x%08x FFCR(4428)=0x%08x SWTCR0(4418)=0x%08x PLITIMR(4420)=0x%08x\n",
		readl(sw + 0x4a08), readl(sw + 0x4428), readl(sw + 0x4418), readl(sw + 0x4420));
}

static ssize_t applyFwd_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rtl_eth *p = netdev_priv(ndev);

	if (strncmp(buf, "go", 2))
		return -EINVAL;

	rtl_apply_fwd(p);
	return len;
}
static DEVICE_ATTR_WO(applyFwd);

static int rtl_hwnat_prepare(struct rtl_eth *p)
{
	static const u32 vlan[2][8] = {
		{ 0x00060310 }, /* WAN: logical port4, untag on internal port0, FID1 */
		{ 0x00021f0f }, /* LAN: logical ports0..3 + CPU extension, FID0 */
	};
	u32 netif[8], check[8], swtcr0;
	int err;

	if (p->hwnat_prepared)
		return 0;

	/* Preparation is deliberately inert: enHWRoute and L3/L4 stay off. */
	rtl_pack_netif(netif, RTL_WAN_VID, p->ndev->dev_addr, false);
	err = rtl_tlu_stop(p->swcore, &swtcr0);
	if (err)
		return err;
	err = rtl_tbl_rd(p->tbl, RTL_TBL_ADDR(RTL_TBL_VLAN, RTL_WAN_VID),
			 p->hwnat_saved_vlan[0], 8);
	if (!err)
		err = rtl_tbl_rd(p->tbl,
				 RTL_TBL_ADDR(RTL_TBL_VLAN, RTL_LAN_VID),
				 p->hwnat_saved_vlan[1], 8);
	if (!err)
		err = rtl_tbl_rd(p->tbl, RTL_TBL_ADDR(RTL_TBL_NETIF, 1),
				 p->hwnat_saved_netif, 8);
	if (!err)
		err = rtl_tbl_wr(p->swcore,
				 RTL_TBL_ADDR(RTL_TBL_VLAN, RTL_WAN_VID), vlan[0], 8);
	if (!err)
		err = rtl_tbl_wr(p->swcore,
				 RTL_TBL_ADDR(RTL_TBL_VLAN, RTL_LAN_VID), vlan[1], 8);
	if (!err)
		err = rtl_tbl_wr(p->swcore, RTL_TBL_ADDR(RTL_TBL_NETIF, 1),
				 netif, 8);
	if (!err)
		err = rtl_tbl_rd(p->tbl,
				 RTL_TBL_ADDR(RTL_TBL_VLAN, RTL_WAN_VID), check, 8);
	if (!err && memcmp(check, vlan[0], sizeof(check)))
		err = -EIO;
	if (!err)
		err = rtl_tbl_rd(p->tbl,
				 RTL_TBL_ADDR(RTL_TBL_VLAN, RTL_LAN_VID), check, 8);
	if (!err && memcmp(check, vlan[1], sizeof(check)))
		err = -EIO;
	if (!err)
		err = rtl_tbl_rd(p->tbl, RTL_TBL_ADDR(RTL_TBL_NETIF, 1),
				 check, 8);
	if (!err && memcmp(check, netif, sizeof(check)))
		err = -EIO;
	if (!err)
		err = rtl_hwnat_ext_prepare(p);
	if (err) {
		rtl_hwnat_ext_restore(p);
		rtl_tbl_wr(p->swcore, RTL_TBL_ADDR(RTL_TBL_VLAN, RTL_WAN_VID),
			   p->hwnat_saved_vlan[0], 8);
		rtl_tbl_wr(p->swcore, RTL_TBL_ADDR(RTL_TBL_VLAN, RTL_LAN_VID),
			   p->hwnat_saved_vlan[1], 8);
		rtl_tbl_wr(p->swcore, RTL_TBL_ADDR(RTL_TBL_NETIF, 1),
			   p->hwnat_saved_netif, 8);
	} else {
		p->hwnat_prepared = true;
	}
	rtl_tlu_start(p->swcore, swtcr0);

	return err;
}

static void rtl_hwnat_off(struct rtl_eth *p)
{
	u32 swtcr0;
	unsigned int i;

	writel(readl(p->swcore + SW_MODULE_SW_CONTROL) &
	       ~(SW_L3_ENGINE_ENABLE | SW_L4_ENGINE_ENABLE),
	       p->swcore + SW_MODULE_SW_CONTROL);
	if ((!p->hwnat_prepared && !p->hwnat_active) ||
	    rtl_tlu_stop(p->swcore, &swtcr0))
		return;
	for (i = 0; i < RTL_HWNAT_MAX_FLOWS; i++) {
		struct rtl_hwnat_flow *flow = &p->hwnat_flow[i];

		if (!flow->used)
			continue;
		rtl_tbl_wr(p->swcore, RTL_TBL_ADDR(RTL_TBL_NAPT,
						     flow->out_index),
			   flow->saved_out, 8);
		rtl_tbl_wr(p->swcore, RTL_TBL_ADDR(RTL_TBL_NAPT,
						     flow->in_index),
			   flow->saved_in, 8);
		memset(flow, 0, sizeof(*flow));
	}
	if (p->hwnat_active)
		swtcr0 = rtl_hwnat_base_stop(p);
	rtl_tbl_wr(p->swcore, RTL_TBL_ADDR(RTL_TBL_VLAN, RTL_WAN_VID),
		   p->hwnat_saved_vlan[0], 8);
	rtl_tbl_wr(p->swcore, RTL_TBL_ADDR(RTL_TBL_VLAN, RTL_LAN_VID),
		   p->hwnat_saved_vlan[1], 8);
	rtl_tbl_wr(p->swcore, RTL_TBL_ADDR(RTL_TBL_NETIF, 1),
		   p->hwnat_saved_netif, 8);
	rtl_tlu_start(p->swcore, swtcr0);
	rtl_hwnat_ext_restore(p);
	p->hwnat_prepared = false;
	memset(p->hwnat_pending, 0, sizeof(p->hwnat_pending));
}

static ssize_t hwnat_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rtl_eth *p = netdev_priv(ndev);
	u32 mscr = readl(p->swcore + SW_MODULE_SW_CONTROL);

	return sysfs_emit(buf,
			  "prepared=%u active=%u flows=%u l2=%u l3=%u l4=%u\n",
			  p->hwnat_prepared, p->hwnat_active,
			  p->hwnat_flow_count, !!(mscr & SW_L2_ENGINE_ENABLE),
			  !!(mscr & SW_L3_ENGINE_ENABLE),
			  !!(mscr & SW_L4_ENGINE_ENABLE));
}

static ssize_t hwnat_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t len)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rtl_eth *p = netdev_priv(ndev);
	int err = 0;

	mutex_lock(&p->tbl_lock);
	if (sysfs_streq(buf, "prepare"))
		err = rtl_hwnat_prepare(p);
	else if (sysfs_streq(buf, "off"))
		rtl_hwnat_off(p);
	else
		err = -EINVAL;
	mutex_unlock(&p->tbl_lock);

	return err ?: len;
}
static DEVICE_ATTR_RW(hwnat);

static ssize_t hswatch_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rtl_eth *p = netdev_priv(ndev);
	ssize_t len;
	int i;

	len = sysfs_emit(buf,
		"state=%u sip=%08x dip=%08x hwfwrd=%u dp=%02x why=%04x cputag=%u rmdp=%02x\nHSB:",
		p->hsb_watch_state, p->hsb_watch_sip, p->hsb_watch_dip,
		!!(p->hsa_watch[6] & BIT(8)), p->hsa_watch[8] & 0x7f,
		(p->hsa_watch[6] >> 18) | ((p->hsa_watch[7] & 3) << 14),
		p->hsa_watch[10] & 1, (p->hsa_watch[10] >> 8) & 0x3f);
	for (i = 0; i < RTL_HS_WORDS; i++)
		len += sysfs_emit_at(buf, len, " %08x", p->hsb_watch[i]);
	len += sysfs_emit_at(buf, len, "\nHSA:");
	for (i = 0; i < RTL_HS_WORDS; i++)
		len += sysfs_emit_at(buf, len, " %08x", p->hsa_watch[i]);
	return len + sysfs_emit_at(buf, len, "\n");
}

static ssize_t hswatch_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rtl_eth *p = netdev_priv(ndev);
	unsigned long deadline = jiffies + 10 * HZ;
	unsigned long flags;
	unsigned long spins = 0;
	u32 hsb[4];
	u32 hsb8;
	int i;

	if (sscanf(buf, "%x %x", &p->hsb_watch_sip, &p->hsb_watch_dip) != 2)
		return -EINVAL;
	p->hsb_watch_state = 0;
	memset(p->hsb_watch, 0, sizeof(p->hsb_watch));
	memset(p->hsa_watch, 0, sizeof(p->hsa_watch));
	do {
		for (i = 0; i < 4; i++)
			hsb[i] = readl(p->swcore + 0x6280 + i * 4);
		hsb8 = readl(p->swcore + 0x62a0);
		if ((p->hsb_watch_sip || p->hsb_watch_dip) ?
		    (!(hsb8 & BIT(24)) &&
		     rtl_hsb_ipv4_match(hsb, p->hsb_watch_sip,
					p->hsb_watch_dip)) :
		    (!(hsb8 & BIT(24)) && !(hsb[0] & 7) &&
		     (hsb[1] || hsb[2] || hsb[3]))) {
			local_irq_save(flags);
			for (i = 0; i < 100 &&
			     (readl(p->swcore + 0x6300) & BIT(2)); i++)
				cpu_relax();
			p->hsa_watch[6] = readl(p->swcore + 0x6218);
			p->hsa_watch[8] = readl(p->swcore + 0x6220);
			p->hsa_watch[10] = readl(p->swcore + 0x6228);
			for (i = 0; i < RTL_HS_WORDS; i++) {
				p->hsb_watch[i] = i < 4 ? hsb[i] : i == 8 ? hsb8 :
					readl(p->swcore + 0x6280 + i * 4);
				if (i != 6 && i != 8 && i != 10)
					p->hsa_watch[i] =
						readl(p->swcore + 0x6200 + i * 4);
			}
			local_irq_restore(flags);
			p->hsb_watch_state = 1;
			return len;
		}
		cpu_relax();
		if (!(++spins & 0xfff))
			cond_resched();
	} while (time_before(jiffies, deadline));
	p->hsb_watch_state = 2;
	return len;
}
static DEVICE_ATTR_RW(hswatch);

static ssize_t txcap_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rtl_eth *p = netdev_priv(ndev);
	unsigned int off, len, seq;
	ssize_t pos = 0;

	spin_lock_bh(&p->tx_lock);
	len = p->txcap_len;
	seq = p->txcap_seq;
	pos += sysfs_emit_at(buf, pos, "seq=%u len=%u\n", seq, len);
	for (off = 0; off < len; off += 16) {
		unsigned int n = min_t(unsigned int, 16, len - off);

		pos += sysfs_emit_at(buf, pos, "%04x: %*ph\n",
				     off, n, p->txcap + off);
	}
	spin_unlock_bh(&p->tx_lock);

	return pos;
}
static DEVICE_ATTR_RO(txcap);

static int rtl_eth_probe(struct platform_device *pdev)
{
	struct net_device *ndev;
	struct rtl_eth *p;
	int ret;
	unsigned int i;

	ndev = devm_alloc_etherdev(&pdev->dev, sizeof(*p));
	if (!ndev)
		return -ENOMEM;

	SET_NETDEV_DEV(ndev, &pdev->dev);
	platform_set_drvdata(pdev, ndev);

	p = netdev_priv(ndev);
	p->ndev = ndev;
	p->dev = &pdev->dev;
	/* the zeroed priv would read as IRQ 0; stay polled until one is claimed */
	p->irq = -1;
	spin_lock_init(&p->tx_lock);
	mutex_init(&p->tbl_lock);
	if (!rtl_hwnat_selftest())
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "hardware NAT table self-test failed\n");

	/* External RTL8367 SMI pins, discovered empirically (sweep): pin18=portC.2
	 * = SCK, pin19=portC.3 = SDA.  gpioId = port<<8 | pin. */
	p->smi_sck = (2 << 8) | 2;
	p->smi_sda = (2 << 8) | 3;

	p->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(p->base))
		return PTR_ERR(p->base);

	/* Fixed SoC blocks needed to start the engine: the system register
	 * block (switch/NIC core clock gate) and the switch core (SIRR). */
	p->sysreg = devm_ioremap(&pdev->dev, SYSREG_BASE_PHYS, SYSREG_SIZE);
	p->swcore = devm_ioremap(&pdev->dev, SWCORE_BASE_PHYS, SWCORE_SIZE);
	p->tbl = devm_ioremap(&pdev->dev, 0x1b000000, 0x100000);
	if (!p->sysreg || !p->swcore || !p->tbl)
		return -ENOMEM;

	rtl_regdump("ETH-PROBE (before any eth switch access)", p->swcore, p->sysreg);

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	for (i = 0; i < NUM_RX_RING; i++) {
		p->rx_ring[i] = dmam_alloc_coherent(&pdev->dev,
					 rtl_rx_ring_size(i) * sizeof(struct rtl_desc),
					 &p->rx_ring_dma[i], GFP_KERNEL);
		if (!p->rx_ring[i])
			return -ENOMEM;
	}
	p->tx_ring = dmam_alloc_coherent(&pdev->dev,
					 RTL_RING_SIZE * sizeof(struct rtl_desc),
					 &p->tx_ring_dma, GFP_KERNEL);
	if (!p->tx_ring)
		return -ENOMEM;
	p->tx_ring1 = dmam_alloc_coherent(&pdev->dev,
					  RTL_TX_RING1_SIZE * sizeof(struct rtl_desc),
					  &p->tx_ring1_dma, GFP_KERNEL);
	if (!p->tx_ring1)
		return -ENOMEM;

	ret = of_get_ethdev_address(pdev->dev.of_node, ndev);
	if (ret == -EPROBE_DEFER)
		return ret;
	if (ret)
		eth_hw_addr_random(ndev);

	ndev->netdev_ops = &rtl_netdev_ops;
	ndev->hw_features |= NETIF_F_RXCSUM | NETIF_F_IP_CSUM |
			     NETIF_F_IPV6_CSUM | NETIF_F_HW_TC;
	ndev->features |= NETIF_F_RXCSUM | NETIF_F_IP_CSUM |
			  NETIF_F_IPV6_CSUM | NETIF_F_HW_TC;
	ndev->vlan_features |= NETIF_F_RXCSUM | NETIF_F_IP_CSUM |
			       NETIF_F_IPV6_CSUM;
	/* Allow the extra headroom DSA needs for its CPU tag (rtl8365mb uses an
	 * 8-byte RTL8_4 tag): as the DSA conduit we must accept MTU 1508. The
	 * 2 KB RX buffers already hold it. */
	ndev->max_mtu = RTL_BUF_SIZE - ETH_HLEN - ETH_FCS_LEN;
	netif_napi_add(ndev, &p->napi, rtl_rx_poll);

	/*
	 * The switch core is wired straight to a CPU IP (see the ictl comment
	 * near ICTL_GIMR), so this is a plain cpuintc interrupt. Claim it here
	 * but leave it masked at the ictl until rtl_dma_enable() arms the rings;
	 * if the DT carries no interrupt we fall back to the old polled timer.
	 */
	p->irq = platform_get_irq_optional(pdev, 0);
	if (p->irq >= 0) {
		ret = devm_request_irq(&pdev->dev, p->irq, rtl_isr, 0,
				       dev_name(&pdev->dev), p);
		if (ret) {
			dev_warn(&pdev->dev,
				 "IRQ %d unavailable (%d), falling back to polling\n",
				 p->irq, ret);
			p->irq = -1;
		}
	}

	ret = devm_register_netdev(&pdev->dev, ndev);
	if (ret)
		return ret;

	/* on-demand PHY status + interactive MDIO poke files */
	device_create_file(&pdev->dev, &dev_attr_phy);
	device_create_file(&pdev->dev, &dev_attr_mdio);
	device_create_file(&pdev->dev, &dev_attr_reg);
	device_create_file(&pdev->dev, &dev_attr_smi);
	device_create_file(&pdev->dev, &dev_attr_dma);
	device_create_file(&pdev->dev, &dev_attr_probe);
	device_create_file(&pdev->dev, &dev_attr_applyA);
	device_create_file(&pdev->dev, &dev_attr_applyFwd);
	device_create_file(&pdev->dev, &dev_attr_hwnat);
	device_create_file(&pdev->dev, &dev_attr_hswatch);
	device_create_file(&pdev->dev, &dev_attr_txcap);

	dev_info(&pdev->dev, "Realtek RTL8197F CPU NIC at %pR (%s)\n",
		 platform_get_resource(pdev, IORESOURCE_MEM, 0),
		 p->irq >= 0 ? "irq" : "polled");
	return 0;
}

static const struct of_device_id rtl_eth_of_match[] = {
	{ .compatible = "realtek,rtl8197f-eth" },
	{ }
};
MODULE_DEVICE_TABLE(of, rtl_eth_of_match);

static struct platform_driver rtl_eth_driver = {
	.probe = rtl_eth_probe,
	.driver = {
		.name = "rtl8197f-eth",
		.of_match_table = rtl_eth_of_match,
	},
};
module_platform_driver(rtl_eth_driver);

MODULE_DESCRIPTION("Realtek RTL8197F CPU NIC (swNic) driver");
MODULE_LICENSE("GPL");
