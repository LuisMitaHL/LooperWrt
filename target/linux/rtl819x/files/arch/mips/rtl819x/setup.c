// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek RTL819x-class platform hook scaffold.
 *
 * These stubs only satisfy mandatory MIPS platform link hooks while the
 * RTL8197F Stage 0A/0B boot path is being forward-ported.
 */

#include <linux/init.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/memblock.h>
#include <linux/of.h>
#include <linux/of_clk.h>
#include <linux/sizes.h>
#include <linux/string.h>
#include <linux/types.h>

#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/page.h>
#include <asm/prom.h>
#include <asm/reboot.h>
#include <asm/thread_info.h>
#include <asm/irq.h>
#include <asm/time.h>

#define RTL8197F_STAGE0_CMDLINE	"console=ttyS0,38400"
#define RTL8197F_GIMR		0xb8003000
#define RTL8197F_GIMR2		0xb8003020
#define RTL8197F_WDTCNR		0xb800311c

/*
 * RTL8197F UART (KSEG1 0xb8147000): LSR at +0x14 (THRE = 0x20), and the RX/TX
 * data register at +0x24 -- not the standard 16550 +0x00. This provides
 * CONFIG_EARLY_PRINTK output from setup_early_printk() onward, independent of
 * the DT-matched earlycon, so an early boot hang is visible on UART.
 */
void prom_putchar(char c)
{
	volatile u32 *lsr = (volatile u32 *)0xb8147014;
	volatile u32 *thr = (volatile u32 *)0xb8147024;

	while (!(*lsr & 0x20))
		;
	*thr = (u8)c;
}

const char *get_system_type(void)
{
	return "Realtek RTL8197F";
}

static void rtl8197f_restart(char *command)
{
	*(volatile u32 *)RTL8197F_GIMR = 0;
	*(volatile u32 *)RTL8197F_GIMR2 = 0;
	local_irq_disable();
	*(volatile u32 *)RTL8197F_WDTCNR = 0;

	for (;;)
		;
}

void __init prom_init(void)
{
	/* TODO: parse bootloader arguments once the firmware ABI is defined. */
	strscpy(arcs_cmdline, RTL8197F_STAGE0_CMDLINE, COMMAND_LINE_SIZE);
}

void __init plat_mem_setup(void)
{
	void *dtb;

	_machine_restart = rtl8197f_restart;

	/*
	 * Load the appended device tree. Without this the kernel runs with NO
	 * live DT: of_find_compatible_node()/of_irq_init() find nothing, so the
	 * cpuintc (mti,cpu-interrupt-controller) domain is never created and the
	 * timer IRQ has no chip. get_fdt() returns &__appended_dtb for
	 * CONFIG_MIPS_RAW_APPENDED_DTB; __dt_setup_arch() sets initial_boot_params
	 * (and, with CMDLINE_FROM_DTB, the kernel cmdline), and the MIPS core
	 * arch_mem_init()->device_tree_init() then unflattens it.
	 */
	dtb = get_fdt();
	if (dtb)
		__dt_setup_arch(dtb);

	/* DT has no memory node yet; keep the fixed 64 MB until RAM is modelled. */
	memblock_add(0, SZ_64M);
}

void __init plat_time_init(void)
{
	/*
	 * Register the device-tree fixed-clocks (spic_clk, uart_clk, ...). The
	 * generic of_platform populate does not turn the /clocks child nodes into
	 * platform devices, and nothing else calls of_clk_init here, so without
	 * this the SPI controller (clocks = <&spic_clk>) probe-defers forever and
	 * the boot flash / rootfs never appear.
	 */
	of_clk_init(NULL);

	/*
	 * CP0 Count/Compare timer frequency. On MIPS 24Kc the Count register runs
	 * at half the CPU clock (bootloader reports 999 MHz). TODO: derive from a
	 * DT CPU clock once modelled.
	 */
	mips_hpt_frequency = 999000000 / 2;

	/*
	 * On the RTL8197F the CP0 compare timer's SI_TimerInt is NOT delivered on a
	 * CP0 IP directly; it is fed into the SoC interrupt controller (ictl at
	 * 0x18003000) as GISR2 bit 15 (CPU_SI_TIMER). The Realtek BSP routes that
	 * source to CPU IP7 via the ictl routing register IRR5 and enables it in
	 * GIMR2, then handles it as the r4k compare IRQ on IP7 (BSP_COMPARE_IRQ =
	 * CPU_BASE+7). Without this the compare interrupt never reaches the CPU and
	 * the boot hangs in calibrate_delay(). Program the ictl the same way (KSEG1
	 * regs), then point the r4k clockevent at IP7. (IntCtl.IPTI reads 2 here and
	 * is read-only / irrelevant to this routing.)
	 */
	{
		volatile u32 *gimr2 = (volatile u32 *)0xb8003020;
		volatile u32 *irr5  = (volatile u32 *)0xb800302c;

		*irr5 = 0x70000000;		/* CPU_SI_TIMER (src47) -> CPU IP7 */
		*gimr2 |= (1u << 15);		/* enable CPU_SI_TIMER */
	}
	cp0_compare_irq = 7;
	cp0_perfcount_irq = -1;
}

void __init arch_init_irq(void)
{
	/* TODO: confirm RTL8197F ICTL IRR0..7 routing before boot use. */
	irqchip_init();
}

/*
 * The MIPS CPU interrupt controller (mti,cpu-interrupt-controller) domain maps
 * hwirq N -> virq N (legacy, base 0). Hand the r4k clockevent the virq for the
 * CP0 compare hwirq (cp0_compare_irq, forced to IP7 above) via that domain so it
 * carries the mips_cpu_irq_controller chip.
 */
unsigned int get_c0_compare_int(void)
{
	struct device_node *np;
	struct irq_domain *d;

	np = of_find_compatible_node(NULL, NULL, "mti,cpu-interrupt-controller");
	d = np ? irq_find_host(np) : NULL;
	of_node_put(np);

	if (d)
		return irq_create_mapping(d, cp0_compare_irq);

	return MIPS_CPU_IRQ_BASE + cp0_compare_irq;
}
