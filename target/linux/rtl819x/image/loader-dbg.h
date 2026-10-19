/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Minimal UART progress output for the A2004MU first-stage loader.
 *
 * The generic lzma-loader is silent, so a boot that stops after the bootloader's
 * "Jump to image start=0x80a00000..." line gives no way to tell a slow
 * decompression apart from a hang. These milestones mirror the stock loader's
 * "decompressing kernel:" / "done" markers.
 *
 * Register layout was recovered from the stock first-stage loader's own putc
 * (disassembled from the stock image, RAM 0x80a00e40..0x80a00e80):
 *
 *     lui  a1, 0xb814 ; ori a1, a1, 0x7014     ; a1 = LSR  = 0xb8147014
 *   1:lbu  a2, 0(a1)  ; andi a2, a2, 0x20      ; THRE bit  = 0x20
 *     beq  a2, zero, 1b                        ; wait for THRE
 *     lui  v0, 0xb814 ; ori v0, v0, 0x7024     ; v0 = THR  = 0xb8147024
 *     sb   a0, 0(v0)                           ; write char
 *
 * Note THR is at base+0x24, NOT the standard 16550 base+0x00. The bootloader
 * has already configured the UART, so this only polls THRE and writes THR --
 * no re-init, no clock maths. Accesses are byte-wide to match the stock loader.
 */
#ifndef A2004MU_LOADER_DBG_H
#define A2004MU_LOADER_DBG_H

#define DBG_UART_LSR (*(volatile unsigned char *)0xb8147014u)
#define DBG_UART_THR (*(volatile unsigned char *)0xb8147024u)
#define DBG_UART_LSR_THRE 0x20u

static inline void dbg_putc(char c)
{
	while ((DBG_UART_LSR & DBG_UART_LSR_THRE) == 0)
		;
	DBG_UART_THR = (unsigned char)c;
}

static inline void dbg_puts(const char *s)
{
	while (*s) {
		if (*s == '\n')
			dbg_putc('\r');
		dbg_putc(*s++);
	}
}

static inline void dbg_puthex(unsigned int v)
{
	static const char hex[] = "0123456789abcdef";
	int i;

	dbg_putc('0');
	dbg_putc('x');
	for (i = 28; i >= 0; i -= 4)
		dbg_putc(hex[(v >> i) & 0xf]);
}

/*
 * Report the divisor the bootloader already programmed for a clean 38400 baud.
 * uart_clock = divisor * 16 * 38400. LCR is saved and restored around the DLAB
 * toggle so subsequent prints stay clean.
 */
static inline void dbg_report_uart_divisor(void)
{
	volatile unsigned char *lcr = (volatile unsigned char *)0xb814700cu;
	volatile unsigned char *dll = (volatile unsigned char *)0xb8147000u;
	volatile unsigned char *dlm = (volatile unsigned char *)0xb8147004u;
	unsigned char saved = *lcr;
	unsigned int lo, hi;

	*lcr = saved | 0x80;
	lo = *dll;
	hi = *dlm;
	*lcr = saved;

	dbg_puts("loader: uart divisor=");
	dbg_puthex((hi << 8) | lo);
	dbg_puts(" (clk = divisor*614400)\n");
}

/*
 * Dump the UART register block at both DLAB states. The stock loader transmits
 * via offset 0x24, not the standard 16550 0x00, so the map is non-standard and
 * needs to be seen directly. All registers are captured first, then DLAB is
 * restored to 0, then the captured values are printed -- so the prints (which
 * go through TX at 0x24) are never disturbed by the DLAB toggle.
 */
static inline void dbg_dump_uart(void)
{
	volatile unsigned char *base = (volatile unsigned char *)0xb8147000u;
	unsigned char saved = base[0x0c];
	unsigned char d0[11], d1[11];
	int i;
	static const unsigned char off[11] = {
		0x00, 0x04, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x1c, 0x20, 0x24, 0x28
	};

	base[0x0c] = saved & ~0x80u;		/* DLAB = 0 */
	for (i = 0; i < 11; i++)
		d0[i] = base[off[i]];
	base[0x0c] = saved | 0x80u;		/* DLAB = 1 */
	for (i = 0; i < 11; i++)
		d1[i] = base[off[i]];
	base[0x0c] = saved;			/* restore */

	dbg_puts("loader: uart regs dlab0:");
	for (i = 0; i < 11; i++) {
		dbg_putc(' ');
		dbg_putc("0123456789abcdef"[(off[i] >> 4) & 0xf]);
		dbg_putc("0123456789abcdef"[off[i] & 0xf]);
		dbg_putc('=');
		dbg_putc("0123456789abcdef"[(d0[i] >> 4) & 0xf]);
		dbg_putc("0123456789abcdef"[d0[i] & 0xf]);
	}
	dbg_puts("\nloader: uart regs dlab1:");
	for (i = 0; i < 11; i++) {
		dbg_putc(' ');
		dbg_putc("0123456789abcdef"[(off[i] >> 4) & 0xf]);
		dbg_putc("0123456789abcdef"[off[i] & 0xf]);
		dbg_putc('=');
		dbg_putc("0123456789abcdef"[(d1[i] >> 4) & 0xf]);
		dbg_putc("0123456789abcdef"[d1[i] & 0xf]);
	}
	dbg_putc('\n');
}

#endif /* A2004MU_LOADER_DBG_H */
