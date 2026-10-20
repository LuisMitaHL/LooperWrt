// SPDX-License-Identifier: GPL-2.0-only
/*
 * RTL8197F RW407 firmware-container loader.
 *
 * The stock stage-2 boot code copies the cvimg body to 0x85000000 and jumps
 * here.  The body holds a small header, the compressed kernel split around the
 * fixed rootfs at flash 0x260000, and the real rootfs.  This loader reassembles
 * the two kernel fragments in RAM and LZMA-decompresses them to 0x80000000.
 */

#include "LzmaDecode.h"
#include "loader-dbg.h"

#define LOADER_BASE	0x85000000
#define HEADER_OFF	0x10000
#define SCRATCH_ADDR	0x86000000
#define RAMSTART	0x80000000
#define RAMSIZE		0x08000000

#define KSEG0		0x80000000

#define Index_Invalidate_I	0x00
#define Index_Writeback_Inv_D	0x01

#define cache_unroll(base,op)	\
	__asm__ __volatile__(	\
		".set noreorder;\n"	\
		".set mips3;\n"	\
		"cache %1, (%0);\n"	\
		".set mips0;\n"	\
		".set reorder\n"	\
		:			\
		: "r" (base),		\
		  "i" (op));

struct rw407_header {
	unsigned int magic;		/* 'KNWR' */
	unsigned int version;
	unsigned int header_size;
	unsigned int kernel_a_off;	/* body-relative */
	unsigned int kernel_a_len;
	unsigned int kernel_b_off;	/* body-relative */
	unsigned int kernel_b_len;
	unsigned int kernel_load;
	unsigned int kernel_entry;
	unsigned int rootfs_off;	/* body-relative; 0x22fff0 for 0x260000 */
	unsigned int rootfs_len;	/* squashfs bytes_used */
	unsigned int rootfs_data_off;	/* body-relative */
	unsigned int body_len;		/* data bytes before cvimg checksum */
	unsigned int reserved[3];
};

struct rw407_header header_placeholder __attribute__((section(".header")));

static unsigned char *data;
extern char __bss_start[], __bss_end[];

static void mem_copy(void *dst, const void *src, unsigned int len)
{
	unsigned char *d = dst;
	const unsigned char *s = src;
	while (len--)
		*d++ = *s++;
}

static void zero_bss(void)
{
	unsigned char *p;
	for (p = (unsigned char *)__bss_start; p < (unsigned char *)__bss_end; p++)
		*p = 0;
}

static void blast_icache(unsigned long size, unsigned long lsize)
{
	unsigned long start = KSEG0;
	unsigned long end = start + size;
	while (start < end) {
		cache_unroll(start, Index_Invalidate_I);
		start += lsize;
	}
}

static void blast_dcache(unsigned long size, unsigned long lsize)
{
	unsigned long start = KSEG0;
	unsigned long end = start + size;
	while (start < end) {
		cache_unroll(start, Index_Writeback_Inv_D);
		start += lsize;
	}
}

static int read_byte(void *object, const unsigned char **buffer, UInt32 *bufferSize)
{
	*bufferSize = 1;
	*buffer = data;
	++data;
	return LZMA_RESULT_OK;
}

static unsigned char get_byte(void)
{
	const unsigned char *buffer;
	UInt32 fake;
	read_byte(0, &buffer, &fake);
	return *buffer;
}

static void fail(const char *msg)
{
	dbg_puts("rw407 loader: ");
	dbg_puts(msg);
	dbg_puts("\n");
	for (;;)
		;
}

void entry(unsigned long icache_size, unsigned long icache_lsize,
	   unsigned long dcache_size, unsigned long dcache_lsize)
{
	struct rw407_header *h = (struct rw407_header *)(LOADER_BASE + HEADER_OFF);
	unsigned char *scratch = (unsigned char *)SCRATCH_ADDR;
	unsigned int total, osize, i;
	volatile unsigned int arg0, arg1, arg2, arg3;
	ILzmaInCallback callback;
	CLzmaDecoderState vs;

	zero_bss();

	dbg_puts("rw407 loader: start\n");

	__asm__ __volatile__ ("ori %0, $12, 0" : "=r"(arg0));
	__asm__ __volatile__ ("ori %0, $13, 0" : "=r"(arg1));
	__asm__ __volatile__ ("ori %0, $14, 0" : "=r"(arg2));
	__asm__ __volatile__ ("ori %0, $15, 0" : "=r"(arg3));

	if (h->magic != 0x52574e4b || h->version != 1)
		fail("bad header");

	total = h->kernel_a_len + h->kernel_b_len;
	if (!h->kernel_a_len || total > 0x02000000 || h->kernel_load < 0x80000000 ||
	    h->kernel_load > 0x87f00000 || h->kernel_entry < 0x80000000 ||
	    h->kernel_entry > 0x87f00000)
		fail("bad kernel geometry");

	if ((unsigned int)scratch + total > (RAMSTART + RAMSIZE - 0x20000))
		fail("kernel too large");

	mem_copy(scratch, (void *)(LOADER_BASE + h->kernel_a_off), h->kernel_a_len);
	if (h->kernel_b_len)
		mem_copy(scratch + h->kernel_a_len,
			 (void *)(LOADER_BASE + h->kernel_b_off), h->kernel_b_len);

	dbg_puts("rw407 loader: kernel A=");
	dbg_puthex(h->kernel_a_len);
	dbg_puts(" B=");
	dbg_puthex(h->kernel_b_len);
	dbg_puts(" load=");
	dbg_puthex(h->kernel_load);
	dbg_puts(" entry=");
	dbg_puthex(h->kernel_entry);
	dbg_puts("\n");

	data = scratch;
	callback.Read = read_byte;

	i = get_byte();
	vs.Properties.lc = i % 9;
	i = i / 9;
	vs.Properties.lp = i % 5;
	vs.Properties.pb = i / 5;
	vs.Probs = (CProb *)(RAMSTART + RAMSIZE - 0x00020000);

	for (i = 0; i < 4; i++)
		get_byte();

	osize = ((unsigned int)get_byte()) +
		((unsigned int)get_byte() << 8) +
		((unsigned int)get_byte() << 16) +
		((unsigned int)get_byte() << 24);
	for (i = 0; i < 4; i++)
		get_byte();

	if (LzmaDecode(&vs, &callback, (unsigned char *)h->kernel_load,
		       osize, &osize) != LZMA_RESULT_OK)
		fail("lzma failed");

	blast_dcache(dcache_size, dcache_lsize);
	blast_icache(icache_size, icache_lsize);

	dbg_puts("rw407 loader: jump\n");
	((void (*)(int, int, int, int))h->kernel_entry)(arg0, arg1, arg2, arg3);

	fail("returned");
}
