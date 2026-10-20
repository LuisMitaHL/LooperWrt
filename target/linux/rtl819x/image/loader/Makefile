LOADADDR ?= 0x85000000
GENERIC ?= $(TOPDIR)/target/linux/generic/image/lzma-loader/src
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy

CFLAGS := -march=mips32r2 -mno-abicalls -fno-pic -fno-builtin -G0 \
	-msoft-float -Os -nostdlib -Wall -Wno-unused-function \
	-D_LZMA_IN_CB -DLOADER_BASE=$(LOADADDR) -DRAMSTART=0x80000000 -DRAMSIZE=0x08000000 \
	-I. -I.. -I$(GENERIC)

all: loader.bin

start.o: $(GENERIC)/start.S
	$(CC) $(CFLAGS) -c $< -o $@

loader.o: loader.c
	$(CC) $(CFLAGS) -c $< -o $@

LzmaDecode.o: $(GENERIC)/LzmaDecode.c
	$(CC) $(CFLAGS) -I$(GENERIC) -c $< -o $@

loader.elf: start.o loader.o LzmaDecode.o loader.lds
	$(LD) -T loader.lds -o $@ start.o loader.o LzmaDecode.o

loader.bin: loader.elf
	$(OBJCOPY) -O binary -R .reginfo -R .MIPS.abiflags -R .note -R .comment \
		-R .mdebug -R .pdr -R .gnu.attributes -S $< $@

clean:
	rm -f *.o loader.elf loader.bin
