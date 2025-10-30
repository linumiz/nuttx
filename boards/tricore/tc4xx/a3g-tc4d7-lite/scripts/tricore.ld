#include <nuttx/config.h>

OUTPUT_FORMAT("elf32-tricore")
OUTPUT_ARCH("tricore")
ENTRY(__start)
EXTERN(__start)

__HEAP_SIZE = (5 * 1024 * 1024);
CONFIG_TRICORE_CSA_COUNT = 512;


MEMORY
{
	dsram0 (w!xp): ORIGIN = 0x70000000, LENGTH = 240K
	psram0 (w!xp): ORIGIN = 0x70100000, LENGTH = 64K
	lmu (wr!xp): ORIGIN = 0x90400000, LENGTH = 5M

	pflash0 (rx!p): ORIGIN = 0x80000000, LENGTH = 4M
}

REGION_ALIAS("CODE_MEM", pflash0)
REGION_ALIAS("DATA_MEM", dsram0)
REGION_ALIAS("BSS_MEM", dsram0)
REGION_ALIAS("CSA_MEM", dsram0)
REGION_ALIAS("HEAP_MEM", lmu)
/* REGION_ALIAS("HEAP_MEM", dsram0) */

SECTIONS
{
	.start :
	{
		KEEP(*(.start))
		. = ALIGN(4);
	} > CODE_MEM

	.traptab :
	{
		KEEP(*(.traptab))
		. = ALIGN(4);
	} > CODE_MEM

	.ivtab :
	{
		KEEP(*(.ivtab))
		. = ALIGN(4);
	} > CODE_MEM

	.text :
	{
		KEEP(*(.text))
		. = ALIGN(2);
	} > CODE_MEM

	.data :
	{
		_sdata = ABSOLUTE(.);
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		CONSTRUCTORS
		. = ALIGN(4);
		_edata = ABSOLUTE(.);
	} > DATA_MEM AT> CODE_MEM
	_data_flash = LOADADDR(.data);

	.bss :
	{
		_sbss = ABSOLUTE(.);
		*(.bss .bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = ABSOLUTE(.);
	} > BSS_MEM

	.ustack . : ALIGN(8) FLAGS(aw)
	{
		. +=  CONFIG_IDLETHREAD_STACKSIZE ;
		__USTACK =  . ;
	} > BSS_MEM

#if defined(CONFIG_ARCH_INTERRUPTSTACK)
	.istack . : ALIGN(8) FLAGS(aw)
	{
		. += CONFIG_ARCH_INTERRUPTSTACK;
		__istack =  . ;
	} > BSS_MEM
#endif
	.csa : ALIGN(64) FLAGS(aw)
	{
		__CSA_BEGIN = . ;
		. +=  CONFIG_TRICORE_CSA_COUNT * 64 ;
		__CSA_END = .;
	} > CSA_MEM
	__CSA_SIZE = __CSA_END - __CSA_BEGIN;

	.heap : ALIGN(8) FLAGS(aw)
	{
		_sheap =  . ;
		. +=  __HEAP_SIZE;
		_eheap =  . ;
	} > HEAP_MEM
	_heap_size = _sheap - _eheap;

#if 0
	/DISCARD/ : { *(.comment) }
	/DISCARD/ : { *(.callinfo) }
#endif
}
