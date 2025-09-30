OUTPUT_FORMAT("elf32-tricore")
OUTPUT_ARCH("tricore")
ENTRY(__start)

__USTACK_SIZE = 2K;
__ISTACK_SIZE = 1k;
__HEAP_MIN = 4K;
CONFIG_TRICORE_CSA_COUNT = 128 ;


MEMORY
{
	dsram0 (w!xp): ORIGIN = 0x70000000, LENGTH = 240K
	psram0 (w!xp): ORIGIN = 0x70100000, LENGTH = 64K

	pflash0 (rx!p): ORIGIN = 0x80000000, LENGTH = 4M
}

REGION_ALIAS("CODE_MEM", pflash0)
REGION_ALIAS("BSS_MEM", dsram0)
REGION_ALIAS("CSA_MEM", dsram0)

SECTIONS
{
	.start :
	{
		KEEP(*(.start))
		. = ALIGN(8);
	} > CODE_MEM

	.traptab : FLAGS(axl)
	{
		KEEP(*(.traptab))
		. = ALIGN(8);
	} > CODE_MEM

	.ustack : ALIGN(8) FLAGS(aw)
	{
		. +=  __USTACK_SIZE ;
		. = ALIGN(8) ;
		__USTACK =  . ;
	} > BSS_MEM

	.istack : ALIGN(8) FLAGS(aw)
	{
		. +=  __ISTACK_SIZE ;
		. = ALIGN(8) ;
		__ISTACK =  . ;
	} > BSS_MEM

	.csa : ALIGN(64) FLAGS(aw)
	{
		__CSA_BEGIN = . ;
		. +=  CONFIG_TRICORE_CSA_COUNT * 64 ;
		__CSA_END = .;
	} > CSA_MEM
	__CSA_SIZE = __CSA_END - __CSA_BEGIN;
}
