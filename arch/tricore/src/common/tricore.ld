OUTPUT_FORMAT("elf32-tricore")
OUTPUT_ARCH("tricore")
ENTRY(__start)

STARTPTR_NC_CPU0 = 0xA0000000;
MEMORY
{
	dsram0 (w!xp): ORIGIN = 0x70000000, LENGTH = 240K
	psram0 (w!xp): ORIGIN = 0x70100000, LENGTH = 64K

	pflash0 (rx!p): ORIGIN = 0x80000000, LENGTH = 4M
}

SECTIONS
{
	.start (STARTPTR_NC_CPU0) :
	{
		KEEP(*(.start));
	} > pflash0

	.csa : ALIGN(64)
	{
		_csa_begin = .;
		. +=  CONFIG_TRICORE_CSA_COUNT * 64 ;
		_csa_end = .;
	} >dsram0

	.ustack
	{
		_ustack = .;		
	} >dsram0

	.istack
	{
		_istack = .;
	} >dsram0

	/*
		- CSA
		- User stack
		- ISR stack
	*/
}
