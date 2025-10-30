/* ---- configurable knobs ---- */
N_CPUS                 = 7;
__USTACK_SIZE_PERCPU   = 0x1000;      /* bytes per CPU */
__ISTACK_SIZE_PERCPU   = 0x0800;      /* bytes per CPU */
__CSA_FRAMES_PERCPU    = CONFIG_TRICORE_CSA_COUNT; /* frames per CPU (64B each) */

/* your MEMORY{} and REGION_ALIAS{} stay as-is */

/* ---------------- sections ---------------- */
SECTIONS
{
	/* as you had */
	.start : { KEEP(*(.start)) . = ALIGN(4); } > CODE_MEM

	.traptab : { KEEP(*(.traptab)) . = ALIGN(4); } > CODE_MEM
	.ivtab   : { KEEP(*(.ivtab))   . = ALIGN(4); } > CODE_MEM

	.text : { KEEP(*(.text*)) . = ALIGN(4); } > CODE_MEM

	.rodata : { *(.rodata*) . = ALIGN(4); } > CODE_MEM

	.data : AT> CODE_MEM
	{
		_sdata = .;
		*(.data*)
			_edata = .;
	} > DATA_MEM
	_data_load = LOADADDR(.data);

	.bss (NOLOAD) :
	{
		_sbss = .;
		*(.bss*)
		*(COMMON)
		_ebss = .;
	} > BSS_MEM

	/* -------- per-CPU stacks in one slab -------- */
	.ustacks (NOLOAD) : ALIGN(8)
	{
		__ustacks_base = .;
		. = ALIGN(8);
		. += N_CPUS * ((__USTACK_SIZE_PERCPU + 7) & ~7);
		__ustacks_end  = .;
	} > BSS_MEM
	__ustack_stride = ((__USTACK_SIZE_PERCPU + 7) & ~7);

	.istacks (NOLOAD) : ALIGN(8)
	{
		__istacks_base = .;
		. = ALIGN(8);
		. += N_CPUS * ((__ISTACK_SIZE_PERCPU + 7) & ~7);
		__istacks_end  = .;
	} > BSS_MEM
	__istack_stride = ((__ISTACK_SIZE_PERCPU + 7) & ~7);

	/* -------- per-CPU CSA pool in one slab -------- */
	.csa (NOLOAD) : ALIGN(64)
	{
		__CSA_BASE = .;
		. = ALIGN(64);
		. += N_CPUS * (__CSA_FRAMES_PERCPU * 64);
		__CSA_END  = .;
	} > CSA_MEM
	__CSA_STRIDE = (__CSA_FRAMES_PERCPU * 64);
	__CSA_SIZE   = __CSA_END - __CSA_BASE;

	/* -------- optional per-CPU data (like Linux .percpu) -------- */
	.percpu (NOLOAD) : ALIGN(64)
	{
		__percpu_unit_start = .;
		*(.data.percpu*)    /* put per-CPU variables here via __attribute__((section)) */
		*(.bss.percpu*)
		. = ALIGN(64);
		__percpu_unit_end = .;
	} > BSS_MEM
	__percpu_unit_size = (__percpu_unit_end - __percpu_unit_start);
	__percpu_base      = __percpu_unit_start;  /* first CPU’s unit */
}
