#include "asm_macros.inc"

	.section .traptab, "ax"
	.global	btv_vector

btv_vector:
	btv_vector_class 0
	btv_vector_class 1
	btv_vector_class 2
	btv_vector_class 3
	btv_vector_class 4
	btv_vector_class 5
	btv_vector_class 6
	btv_vector_class 7

tricore_trap_handler:
	fret
