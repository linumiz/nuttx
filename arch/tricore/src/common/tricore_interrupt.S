#include <tricore_cpu.h>
	.section .ivtab, "ax", %progbits
	.globl  biv_vector
	.type   biv_vector, %object
	.p2align 5

biv_vector:
	.set N, 255

	.rept N
1:
	svlcx
	j _isr_wrapper
	rslcx
	rfe

	.space (32 - (.-1b))
	.endr

_isr_wrapper:
	mov	%d14, lo:IFX_LWSRZ_VMY
	addih	%d14, %d14, hi:IFX_LWSRZ_VMY

	mfcr	%d15, IFX_CPU_CORE_ID
	extr.u	%d15, %d15, 0, 3
	madd	%d14, %d14, %d15, 0x34

	mov.a	%a2, %d14
	ld.w	%d15, [%a2]
	extr.u	%d15, %d15, 16, 11

	mov	%d4, %d15
	call	tricore_isr_handler
	rslcx
	rfe
