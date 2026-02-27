#include <arch/arch.h>
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

	.space (32 - (.-1b))
	.endr

_isr_wrapper:
	mov	%d14, lo:IFX_IR_LASR_ADDR
	addih	%d14, %d14, hi:IFX_IR_LASR_ADDR

	mfcr	%d15, IFX_CPU_CORE_ID
	extr.u	%d15, %d15, 0, 3
	madd	%d14, %d14, %d15, IFX_IR_LASR_TOS_STRIDE

	mov.a	%a2, %d14
	ld.w	%d15, [%a2]
	extr.u	%d15, %d15, 16, IFX_IR_LASR_PIPN_WIDTH

	mov	%d4, %d15
        movh.a  %a14, hi:tricore_isr_handler
        lea     %a14, [%a14]lo:tricore_isr_handler
	ji	%a14
