#include <arch/arch.h>

/*
	.section .traptab, "ax", %progbits
	.global	btv_vector
	.type   btv_vector, %object
	.p2align 5

btv_vector:
	.set N, 8
	.set I, 0

	.rept N
1:
	svlcx
	mov %d4, I // Trap Class
	mov %d5, %d15 // TIN

	movh.a  %a14, hi:tricore_trap_common_entry
	lea     %a14, [%a2]lo:tricore_trap_common_entry
	ji %a14

	.space (32 - (.-1b))
	.set I, I + 1
	.endr


	.extern g_current_regs
tricore_trap_common_entry:
	mfcr %d0, IFX_CPU_PCXI

	mov     %d1, 0
	insert  %d1, %d1, %d0, 6, 16
	extr.u  %d2, %d0, 16, 4
	insert  %d1, %d1, %d2, 28, 4

	movh.a %a14, hi:g_current_regs
	lea %a14, [%a14]lo:g_current_regs
	st.w [%a14], %d1

	call tricore_trap_handler

	movh.a %a14, hi:g_current_regs
	lea %a14, [%a14]lo:g_current_regs
	mov %d0, 0
	st.w [%a14], %d0

	rslcx
	rfe
*/

        .section .traptab, "ax", %progbits
        .global btv_vector
        .type   btv_vector, %object
        .p2align 8

        .macro TRAP_SLOT I, handler
1:
        mov     %d4, \I
        mov     %d5, %d15
        movh.a  %a2, hi:\handler
        lea     %a2, [%a2]lo:\handler
        ji      %a2
        .space  (32 - (.-1b))
        .endm

btv_vector:
        TRAP_SLOT 0, trap_default
        TRAP_SLOT 1, trap_default
        TRAP_SLOT 2, trap_default
        TRAP_SLOT 3, trap_default
        TRAP_SLOT 4, trap_default
        TRAP_SLOT 5, trap_default
        TRAP_SLOT 6, tricore_svcall_local
        TRAP_SLOT 7, trap_default

        .text
        .p2align 2
        .global trap_default
trap_default:
//	rslcx
        rfe

tricore_svcall_local:
        svlcx
	call tricore_svcall
	rslcx
	rfe
