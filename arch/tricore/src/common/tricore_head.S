#include <tricore_cpu.h>
/*
1.
sp-> PSPR0
trap table
virt disable
CSA -> PSPR0 (func call)
PSW reg init ?
A0, A1, A8, A9 -> init
reset cause
*/
/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.section .start, "ax"

	.global	btv_vector
	.global __start; .type __start, %function

__start:
	/* Phase 1: Start */
	/* Main stack pointer to PSPR0 */
	/* TODO
		- Should we care use DSPR0?
		- Fix the address coming from xx?
	*/
	movh.a	%a10, hi:(IFX_CFG_SSW_CSA_USTACK_PTR)
	lea	%a10, [%a10]lo:(IFX_CFG_SSW_CSA_USTACK_PTR)
	dsync	/* FIXME why? */

// TODO: disable endinit

	/* Trap table setup */
	mov	%d15, lo:btv_vector
	addih   %d15, %d15, hi:btv_vector
	mtcr	IFX_CPU_BTV, %d15
	isync

	/* CSA init to PSPR0 */
	/* TODO should we use DSPR0 ? */
	movh.a	%a15, hi:(IFX_CFG_SSW_CSA_BOOT_PTR_START)
	lea	%a15, [%a15]lo:(IFX_CFG_SSW_CSA_BOOT_PTR_START)

	mov.d   %d14, %a15
	extr.u  %d13, %d14, 12, 20
	extr.u  %d12, %d14, 6, 16
	insert  %d13, %d13, %d12, 0, 16
	mtcr    IFX_CPU_FCX, %d13 
	isync

	/* Disable virtualization */
	call disable_virtualization

	/* Re-init trap table setup */
	mov	%d15, lo:btv_vector
	addih   %d15, %d15, hi:btv_vector
	mtcr	IFX_CPU_BTV, %d15
	isync

	/* Setup PSW */
	mov %d15, lo:IFX_CFG_SSW_PSW_DEFAULT
	mtcr IFX_CPU_PSW, %d15
	isync

	/* PSW.IS = 0, enable ISP stack */
	mov	%d15, lo:__ISTACK
	addih   %d15, %d15, hi:__ISTACK
	mtcr	IFX_CPU_ISP, %d15
	isync

	/* TODO: A0/A1/A8/A9 - init global registers */

	/* TODO: reset evaluation */
	/* Phase 1: END */

	/* Phase 2: Start */
	/* Phase 2: END */

	/* Phase 3: Start */
	/* Main stack pointer to */
	movh.a	%a10, hi:__USTACK
	lea	%a10, [%a10]lo:__USTACK
	dsync	/* FIXME why? */

	/* Initialize context save areas */
	movh.a  %a15, hi:__CSA_BEGIN
	lea     %a15, [%a15], lo:__CSA_BEGIN
	lea     %a14, [%a15], __CSA_SIZE
	lea     %a13, 64

	mov.d   %d14, %a15
	extr.u  %d13, %d14, 12, 20
	extr.u  %d12, %d14, 6, 16
	insert  %d13, %d13, %d12, 0, 16
	mtcr    IFX_CPU_FCX, %d13 
	isync

csa_loop:
	addi    %d14, %d14, 64
	extr.u  %d13, %d14, 12, 20
	extr.u  %d12, %d14, 6, 16
	insert  %d13, %d13, %d12, 0, 16
	st.w    [%a15], %d13
	add.a   %a15, %a13
	jne.a   %a15, %a14, csa_loop

	/* Store LCX with 3 less entries for context depletion trap */
	addi    %d13, %d13, -3
	mtcr    IFX_CPU_LCX, %d13
	isync
	dsync

	/* copy .data */
/*
	- src addr
	- size
	- dst addr
*/
	movh.a %a0, hi:_sdata // dest
	lea %a0, [%a0]lo:_sdata

	movh.a %a1, hi:_edata // source end
	lea %a1, [%a1]lo:_edata

	movh.a %a2, hi:_data_load // src
	lea %a2, [%a2]lo:_data_load

	jeq.a %a0, %a1, skip_data // no data section
copy_data:
	ld.w %d0, [%a0+]
	st.w [%a2+], %d0
	jne.a %a0, %a1, copy_data

skip_data:
	/* clear bss */
	movh.a %a0, hi:_sbss // start
	lea %a0, [%a0]lo:_sbss

	movh.a %a1, hi:_ebss // end
	lea %a1, [%a1]lo:_ebss

	jeq.a %a2, %a1, skip_bss // no bss section
	mov %d0, 0
clear_bss:
	st.w [%a0+], %d0
	jne.a %a0, %a1, clear_bss

skip_bss:
	/* Interrupt vector table setup */
	mov	%d15, lo:biv_vector
	addih   %d15, %d15, hi:biv_vector
	mtcr	IFX_CPU_BIV, %d15
	isync

// TODO: enable endinit

	// Testing INT START
	call up_irqinitialize
	call up_timer_initialize
/*

	mov %d4, 1
	call up_enable_irq

	mov %d4, 1
	mov %d5, 0
	call up_trigger_irq

	mov	%d15, lo:0x120000FF
	addih	%d15, %d15, hi:0x120000FF

	movh.a %a0, hi:IFX_SRCR1
	lea %a0, [%a0]lo:IFX_SRCR1
	st.w [%a0], %d15

	ld.w  %d15, [%a0]
	insert %d15, %d15, 1, 23, 1
	st.w [%a0], %d15

	ld.w  %d15, [%a0]
	insert %d15, %d15, 1, 26, 1
	st.w [%a0], %d15
*/
	// Testing INT END

/*
	mov %d4, 0
	call tricore_live
*/

live:
	j live
//	j nx_start

	/* Phase 3: END */

disable_virtualization:
	mov %d15, 0
	mtcr IFX_CPU_VCON0, %d15
	isync
	mov %d15, 0xB00
	mtcr IFX_CPU_PSW, %d15
	isync
	rfh

/*
2.
EVR/PMIC based on external components

3.
CSA, SP -> DSPR0
LBIST test
...
security cluster sync point B
C runtime routines 

4.
unused regulator turn off
PMIC core 1.023v

5.
security cluster sync point C
Clock init & config flash

6. optional

7. access protection

8.
Cache & interrupt enablement
xfer to OS -> startcore & startOS on every CPU
*/
