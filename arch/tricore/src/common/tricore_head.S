/****************************************************************************
 * arch/tricore/src/common/tricore_head.S
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * TriCore NuttX Boot Initialization
 *
 * Supports:
 *   - TC1.8 architecture (AURIX TC4x)
 *   - TC1.6 architecture (AURIX TC3x)
 *
 ****************************************************************************/

#include <arch/arch.h>

#define PSW_CDE         (1 << 7)
#define PSW_GW          (1 << 8)
#define PSW_IS          (1 << 9)
#define PSW_IO_SV       (2 << 10)

#define PSW_DEFAULT     (PSW_CDE | PSW_GW | PSW_IO_SV)
#define PSW_DEFAULT_IS  (PSW_DEFAULT | PSW_IS)

	.extern btv_vector
	.extern biv_vector
	.extern tricore_wdt_disable
	.extern up_clockconfig
	.extern tricore_start

	.extern tricore_endinit_disable
	.extern tricore_endinit_enable
	.extern tricore_safety_endinit_enable

#if defined(CONFIG_TRICORE_TC18)
	.extern __tricore_virt_disable
#endif

.macro encode_csa_link addr_reg, tmp1, tmp2
	extr.u	\tmp1, \addr_reg, 6, 16
	extr.u	\tmp2, \addr_reg, 12, 20
	insert	\addr_reg, \tmp2, \tmp1, 0, 16
.endm

	/* Reset Entry Point */
	.section .start, "ax"
	.global __start
	.type   __start, %function

__start:
	movh.a	%a15, hi:__start_stage2
	lea	%a15, [%a15], lo:__start_stage2
	ji	%a15

	.section .text, "ax"
	.type   __start_stage2, %function

__start_stage2:

	/* Disable endinit protection */
	jl	tricore_endinit_disable

	movh.a	%a10, hi:__PSRAM_USTACK
	lea	%a10, [%a10], lo:__PSRAM_USTACK

	/* Trap vector table (BTV) */
	mov	%d15, lo:btv_vector
	addih	%d15, %d15, hi:btv_vector
	mtcr	IFX_CPU_BTV, %d15
	isync

	/* Boot CSA in PSPR0 */
	movh.a	%a15, hi:__PSRAM_BOOT_CSA
	lea	%a15, [%a15], lo:__PSRAM_BOOT_CSA

	mov.d	%d15, %a15
	encode_csa_link %d15, %d14, %d13
	mtcr	IFX_CPU_FCX, %d15
	isync

#if defined(CONFIG_TRICORE_TC18)

	/* Disable Virtualization */
	call	__tricore_virt_disable

	/* Re-init trap table setup because clobbered BTV and FCX after RFH */
	mov	%d15, lo:btv_vector
	addih	%d15, %d15, hi:btv_vector
	mtcr	IFX_CPU_BTV, %d15

	movh.a	%a15, hi:__PSRAM_BOOT_CSA
	lea	%a15, [%a15], lo:__PSRAM_BOOT_CSA
	mov.d	%d15, %a15
	encode_csa_link %d15, %d14, %d13
	mtcr	IFX_CPU_FCX, %d15
	isync

#endif /* CONFIG_TRICORE_TC18 */

#if defined(CONFIG_ARCH_INTERRUPTSTACK)
	/* Setup PSW */
	mov	%d15, lo:PSW_DEFAULT
	mtcr	IFX_CPU_PSW, %d15
	isync

	/* PSW.IS = 0, enable ISP stack */
	mov	%d15, lo:__istack
	addih	%d15, %d15, hi:__istack
	mtcr	IFX_CPU_ISP, %d15
	isync
#else
	/* PSW.IS = 0, enable user stack */
	mov	%d15, lo:PSW_DEFAULT_IS
	mtcr	IFX_CPU_PSW, %d15
	isync
#endif

	/* Stack pointer -> DSPR0 (final user stack) */
	movh.a	%a10, hi:__USTACK
	lea	%a10, [%a10], lo:__USTACK

	/* Initialize context save areas */
	movh.a	%a15, hi:__CSA_BEGIN
	lea	%a15, [%a15], lo:__CSA_BEGIN
	movh.a	%a14, hi:__CSA_END
	lea	%a14, [%a14], lo:__CSA_END

	/* Encode first entry -> FCX */
	mov.d	%d15, %a15
	encode_csa_link %d15, %d14, %d13
	mtcr	IFX_CPU_FCX, %d15
	isync

	/* Build free list */
csa_loop:
	addi	%d15, %d15, 1
	st.w	[%a15], %d15
	lea	%a15, [%a15], 64
	jne.a	%a15, %a14, csa_loop

	/* Store LCX with 3 less entries for context depletion trap */
	addi	%d15, %d15, -3
	mtcr	IFX_CPU_LCX, %d15
	isync

	/* Copy .data from flash (LMA) to RAM (VMA) */
	movh.a	%a0, hi:_data_flash
	lea	%a0, [%a0], lo:_data_flash
	movh.a	%a1, hi:_sdata
	lea	%a1, [%a1], lo:_sdata
	movh.a	%a2, hi:_edata
	lea	%a2, [%a2], lo:_edata
	jeq.a	%a1, %a2, skip_data

copy_data:
	ld.w	%d0, [%a0+]
	st.w	[%a1+], %d0
	jne.a	%a1, %a2, copy_data

skip_data:

	/* clear bss */
	movh.a	%a0, hi:_sbss
	lea	%a0, [%a0], lo:_sbss
	movh.a	%a1, hi:_ebss
	lea	%a1, [%a1], lo:_ebss
	jeq.a	%a0, %a1, skip_bss
	mov	%d0, 0

clear_bss:
	st.w	[%a0+], %d0
	jne.a	%a0, %a1, clear_bss

skip_bss:

	/* Interrupt vector table setup */
	mov	%d15, lo:biv_vector
	addih	%d15, %d15, hi:biv_vector
	mtcr	IFX_CPU_BIV, %d15
	isync

	/* Re-enable endinit protection */
	jl	tricore_endinit_enable
	jl	tricore_safety_endinit_enable

	/* Disable watchdog */
	call	tricore_wdt_disable

	mfcr	%d4, IFX_CPU_CORE_ID
	extr.u	%d4, %d4, 0, 3
	j	tricore_start
