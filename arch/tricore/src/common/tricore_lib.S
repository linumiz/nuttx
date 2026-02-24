/****************************************************************************
 * arch/tricore/src/common/tricore_lib.S
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Low-level architecture helpers for TriCore NuttX.
 *
 ****************************************************************************/

#include <arch/arch.h>

	.global tricore_endinit_disable
	.global tricore_endinit_enable
	.global tricore_safety_endinit_enable

#if defined(CONFIG_TRICORE_TC18)
#define CPU_PROTSFR	0xF880E080

	.type	tricore_endinit_disable, %function
tricore_endinit_disable:
	mfcr    %d4, IFX_CPU_CORE_ID
	extr.u  %d4, %d4, 0, 3
	sh      %d4, %d4, 18
	movh.a  %a15, hi:CPU_PROTSFR
	lea     %a15, [%a15], lo:CPU_PROTSFR
	addsc.a %a15, %a15, %d4, 0

	ld.w	%d15, [%a15]
	extr.u	%d15, %d15, 0, 3
	jlt.u	%d15, 3, already_config

	/* Transition: Run -> Config (unlock) */
	insert	%d15, %d15, 9, 0, 16
	st.w	[%a15], %d15
	dsync

already_config:
	ji	%a11

	.type	tricore_endinit_enable, %function
tricore_endinit_enable:
	mfcr    %d4, IFX_CPU_CORE_ID
	extr.u  %d4, %d4, 0, 3
	sh      %d4, %d4, 18
	movh.a  %a15, hi:CPU_PROTSFR
	lea     %a15, [%a15], lo:CPU_PROTSFR
	addsc.a %a15, %a15, %d4, 0

	ld.w	%d15, [%a15]
	extr.u	%d15, %d15, 0, 3
	jge.u	%d15, 2, already_locked

	/* Transition: Config -> Run (lock) */
	insert	%d15, %d15, 12, 0, 16
	st.w	[%a15], %d15
	dsync

already_locked:
	ji	%a11

	.type	tricore_safety_endinit_enable, %function
tricore_safety_endinit_enable:
	ji	%a11

	/* Virtualization disable */

	.global __tricore_virt_disable
	.type	__tricore_virt_disable, %function
__tricore_virt_disable:
	mov	%d15, 0
	mtcr	IFX_CPU_VCON0, %d15

	mov	%d15, 0xB00
	mtcr	IFX_CPU_PSW, %d15
	isync

	rfh

#else /* TC1.6 (AURIX TC3x) */

/* TC1.6 (AURIX TC3x) — WDT-based endinit */

#define SCU_BASE	0xF0036000
#define WDTCPU0CON0	(SCU_BASE + 0x024C)
#define WDTSCON0	(SCU_BASE + 0x02A8)

	.type	tricore_endinit_disable, %function
tricore_endinit_disable:
        mfcr    %d4, IFX_CPU_CORE_ID
        extr.u  %d4, %d4, 0, 3
        sh      %d5, %d4, 1
        add     %d4, %d5, %d4
	movh.a	%a15, hi:WDTCPU0CON0
	lea	%a15, [%a15], lo:WDTCPU0CON0
        addsc.a %a15, %a15, %d4, 2

	/* Read current WDT register, invert password bits */
	ld.w	%d15, [%a15]
	xor	%d15, %d15, 0xFC

	insert	%d15, %d15, 1, 0, 2
	st.w	[%a15], %d15

	insert	%d15, %d15, 2, 0, 2
	st.w	[%a15], %d15
	dsync

	ji	%a11

	.type	tricore_endinit_enable, %function
tricore_endinit_enable:
        mfcr    %d4, IFX_CPU_CORE_ID
        extr.u  %d4, %d4, 0, 3
        sh      %d5, %d4, 1
        add     %d4, %d5, %d4
	movh.a	%a15, hi:WDTCPU0CON0
	lea	%a15, [%a15], lo:WDTCPU0CON0
        addsc.a %a15, %a15, %d4, 2

	/* Read current WDT register, invert password bits */
	ld.w	%d15, [%a15]
	xor	%d15, %d15, 0xFC

	insert	%d15, %d15, 1, 0, 2
	st.w	[%a15], %d15

	insert	%d15, %d15, 3, 0, 2
	st.w	[%a15], %d15
	dsync

	ji	%a11

	.type	tricore_safety_endinit_enable, %function
tricore_safety_endinit_enable:
	movh.a	%a15, hi:WDTSCON0
	lea	%a15, [%a15], lo:WDTSCON0

	ld.w	%d15, [%a15]
	xor	%d15, %d15, 0xFC

	insert	%d15, %d15, 1, 0, 2
	st.w	[%a15], %d15

	insert	%d15, %d15, 3, 0, 2
	st.w	[%a15], %d15
	dsync

	ji	%a11

#endif /* CONFIG_TRICORE_TC18 */
