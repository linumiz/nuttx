/*
 * Copyright (c) 2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TRICORE_INTERRUPT_CONTROLLER_INTC_TC_IR_PRIV_H_
#define TRICORE_INTERRUPT_CONTROLLER_INTC_TC_IR_PRIV_H_

#include <stdint.h>

#define NR_IRQS	2048
typedef struct SRCR_Bits {
	uint32_t SRPN: 8;
	uint32_t VM: 3;
	uint32_t CS: 1;
	uint32_t TOS: 4;
	uint32_t reserved_16: 7;
	uint32_t SRE: 1;
	uint32_t SRR: 1;
	uint32_t CLRR: 1;
	uint32_t SETR: 1;
	uint32_t IOV: 1;
	uint32_t IOVCLR: 1;
	uint32_t reserved_29: 3;
} SRCR_Bits;
#define GET_TOS(coreId) coreId

typedef union {
	uint32_t U;
	int32_t I;
	SRCR_Bits B;
} SRCR;

#define IFX_IR_INT_BASE 0xf4430000
#define IFX_IR_SRC_BASE 0xf4432000

#define IFX_IR_GET_SRC(irq)	(IFX_IR_SRC_BASE + (irq)*4)
#define TRICORE_DEFAULT_IR_PRIO	255
#define TRICORE_DEFAULT_IR_TOS	0

#define IFX_STM_IR_OFFSET 0x20
#define IFX_STM_IR_CPUw_SRx(w, x) IFX_IR_SRC_BASE + IFX_STM_IR_OFFSET + (w * 0x40) + (x * 0x4) //0xF4432028u
#define IFX_STM_IR_CPU0_SR(x) IFX_STM_CPUw_SRx(0, x)

#define IFX_STM_BASE_SRN 8

#define IFX_STM_IR_SRN(x) (IFX_STM_BASE_SRN + x)

#endif
