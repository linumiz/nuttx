/****************************************************************************
 * arch/tricore/include/tc4xx/chip.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_TRICORE_INCLUDE_TC4XX_CHIP_H
#define __ARCH_TRICORE_INCLUDE_TC4XX_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* System timer (STM) of CPU0, part of the CPU0 SFR space, and its
 * interrupt
 */

#define TRICORE_STM0_BASE          0xf8800000  /* MODULE_CPU0 */
#define TRICORE_STM0_IRQ           10          /* SRC_STMCPU0_SR2 */

#define TRICORE_STM_ABS_OFFSET     0x20        /* 64-bit timer */

/* System timer (STM) register offsets and bits */

#define TRICORE_STM_CMP0_OFFSET    0x120       /* Compare register 0 */
#define TRICORE_STM_CMCON_OFFSET   0x128       /* Compare match control */
#define TRICORE_STM_ICR_OFFSET     0x12c       /* Interrupt control */
#define TRICORE_STM_ISCR_OFFSET    0x130       /* Interrupt set/clear */
#define TRICORE_STM_OCS_OFFSET     0x4         /* OCDS control and status */

#define TRICORE_STM_CMCON_MSIZE0   (0x1f << 0) /* CMP0 compare size - 1 */
#define TRICORE_STM_CMCON_MSTART0  (0x1f << 8) /* CMP0 first compared bit */
#define TRICORE_STM_ICR_CMP0EN     (1 << 0)    /* CMP0 interrupt enable */
#define TRICORE_STM_ICR_CMP0OS     (1 << 1)    /* CMP0 output, 0 is STMIR0 */
#define TRICORE_STM_ISCR_CMP0IRR   (1 << 0)    /* Clear CMP0 request */
#define TRICORE_STM_OCS_SUS_HARD   (1 << 24)   /* Hard suspend on halt */
#define TRICORE_STM_OCS_SUS_MASK   (0xf << 24)
#define TRICORE_STM_OCS_SUS_P      (1 << 28)   /* SUS write protection */

/* ASCLIN0 (UART) and its register offsets */

#define TRICORE_ASCLIN0_BASE       0xf46c0000

#define TRICORE_ASCLIN_CLC_OFFSET  0x000
#define TRICORE_ASCLIN_IOCR_OFFSET 0x100
#define TRICORE_ASCLIN_TXFIFOCON_OFFSET 0x104
#define TRICORE_ASCLIN_RXFIFOCON_OFFSET 0x108
#define TRICORE_ASCLIN_BITCON_OFFSET 0x10c
#define TRICORE_ASCLIN_FRAMECON_OFFSET 0x110
#define TRICORE_ASCLIN_DATCON_OFFSET 0x114
#define TRICORE_ASCLIN_BRG_OFFSET  0x118
#define TRICORE_ASCLIN_FLAGSCLEAR_OFFSET 0x134
#define TRICORE_ASCLIN_FLAGSENABLE_OFFSET 0x138
#define TRICORE_ASCLIN_CSR_OFFSET  0x13c
#define TRICORE_ASCLIN_TXDATA_OFFSET 0x140
#define TRICORE_ASCLIN_RXDATA_OFFSET 0x160

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_TRICORE_INCLUDE_TC4XX_CHIP_H */
