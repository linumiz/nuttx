/****************************************************************************
 * arch/tricore/src/common/tricore_stm.h
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

#ifndef __ARCH_TRICORE_SRC_COMMON_TRICORE_STM_H
#define __ARCH_TRICORE_SRC_COMMON_TRICORE_STM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/bits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_ARCH_CHIP_FAMILY_TC4XX)

#define IFX_STM_CPU_BASE            0xf8800000
#define IFX_STM_CPU_STRIDE          0x40000

#define IFX_STM_DEFAULT             2

#define IFX_STM_BASE(core)   (IFX_STM_CPU_BASE + (core * IFX_STM_CPU_STRIDE))

#define IFX_STM_ABS(core) (IFX_STM_BASE(core) + 0x20)
#define IFX_STM_OCS(core) (IFX_STM_BASE(core) + 0x04)

#define IFX_STM_VM_BANK(core, stm) \
  (IFX_STM_BASE(core) + ((stm >> 1) * 0x20))

#define IFX_STM_CMP0(core, stm)     (IFX_STM_VM_BANK(core, stm) + 0x100)
#define IFX_STM_CMP1(core, stm)     (IFX_STM_VM_BANK(core, stm) + 0x104)
#define IFX_STM_CMCON(core, stm)    (IFX_STM_VM_BANK(core, stm) + 0x108)
#define IFX_STM_ICR(core, stm)      (IFX_STM_VM_BANK(core, stm) + 0x10C)
#define IFX_STM_ISCR(core, stm)     (IFX_STM_VM_BANK(core, stm) + 0x110)
#define IFX_STM_ISR(core, stm)      (IFX_STM_VM_BANK(core, stm) + 0x114)

#define IFX_STM_SRC_INDEX(core, stm) (8 + stm)

#define IFX_STM_FREQ                (500 * 1000 * 1000)
#define IFX_STM_HAS_64BIT_READ     1

#elif defined(CONFIG_ARCH_CHIP_FAMILY_TC3XX)

#define IFX_STM_PERIPH_BASE         0xF0001000
#define IFX_STM_PERIPH_STRIDE       0x100

#define IFX_STM_DEFAULT            0

#define IFX_STM_BASE(core) \
  (IFX_STM_PERIPH_BASE + ((core) * IFX_STM_PERIPH_STRIDE))

#define IFX_STM_TIM0(core)          (IFX_STM_BASE(core) + 0x10)
#define IFX_STM_TIM0SV(core)        (IFX_STM_BASE(core) + 0x50)
#define IFX_STM_CAPSV(core)         (IFX_STM_BASE(core) + 0x54)

#define IFX_STM_CLC(core)           (IFX_STM_BASE(core) + 0x00)
#define IFX_STM_OCS(core)           (IFX_STM_BASE(core) + 0xe8)

#define IFX_STM_CMP0(core, stm)      (IFX_STM_BASE(core) + 0x30)
#define IFX_STM_CMP1(core, stm)      (IFX_STM_BASE(core) + 0x34)
#define IFX_STM_CMCON(core, stm)     (IFX_STM_BASE(core) + 0x38)
#define IFX_STM_ICR(core, stm)       (IFX_STM_BASE(core) + 0x3c)
#define IFX_STM_ISCR(core, stm)      (IFX_STM_BASE(core) + 0x40)

#define IFX_STM_SRC_BASE            192
#define IFX_STM_SRC_INDEX(core, stm) \
  (IFX_STM_SRC_BASE + (core) * 2)

#define IFX_STM_FREQ                (100 * 1000 * 1000)

#define IFX_STM_HAS_64BIT_READ     0
#endif

#define IFX_STM_CMCON_MSIZE0_SHIFT  0
#define IFX_STM_CMCON_MSIZE0_MASK   GENMASK(4, 0)
#define IFX_STM_CMCON_MSTART0_SHIFT 8
#define IFX_STM_CMCON_MSTART0_MASK  GENMASK(12, 8)

#define IFX_STM_CMCON_MSIZE1_SHIFT  16
#define IFX_STM_CMCON_MSIZE1_MASK   GENMASK(20, 16)
#define IFX_STM_CMCON_MSTART1_SHIFT 24
#define IFX_STM_CMCON_MSTART1_MASK  GENMASK(28, 24)

#define IFX_STM_ICR_CMP0EN          BIT(0)
#define IFX_STM_ICR_CMP0OS          BIT(1)
#define IFX_STM_ICR_CMP1EN          BIT(4)
#define IFX_STM_ICR_CMP1OS          BIT(5)

#define IFX_STM_ISCR_CMP0IRR        BIT(0)
#define IFX_STM_ISCR_CMP0IRS        BIT(1)
#define IFX_STM_ISCR_CMP1IRR        BIT(2)
#define IFX_STM_ISCR_CMP1IRS        BIT(3)

#define IFX_STM_ISR_CMP0IR          BIT(0)
#define IFX_STM_ISR_CMP1IR          BIT(1)

#define IFX_STM_CLC_DISR            BIT(0)
#define IFX_STM_CLC_DISS            BIT(1)

#define IFX_STM_OCS_SUS_W           0x12000000

#endif /* __ARCH_TRICORE_SRC_COMMON_TRICORE_STM_H */
