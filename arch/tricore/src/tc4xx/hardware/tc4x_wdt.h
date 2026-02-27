/****************************************************************************
 * arch/tricore/src/tc4xx/tc4x_wdt.h
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

#ifndef __ARCH_TRICORE_SRC_TC4XX_TC4X_WDT_H
#define __ARCH_TRICORE_SRC_TC4XX_TC4X_WDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TC4X_WTU_BASE            0xF0000000

#define TC4X_WDTCPU0_BASE        0xF0000018
#define TC4X_WDTCPU1_BASE        0xF0000048
#define TC4X_WDTCPU2_BASE        0xF0000078
#define TC4X_WDTCPU3_BASE        0xF00000A8
#define TC4X_WDTCPU4_BASE        0xF00000D8
#define TC4X_WDTCPU5_BASE        0xF0000108
#define TC4X_WDTSYS_BASE         0xF0000184

#define TC4X_WDT_CTRLA_OFFSET    0x24
#define TC4X_WDT_CTRLB_OFFSET    0x28
#define TC4X_WDT_STAT_OFFSET     0x2C

#define WDT_CTRLA_LCK            BIT(0)
#define WDT_CTRLA_PW_SHIFT       1
#define WDT_CTRLA_PW_MASK        (0x7fff << 1)
#define WDT_CTRLA_PW_LOW_MASK    (0x7f << 1)
#define WDT_CTRLA_PW_HIGH_MASK   (0xff << 8)
#define WDT_CTRLA_TCVI_SHIFT     16
#define WDT_CTRLA_TCVI_MASK      (0xffff << 16)

#define WDT_CTRLB_DR             BIT(0)
#define WDT_CTRLB_IFSR_SHIFT     4
#define WDT_CTRLB_IFSR_MASK      (0x3 << 4)
#define WDT_CTRLB_URR            BIT(6)
#define WDT_CTRLB_PAR            BIT(7)
#define WDT_CTRLB_TCR            BIT(8)
#define WDT_CTRLB_TCTR_SHIFT     9
#define WDT_CTRLB_TCTR_MASK      (0x7f << 9)
#define WDT_CTRLB_TIMR_SHIFT     16
#define WDT_CTRLB_TIMR_MASK      (0xffff << 16)

#define WDT_STAT_D               BIT(0)
#define WDT_STAT_TOM             BIT(1)
#define WDT_STAT_OE              BIT(2)
#define WDT_STAT_AE              BIT(3)
#define WDT_STAT_IFS_SHIFT       4
#define WDT_STAT_IFS_MASK        (0x3 << 4)
#define WDT_STAT_UR              BIT(6)
#define WDT_STAT_PA              BIT(7)
#define WDT_STAT_TC              BIT(8)
#define WDT_STAT_TCT_SHIFT       9
#define WDT_STAT_TCT_MASK        (0x7f << 9)
#define WDT_STAT_TIM_SHIFT       16
#define WDT_STAT_TIM_MASK        (0xffff << 16)

#define WDT_IFS_DIV16384         0
#define WDT_IFS_DIV256           1
#define WDT_IFS_DIV64            2

#define WDT_DIVIDER_16384        16384
#define WDT_DIVIDER_256          256
#define WDT_DIVIDER_64           64

#define WDT_TIMER_MAX            65536

/* WDT instance enumeration */

#define TC4X_WDT_CPU0            0
#define TC4X_WDT_CPU1            1
#define TC4X_WDT_CPU2            2
#define TC4X_WDT_CPU3            3
#define TC4X_WDT_CPU4            4
#define TC4X_WDT_CPU5            5
#define TC4X_WDT_SYS             6
#define TC4X_WDT_COUNT           7

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_TC4X_WDT

/****************************************************************************
 * Name: tc4x_wdt_initialize
 *
 * Description:
 *   Initialize a WDT instance and register it as /dev/watchdogN.
 *
 * Input Parameters:
 *   devpath - The device path string, e.g. "/dev/watchdog0"
 *   wdtid   - WDT instance id (TC4X_WDT_CPU0 .. TC4X_WDT_SYS)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 *
 ****************************************************************************/

int tc4x_wdt_initialize(const char *devpath, int wdtid);

#endif /* CONFIG_TC4X_WDT */
#endif /* __ARCH_TRICORE_SRC_TC4XX_TC4X_WDT_H */
