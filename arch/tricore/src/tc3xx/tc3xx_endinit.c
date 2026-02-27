/****************************************************************************
 * arch/tricore/src/tc3xx/tc3xx_endinit.c
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

#include <nuttx/bits.h>
#include "tricore_internal.h"

#define SCU_BASE        0xF0036000
#define WDTCPU0CON0     (SCU_BASE + 0x024C)
#define WDTSCON0        (SCU_BASE + 0x02A8)

static void wdt_modify(uintptr_t con0, bool endinit)
{
  uint32_t val = getreg32(con0);
  uint32_t pw = ((val >> 2) & 0x3FFF) ^ 0x003F;

  /* Password access: LCK=1, ENDINIT=1 */
  if (val & BIT(1))
    {
      putreg32((val & 0xFFFF0000) | (pw << 2) | BIT(0), con0);
    }

  /* Modify access: LCK=1, ENDINIT=desired */
  putreg32((val & 0xFFFF0000) | (pw << 2) | BIT(1) |
           (endinit ? BIT(0) : 0), con0);

  /* Wait for ENDINIT to take effect */
  while ((getreg32(con0) & BIT(0)) != (endinit ? 1 : 0));
}

void aurix_cpu_endinit_enable(bool enable)
{
  wdt_modify(WDTCPU0CON0, enable);
}

void aurix_safety_endinit_enable(bool enable)
{
  wdt_modify(WDTSCON0, enable);
}
