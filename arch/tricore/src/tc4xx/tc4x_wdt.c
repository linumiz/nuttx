/****************************************************************************
 * arch/tricore/src/tc4xx/tc4x_wdt.c
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
#include <nuttx/arch.h>

#include "tricore_internal.h"

#define CTRLA_LCK       BIT(0)
#define CTRLA_PW_MASK   (0x7F << 1)   /* PW[6:0] in bits[7:1] */

#define CTRLB_DR        BIT(0)       /* Disable Request */

/* Register addresses */
#define WDT_CPU0_CTRLA  0xF000003C
#define WDT_CPU0_CTRLB  0xF0000040
#define WDT_SYS_CTRLA   0xF00001A8
#define WDT_SYS_CTRLB   0xF00001AC

static void tc4x_wdt_disable(uintptr_t ctrla_addr, uintptr_t ctrlb_addr)
{
  uint32_t ctrla;

  /* Unlock: read CTRLA, invert PW, clear LCK */
  ctrla = getreg32(ctrla_addr);
  if (ctrla & CTRLA_LCK)
    {
      ctrla &= ~CTRLA_LCK;
      ctrla ^= CTRLA_PW_MASK;
      putreg32(ctrla, ctrla_addr);
    }

  /* Set disable request */
  putreg32(CTRLB_DR, ctrlb_addr);

  /* Lock to apply */
  ctrla = getreg32(ctrla_addr);
  ctrla |= CTRLA_LCK;
  putreg32(ctrla, ctrla_addr);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void tricore_wdt_disable(void)
{
  tc4x_wdt_disable(WDT_SYS_CTRLA, WDT_SYS_CTRLB);
  tc4x_wdt_disable(WDT_CPU0_CTRLA, WDT_CPU0_CTRLB);
}
