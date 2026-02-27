/****************************************************************************
 * arch/tricore/src/tc3xx/tc3x_wdt.c
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

#include <nuttx/arch.h>
#include <nuttx/bits.h>
#include "tricore_internal.h"

#define CON1_DR         (1u << 3)       /* Disable Request */

#define SCU_BASE        0xF0036000
#define WDTCPU0_CON1    (SCU_BASE + 0x0250)
#define WDTS_CON1       (SCU_BASE + 0x02AC)

void tricore_wdt_disable(void)
{
  aurix_safety_endinit_enable(false);
  putreg32(CON1_DR, WDTS_CON1);
  aurix_safety_endinit_enable(true);

  aurix_cpu_endinit_enable(false);
  putreg32(CON1_DR, WDTCPU0_CON1);
  aurix_cpu_endinit_enable(true);
}
