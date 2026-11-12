/****************************************************************************
 * arch/tricore/src/tc3x/tc3x_start.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/arch.h>

#include "tricore_internal.h"
#include "hardware/tc3x_cpu.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Provided by the linker script */

extern uintptr_t __start1[];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_ram_init
 *
 * Description:
 *   TC3xx has no RAM to initialize: the boot firmware initializes the CPU
 *   RAMs, and NuttX uses no other.
 *
 ****************************************************************************/

void tricore_ram_init(void)
{
}

/****************************************************************************
 * Name: tricore_start_cores
 *
 * Description:
 *   Release CPU1 at the start of its own image, as the iLLD startup code
 *   does.
 *
 ****************************************************************************/

void tricore_start_cores(void)
{
  putreg32((getreg32(TC3X_CPU1_PC) & 1) | ((uintptr_t)__start1 & ~1),
           TC3X_CPU1_PC);
  if ((getreg32(TC3X_CPU1_SYSCON) & TC3X_SYSCON_BHALT) != 0)
    {
      putreg32(getreg32(TC3X_CPU1_SYSCON) & ~TC3X_SYSCON_BHALT,
               TC3X_CPU1_SYSCON);
    }
}
