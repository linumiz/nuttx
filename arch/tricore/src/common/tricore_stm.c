/****************************************************************************
 * arch/tricore/src/common/tricore_stm.c
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
#include <time.h>

#include <arch/arch.h>
#include <nuttx/irq.h>
#include <nuttx/tls.h>
#include <nuttx/clock.h>
#include <nuttx/sched.h>
#include <nuttx/arch.h>

#include "tricore_irq.h"
#include "tricore_stm.h"
#include "tricore_internal.h"

#define CYCLES_PER_SEC  IFX_STM_FREQ
#define CYCLES_PER_TICK (CONFIG_USEC_PER_TICK * (CYCLES_PER_SEC / USEC_PER_SEC))

#define cycle_diff_t    unsigned long

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint64_t g_last_count;
static uint64_t g_last_ticks;
static uint8_t  g_core_id;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint64_t stm_get_time64(void)
{
#if IFX_STM_HAS_64BIT_READ
  return getreg64(IFX_STM_ABS(g_core_id));
#else
  uint64_t lo;
  uint64_t hi;

  lo = (uint64_t)getreg32(IFX_STM_TIM0SV(g_core_id));
  hi = (uint64_t)getreg32(IFX_STM_CAPSV(g_core_id));

  return (hi << 32) | lo;
#endif
}

static inline void stm_set_compare(uint32_t cmp)
{
  putreg32(cmp, IFX_STM_CMP0(g_core_id, IFX_STM_DEFAULT));
}

static int tricore_timerisr(int irq, uint32_t *regs, void *arg)
{
  uint32_t val;
  uint64_t now;
  uint64_t dcycles;
  uint32_t dticks;
  uint64_t next;

  /* Clear the compare match interrupt */
  val = getreg32(IFX_STM_ISCR(g_core_id, IFX_STM_DEFAULT));
  val |= IFX_STM_ISCR_CMP0IRR;
  putreg32(val, IFX_STM_ISCR(g_core_id, IFX_STM_DEFAULT));

  /* Calculate elapsed ticks */

  now     = stm_get_time64();
  dcycles = now - g_last_count;
  dticks  = (cycle_diff_t)dcycles / CYCLES_PER_TICK;

  g_last_count += (cycle_diff_t)dticks * CYCLES_PER_TICK;
  g_last_ticks += dticks;

  next = g_last_count + CYCLES_PER_TICK;
  stm_set_compare((uint32_t)next);

  nxsched_process_timer();

  return 0;
}

static inline unsigned int tricore_get_core_id(void)
{
  unsigned int id;

  __asm__ __volatile__("mfcr %0, %1"
                       : "=d"(id)
                       : "i"(IFX_CPU_CORE_ID)
                       : "memory");

  return id & IFX_CPU_CORE_ID_MASK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   Initialize the STM as the system tick timer.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint32_t val;
  int stm_irq;

  g_core_id = tricore_get_core_id();
  stm_irq   = IFX_STM_SRC_INDEX(g_core_id, IFX_STM_DEFAULT);

  val  = getreg32(IFX_STM_CMCON(g_core_id, IFX_STM_DEFAULT));
  val &= ~(IFX_STM_CMCON_MSIZE0_MASK | IFX_STM_CMCON_MSTART0_MASK);
  val |= (31u << IFX_STM_CMCON_MSIZE0_SHIFT);
  putreg32(val, IFX_STM_CMCON(g_core_id, IFX_STM_DEFAULT));

  /* Clear any pending compare interrupt */

  val = getreg32(IFX_STM_ISCR(g_core_id, IFX_STM_DEFAULT));
  val |= IFX_STM_ISCR_CMP0IRR;
  putreg32(val, IFX_STM_ISCR(g_core_id, IFX_STM_DEFAULT));

  /* Snapshot current time for tick accounting */

  g_last_ticks = stm_get_time64() / CYCLES_PER_TICK;
  g_last_count = g_last_ticks * CYCLES_PER_TICK;

  val  = getreg32(IFX_STM_ICR(g_core_id, IFX_STM_DEFAULT));
  val &= ~IFX_STM_ICR_CMP0OS;
  val |= IFX_STM_ICR_CMP0EN;
  putreg32(val, IFX_STM_ICR(g_core_id, IFX_STM_DEFAULT));

#ifdef CONFIG_DEBUG_FEATURES
  /* Freeze STM on debug halt */

  putreg32(IFX_STM_OCS_SUS_W, IFX_STM_OCS(g_core_id));
#endif

  /* Attach and enable the timer interrupt */

  irq_attach(stm_irq, (xcpt_t)tricore_timerisr, NULL);
  stm_set_compare((uint32_t)g_last_count + (CYCLES_PER_TICK * 5));
  up_enable_irq(stm_irq);
}
