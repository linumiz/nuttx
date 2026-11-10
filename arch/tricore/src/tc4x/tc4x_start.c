/****************************************************************************
 * arch/tricore/src/tc4x/tc4x_start.c
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

#include <stdbool.h>
#include <stdint.h>

#include <arch/arch.h>

#include "tricore_internal.h"
#include "hardware/tc4x_clock.h"
#include "hardware/tc4x_cpu.h"
#include "hardware/tc4x_vmt.h"
#include "tricore_stm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* STM of CPU0 */

#define TC4X_STM0_ABS   (TRICORE_STM0_BASE + TRICORE_STM_ABS_OFFSET)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Provided by the linker script */

extern uintptr_t __START1[];
extern uintptr_t __START2[];
extern uintptr_t __START3[];
extern uintptr_t __START4[];
extern uintptr_t __START5[];

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SSH (MBIST controller) selectors of the RAMs that tricore_ram_init
 * clears: the boot firmware initializes the CPU RAMs and the DLMUs
 * (UCB0_RAMINIT), but not the LMU0-9 and MCAN0-4 RAMs.
 */

static const uint8_t g_tc4x_sram[] =
{
  0x17, 0x18, 0x20, 0x21, 0x19, 0x1a, 0x1b, 0x22, 0x23, 0x24, /* LMU0-9 */
  0xa2, 0xa3, 0xa4, 0xa5, 0xa6                                /* MCAN0-4 */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc4x_sram_clear
 *
 * Description:
 *   Fill a whole SRAM with ECC correct zeros through its SSH, with
 *   MCONTROL.SRAM_CLR, as IfxVmt_clearSram does.
 *
 ****************************************************************************/

static void tc4x_sram_clear(uint32_t sel)
{
  uintptr_t vmt = TC4X_VMT_BASE + (sel >> 5) * TC4X_VMT_SIZE;
  uintptr_t mc = vmt + TC4X_VMT_MC(sel % 32);
  uint32_t mask = 1 << (sel % 32);

  if ((getreg32(vmt + TC4X_VMT_CLC) & TC4X_VMT_CLC_DISS) != 0)
    {
      putreg32(0, vmt + TC4X_VMT_CLC);
    }

  /* Enable the MBIST shell and start the clear */

  putreg32(getreg32(mc + TC4X_VMT_MC_ALMSRCS) & ~TC4X_ALMSRCS_ALARMS,
           mc + TC4X_VMT_MC_ALMSRCS);
  putreg32(getreg32(vmt + TC4X_VMT_MEMTEST) | mask,
           vmt + TC4X_VMT_MEMTEST);
  while ((getreg32(vmt + TC4X_VMT_MEMSTAT) & mask) != 0);

  putreg32((1 << 15) | (1 << 0), mc + TC4X_VMT_MC_MCONTROL);
  putreg32(getreg32(mc + TC4X_VMT_MC_MCONTROL) & ~(1 << 0),
           mc + TC4X_VMT_MC_MCONTROL);
  while ((getreg32(vmt + TC4X_VMT_MEMDONE) & mask) == 0);

  /* Read address 0 once, then disable the MBIST shell again */

  putreg32(1 << 3, mc + TC4X_VMT_MC_MCONTROL);
  putreg32((1 << 12) | (1 << 0), mc + TC4X_VMT_MC_CONFIG0);
  putreg32(0, mc + TC4X_VMT_MC_CONFIG1);
  putreg32(0, mc + TC4X_VMT_MC_RANGE);
  putreg32((1 << 3) | (1 << 0), mc + TC4X_VMT_MC_MCONTROL);
  putreg32(1 << 3, mc + TC4X_VMT_MC_MCONTROL);
  while ((getreg32(vmt + TC4X_VMT_MEMDONE) & mask) == 0);

  putreg32(getreg32(vmt + TC4X_VMT_MEMTEST) & ~mask,
           vmt + TC4X_VMT_MEMTEST);
  while ((getreg32(vmt + TC4X_VMT_MEMSTAT) & mask) != 0);
  putreg32(getreg32(mc + TC4X_VMT_MC_ALMSRCS) | TC4X_ALMSRCS_ALARMS,
           mc + TC4X_VMT_MC_ALMSRCS);
}

/****************************************************************************
 * Name: tc4x_stm_frequency
 *
 * Description:
 *   STM frequency as the iLLD startup code computes it for its delays.
 *
 ****************************************************************************/

static uint32_t tc4x_stm_frequency(void)
{
  uint32_t insel = (getreg32(TC4X_CLOCK_OSCCON) &
                    TC4X_OSCCON_INSEL_MASK) >> TC4X_OSCCON_INSEL_SHIFT;
  uint32_t clksels = getreg32(TC4X_CLOCK_CCUSTAT) & TC4X_CCU_CLKSELS_MASK;
  uint32_t con0 = getreg32(TC4X_CLOCK_SYSPLLCON0);
  uint32_t stat = getreg32(TC4X_CLOCK_RAMPSTAT);
  uint32_t fstat = (stat & TC4X_RAMPSTAT_FSTAT_MASK) >>
                   TC4X_RAMPSTAT_FSTAT_SHIFT;
  uint32_t osc;
  uint32_t freq;

  osc = insel == 2 ? 20000000 : insel == 1 ? 25000000 : 100000000;
  if (clksels == TC4X_CCU_CLKSELS_PLL)
    {
      uint32_t ndiv = (con0 & TC4X_PLLCON0_NDIV_MASK) >>
                      TC4X_PLLCON0_NDIV_SHIFT;
      uint32_t pdiv = (con0 & TC4X_PLLCON0_PDIV_MASK) >>
                      TC4X_PLLCON0_PDIV_SHIFT;
      uint32_t k2div = getreg32(TC4X_CLOCK_SYSPLLCON1) &
                       TC4X_PLLCON1_K2DIV_MASK;

      freq = (uint32_t)((float)osc * (ndiv + 1) /
                        ((pdiv + 1) * (k2div + 1)) + 0.5f);
    }
  else if (clksels == TC4X_CCU_CLKSELS_RAMP)
    {
      freq = 0;
      if ((stat & TC4X_RAMPSTAT_SSTAT_MASK) == 0)
        {
          if (fstat == 2)
            {
              freq = (getreg32(TC4X_CLOCK_RAMPCON0) &
                      TC4X_RAMPCON0_UFL_MASK) * 1000000;
            }
          else if (fstat == 1)
            {
              freq = 100000000;
            }
        }
    }
  else
    {
      freq = 100000000;
    }

  return freq / ((getreg32(TC4X_CLOCK_SYSCCUCON0) &
                  TC4X_SYSCCUCON0_STMDIV_MASK) >>
                 TC4X_SYSCCUCON0_STMDIV_SHIFT);
}

/****************************************************************************
 * Name: tc4x_delay
 *
 * Description:
 *   Wait 100 us on the STM between the starts of the cores.
 *
 ****************************************************************************/

static void tc4x_delay(void)
{
  uint32_t ticks = tc4x_stm_frequency() / 10000;
  uint32_t start = getreg32(TC4X_STM0_ABS);

  while (getreg32(TC4X_STM0_ABS) - start < ticks);
}

/****************************************************************************
 * Name: tc4x_start_core
 ****************************************************************************/

static void tc4x_start_core(int cpu, uintptr_t pc)
{
  putreg32((getreg32(TC4X_CPU_HRA_PC(cpu)) & 1) | (pc & ~1),
           TC4X_CPU_HRA_PC(cpu));
  if ((getreg32(TC4X_CPU_HRA_BOOTCON(cpu)) & TC4X_BOOTCON_BHALT) != 0)
    {
      putreg32(getreg32(TC4X_CPU_HRA_BOOTCON(cpu)) & ~TC4X_BOOTCON_BHALT,
               TC4X_CPU_HRA_BOOTCON(cpu));
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_ram_init
 *
 * Description:
 *   Initialize the RAMs that the boot firmware leaves uninitialized, so
 *   that they can be read before they are written.  None of them holds the
 *   stack, the CSAs, .data or .bss.
 *
 ****************************************************************************/

void tricore_ram_init(void)
{
  int i;

  for (i = 0; i < sizeof(g_tc4x_sram); i++)
    {
      tc4x_sram_clear(g_tc4x_sram[i]);
    }
}

/****************************************************************************
 * Name: tricore_start_cores
 *
 * Description:
 *   Release CPU1 to CPU5 at the start of their own images, 100 us apart, as
 *   the iLLD startup code does.
 *
 ****************************************************************************/

void tricore_start_cores(void)
{
  tc4x_start_core(1, (uintptr_t)__START1);
  tc4x_delay();
  tc4x_start_core(2, (uintptr_t)__START2);
  tc4x_delay();
  tc4x_start_core(3, (uintptr_t)__START3);
  tc4x_delay();
  tc4x_start_core(4, (uintptr_t)__START4);
  tc4x_delay();
  tc4x_start_core(5, (uintptr_t)__START5);
}
