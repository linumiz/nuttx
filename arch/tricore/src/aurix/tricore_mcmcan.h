/****************************************************************************
 * arch/tricore/src/aurix/tricore_mcmcan.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_TRICORE_SRC_TRICORE_MCMCAN_H
#define __ARCH_TRICORE_SRC_TRICORE_MCMCAN_H

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/can/can.h>
#include <nuttx/can/can_mcan.h>

#if defined(CONFIG_ARCH_CHIP_FAMILY_TC4XX)
#  include "hardware/tc4xx_mcanreg.h"
#elif defined(CONFIG_ARCH_CHIP_FAMILY_TC3XX)
#  include "hardware/tc3xx_mcanreg.h"
#endif

#define MCMCAN_MCR_MRAM_BUSY BIT(28)
#define MCMCAN_MCR_INIT_MRAM BIT(29)
#define MCMCAN_MCR_CLE       BIT(30)
#define MCMCAN_MCR_CLKSEL    BIT(31)

#define AURIX_CLC_DISR  BIT(0)   /* disable request */
#define AURIX_CLC_DISS  BIT(1)   /* disable status  */

struct mcmcan_node_pincfg
{
  bool     present;   /* node is wired on this board */
  uint32_t npcr_val;  /* value for NPCR (RX input alt select) */
};

struct mcmcan_config
{
  uintptr_t base;

  struct mcmcan_node_pincfg node_pin[4];
};

struct mcmcan_node_priv_s
{
  struct can_mcan_priv_s     mcan;         /* handle for can_mcan.c MUST remain the first member */
  struct can_mcan_config     mcan_config;
  struct can_mcan_data       mcan_data;

  uintptr_t  node_base;
  uintptr_t  mram_base;
  int        irq_line0;    /* bus/TX events IRQ         */
  int        irq_line1;    /* RX FIFO IRQ               */

  /* NuttX upper half */
  struct can_dev_s          *candev;

  int  rxfilter_std;
  int  rxfilter_ext;

  bool initialized;

  const struct mcmcan_config *module_config;
};

/****************************************************************************
 * Public init API
 ****************************************************************************/

/* Initialize the MCMCAN parent module (clock, MRAM clear, GRINT).
 * Must be called before any mcmcan_node_initialize().                */
int mcmcan_module_init(const struct mcmcan_config *config);

/* All hardware initialization (can_mcan_init, can_mcan_configure_mram,
 * irq_attach, RX filter install) is deferred to co_setup for the
 * NuttX upper-half lifecycle.
 */
void mcmcan_node_initialize(struct mcmcan_node_priv_s *np,
                            struct can_dev_s *candev);

#endif /* __ARCH_TRICORE_SRC_TRICORE_MCMCAN_H */
