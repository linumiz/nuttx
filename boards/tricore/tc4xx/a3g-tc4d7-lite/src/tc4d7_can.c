/****************************************************************************
 * boards/tricore/tc4xx/a3g-tc4d7-lite/src/tc4d7_can.c
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
#include <stdbool.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>

#include "tc4xx_gpio.h"
#include "tricore_mcmcan.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
//TODO all in Kconfig
#define CONFIG_MCMCAN_BASE_ADDR             0xF4710000
#define CONFIG_MCMCAN_NODE0_BASE_ADDR       0xF4710200
#define CONFIG_MCMCAN_NODE0_MRAM_ADDR       0xF4700000
#define CONFIG_MCMCAN_NODE0_MRAM_SIZE       0x9000

#define CONFIG_MCMCAN_NODE0_IRQ_LINE0       788
// TC4X has one ISR line for TX & RX
#define CONFIG_MCMCAN_NODE0_IRQ_LINE1       0

#define CONFIG_MCMCAN_NODE0_BITRATE         875
#define CONFIG_MCMCAN_NODE0_SAMPLE_POINT    875
#define CONFIG_MCMCAN_NODE0_STD_FILTERS     32
#define CONFIG_MCMCAN_NODE0_EXT_FILTERS     32
#define CONFIG_MCMCAN_NODE0_RX_FIFO0        32
#define CONFIG_MCMCAN_NODE0_RX_FIFO1        32
#define CONFIG_MCMCAN_NODE0_TX_EVENTS       16
#define CONFIG_MCMCAN_NODE0_TX_SLOTS        16
#define CONFIG_MCMCAN_NODE0_NPCR            0x2

static struct can_dev_s g_node0_candev;

CAN_MCAN_CALLBACKS_DEFINE(g_node0_cbs,
                           CONFIG_MCMCAN_NODE0_TX_SLOTS,
                           CONFIG_MCMCAN_NODE0_STD_FILTERS,
                           CONFIG_MCMCAN_NODE0_EXT_FILTERS);

static const struct mcmcan_config g_mcmcan_module =
{
  .base       = CONFIG_MCMCAN_BASE_ADDR,
  .node_pin   =
    {
      [0] = { .present = true,  .npcr_val = CONFIG_MCMCAN_NODE0_NPCR },
      [1] = { .present = false, .npcr_val = 0 },
      [2] = { .present = false, .npcr_val = 0 },
      [3] = { .present = false, .npcr_val = 0 },
    },
};

static struct mcmcan_node_priv_s g_node0_priv =
{
  .node_base     = CONFIG_MCMCAN_NODE0_BASE_ADDR,
  .mram_base     = CONFIG_MCMCAN_NODE0_MRAM_ADDR,
  .irq_line0     = CONFIG_MCMCAN_NODE0_IRQ_LINE0,
  .irq_line1     = CONFIG_MCMCAN_NODE0_IRQ_LINE1,
  .module_config = &g_mcmcan_module,
  .initialized   = false,

  .mcan_config =
    {
      .callbacks    = &g_node0_cbs,
      .bitrate      = CONFIG_MCMCAN_NODE0_BITRATE,
      .sample_point = CONFIG_MCMCAN_NODE0_SAMPLE_POINT,
#ifdef CONFIG_CAN_FD_MODE
      .bitrate_data      = CONFIG_MCMCAN_NODE0_BITRATE_DATA,
      .sample_point_data = CONFIG_MCMCAN_NODE0_SAMPLE_POINT_DATA,
#endif
      .phy_present  = false,
      .mram_elements =
        {
          0,
          CONFIG_MCMCAN_NODE0_STD_FILTERS,
          CONFIG_MCMCAN_NODE0_EXT_FILTERS,
          CONFIG_MCMCAN_NODE0_RX_FIFO0,
          CONFIG_MCMCAN_NODE0_RX_FIFO1,
          0,
          CONFIG_MCMCAN_NODE0_TX_EVENTS,
          CONFIG_MCMCAN_NODE0_TX_SLOTS,
        },
      /* mram_offsets[] and mram_size are computed at runtime by
       * mcmcan_compute_mram_layout() in mcmcan_hw_init_once().       */
    },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void tc4d7_config_can_pins(void)
{
  gpio_pinset_t pins[] =
    {
      /* CAN0 TX outputs */
      AURIX_GPIO_FULL(GPIO_PORT1, GPIO_PIN3, GPIO_PERIPH, GPIO_ALT5,
      0, 0, 0, 0, GPIO_PERIPH_OWN_PAD),

      /* CAN0 RX inputs */
      AURIX_GPIO_FULL(GPIO_PORT1, GPIO_PIN4, GPIO_INPUT, GPIO_PULL_NONE,
      0, 0, 0, 0, GPIO_PERIPH_OWN_PAD),
    };

  for (int i = 0; i < (int)(sizeof(pins) / sizeof(pins[0])); i++)
    {
      aurix_config_gpio(pins[i]);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mcmcan_board_initialize(void)
{
  int err;

  tc4d7_config_can_pins();

  err = mcmcan_module_init(&g_mcmcan_module);
  if (err != 0)
  {
    return err;
  }

  mcmcan_node_initialize(&g_node0_priv, &g_node0_candev);

  return can_register("/dev/can0", &g_node0_candev);
}
