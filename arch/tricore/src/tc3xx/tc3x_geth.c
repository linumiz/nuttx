/****************************************************************************
 * arch/tricore/src/tc3xx/tc3x_geth.c
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
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/bits.h>
#include <nuttx/arch.h>
#include <nuttx/net/mdio.h>

#include "tricore_internal.h"
#include "tc3x_geth.h"

int xgmac_initialize(int intf);
int dwxgmac_mac_isr(int irq, FAR void *context, FAR void *arg);

uint32_t xgmac_read(uintptr_t reg)
{
  return getreg32(reg);
}

void xgmac_write(uint32_t value, uintptr_t reg)
{
  putreg32(value, reg);
}

void xgmac_modreg(uint32_t value, uint32_t mask, uintptr_t reg)
{
  modreg32(value, mask, reg);
}

static int init_phy_mode_input_cfg(int port)
{
  /* phy in rgmii mode */
  xgmac_modreg(TC3XX_ETH_PHY_MODE(1), TC3XX_ETH_PHY_MODE_MASK, TC3XX_ETH_GPCTL);

  /* mdio pins in alt1 */
  xgmac_modreg(BIT(1), GENMASK(1, 0), TC3XX_ETH_GPCTL);
  /* all pins in alt0 */
  xgmac_modreg(0, GENMASK(10, 2), TC3XX_ETH_GPCTL);
}

static int init_geth(int port)
{
  int i;

  /* Enable GETH clock */
  aurix_cpu_endinit_enable(false);
  xgmac_modreg(0, BIT(0), TC3XX_ETH_CLC);
  aurix_cpu_endinit_enable(true);
  for (i = 0; i < 100; i++)
    {
      if ((xgmac_read(TC3XX_ETH_CLC) & BIT(1)) == 0)
        break;
      up_mdelay(1);
    }

  if (i == 100)
    return -EIO;

  aurix_cpu_endinit_enable(false);

  /* Kernel reset the GETH */
  xgmac_modreg(BIT(0), BIT(0), TC3XX_ETH_RST_CTRLA);
  xgmac_modreg(BIT(0), BIT(0), TC3XX_ETH_RST_CTRLB);

  for (i = 0; i < 100; i++)
    {
      if (xgmac_read(TC3XX_ETH_RST_CTRLA) & BIT(1))
        break;
      up_mdelay(1);
    }

  xgmac_modreg(BIT(0), BIT(0), TC3XX_ETH_RST_STAT);
  aurix_cpu_endinit_enable(true);

  if (i == 100)
    {
      nerr("GETH kernel reset timeout\n");
      return -EIO;
    }

  up_mdelay(1);

  /* Open all access protection registers */
  xgmac_write(0xFFFFFFFF, TC3XX_ETH_ACCEN0);
  xgmac_write(0xFFFFFFFF, TC3XX_ETH_ACCEN1);

  for (i = 0; i < 3; i++)
    {
      xgmac_write(0xFFFFFFFF, TC3XX_ETH_ACCEN_CH_WRA(i));
      xgmac_write(0xFFFFFFFF, TC3XX_ETH_ACCEN_CH_WRB(i));
    }

  return OK;
}

int tc3x_geth_init(int intf)
{
  int ret;

  /* mode must be set before reset */
  ret = init_phy_mode_input_cfg(intf);
  if (ret < 0)
    return ret;

  ret = init_geth(intf);
  if (ret < 0)
    return ret;

  ret = xgmac_initialize(intf);
  if (ret < 0)
    return ret;

  return OK;
}
