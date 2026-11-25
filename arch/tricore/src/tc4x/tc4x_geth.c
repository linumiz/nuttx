/****************************************************************************
 * arch/tricore/src/tc4x/tc4x_geth.c
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

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/net/dwxgmac.h>

#include "tricore_internal.h"
#include "tc4x_geth.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TC4X_GETH_TIMEOUT          100     /* Polls of 1 ms */
#define TC4X_HSPHY_PWRUP_US        25      /* PHY power up before reset */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void tc4x_geth_modify(uintptr_t reg, uint32_t clear, uint32_t set)
{
  putreg32((getreg32(reg) & ~clear) | set, reg);
}

/****************************************************************************
 * Name: tc4x_geth_wait
 *
 * Description:
 *   Wait until the bits of a register equal a value.
 *
 ****************************************************************************/

static int tc4x_geth_wait(uintptr_t reg, uint32_t mask, uint32_t value)
{
  int i;

  for (i = 0; i < TC4X_GETH_TIMEOUT; i++)
    {
      if ((getreg32(reg) & mask) == value)
        {
          return OK;
        }

      up_mdelay(1);
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: tc4x_geth_module_on
 *
 * Description:
 *   Clock a module and run its kernel reset.
 *
 ****************************************************************************/

static int tc4x_geth_module_on(uintptr_t clc, uintptr_t rsta,
                               uintptr_t rstb, uintptr_t rststat)
{
  int ret;

  tc4x_geth_modify(clc, TC4X_GETH_CLC_DISR, 0);
  ret = tc4x_geth_wait(clc, TC4X_GETH_CLC_DISS, 0);
  if (ret < 0)
    {
      return ret;
    }

  tc4x_geth_modify(rstb, 0, TC4X_GETH_RST_STATCLR);
  tc4x_geth_modify(rsta, 0, TC4X_GETH_RST_KRST);
  tc4x_geth_modify(rstb, 0, TC4X_GETH_RST_KRST);
  ret = tc4x_geth_wait(rststat, TC4X_GETH_RST_KRST, TC4X_GETH_RST_KRST);
  tc4x_geth_modify(rstb, 0, TC4X_GETH_RST_STATCLR);
  return ret;
}

/****************************************************************************
 * Name: tc4x_geth_supply
 *
 * Description:
 *   Enable the monitor of a supply, wait until the supply is above its
 *   reset level and release the reset of the monitor.
 *
 ****************************************************************************/

static void tc4x_geth_supply(uintptr_t con, uintptr_t rst, uintptr_t stat)
{
  uint32_t regval;
  int i;

  tc4x_geth_modify(con, 0, TC4X_PMS_VMON_OVE);

  if (stat != 0)
    {
      for (i = 0; i < TC4X_GETH_TIMEOUT; i++)
        {
          up_mdelay(1);
          regval = getreg32(stat);
          if (((regval & TC4X_PMS_VMON_RESULT) >> 8) >
              (regval & TC4X_PMS_VMON_RESETVAL))
            {
              break;
            }
        }
    }

  if (rst != 0)
    {
      tc4x_geth_modify(rst, TC4X_PMS_VMON_RESETOFF,
                       TC4X_PMS_VMON_RESETOFF_P);
    }
}

/****************************************************************************
 * Name: tc4x_geth_hsphy_rmii
 *
 * Description:
 *   Bring up the HSPHY and set a port to RMII with the board's input
 *   selection.
 *
 ****************************************************************************/

static int tc4x_geth_hsphy_rmii(int port,
                                FAR const struct tc4x_geth_rmii_s *rmii)
{
  uintptr_t eth = TC4X_HSPHY_ETH(port);
  int ret;

  ret = tc4x_geth_module_on(TC4X_HSPHY_CLC, TC4X_HSPHY_RST_CTRLA,
                            TC4X_HSPHY_RST_CTRLB, TC4X_HSPHY_RST_STAT);
  if (ret < 0)
    {
      return ret;
    }

  tc4x_geth_modify(TC4X_HSPHY_PHYCTRL1(port), TC4X_HSPHY_PHYCTRL1_PWRDWN, 0);
  up_udelay(TC4X_HSPHY_PWRUP_US);
  tc4x_geth_modify(TC4X_HSPHY_PHYCTRL1(port), TC4X_HSPHY_PHYCTRL1_RST, 0);

  tc4x_geth_modify(eth, TC4X_HSPHY_ETH_MODE, TC4X_HSPHY_ETH_MODE_RMII);
  tc4x_geth_modify(eth,
                   TC4X_HSPHY_ETH_CRSDV | TC4X_HSPHY_ETH_RXD0 |
                   TC4X_HSPHY_ETH_RXD1 | TC4X_HSPHY_ETH_REFCLK |
                   TC4X_HSPHY_ETH_MDIO,
                   ((uint32_t)rmii->crsdv << 8) |
                   ((uint32_t)rmii->rxd0 << 12) |
                   ((uint32_t)rmii->rxd1 << 14) |
                   ((uint32_t)rmii->refclk << 22) |
                   rmii->mdio | TC4X_HSPHY_ETH_MDIOEN);
  tc4x_geth_modify(TC4X_HSPHY_CMNCFG, 0, TC4X_HSPHY_CMNCFG_FSR);
  return OK;
}

/****************************************************************************
 * Name: tc4x_geth_lmu_access
 *
 * Description:
 *   The DMA descriptors and buffers of the boards are in LMU0, whose
 *   access protection only lets the CPU masters write after reset.  Give
 *   the GETH DMA masters write access to region 0 of LMU0 too.
 *
 ****************************************************************************/

static int tc4x_geth_lmu_access(void)
{
  uint32_t state = getreg32(TC4X_LMU0_PROTRGN) & TC4X_LMU_PROTRGN_STATE;

  if (state != TC4X_LMU_PROT_INIT && state != TC4X_LMU_PROT_CONFIG)
    {
      return -EACCES;
    }

  tc4x_geth_modify(TC4X_LMU0_PROTRGN, TC4X_LMU_PROTRGN_SEL, 0);
  tc4x_geth_modify(TC4X_LMU0_ACCENCFG_WRB, 0, TC4X_LMU_WRB_GETH);
  return OK;
}

/****************************************************************************
 * Name: tc4x_geth_attach
 ****************************************************************************/

static int tc4x_geth_attach(int irq)
{
  int ret;

  ret = irq_attach(irq, dwxgmac_interrupt, (FAR void *)0);
  if (ret < 0)
    {
      return ret;
    }

  up_enable_irq(irq);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc4x_geth_initialize
 ****************************************************************************/

int tc4x_geth_initialize(int port, FAR const struct tc4x_geth_rmii_s *rmii)
{
  struct dwxgmac_config_s config =
  {
    0
  };

  int ret;
  int i;

  if (port != 0 || rmii == NULL)
    {
      return -EINVAL;
    }

  tc4x_geth_supply(TC4X_PMS_VDDHSIFCON, TC4X_PMS_VDDHSIFRST,
                   TC4X_PMS_VDDHSIFSTAT);
  tc4x_geth_supply(TC4X_PMS_VDDPHY0CON, 0, TC4X_PMS_VDDPHY0STAT);
  tc4x_geth_supply(TC4X_PMS_VDDPHY1CON, 0, 0);
  tc4x_geth_supply(TC4X_PMS_VDDPHY2CON, 0, 0);

  ret = tc4x_geth_hsphy_rmii(port, rmii);
  if (ret < 0)
    {
      nerr("ERROR: HSPHY bring up failed: %d\n", ret);
      return ret;
    }

  ret = tc4x_geth_module_on(TC4X_GETH_CLC, TC4X_GETH_RST_CTRLA,
                            TC4X_GETH_RST_CTRLB, TC4X_GETH_RST_STAT);
  if (ret < 0)
    {
      nerr("ERROR: GETH bring up failed: %d\n", ret);
      return ret;
    }

  tc4x_geth_modify(TC4X_GETH_MACEN, 0, 1 << port);

  /* Write access to the GETH for all masters */

  putreg32(0xffffffff, TC4X_GETH_ACCEN_GBL_WRA);
  putreg32(0xffffffff, TC4X_GETH_ACCEN_GBL_WRB);
  putreg32(0xffffffff, TC4X_GETH_ACCEN_MAC_WRA(port));
  putreg32(0xffffffff, TC4X_GETH_ACCEN_MAC_WRB(port));

  for (i = 0; i < TC4X_GETH_NDMACH; i++)
    {
      putreg32(0xffffffff, TC4X_GETH_ACCEN_CH_WRA(i));
      putreg32(0xffffffff, TC4X_GETH_ACCEN_CH_WRB(i));
    }

  ret = tc4x_geth_lmu_access();
  if (ret < 0)
    {
      nerr("ERROR: No GETH write access to LMU0: %d\n", ret);
      return ret;
    }

  config.base = TC4X_GETH_MAC_BASE(port);
  config.dmabase = TC4X_GETH_DMA_BASE;

  ret = dwxgmac_initialize(0, &config);
  if (ret < 0)
    {
      return ret;
    }

  /* All DMA channels and queues on port 0, no loopback or flush */

  putreg32(0, TC4X_GETH_BRIDGE_DMASEL);
  putreg32(TC4X_GETH_BRIDGE_PORT0_ALL, TC4X_GETH_BRIDGE_PORT0);
  putreg32(0, TC4X_GETH_BRIDGE_FLUSH);

  /* The TX and the RX side of each DMA channel raise their own interrupt,
   * all of them served by the interrupt handler of the driver.
   */

  for (i = 0; i < CONFIG_NET_DWXGMAC_NTXCHANNELS; i++)
    {
      ret = tc4x_geth_attach(TC4X_GETH_TX_IRQ(i));
      if (ret < 0)
        {
          return ret;
        }
    }

  for (i = 0; i < CONFIG_NET_DWXGMAC_NRXCHANNELS; i++)
    {
      ret = tc4x_geth_attach(TC4X_GETH_RX_IRQ(i));
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}
