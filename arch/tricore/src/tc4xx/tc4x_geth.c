/****************************************************************************
 * drivers/net/tc4x_geth.c
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
#include "tc4x_geth.h"

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

/****************************************************************************
 * PMS / HSPHY power-up
 ****************************************************************************/
static void pms_enable_supply(uintptr_t con, uintptr_t rst, uintptr_t stat)
{
  int i;

  xgmac_modreg(PMS_VMONP_OVE, PMS_VMONP_OVE, con);

  /* Wait for supply to settle — poll STAT.RESULT > STAT.RESETVAL */
  if (stat) {
    for (i = 0; i < 100; i++)
      {
        up_mdelay(1);
        uint32_t s = xgmac_read(stat);
        uint8_t result = (s >> 8) & 0xFF;
        uint8_t resetval = (s >> 0) & 0xFF;
        if (result > resetval)
          break;
      }
  }

  if (rst) {
    xgmac_modreg(PMS_VMONP_RESETOFF_P,
                 PMS_VMONP_RESETOFF | PMS_VMONP_RESETOFF_P,
                 rst);
  }
}

static void init_pms_supplies(void)
{
  pms_enable_supply(PMS_VMONP_VDDHSIFCON, PMS_VMONP_VDDHSIFRST,
                    PMS_VMONP_VDDHSIFSTAT);

  pms_enable_supply(PMS_VMONP_VDDPHY0CON, 0, PMS_VMONP_VDDPHY0STAT);
  pms_enable_supply(PMS_VMONP_VDDPHY1CON, 0, 0);
  pms_enable_supply(PMS_VMONP_VDDPHY2CON, 0, 0);
}

static int init_hsphy_for_rmii(int port)
{
  int i;

  /* Enable HSPHY clock */
  xgmac_modreg(0, BIT(0), TC4XX_HSPHY_CLC);
  for (i = 0; i < 100; i++)
    {
      if ((xgmac_read(TC4XX_HSPHY_CLC) & BIT(1)) == 0)
        break;
      up_mdelay(1);
    }

  if (i == 100)
    {
      nerr("HSPHY CLC enable timeout\n");
      return -EIO;
    }

  /* Kernel reset HSPHY */
  xgmac_modreg(BIT(31), BIT(31), TC4XX_HSPHY_RSTB);
  xgmac_modreg(BIT(0), BIT(0), TC4XX_HSPHY_RSTA);
  xgmac_modreg(BIT(0), BIT(0), TC4XX_HSPHY_RSTB);

  for (i = 0; i < 100; i++)
    {
      if (xgmac_read(TC4XX_HSPHY_RSTSTAT) & BIT(0))
        break;
      up_mdelay(1);
    }

  if (i == 100)
    {
      nerr("HSPHY kernel reset timeout\n");
      return -EIO;
    }

  xgmac_modreg(0, BIT(8), TC4XX_HSPHY_PHYCTRL1(port));
  up_udelay(25);
  xgmac_modreg(0, BIT(7), TC4XX_HSPHY_PHYCTRL1(port));

  xgmac_modreg(BIT(29), GENMASK(30,28), TC4XX_HSPHY_ETH(port)); // rmii
  xgmac_modreg(GENMASK(15,12), GENMASK(15,12), TC4XX_HSPHY_ETH(port)); // RXD0D, RXD1D
  xgmac_modreg(GENMASK(23,22), GENMASK(23,22),  TC4XX_HSPHY_ETH(port)); // REFCLKD
  xgmac_modreg(BIT(9), GENMASK(9,8),  TC4XX_HSPHY_ETH(port)); // CRSDVC 

  /* Select RMII/RGMII vs xSPI in CMNCFG: bit4 = 0 selects RGMII/RMII */
  xgmac_modreg(BIT(4), BIT(4), TC4XX_HSPHY_CMNCFG);

  uint32_t dll = xgmac_read(TC4XX_HSPHY_DLLCFG);
  dll |= BIT(31);  /* POWER on */
  dll |= BIT(24);  /* RX enable */
  dll |= BIT(23);  /* PMODE */
  dll |= BIT(22);  /* 3.3V mode */
  dll |= BIT(16);  /* TX enable */
  xgmac_write(dll, TC4XX_HSPHY_DLLCFG);

  return OK;
}

static int xgmac_mac_bridge_isr(int irq, FAR void *context, FAR void *arg)
{
  int32_t bridge_sts;

  bridge_sts = xgmac_read(TC4XX_BRIDGE_INTR_STATUS);

  if (!(bridge_sts & BIT(16)))
    return OK;  /* Not a DMA interrupt */

  dwxgmac_mac_isr(irq, context, arg);

  return OK;
}

static int init_geth(int port)
{
  int i;

  /* Enable GETH clock */
  xgmac_modreg(0, BIT(0), TC4XX_ETH_CLC);
  for (i = 0; i < 100; i++)
    {
      if ((xgmac_read(TC4XX_ETH_CLC) & BIT(1)) == 0)
        break;
      up_mdelay(1);
    }

  if (i == 100)
    return -EIO;

  /* Enable the MAC port */
  xgmac_modreg(BIT(port), BIT(port), TC4XX_ETH_MACEN);

  /* Kernel reset the GETH */
  xgmac_modreg(BIT(31), BIT(31), TC4XX_ETH_RST_CTRLB);
  xgmac_modreg(BIT(0), BIT(0), TC4XX_ETH_RST_CTRLA);
  xgmac_modreg(BIT(0), BIT(0), TC4XX_ETH_RST_CTRLB);

  for (i = 0; i < 100; i++)
    {
      if (xgmac_read(TC4XX_ETH_RST_STAT) & BIT(0))
        break;
      up_mdelay(1);
    }

  if (i == 100)
    {
      nerr("GETH kernel reset timeout\n");
      return -EIO;
    }

  xgmac_modreg(BIT(31), BIT(31), TC4XX_ETH_RST_CTRLB);

  /* Open all access protection registers */
  xgmac_write(0xFFFFFFFF, TC4XX_ETH_ACCEN_GBL_WRA);
  xgmac_write(0xFFFFFFFF, TC4XX_ETH_ACCEN_GBL_WRB);

  /* MAC0 port-specific access enable — was missing */
  xgmac_write(0xFFFFFFFF, TC4XX_ETH_ACCEN_MAC0_WRA);
  xgmac_write(0xFFFFFFFF, TC4XX_ETH_ACCEN_MAC0_WRB);

  for (i = 0; i < 8; i++)
    {
      xgmac_write(0xFFFFFFFF, TC4XX_ETH_ACCEN_CH_WRA(i));
      xgmac_write(0xFFFFFFFF, TC4XX_ETH_ACCEN_CH_WRB(i));
    }

  xgmac_write(0, TC4XX_BRIDGE_FWDCTRL);

  /* enable RX ch0  + TX ch0 */
  xgmac_write(BIT(8) | BIT(24), TC4XX_BRIDGE_PORT0_CTRL);

  return OK;
}

int tc4x_geth_init(int intf)
{
  int ret;

  init_pms_supplies();

  ret = init_hsphy_for_rmii(intf);
  if (ret < 0)
    return ret;

  ret = init_geth(intf);
  if (ret < 0)
    return ret;

  ret = xgmac_initialize(intf);
  if (ret < 0)
    return ret;

  ret = irq_attach(CONFIG_TRICORE_GETH_BRIDGE_IRQ, xgmac_mac_bridge_isr, (void *)intf);
  if (ret < 0)
    return ret;

  up_enable_irq(CONFIG_TRICORE_GETH_BRIDGE_IRQ);

  return OK;
}
