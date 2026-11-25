/****************************************************************************
 * boards/tricore/tc4x/a3g-tc4d7-lite/src/tc4d7_ethernet.c
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
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/dwxgmac.h>
#include <nuttx/net/mdio.h>
#include <nuttx/net/mii.h>

#include "tc4x_geth.h"
#include "tc4d7.h"

#ifdef CONFIG_TC4X_GETH

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DP83825I PHY on MDIO address 0 */

#define TC4D7_PHY_ADDR        0
#define TC4D7_PHY_RESET_MS    100
#define TC4D7_PHY_POLL        SEC2TICK(1)

#ifndef CONFIG_SCHED_LPWORK
#  error The PHY link poll of the Ethernet needs CONFIG_SCHED_LPWORK
#endif

/* RMII pins: TXD0, TXD1 and TXEN on P16.6, P16.8 and P16.13, alternate
 * output 10; RXD1, CRS_DV, REFCLK and RXD0 on P16.0, P16.1, P16.2 and
 * P16.4; MDC on P21.2, alternate output 8, and MDIO on P21.3.
 */

#define GPIO_ETH_TXD0         (GPIO_PORT(16) | GPIO_PIN(6) | GPIO_OUTPUT | \
                               GPIO_ALT(10))
#define GPIO_ETH_TXD1         (GPIO_PORT(16) | GPIO_PIN(8) | GPIO_OUTPUT | \
                               GPIO_ALT(10))
#define GPIO_ETH_TXEN         (GPIO_PORT(16) | GPIO_PIN(13) | GPIO_OUTPUT | \
                               GPIO_ALT(10))
#define GPIO_ETH_RXD1         (GPIO_PORT(16) | GPIO_PIN(0) | GPIO_INPUT)
#define GPIO_ETH_CRSDV        (GPIO_PORT(16) | GPIO_PIN(1) | GPIO_INPUT)
#define GPIO_ETH_REFCLK       (GPIO_PORT(16) | GPIO_PIN(2) | GPIO_INPUT)
#define GPIO_ETH_RXD0         (GPIO_PORT(16) | GPIO_PIN(4) | GPIO_INPUT)
#define GPIO_ETH_MDC          (GPIO_PORT(21) | GPIO_PIN(2) | GPIO_OUTPUT | \
                               GPIO_ALT(8))
#define GPIO_ETH_MDIO         (GPIO_PORT(21) | GPIO_PIN(3) | GPIO_INPUT)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* CRS_DV is input C, the other RMII inputs and MDIO are input D */

static const struct tc4x_geth_rmii_s g_tc4d7_rmii =
{
  .crsdv  = 2,
  .rxd0   = 3,
  .rxd1   = 3,
  .refclk = 3,
  .mdio   = 3,
};

static const uint32_t g_tc4d7_ethpins[] =
{
  GPIO_ETH_TXD0, GPIO_ETH_TXD1, GPIO_ETH_TXEN, GPIO_ETH_RXD1,
  GPIO_ETH_CRSDV, GPIO_ETH_REFCLK, GPIO_ETH_RXD0, GPIO_ETH_MDC,
  GPIO_ETH_MDIO
};

static FAR struct mdio_bus_s *g_tc4d7_mdio;
static struct work_s g_tc4d7_phywork;
static bool g_tc4d7_link;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc4d7_phy_poll
 *
 * Description:
 *   Report the link state and speed of the PHY to the MAC when they
 *   change, once a second.
 *
 ****************************************************************************/

static void tc4d7_phy_poll(FAR void *arg)
{
  uint16_t msr = 0;
  uint16_t physts = 0;
  bool link;

  mdio_read(g_tc4d7_mdio, TC4D7_PHY_ADDR, MII_MSR, &msr);
  link = (msr & MII_MSR_LINKSTATUS) != 0;

  if (link != g_tc4d7_link)
    {
      int speed = 100;

      if (link)
        {
          mdio_read(g_tc4d7_mdio, TC4D7_PHY_ADDR, MII_DP83825I_PHYSTS,
                    &physts);
          if ((physts & MII_DP83825I_PHYSTS_SPEED) != 0)
            {
              speed = 10;
            }

          ninfo("Link up, %d Mbps %s duplex\n", speed,
                (physts & MII_DP83825I_PHYSTS_DUPLEX) != 0 ?
                "full" : "half");
        }
      else
        {
          ninfo("Link down\n");
        }

      g_tc4d7_link = link;
      dwxgmac_carrier(0, link, speed);
    }

  work_queue(LPWORK, &g_tc4d7_phywork, tc4d7_phy_poll, NULL,
             TC4D7_PHY_POLL);
}

/****************************************************************************
 * Name: tc4d7_phy_initialize
 *
 * Description:
 *   Reset the PHY, advertise 10 and 100 Mbps and start the
 *   autonegotiation.
 *
 ****************************************************************************/

static int tc4d7_phy_initialize(void)
{
  uint16_t mcr = MII_MCR_RESET;
  int ret;
  int i;

  ret = mdio_write(g_tc4d7_mdio, TC4D7_PHY_ADDR, MII_MCR, MII_MCR_RESET);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < TC4D7_PHY_RESET_MS && (mcr & MII_MCR_RESET) != 0; i++)
    {
      up_mdelay(1);
      ret = mdio_read(g_tc4d7_mdio, TC4D7_PHY_ADDR, MII_MCR, &mcr);
      if (ret < 0)
        {
          return ret;
        }
    }

  if ((mcr & MII_MCR_RESET) != 0)
    {
      return -ETIMEDOUT;
    }

  /* Receive elasticity buffer of 2 bits, as for a 50 MHz reference clock
   * from the board
   */

  mdio_write(g_tc4d7_mdio, TC4D7_PHY_ADDR, MII_DP83825I_RCSR,
             MII_DP83825I_RCSC_ELAST_2);
  mdio_write(g_tc4d7_mdio, TC4D7_PHY_ADDR, MII_ADVERTISE,
             MII_ADVERTISE_100BASETXFULL | MII_ADVERTISE_100BASETXHALF |
             MII_ADVERTISE_10BASETXFULL | MII_ADVERTISE_10BASETXHALF |
             MII_ADVERTISE_CSMA);
  return mdio_write(g_tc4d7_mdio, TC4D7_PHY_ADDR, MII_MCR,
                    MII_MCR_ANENABLE | MII_MCR_ANRESTART);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc4d7_ethernet_initialize
 *
 * Description:
 *   Set up the RMII pins, the GETH and the PHY, and start the link poll.
 *
 ****************************************************************************/

int tc4d7_ethernet_initialize(void)
{
  int ret;
  int i;

  for (i = 0; i < sizeof(g_tc4d7_ethpins) / sizeof(g_tc4d7_ethpins[0]); i++)
    {
      tricore_gpio_config(g_tc4d7_ethpins[i]);
    }

  ret = tc4x_geth_initialize(0, &g_tc4d7_rmii);
  if (ret < 0)
    {
      return ret;
    }

  g_tc4d7_mdio = dwxgmac_mdio_initialize(0, TC4X_GETH_MAC_BASE(0));
  if (g_tc4d7_mdio == NULL)
    {
      return -ENODEV;
    }

  ret = tc4d7_phy_initialize();
  if (ret < 0)
    {
      nerr("ERROR: PHY initialization failed: %d\n", ret);
      return ret;
    }

  return work_queue(LPWORK, &g_tc4d7_phywork, tc4d7_phy_poll, NULL, 0);
}

#endif /* CONFIG_TC4X_GETH */
