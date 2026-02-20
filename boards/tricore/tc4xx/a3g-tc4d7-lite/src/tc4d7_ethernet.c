/****************************************************************************
 * boards/tricore/tc4dx/tc4d7-lite/src/tc4d7_ethernet.c
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
#include <nuttx/net/mii.h>
#include <nuttx/net/mdio.h>

#include "tc4xx_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DP83825I PHY configuration */
#define BOARD_PHY_NAME        "DP83825I"
#define BOARD_PHY_ADDR        0           /* PHY address on MDIO bus */
#define BOARD_PHYID1          MII_PHYID1_DP83825I
#define BOARD_PHYID2          MII_PHYID2_DP83825I

#define PHY_RESET_TIMEOUT_MS  100
#define PHY_LINK_TIMEOUT      0x4FFFF
#define PHY_ANEG_TIMEOUT      0x4FFFF

struct mdio_bus_s *dwxgmac_mdio_bus_initialize(uint8_t port, uintptr_t base);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void tc4d7_config_geth_pins(void)
{
  gpio_pinset_t pins[] =
    {
      /* RMII TX outputs */
      AURIX_GPIO(GPIO_PORT16, GPIO_PIN6, GPIO_PERIPH, GPIO_ALT10),
      AURIX_GPIO(GPIO_PORT16, GPIO_PIN8, GPIO_PERIPH, GPIO_ALT10),
      AURIX_GPIO(GPIO_PORT16, GPIO_PIN13, GPIO_PERIPH, GPIO_ALT10),

      /* RMII RX inputs */
      AURIX_GPIO(GPIO_PORT16, GPIO_PIN4, GPIO_INPUT, GPIO_PULL_NONE),
      AURIX_GPIO(GPIO_PORT16, GPIO_PIN0, GPIO_INPUT, GPIO_PULL_NONE),
      AURIX_GPIO(GPIO_PORT16, GPIO_PIN1, GPIO_INPUT, GPIO_PULL_NONE),
      AURIX_GPIO(GPIO_PORT16, GPIO_PIN2, GPIO_INPUT, GPIO_PULL_NONE),

      /* MDIO */
      AURIX_GPIO(GPIO_PORT21, GPIO_PIN2, GPIO_PERIPH, GPIO_ALT8),
      AURIX_GPIO(GPIO_PORT21, GPIO_PIN3, GPIO_INPUT, GPIO_PULL_NONE),
    };

  for (int i = 0; i < (int)(sizeof(pins) / sizeof(pins[0])); i++)
    {
      aurix_config_gpio(pins[i]);
    }
}

static int tc4d7_init_phy(struct mdio_bus_s *mdio)
{
  uint8_t phyaddr = BOARD_PHY_ADDR;
  uint16_t phyval;
  int ret;
  int to;
#ifdef CONFIG_NET_AUTONEG
  volatile uint32_t timeout;
#endif

  ret = mdio_write(mdio, phyaddr, MII_MCR, MII_MCR_RESET);
  if (ret < 0)
    {
      nerr("ERROR: PHY reset write failed: %d\n", ret);
      return ret;
    }

  for (to = PHY_RESET_TIMEOUT_MS; to > 0; to -= 10)
    {
      up_mdelay(10);
      phyval = 0xFFFF;
      ret = mdio_read(mdio, phyaddr, MII_MCR, &phyval);
      if (ret < 0)
        {
          nerr("ERROR: PHY MCR read failed: %d\n", ret);
          return ret;
        }

      if ((phyval & MII_MCR_RESET) == 0)
        break;
    }

  if (to <= 0)
    {
      nerr("ERROR: PHY reset timeout\n");
      return -ETIMEDOUT;
    }

  ninfo("PHY reset complete in %d ms\n", PHY_RESET_TIMEOUT_MS - to);
  ret = mdio_read(mdio, phyaddr, MII_PHYID1, &phyval);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read PHYID1: %d\n", ret);
      return ret;
    }

  if (phyval != BOARD_PHYID1)
    {
      nerr("ERROR: Wrong PHYID1: 0x%04x expected 0x%04x\n",
           phyval, BOARD_PHYID1);
      return -ENXIO;
    }

  ninfo("DP83825I detected, PHYID1=0x%04x\n", phyval);

  /*  Configure RMII mode:
   */
  mdio_write(mdio, phyaddr, MII_DP83825I_RCSR,
             MII_DP83825I_RCSC_ELAST_2 | MII_DP83825I_RCSC_RMIICS);

  mdio_write(mdio, phyaddr, MII_ADVERTISE,
             MII_ADVERTISE_100BASETXFULL |
             MII_ADVERTISE_100BASETXHALF |
             MII_ADVERTISE_10BASETXFULL  |
             MII_ADVERTISE_10BASETXHALF  |
             MII_ADVERTISE_CSMA);

#ifdef CONFIG_NET_AUTONEG

  ninfo("%s: Waiting for link...\n", BOARD_PHY_NAME);

  for (timeout = 0; timeout < PHY_LINK_TIMEOUT; timeout++)
    {
      ret = mdio_read(mdio, phyaddr, MII_MSR, &phyval);
      if (ret < 0)
        {
          nerr("ERROR: PHY MSR read failed: %d\n", ret);
          return ret;
        }

      if (phyval & MII_MSR_LINKSTATUS)
        break;

      nxsig_usleep(100);
    }

  if (timeout >= PHY_LINK_TIMEOUT)
    {
      nerr("ERROR: Link timeout (MSR=0x%04x)\n", phyval);
      return -ETIMEDOUT;
    }

  ninfo("Link up (MSR=0x%04x)\n", phyval);

  ret = mdio_write(mdio, phyaddr, MII_MCR, MII_MCR_ANENABLE);
  if (ret < 0)
    {
      nerr("ERROR: Failed to enable autoneg: %d\n", ret);
      return ret;
    }

  /* Wait for autoneg complete */
  for (timeout = 0; timeout < PHY_ANEG_TIMEOUT; timeout++)
    {
      ret = mdio_read(mdio, phyaddr, MII_MSR, &phyval);
      if (ret < 0)
        return ret;

      if (phyval & MII_MSR_ANEGCOMPLETE)
        break;

      nxsig_usleep(100);
    }

  if (timeout >= PHY_ANEG_TIMEOUT)
    {
      nerr("ERROR: Autoneg timeout\n");
      return -ETIMEDOUT;
    }

  ninfo("Autoneg complete (MSR=0x%04x)\n", phyval);

  ret = mdio_read(mdio, phyaddr, MII_DP83825I_PHYSTS, &phyval);
  if (ret == OK)
    {
      bool is100   = ((phyval & MII_DP83825I_PHYSTS_SPEED) == 0);
      bool duplex  = ((phyval & MII_DP83825I_PHYSTS_DUPLEX) != 0);

      ninfo("Link: %s %s-duplex\n",
            is100 ? "100Mbps" : "10Mbps",
            duplex ? "full" : "half");
    }
#else

  /* Force 100Mbps full duplex if autoneg is disabled */
  mdio_write(mdio, phyaddr, MII_MCR,
             MII_MCR_SPEED100 | MII_MCR_FULLDPLX);
  up_mdelay(100);
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc4d7_ethernet_initialize
 *
 * Description:
 *   Board-level Ethernet initialization for TC4D7-Lite.
 *
 ****************************************************************************/

int tc4d7_ethernet_initialize(void)
{
  int ret;
  struct mdio_bus_s *mdio;

  tc4d7_config_geth_pins();

  ret = tc4x_geth_init(0);
  if (ret < 0)
    {
      nerr("ERROR: xgmac_initialize failed: %d\n", ret);
      return ret;
    }

  mdio = dwxgmac_mdio_bus_initialize(0, CONFIG_DWXGMAC_BASE);
  if (mdio == NULL)
    {
      printf("ERROR: MDIO bus init failed\n");
      return -ENODEV;
    }

  tc4d7_init_phy(mdio);

  return OK;
}
