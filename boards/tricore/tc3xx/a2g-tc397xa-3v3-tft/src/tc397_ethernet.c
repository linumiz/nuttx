/****************************************************************************
 * boards/tricore/tc3xx/a2g-tc397xa-3v3-tft/src/tc397_ethernet.c
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

#include "tc3xx_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RTL8211F PHY configuration */
#define BOARD_PHY_NAME        "RTL8211F"
#define BOARD_PHY_ADDR        0           /* PHY address on MDIO bus */
#define BOARD_PHYID1          MII_PHYID1_RTL8211F

#define PHY_RESET_TIMEOUT_MS  100
#define PHY_LINK_TIMEOUT      0x4FFFF
#define PHY_ANEG_TIMEOUT      0x4FFFF

struct mdio_bus_s *dwxgmac_mdio_bus_initialize(uint8_t port, uintptr_t base);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int tc3x_geth_init(int intf);

static void tc397_config_geth_pins(void)
{
  gpio_pinset_t pins[] =
    {
      /* RMII TX outputs */
      AURIX_GPIO_FULL(GPIO_PORT11, GPIO_PIN4, GPIO_PERIPH, GPIO_ALT7,
      0, 0, 3, 0, GPIO_PERIPH_OWN_PAD),
      AURIX_GPIO_FULL(GPIO_PORT11, GPIO_PIN6, GPIO_PERIPH, GPIO_ALT6,
      0, 0, 3, 0, GPIO_PERIPH_OWN_PAD),
      AURIX_GPIO_FULL(GPIO_PORT11, GPIO_PIN0, GPIO_PERIPH, GPIO_ALT6,
      0, 0, 3, 0, GPIO_PERIPH_OWN_PAD),
      AURIX_GPIO_FULL(GPIO_PORT11, GPIO_PIN1, GPIO_PERIPH, GPIO_ALT6,
      0, 0, 3, 0, GPIO_PERIPH_OWN_PAD),
      AURIX_GPIO_FULL(GPIO_PORT11, GPIO_PIN2, GPIO_PERIPH, GPIO_ALT6,
      0, 0, 3, 0, GPIO_PERIPH_OWN_PAD),
      AURIX_GPIO_FULL(GPIO_PORT11, GPIO_PIN3, GPIO_PERIPH, GPIO_ALT6,
      0, 0, 3, 0, GPIO_PERIPH_OWN_PAD),

      /* RMII RX inputs */
      AURIX_GPIO_FULL(GPIO_PORT11, GPIO_PIN5, GPIO_INPUT, GPIO_PULL_NONE,
      0, 0, 3, 0, GPIO_PERIPH_OWN_PAD),
      AURIX_GPIO_FULL(GPIO_PORT11, GPIO_PIN11, GPIO_INPUT, GPIO_PULL_NONE,
      0, 0, 3, 0, GPIO_PERIPH_OWN_PAD),
      AURIX_GPIO_FULL(GPIO_PORT11, GPIO_PIN12, GPIO_INPUT, GPIO_PULL_NONE,
      0, 0, 3, 0, GPIO_PERIPH_OWN_PAD),
      AURIX_GPIO_FULL(GPIO_PORT11, GPIO_PIN7, GPIO_INPUT, GPIO_PULL_NONE,
      0, 0, 3, 0, GPIO_PERIPH_OWN_PAD),
      AURIX_GPIO_FULL(GPIO_PORT11, GPIO_PIN8, GPIO_INPUT, GPIO_PULL_NONE,
      0, 0, 3, 0, GPIO_PERIPH_OWN_PAD),
      AURIX_GPIO_FULL(GPIO_PORT11, GPIO_PIN9, GPIO_INPUT, GPIO_PULL_NONE,
      0, 0, 3, 0, GPIO_PERIPH_OWN_PAD),
      AURIX_GPIO_FULL(GPIO_PORT11, GPIO_PIN10, GPIO_INPUT, GPIO_PULL_NONE,
      0, 0, 3, 0, GPIO_PERIPH_OWN_PAD),

      /* MDIO */
      AURIX_GPIO(GPIO_PORT12, GPIO_PIN0, GPIO_PERIPH, GPIO_ALT6),
      AURIX_GPIO(GPIO_PORT12, GPIO_PIN1, GPIO_INPUT, GPIO_PULL_NONE),
    };

  for (int i = 0; i < (int)(sizeof(pins) / sizeof(pins[0])); i++)
    {
      aurix_config_gpio(pins[i]);
    }
}

static int tc397_init_phy(struct mdio_bus_s *mdio)
{
  uint8_t phyaddr = BOARD_PHY_ADDR;
  uint16_t phyval;
  int ret;
  int to;
#ifdef CONFIG_BOARD_PHY_AUTONEG
  volatile uint32_t timeout;
#endif

  ret = mdio_write(mdio, phyaddr, MII_MCR, MII_MCR_RESET);
  if (ret < 0)
    {
      printf("ERROR: PHY reset write failed: %d\n", ret);
      return ret;
    }

  for (to = PHY_RESET_TIMEOUT_MS; to > 0; to -= 10)
    {
      up_mdelay(10);
      phyval = 0xFFFF;
      ret = mdio_read(mdio, phyaddr, MII_MCR, &phyval);
      if (ret < 0)
        {
          printf("ERROR: PHY MCR read failed: %d\n", ret);
          return ret;
        }

      if ((phyval & MII_MCR_RESET) == 0)
        break;
      printf("phy val %x\n", phyval);
    }

  if (to <= 0)
    {
      printf("ERROR: PHY reset timeout\n");
      return -ETIMEDOUT;
    }

  printf("PHY reset complete in %d ms\n", PHY_RESET_TIMEOUT_MS - to);
  ret = mdio_read(mdio, phyaddr, MII_PHYID1, &phyval);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read PHYID1: %d\n", ret);
      return ret;
    }

  if (phyval != BOARD_PHYID1)
    {
      printf("ERROR: Wrong PHYID1: 0x%04x expected 0x%04x\n",
           phyval, BOARD_PHYID1);
      return -ENXIO;
    }

  printf("rtl8211f detected, PHYID1=0x%04x\n", phyval);

  mdio_write(mdio, phyaddr, MII_RTL8211F_MDIO_PAGSR, 0xd08);
  mdio_read(mdio, phyaddr, 0x11, &phyval);
  mdio_write(mdio, phyaddr, 0x11, (phyval | 0x100)); // enable TX delay
  mdio_write(mdio, phyaddr, MII_RTL8211F_MDIO_PAGSR, 0);

  /* set LEDs */
  mdio_write(mdio, phyaddr, MII_RTL8211F_MDIO_PAGSR, 0xd04);
  mdio_write(mdio, phyaddr, MII_RTL8211F_MDIO_LCR, 0x8170);
  mdio_write(mdio, phyaddr, MII_RTL8211F_MDIO_EEELCR, 0x0);
  mdio_write(mdio, phyaddr, MII_RTL8211F_MDIO_PAGSR, 0);

#ifdef CONFIG_BOARD_PHY_AUTONEG

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

  ret = mdio_read(mdio, phyaddr, MII_RTL8211F_MDIO_PHYSR, &phyval);
  if (ret == OK)
    {
      bool is1000  = ((phyval & MII_RTL8211F_PHYSTS_SPEED1) != 0);
      bool is100   = ((phyval & MII_RTL8211F_PHYSTS_SPEED) != 0);
      bool duplex  = ((phyval & MII_RTL8211F_PHYSTS_DUPLEX) != 0);

      printf("Link: %s %s-duplex\n",
            is1000 ? "1000Mbps" : (is100 ? "100Mbps" : "10Mbps"),
            duplex ? "full" : "half");
    }

#else
  /* Force 100Mbps full duplex if autoneg is disabled */
  mdio_write(mdio, phyaddr, MII_MCR,
             MII_MCR_SPEED1000 | MII_MCR_FULLDPLX);
  up_mdelay(100);
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc397_ethernet_initialize
 *
 * Description:
 *   Board-level Ethernet initialization for TC397-Lite.
 *
 ****************************************************************************/

int tc397_ethernet_initialize(void)
{
  int ret;
  struct mdio_bus_s *mdio;

  tc397_config_geth_pins();

  ret = tc3x_geth_init(0);
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

  tc397_init_phy(mdio);

  return OK;
}
