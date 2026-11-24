/****************************************************************************
 * drivers/net/dwxgmac/dwxgmac_mdio.c
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
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/net/dwxgmac.h>
#include <nuttx/net/mdio.h>

#include "dwxgmac.h"
#include "xgmac_priv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MDIO single command registers, relative to the MAC base */

#define XGMAC_MDIO_SCA          0x200   /* Address */
#define XGMAC_MDIO_SCCD         0x204   /* Control and data */
#define XGMAC_MDIO_C22P         0x220   /* Clause 22 ports */

#define XGMAC_MDIO_SCA_RA       GENMASK(4, 0)    /* Register address */
#define XGMAC_MDIO_SCA_PA       GENMASK(20, 16)  /* PHY address */
#define XGMAC_MDIO_SCCD_SDATA   GENMASK(15, 0)
#define XGMAC_MDIO_SCCD_CMD     GENMASK(17, 16)
#define XGMAC_MDIO_SCCD_WRITE   FIELD_PREP(XGMAC_MDIO_SCCD_CMD, 1)
#define XGMAC_MDIO_SCCD_READ    FIELD_PREP(XGMAC_MDIO_SCCD_CMD, 3)
#define XGMAC_MDIO_SCCD_SBUSY   BIT(22)

#define XGMAC_MDIO_TIMEOUT      50      /* Milliseconds */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dwxgmac_mdio_s
{
  struct mdio_lowerhalf_s lower;        /* Must be first */
  uintptr_t base;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int dwxgmac_mdio_read(FAR struct mdio_lowerhalf_s *lower,
                             uint8_t phyaddr, uint8_t regaddr,
                             FAR uint16_t *value);
static int dwxgmac_mdio_write(FAR struct mdio_lowerhalf_s *lower,
                              uint8_t phyaddr, uint8_t regaddr,
                              uint16_t value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct mdio_ops_s g_dwxgmac_mdio_ops =
{
  .read  = dwxgmac_mdio_read,
  .write = dwxgmac_mdio_write,
};

static struct dwxgmac_mdio_s g_dwxgmac_mdio[CONFIG_NET_DWXGMAC_NINTERFACES];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int dwxgmac_mdio_wait(FAR struct dwxgmac_mdio_s *priv)
{
  int i;

  for (i = 0; i < XGMAC_MDIO_TIMEOUT; i++)
    {
      if ((dwmac_getreg(priv->base + XGMAC_MDIO_SCCD) &
           XGMAC_MDIO_SCCD_SBUSY) == 0)
        {
          return OK;
        }

      up_mdelay(1);
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: dwxgmac_mdio_command
 *
 * Description:
 *   Run one clause 22 command on the bus and wait until it is done.
 *
 ****************************************************************************/

static int dwxgmac_mdio_command(FAR struct dwxgmac_mdio_s *priv,
                                uint8_t phyaddr, uint8_t regaddr,
                                uint32_t cmd)
{
  int ret;

  ret = dwxgmac_mdio_wait(priv);
  if (ret < 0)
    {
      return ret;
    }

  dwmac_putreg(1 << phyaddr, priv->base + XGMAC_MDIO_C22P);
  dwmac_putreg(FIELD_PREP(XGMAC_MDIO_SCA_PA, phyaddr) |
               FIELD_PREP(XGMAC_MDIO_SCA_RA, regaddr),
               priv->base + XGMAC_MDIO_SCA);
  dwmac_modreg(cmd | XGMAC_MDIO_SCCD_SBUSY,
               XGMAC_MDIO_SCCD_CMD | XGMAC_MDIO_SCCD_SBUSY |
               (cmd == XGMAC_MDIO_SCCD_READ ? 0 : XGMAC_MDIO_SCCD_SDATA),
               priv->base + XGMAC_MDIO_SCCD);

  ret = dwxgmac_mdio_wait(priv);
  if (ret < 0)
    {
      nerr("ERROR: MDIO timeout, PHY %u register %u\n", phyaddr, regaddr);
    }

  return ret;
}

static int dwxgmac_mdio_read(FAR struct mdio_lowerhalf_s *lower,
                             uint8_t phyaddr, uint8_t regaddr,
                             FAR uint16_t *value)
{
  FAR struct dwxgmac_mdio_s *priv = (FAR struct dwxgmac_mdio_s *)lower;
  int ret;

  ret = dwxgmac_mdio_command(priv, phyaddr, regaddr,
                             XGMAC_MDIO_SCCD_READ);
  if (ret >= 0)
    {
      *value = dwmac_getreg(priv->base + XGMAC_MDIO_SCCD) &
               XGMAC_MDIO_SCCD_SDATA;
    }

  return ret;
}

static int dwxgmac_mdio_write(FAR struct mdio_lowerhalf_s *lower,
                              uint8_t phyaddr, uint8_t regaddr,
                              uint16_t value)
{
  FAR struct dwxgmac_mdio_s *priv = (FAR struct dwxgmac_mdio_s *)lower;

  return dwxgmac_mdio_command(priv, phyaddr, regaddr,
                              XGMAC_MDIO_SCCD_WRITE | value);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dwxgmac_mdio_initialize
 ****************************************************************************/

FAR struct mdio_bus_s *dwxgmac_mdio_initialize(int bus, uintptr_t base)
{
  FAR struct dwxgmac_mdio_s *priv;

  if (bus < 0 || bus >= CONFIG_NET_DWXGMAC_NINTERFACES)
    {
      return NULL;
    }

  priv = &g_dwxgmac_mdio[bus];
  priv->lower.ops = &g_dwxgmac_mdio_ops;
  priv->base = base;

  return mdio_register(&priv->lower);
}
