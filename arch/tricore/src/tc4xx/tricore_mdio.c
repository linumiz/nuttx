/****************************************************************************
 * arch/tricore/src/common/tricore_mdio.c
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
 * Pre-processor Definitions
 ****************************************************************************/

#define TC4X_GETH_MDIO_SCA(i)     (0x200 + i * 0x2000)
#define TC4X_GETH_MDIO_SCCD(i)    (0x204 + i * 0x2000)
#define TC4X_GETH_MDIO_CLAUSE(i)  (0x220 + i * 0x2000)
#define GETH_MDIO_CMD_READ        BIT(16)
#define GETH_MDIO_CMD_WRITE       GENMASK(17,16)
#define GETH_MDIO_CMD             GENMASK(17,16)
#define GETH_MDIO_SBUSY           BIT(22)

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/mutex.h>
#include <nuttx/config.h>
#include <nuttx/net/mdio.h>
#include <nuttx/bits.h>

#include <debug.h>
#include <errno.h>
#include <inttypes.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tricore_mdio_bus_s
{
  struct mdio_lowerhalf_s lower;
  uint8_t port;
  uintptr_t base;  /* XGMAC register base */

  /* MDIO bus timeout in milliseconds */
  int timeout;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int tricore_c22_read(struct mdio_lowerhalf_s *dev, uint8_t phydev,
                     uint8_t regaddr, uint16_t *value);

static int tricore_c22_write(struct mdio_lowerhalf_s *dev, uint8_t phydev,
                     uint8_t regaddr, uint16_t value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR const struct mdio_ops_s tricore_mdio_ops = {
    .read  = tricore_c22_read,
    .write = tricore_c22_write,
    .reset = NULL,
};

static struct tricore_mdio_bus_s g_tricore_mdio_bus[2] =
{
  {
   .lower.ops = &tricore_mdio_ops,
   .timeout = 10
  },

  {
   .lower.ops = &tricore_mdio_ops,
   .timeout = 10
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int tricore_c22_read(struct mdio_lowerhalf_s *dev, uint8_t phyaddr,
                            uint8_t regaddr, uint16_t *value)
{
  int to = 5;
  int retval = -ETIMEDOUT;
  struct tricore_mdio_bus_s *priv = (struct tricore_mdio_bus_s *)dev;
  uint32_t cmd_addr = (phyaddr << 16) | (regaddr & 0x1F);
  uint32_t cmd_data = GETH_MDIO_CMD_READ | GETH_MDIO_SBUSY;
  uint8_t port = priv->port;

  do {
      if ((xgmac_read(priv->base + TC4X_GETH_MDIO_SCCD(port)) & GETH_MDIO_SBUSY) == 0)
        {
          retval = OK;
          break;
        }

      up_mdelay(5);
    } while(to--);

  if (to <= 0)
    {
       return retval;
    }

  xgmac_write(BIT(phyaddr), priv->base + TC4X_GETH_MDIO_CLAUSE(port));
  xgmac_write(cmd_addr, priv->base + TC4X_GETH_MDIO_SCA(port));
  xgmac_modreg(cmd_data, GETH_MDIO_CMD | GETH_MDIO_SBUSY, priv->base + TC4X_GETH_MDIO_SCCD(port));

  /* Wait for the transfer to complete */
  for (to = priv->timeout; to >= 0; to--)
    {
      if ((xgmac_read(priv->base + TC4X_GETH_MDIO_SCCD(port)) & GETH_MDIO_SBUSY) == 0)
        {
          *value = (uint16_t)xgmac_read(TC4X_GETH_MDIO_SCCD(port));
          retval = OK;
          break;
        }

      up_mdelay(5);
    }

  if (to <= 0)
    {
      ninfo("MII transfer timed out: phyaddr: %04x regaddr: %04x\n",
            phyaddr, regaddr);
    }

  return retval;
}

static int tricore_c22_write(struct mdio_lowerhalf_s *dev, uint8_t phyaddr,
                             uint8_t regaddr, uint16_t value)
{
  int retval = -ETIMEDOUT;
  struct tricore_mdio_bus_s *priv = (struct tricore_mdio_bus_s *)dev;
  uint32_t cmd_addr = (phyaddr << 16) | (regaddr & 0x1F);
  uint32_t cmd_data = GETH_MDIO_CMD_WRITE | GETH_MDIO_SBUSY | value;
  int to = priv->timeout;
  uint8_t port = priv->port;

  do {
      if ((xgmac_read(priv->base + TC4X_GETH_MDIO_SCCD(port)) & GETH_MDIO_SBUSY) == 0)
        {
          retval = OK;
          break;
        }

      up_mdelay(5);
    } while(to--);

  if (to <= 0)
    {
       return retval;
    }

  xgmac_write(BIT(phyaddr), priv->base + TC4X_GETH_MDIO_CLAUSE(port));
  xgmac_write(cmd_addr, priv->base + TC4X_GETH_MDIO_SCA(port));
  xgmac_modreg(cmd_data, GETH_MDIO_CMD | GETH_MDIO_SBUSY | 0xFF, priv->base + TC4X_GETH_MDIO_SCCD(port));

  /* Wait for the transfer to complete */
  for (to = priv->timeout; to >= 0; to--)
    {
      if ((xgmac_read(priv->base + TC4X_GETH_MDIO_SCCD(port)) & GETH_MDIO_SBUSY) == 0)
        {
          retval = OK;
          break;
        }

      up_mdelay(5);
    }

  if (to <= 0)
    {
      ninfo("MII transfer timed out: phyaddr: %04x regaddr: %04x\n",
            phyaddr, regaddr);
    }

  return retval;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_mdio_bus_initialize
 *
 * Description:
 *   Initialize the MDIO bus
 *
 * Returned Value:
 *   Initialized MDIO bus structure or NULL on failure
 *
 ****************************************************************************/

struct mdio_bus_s * __attribute__((optimize("O0"))) tricore_mdio_bus_initialize(uint8_t port, uintptr_t base)
{

  if (port >= 2)
     return NULL;

  g_tricore_mdio_bus[port].base = base;
  g_tricore_mdio_bus[port].port = port;

  return mdio_register(&g_tricore_mdio_bus[port].lower);
}
