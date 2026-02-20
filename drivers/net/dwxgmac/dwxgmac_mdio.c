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
 * Pre-processor Definitions
 ****************************************************************************/

#define DWXGMAC_MDIO_SCA(i)     (0x200 + (i) * 0x2000)
#define DWXGMAC_MDIO_SCCD(i)    (0x204 + (i) * 0x2000)
#define DWXGMAC_MDIO_CLAUSE(i)  (0x220 + (i) * 0x2000)

#define DWXGMAC_MDIO_CMD_READ        GENMASK(17, 16)
#define DWXGMAC_MDIO_CMD_WRITE       BIT(16)
#define DWXGMAC_MDIO_CMD             GENMASK(17, 16)
#define DWXGMAC_MDIO_SBUSY           BIT(22)
#define DWXGMAC_MDIO_DATA_MASK       GENMASK(15, 0)

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

struct dwxgmac_mdio_bus_s
{
  struct mdio_lowerhalf_s lower;
  uint8_t port;
  uintptr_t base;

  /* MDIO bus timeout in milliseconds */
  int timeout;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int dwxgmac_c22_read(struct mdio_lowerhalf_s *dev, uint8_t phydev,
                            uint8_t regaddr, uint16_t *value);
static int dwxgmac_c22_write(struct mdio_lowerhalf_s *dev, uint8_t phydev,
                             uint8_t regaddr, uint16_t value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR const struct mdio_ops_s dwxgmac_mdio_ops =
{
  .read  = dwxgmac_c22_read,
  .write = dwxgmac_c22_write,
  .reset = NULL,
};

static struct dwxgmac_mdio_bus_s g_dwxgmac_mdio_bus[2] =
{
  {
    .lower.ops = &dwxgmac_mdio_ops,
    .timeout = 50,
  },
  {
    .lower.ops = &dwxgmac_mdio_ops,
    .timeout = 50,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dwxgmac_mdio_wait_idle
 *
 * Description:
 *   Wait until the MDIO bus is idle (SBUSY=0).
 *
 ****************************************************************************/

static int dwxgmac_mdio_wait_idle(struct dwxgmac_mdio_bus_s *priv)
{
  int to;

  for (to = 0; to < priv->timeout; to++)
    {
      if ((xgmac_read(priv->base +
           DWXGMAC_MDIO_SCCD(priv->port)) & DWXGMAC_MDIO_SBUSY) == 0)
        {
          return OK;
        }

      up_mdelay(1);
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: dwxgmac_c22_read
 *
 * Description:
 *   MDIO Clause 22 read operation.
 *
 ****************************************************************************/

static int dwxgmac_c22_read(struct mdio_lowerhalf_s *dev, uint8_t phyaddr,
                            uint8_t regaddr, uint16_t *value)
{
  struct dwxgmac_mdio_bus_s *priv = (struct dwxgmac_mdio_bus_s *)dev;
  uint8_t port = priv->port;
  uint32_t cmd_addr;
  uint32_t cmd_data;
  int ret;

  ret = dwxgmac_mdio_wait_idle(priv);
  if (ret < 0)
    {
      nerr("MDIO read: bus busy before start\n");
      return ret;
    }

  cmd_addr = ((uint32_t)phyaddr << 16) | ((uint32_t)regaddr & 0x1F);

  xgmac_write(BIT(phyaddr), priv->base + DWXGMAC_MDIO_CLAUSE(port));
  xgmac_write(cmd_addr, priv->base + DWXGMAC_MDIO_SCA(port));

  cmd_data = DWXGMAC_MDIO_CMD_READ | DWXGMAC_MDIO_SBUSY;
  xgmac_modreg(cmd_data, DWXGMAC_MDIO_CMD | DWXGMAC_MDIO_SBUSY,
               priv->base + DWXGMAC_MDIO_SCCD(port));

  /* Wait for completion */
  ret = dwxgmac_mdio_wait_idle(priv);
  if (ret < 0)
    {
      nerr("MDIO read timed out: phy=%02x reg=%02x\n", phyaddr, regaddr);
      return ret;
    }

  *value = (uint16_t)(xgmac_read(priv->base +
            DWXGMAC_MDIO_SCCD(port)) & DWXGMAC_MDIO_DATA_MASK);

  return OK;
}

/****************************************************************************
 * Name: dwxgmac_c22_write
 *
 * Description:
 *   MDIO Clause 22 write operation.
 *
 ****************************************************************************/

static int dwxgmac_c22_write(struct mdio_lowerhalf_s *dev, uint8_t phyaddr,
                             uint8_t regaddr, uint16_t value)
{
  struct dwxgmac_mdio_bus_s *priv = (struct dwxgmac_mdio_bus_s *)dev;
  uint8_t port = priv->port;
  uint32_t cmd_addr;
  uint32_t cmd_data;
  int ret;

  ret = dwxgmac_mdio_wait_idle(priv);
  if (ret < 0)
    {
      nerr("MDIO write: bus busy before start\n");
      return ret;
    }

  cmd_addr = ((uint32_t)phyaddr << 16) | ((uint32_t)regaddr & 0x1F);

  xgmac_write(BIT(phyaddr), priv->base + DWXGMAC_MDIO_CLAUSE(port));
  xgmac_write(cmd_addr, priv->base + DWXGMAC_MDIO_SCA(port));

  cmd_data = DWXGMAC_MDIO_CMD_WRITE | DWXGMAC_MDIO_SBUSY | (value & 0xFFFF);
  xgmac_modreg(cmd_data,
               DWXGMAC_MDIO_CMD | DWXGMAC_MDIO_SBUSY | DWXGMAC_MDIO_DATA_MASK,
               priv->base + DWXGMAC_MDIO_SCCD(port));

  ret = dwxgmac_mdio_wait_idle(priv);
  if (ret < 0)
    {
      nerr("MDIO write timed out: phy=%02x reg=%02x\n", phyaddr, regaddr);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dwxgmac_mdio_bus_initialize
 *
 * Description:
 *   Initialize the MDIO bus for a DWXGMAC port.
 *
 * Input Parameters:
 *   port - DWXGMAC port number (0 or 1)
 *   base - MAC register base address (CONFIG_DWXGMAC_BASE)
 *
 * Returned Value:
 *   Pointer to MDIO bus, or NULL on failure.
 *
 ****************************************************************************/
struct mdio_bus_s *dwxgmac_mdio_bus_initialize(uint8_t port, uintptr_t base)
{
  if (port >= 2)
    {
      return NULL;
    }

  g_dwxgmac_mdio_bus[port].base = base;
  g_dwxgmac_mdio_bus[port].port = port;

  return mdio_register(&g_dwxgmac_mdio_bus[port].lower);
}
