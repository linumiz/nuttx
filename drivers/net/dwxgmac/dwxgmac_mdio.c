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
#include <nuttx/bits.h>
#include <nuttx/mutex.h>
#include <nuttx/net/mdio.h>

#include <debug.h>
#include <errno.h>
#include <inttypes.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_DWXGMAC_QOS

/* TC4xx XGMAC MDIO registers, port stride = 0x2000 */

#define DWXGMAC_MDIO_SCA(i)     (0x200 + (i) * 0x2000)
#define DWXGMAC_MDIO_SCCD(i)    (0x204 + (i) * 0x2000)
#define DWXGMAC_MDIO_CLAUSE(i)  (0x220 + (i) * 0x2000)

#define DWXGMAC_MDIO_CMD_READ   GENMASK(17, 16)
#define DWXGMAC_MDIO_CMD_WRITE  BIT(16)
#define DWXGMAC_MDIO_CMD        GENMASK(17, 16)
#define DWXGMAC_MDIO_SBUSY      BIT(22)
#define DWXGMAC_MDIO_DATA_MASK  GENMASK(15, 0)

#else /* CONFIG_DWXGMAC_QOS */

/* TC3xx ETH QoS MDIO registers (GMAC5 standard, no port stride) */

#define EQOS_MDIO_ADDR          0x0200
#define EQOS_MDIO_DATA          0x0204

/* MAC_MDIO_ADDRESSESS fields */

#define EQOS_MDIO_ADDR_PA       GENMASK(25, 21)   /* PHY address */
#define EQOS_MDIO_ADDR_RDA      GENMASK(20, 16)   /* register address */
#define EQOS_MDIO_ADDR_CR       GENMASK(11, 8)    /* MDC clock divider */
#define EQOS_MDIO_ADDR_GOC_1    BIT(3)
#define EQOS_MDIO_ADDR_GOC_0    BIT(2)
#define EQOS_MDIO_ADDR_C45E     BIT(1)
#define EQOS_MDIO_ADDR_GB       BIT(0)             /* busy */

/* GOC encoding: write=0b01, read=0b11 */

#define EQOS_MDIO_GOC_WRITE     (EQOS_MDIO_ADDR_GOC_0)
#define EQOS_MDIO_GOC_READ      (EQOS_MDIO_ADDR_GOC_1 | EQOS_MDIO_ADDR_GOC_0)
#define EQOS_MDIO_GOC_MASK      (EQOS_MDIO_ADDR_GOC_1 | EQOS_MDIO_ADDR_GOC_0)

#define EQOS_MDIO_DATA_MASK     GENMASK(15, 0)

/* MDC clock range - adjust to match your CSR clock. 0x4 = CSR 150-250 MHz */

#define EQOS_MDIO_CR_DEFAULT    0x4

#endif /* CONFIG_DWXGMAC_QOS */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dwxgmac_mdio_bus_s
{
  struct mdio_lowerhalf_s lower;
  uint8_t  port;
  uintptr_t base;
  int      timeout;   /* milliseconds */
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
    .timeout = 50
  },
  {
    .lower.ops = &dwxgmac_mdio_ops,
    .timeout = 50
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int dwxgmac_mdio_wait_idle(struct dwxgmac_mdio_bus_s *priv)
{
  int to;

#ifndef CONFIG_DWXGMAC_QOS
  uintptr_t reg = priv->base + DWXGMAC_MDIO_SCCD(priv->port);
  uint32_t  busy_mask = DWXGMAC_MDIO_SBUSY;
#else
  uintptr_t reg = priv->base + EQOS_MDIO_ADDR;
  uint32_t  busy_mask = EQOS_MDIO_ADDR_GB;
#endif

  for (to = 0; to < priv->timeout; to++)
    {
      if ((xgmac_read(reg) & busy_mask) == 0)
        {
          return OK;
        }

      up_mdelay(1);
    }

  return -ETIMEDOUT;
}

static int dwxgmac_c22_read(struct mdio_lowerhalf_s *dev, uint8_t phyaddr,
                            uint8_t regaddr, uint16_t *value)
{
  struct dwxgmac_mdio_bus_s *priv = (struct dwxgmac_mdio_bus_s *)dev;
  int ret;

  ret = dwxgmac_mdio_wait_idle(priv);
  if (ret < 0)
    {
      nerr("MDIO read: bus busy\n");
      return ret;
    }

#ifndef CONFIG_DWXGMAC_QOS

  uint8_t  port     = priv->port;
  uint32_t cmd_addr = ((uint32_t)phyaddr << 16) |
                      ((uint32_t)regaddr & 0x1f);

  xgmac_write(BIT(phyaddr), priv->base + DWXGMAC_MDIO_CLAUSE(port));
  xgmac_write(cmd_addr, priv->base + DWXGMAC_MDIO_SCA(port));

  xgmac_modreg(DWXGMAC_MDIO_CMD_READ | DWXGMAC_MDIO_SBUSY,
               DWXGMAC_MDIO_CMD | DWXGMAC_MDIO_SBUSY,
               priv->base + DWXGMAC_MDIO_SCCD(port));

  ret = dwxgmac_mdio_wait_idle(priv);
  if (ret < 0)
    {
      nerr("MDIO read timeout: phy=%02x reg=%02x\n", phyaddr, regaddr);
      return ret;
    }

  *value = (uint16_t)(xgmac_read(priv->base + DWXGMAC_MDIO_SCCD(port)) &
                      DWXGMAC_MDIO_DATA_MASK);

#else

  uint32_t addr = ((uint32_t)(phyaddr & 0x1f) << 21) |
                  ((uint32_t)(regaddr & 0x1f) << 16) |
                  ((uint32_t)(EQOS_MDIO_CR_DEFAULT & 0xf) << 8) |
                  EQOS_MDIO_GOC_READ |
                  EQOS_MDIO_ADDR_GB;

  xgmac_write(addr, priv->base + EQOS_MDIO_ADDR);

  ret = dwxgmac_mdio_wait_idle(priv);
  if (ret < 0)
    {
      nerr("MDIO read timeout: phy=%02x reg=%02x\n", phyaddr, regaddr);
      return ret;
    }

  *value = (uint16_t)(xgmac_read(priv->base + EQOS_MDIO_DATA) &
                      EQOS_MDIO_DATA_MASK);

#endif /* CONFIG_DWXGMAC_QOS */

  return OK;
}

static int dwxgmac_c22_write(struct mdio_lowerhalf_s *dev, uint8_t phyaddr,
                             uint8_t regaddr, uint16_t value)
{
  struct dwxgmac_mdio_bus_s *priv = (struct dwxgmac_mdio_bus_s *)dev;
  int ret;

  ret = dwxgmac_mdio_wait_idle(priv);
  if (ret < 0)
    {
      nerr("MDIO write: bus busy\n");
      return ret;
    }

#ifndef CONFIG_DWXGMAC_QOS

  uint8_t  port     = priv->port;
  uint32_t cmd_addr = ((uint32_t)phyaddr << 16) |
                      ((uint32_t)regaddr & 0x1f);

  xgmac_write(BIT(phyaddr), priv->base + DWXGMAC_MDIO_CLAUSE(port));
  xgmac_write(cmd_addr, priv->base + DWXGMAC_MDIO_SCA(port));

  xgmac_modreg(DWXGMAC_MDIO_CMD_WRITE | DWXGMAC_MDIO_SBUSY |
               (uint32_t)(value & 0xffff),
               DWXGMAC_MDIO_CMD | DWXGMAC_MDIO_SBUSY | DWXGMAC_MDIO_DATA_MASK,
               priv->base + DWXGMAC_MDIO_SCCD(port));

#else

  xgmac_write((uint32_t)(value & 0xffff), priv->base + EQOS_MDIO_DATA);

  uint32_t addr = ((uint32_t)(phyaddr & 0x1f) << 21)  |
                  ((uint32_t)(regaddr & 0x1f) << 16) |
                  ((uint32_t)(EQOS_MDIO_CR_DEFAULT & 0xf) << 8) |
                  EQOS_MDIO_GOC_WRITE |
                  EQOS_MDIO_ADDR_GB;

  xgmac_write(addr, priv->base + EQOS_MDIO_ADDR);

#endif /* CONFIG_DWXGMAC_QOS */

  ret = dwxgmac_mdio_wait_idle(priv);
  if (ret < 0)
    {
      nerr("MDIO write timeout: phy=%02x reg=%02x\n", phyaddr, regaddr);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct mdio_bus_s *dwxgmac_mdio_bus_initialize(uint8_t port, uintptr_t base)
{
#ifdef CONFIG_DWXGMAC_QOS
  port = 0;
#else
  if (port >= 2)
    {
      return NULL;
    }
#endif

  g_dwxgmac_mdio_bus[port].base = base;
  g_dwxgmac_mdio_bus[port].port = port;

  return mdio_register(&g_dwxgmac_mdio_bus[port].lower);
}
