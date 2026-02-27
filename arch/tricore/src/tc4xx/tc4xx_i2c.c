/****************************************************************************
 * arch/tricore/src/tc4xx/tc4xx_i2c.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>

#include "tricore_internal.h"
#include "tricore_i2c.h"
#include "hardware/tc4xx_i2c.h"

#ifdef CONFIG_TC4XX_I2C0
static const struct tricore_i2c_config_s g_tc4xx_i2c0_config =
{
  .base          = TC4XX_I2C0_BASE,
  .frequency     = CONFIG_TC4XX_I2C0_FREQUENCY,
  .clk_freq      = CONFIG_TC4XX_I2C_CLOCK_FREQ,
  .bus           = 0,
  .pisel         = CONFIG_TC4XX_I2C0_PISEL,
  .clc_offset    = TC4XX_I2C_CLC_OFFSET,
  .gpctl_offset  = TC4XX_I2C_GPCTL_OFFSET,
};

static struct tricore_i2c_priv_s g_tc4xx_i2c0_priv =
{
  .ops    = &g_tricore_i2c_ops,
  .config = &g_tc4xx_i2c0_config,
  .lock   = NXMUTEX_INITIALIZER,
};
#endif

#ifdef CONFIG_TC4XX_I2C1
static const struct tricore_i2c_config_s g_tc4xx_i2c1_config =
{
  .base          = TC4XX_I2C1_BASE,
  .frequency     = CONFIG_TC4XX_I2C1_FREQUENCY,
  .clk_freq      = CONFIG_TC4XX_I2C_CLOCK_FREQ,
  .bus           = 1,
  .pisel         = CONFIG_TC4XX_I2C1_PISEL,
  .clc_offset    = TC4XX_I2C_CLC_OFFSET,
  .gpctl_offset  = TC4XX_I2C_GPCTL_OFFSET,
};

static struct tricore_i2c_priv_s g_tc4xx_i2c1_priv =
{
  .ops    = &g_tricore_i2c_ops,
  .config = &g_tc4xx_i2c1_config,
  .lock   = NXMUTEX_INITIALIZER,
};
#endif

#ifdef CONFIG_TC4XX_I2C2
static const struct tricore_i2c_config_s g_tc4xx_i2c2_config =
{
  .base          = TC4XX_I2C2_BASE,
  .frequency     = CONFIG_TC4XX_I2C2_FREQUENCY,
  .clk_freq      = CONFIG_TC4XX_I2C_CLOCK_FREQ,
  .bus           = 2,
  .pisel         = CONFIG_TC4XX_I2C2_PISEL,
  .clc_offset    = TC4XX_I2C_CLC_OFFSET,
  .gpctl_offset  = TC4XX_I2C_GPCTL_OFFSET,
};

static struct tricore_i2c_priv_s g_tc4xx_i2c2_priv =
{
  .ops    = &g_tricore_i2c_ops,
  .config = &g_tc4xx_i2c2_config,
  .lock   = NXMUTEX_INITIALIZER,
};
#endif

static int tc4xx_i2c_wait_clk_enable(struct tricore_i2c_priv_s *priv,
                                     uint32_t offset, uint32_t timeout_us)
{
  uint32_t elapsed = 0;

  while ((i2c_getreg(priv, offset) & I2C_CLC1_DISS) != 0)
    {
      if (elapsed >= timeout_us)
        {
          return -ETIMEDOUT;
        }

      up_udelay(I2C_POLL_INTERVAL_US);
      elapsed += I2C_POLL_INTERVAL_US;
    }

  return OK;
}

void weak_function board_aurix_setup_i2c_pin(int bus)
{
}

int tricore_i2c_hw_init(struct tricore_i2c_priv_s *priv)
{
  const struct tricore_i2c_config_s *cfg = priv->config;
  int ret;
  uint32_t elapsed;

  board_aurix_setup_i2c_pin(cfg->bus);

  i2c_modifyreg(priv, cfg->clc_offset, I2C_CLC_DISR, 0);
  ret = tc4xx_i2c_wait_clk_enable(priv, cfg->clc_offset,
                                  I2C_CLK_TIMEOUT_US);
  if (ret < 0)
    {
      i2cerr("I2C%d: Failed to enable module clock (CLC)\n", cfg->bus);
      return ret;
    }

  i2c_modifyreg(priv, TRICORE_I2C_CLC1_OFFSET, I2C_CLC1_RMC_MASK,
                (1 << I2C_CLC1_RMC_SHIFT));

  elapsed = 0;
  while (((i2c_getreg(priv, TRICORE_I2C_CLC1_OFFSET) & I2C_CLC1_RMC_MASK)
          >> I2C_CLC1_RMC_SHIFT) != 1)
    {
      if (elapsed >= I2C_CLK_TIMEOUT_US)
        {
          i2cerr("I2C%d: RMC not ready\n", cfg->bus);
          return -ETIMEDOUT;
        }

      up_udelay(I2C_POLL_INTERVAL_US);
      elapsed += I2C_POLL_INTERVAL_US;
    }

  i2c_modifyreg(priv, TRICORE_I2C_CLC1_OFFSET, I2C_CLC1_DISR, 0);
  ret = tc4xx_i2c_wait_clk_enable(priv, TRICORE_I2C_CLC1_OFFSET,
                                  I2C_CLK_TIMEOUT_US);
  if (ret < 0)
    {
      i2cerr("I2C%d: Failed to enable kernel clock (CLC1)\n", cfg->bus);
      return ret;
    }

  i2c_putreg(priv, cfg->gpctl_offset,
             cfg->pisel & I2C_GPCTL_PISEL_MASK);

  i2c_putreg(priv, TRICORE_I2C_RUNCTRL_OFFSET, 0);

  i2c_putreg(priv, TRICORE_I2C_ERRIRQSM_OFFSET, 0);
  i2c_putreg(priv, TRICORE_I2C_PIRQSM_OFFSET, 0);
  i2c_putreg(priv, TRICORE_I2C_IMSC_OFFSET, 0);

  i2c_putreg(priv, TRICORE_I2C_ADDRCFG_OFFSET,
             I2C_ADDRCFG_MNS | I2C_ADDRCFG_SONA);

  i2c_putreg(priv, TRICORE_I2C_FIFOCFG_OFFSET,
             (2 << I2C_FIFOCFG_RXBS_SHIFT) |
             (2 << I2C_FIFOCFG_TXBS_SHIFT) |
             (2 << I2C_FIFOCFG_RXFA_SHIFT) |
             (2 << I2C_FIFOCFG_TXFA_SHIFT) |
             I2C_FIFOCFG_TXFC |
             I2C_FIFOCFG_RXFC);

  ret = tricore_i2c_set_rate(priv, cfg->frequency);
  if (ret < 0)
    {
      return ret;
    }

  i2c_putreg(priv, TRICORE_I2C_RUNCTRL_OFFSET, I2C_RUNCTRL_RUN);

  i2cinfo("I2C%d: initialized at %lu Hz (PISEL=%d)\n",
          cfg->bus, cfg->frequency, cfg->pisel);
  return OK;
}

void tricore_i2c_hw_deinit(struct tricore_i2c_priv_s *priv)
{
  i2c_putreg(priv, TRICORE_I2C_RUNCTRL_OFFSET, 0);
  i2c_putreg(priv, TRICORE_I2C_IMSC_OFFSET, 0);
  i2c_modifyreg(priv, TRICORE_I2C_CLC1_OFFSET, 0, I2C_CLC1_DISR);
}

FAR struct i2c_master_s *tc4xx_i2cbus_initialize(int bus)
{
  struct tricore_i2c_priv_s *priv = NULL;
  int ret;

  switch (bus)
    {
#ifdef CONFIG_TC4XX_I2C0
      case 0:
        priv = &g_tc4xx_i2c0_priv;
        break;
#endif
#ifdef CONFIG_TC4XX_I2C1
      case 1:
        priv = &g_tc4xx_i2c1_priv;
        break;
#endif
#ifdef CONFIG_TC4XX_I2C2
      case 2:
        priv = &g_tc4xx_i2c2_priv;
        break;
#endif
      default:
        return NULL;
    }

  ret = tricore_i2c_hw_init(priv);
  if (ret < 0)
    {
      i2cerr("I2C%d: hw_init failed: %d\n", bus, ret);
      return NULL;
    }

  return (FAR struct i2c_master_s *)priv;
}
