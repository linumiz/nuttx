/****************************************************************************
 * arch/tricore/src/common/tricore_i2c.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/i2c/i2c_master.h>

#include "tricore_i2c.h"
#include "tricore_internal.h"

static int  tricore_i2c_transfer(FAR struct i2c_master_s *dev,
                                 FAR struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int  tricore_i2c_reset(FAR struct i2c_master_s *dev);
#endif
static int  tricore_i2c_setup(FAR struct i2c_master_s *dev);
static int  tricore_i2c_shutdown(FAR struct i2c_master_s *dev);

const struct i2c_ops_s g_tricore_i2c_ops =
{
  .transfer = tricore_i2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset    = tricore_i2c_reset,
#endif
  .setup    = tricore_i2c_setup,
  .shutdown = tricore_i2c_shutdown,
};

int tricore_i2c_set_rate(struct tricore_i2c_priv_s *priv,
                             uint32_t rate)
{
  uint32_t freq = priv->config->clk_freq;
  uint32_t clkdiv;
  uint32_t dec;
  uint32_t timcfg = 0;

  clkdiv = (i2c_getreg(priv, TRICORE_I2C_CLC1_OFFSET) & I2C_CLC1_RMC_MASK)
        >> I2C_CLC1_RMC_SHIFT;
  if (clkdiv == 0)
    {
      clkdiv = 1;
    }

  if (rate > 400000)
    {
      /* High-speed mode */
      dec = ((freq / rate) * 46 - 92 + 4) / 5;
    }
  else
    {
      /* Standard / Fast mode */
      dec = ((freq / clkdiv / rate) - 3 + 1) / 2;
    }

  if (dec < 6) {
          dec = 6;
  } else if (dec > 0x7ff) {
          dec = 0x7ff;
  }

  if (rate > 400000)
    {
      i2c_putreg(priv, TRICORE_I2C_FDIVCFG_OFFSET,
                 (0x1d2 << I2C_FDIVCFG_DEC_SHIFT) |
                 (5 << I2C_FDIVCFG_INC_SHIFT));
      i2c_putreg(priv, TRICORE_I2C_FDIVHIGHCFG_OFFSET,
                 (dec << I2C_FDIVHIGHCFG_DEC_SHIFT) |
                 (46 << I2C_FDIVHIGHCFG_INC_SHIFT));
    }
  else
    {
      i2c_putreg(priv, TRICORE_I2C_FDIVCFG_OFFSET,
                 (dec << I2C_FDIVCFG_DEC_SHIFT) |
                 (1 << I2C_FDIVCFG_INC_SHIFT));
    }

  timcfg = (0x3f << I2C_TIMCFG_SDA_DEL_HD_DAT_SHIFT);
  if (rate == 400000)
    {
      timcfg |= I2C_TIMCFG_FS_SCL_LOW | I2C_TIMCFG_EN_SCL_LOW_LEN |
                (0x20 << I2C_TIMCFG_SCL_LOW_LEN_SHIFT);
    }

  i2c_putreg(priv, TRICORE_I2C_TIMCFG_OFFSET, timcfg);

  priv->frequency = rate;
  return OK;
}

static int tricore_i2c_wait_fifo_req(struct tricore_i2c_priv_s *priv)
{
  uint32_t elapsed = 0;
  uint32_t val;

  while (elapsed < I2C_TIMEOUT_US)
    {
      /* Check for protocol errors first */
      val = i2c_getreg(priv, TRICORE_I2C_PIRQSS_OFFSET);
      if (val & (I2C_PIRQ_NACK | I2C_PIRQ_AL))
        {
          return -EIO;
        }

      /* Check for FIFO request */
      val = i2c_getreg(priv, TRICORE_I2C_RIS_OFFSET);
      if (val & I2C_INT_DTR_ALL)
        {
          return OK;
        }

      up_udelay(I2C_POLL_INTERVAL_US);
      elapsed += I2C_POLL_INTERVAL_US;
    }

  return -ETIMEDOUT;
}

static int tricore_i2c_wait_pirq(struct tricore_i2c_priv_s *priv)
{
  uint32_t elapsed = 0;
  uint32_t val;

  while (elapsed < I2C_TIMEOUT_US)
    {
      val = i2c_getreg(priv, TRICORE_I2C_PIRQSS_OFFSET);
      if (val & (I2C_PIRQ_NACK | I2C_PIRQ_AL))
        {
          return -EIO;
        }

      if (val & I2C_PIRQ_TX_END)
        {
          return OK;
        }

      up_udelay(I2C_POLL_INTERVAL_US);
      elapsed += I2C_POLL_INTERVAL_US;
    }

  return -ETIMEDOUT;
}

static void tricore_i2c_clear_irqs(struct tricore_i2c_priv_s *priv)
{
  i2c_putreg(priv, TRICORE_I2C_PIRQSC_OFFSET, I2C_PIRQ_ALL);
  i2c_putreg(priv, TRICORE_I2C_ERRIRQSC_OFFSET, I2C_ERRIRQ_ALL);
  i2c_putreg(priv, TRICORE_I2C_ICR_OFFSET, I2C_INT_DTR_ALL);
}

static int tricore_i2c_send_stop(struct tricore_i2c_priv_s *priv)
{
  uint32_t elapsed = 0;

  i2c_putreg(priv, TRICORE_I2C_PIRQSC_OFFSET, I2C_PIRQ_TX_END);

  i2c_putreg(priv, TRICORE_I2C_ENDDCTRL_OFFSET, I2C_ENDDCTRL_SETEND);

  while (elapsed < I2C_TIMEOUT_US)
    {
      if (i2c_getreg(priv, TRICORE_I2C_PIRQSS_OFFSET) & I2C_PIRQ_TX_END)
        {
          i2c_putreg(priv, TRICORE_I2C_PIRQSC_OFFSET, I2C_PIRQ_TX_END);
          return OK;
        }

      up_udelay(I2C_POLL_INTERVAL_US);
      elapsed += I2C_POLL_INTERVAL_US;
    }

  return -ETIMEDOUT;
}

static int tricore_i2c_do_transfer(struct tricore_i2c_priv_s *priv,
                                   uint16_t addr, bool is_read,
                                   FAR struct i2c_msg_s *msgs,
                                   int nmsgs, bool send_stop)
{
  int ret;
  uint32_t total_bytes = 0;
  int offset;

  for (int i = 0; i < nmsgs; i++)
    {
      total_bytes += msgs[i].length;
    }

  tricore_i2c_clear_irqs(priv);

  i2c_putreg(priv, TRICORE_I2C_PIRQSM_OFFSET,
             I2C_PIRQ_AL | I2C_PIRQ_NACK | I2C_PIRQ_TX_END);

  if (is_read)
    {
      i2c_putreg(priv, TRICORE_I2C_TPSCTRL_OFFSET, 1);
      i2c_putreg(priv, TRICORE_I2C_MRPSCTRL_OFFSET, total_bytes);
    }
  else
    {
      i2c_putreg(priv, TRICORE_I2C_TPSCTRL_OFFSET,
                 1 + total_bytes); // +1 for address
      i2c_putreg(priv, TRICORE_I2C_MRPSCTRL_OFFSET, 0);
    }

  ret = tricore_i2c_wait_fifo_req(priv);
  if (ret < 0)
    {
      i2cerr("I2C%d: no fifo req for addr\n", priv->config->bus);
      goto err;
    }

  i2c_putreg(priv, TRICORE_I2C_TXD_OFFSET,
             ((addr << 1) & 0xfe) | (is_read ? 1 : 0));

  i2c_putreg(priv, TRICORE_I2C_ICR_OFFSET, I2C_INT_DTR_ALL);

  offset = 0;

  while (total_bytes > 0)
    {
      uint32_t val;
      uint32_t count = 1;

      ret = tricore_i2c_wait_fifo_req(priv);
      if (ret < 0)
        {
          goto err;
        }

      val = i2c_getreg(priv, TRICORE_I2C_RIS_OFFSET);

      if (val & (I2C_INT_LBREQ | I2C_INT_BREQ))
        {
          count = (total_bytes >= 4) ? 4 : total_bytes;
        }

      for (int i = 0; i < count && total_bytes > 0; i++)
        {
          if (is_read)
            {
              msgs[i].buffer[offset] = i2c_getreg(priv,
                                          TRICORE_I2C_RXD_OFFSET) & 0xff;
            }
          else
            {
              i2c_putreg(priv, TRICORE_I2C_TXD_OFFSET,
                         msgs[i].buffer[offset]);
            }

          offset++;
          total_bytes--;
          if (offset >= msgs[i].length)
            {
              i++;
              offset = 0;
            }
        }

      i2c_putreg(priv, TRICORE_I2C_ICR_OFFSET, I2C_INT_DTR_ALL);
    }

  ret = tricore_i2c_wait_pirq(priv);
  if (ret < 0)
    {
      goto err;
    }

  i2c_putreg(priv, TRICORE_I2C_PIRQSC_OFFSET, I2C_PIRQ_TX_END);

  if (send_stop)
    {
      ret = tricore_i2c_send_stop(priv);
    }

  return ret;

err:
  if ((i2c_getreg(priv, TRICORE_I2C_BUSSTAT_OFFSET) & I2C_BUSSTAT_BS_MASK)
      == I2C_BUSSTAT_BUSYMASTER)
    {
      tricore_i2c_send_stop(priv);
    }

  tricore_i2c_clear_irqs(priv);
  return ret;
}

static int tricore_i2c_setfrequency(struct tricore_i2c_priv_s *priv,
                                    uint32_t frequency)
{
  int ret;

  if (priv->frequency == frequency)
    {
      return OK;
    }

  i2c_putreg(priv, TRICORE_I2C_RUNCTRL_OFFSET, 0);
  ret = tricore_i2c_set_rate(priv, frequency);
  i2c_putreg(priv, TRICORE_I2C_RUNCTRL_OFFSET, I2C_RUNCTRL_RUN);
  return ret;
}

static int tricore_i2c_transfer(FAR struct i2c_master_s *dev,
                                FAR struct i2c_msg_s *msgs, int count)
{
  struct tricore_i2c_priv_s *priv = (struct tricore_i2c_priv_s *)dev;
  int ret;
  int i;

  DEBUGASSERT(priv != NULL && msgs != NULL && count > 0);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if ((i2c_getreg(priv, TRICORE_I2C_BUSSTAT_OFFSET) & I2C_BUSSTAT_BS_MASK)
      != I2C_BUSSTAT_IDLE)
    {
      ret = -EBUSY;
      goto out;
    }

  i = 0;
  while (i < count)
    {
      bool is_read = (msgs[i].flags & I2C_M_READ) != 0;
      uint16_t addr = msgs[i].addr;
      uint32_t freq = msgs[i].frequency;
      int start = i;
      int nmsgs;
      bool stop;

      ret = tricore_i2c_setfrequency(priv, freq);
      if (ret < 0)
        {
          goto out;
        }

      while (i < count)
        {
          bool cur_read = (msgs[i].flags & I2C_M_READ) != 0;

          if (cur_read != is_read)
            {
              break;
            }

          i++;
          if ((i < count) && !(msgs[i - 1].flags & I2C_M_NOSTOP))
            {
              break;
            }
        }

      nmsgs = i - start;

      stop = true;
      if ((msgs[i - 1].flags & I2C_M_NOSTOP) && i < count)
        {
          stop = false;
        }

      if (i >= count)
        {
          stop = true;
        }

      ret = tricore_i2c_do_transfer(priv, addr, is_read,
                                    &msgs[start], nmsgs, stop);
      if (ret < 0)
        {
          break;
        }
    }

out:
  nxmutex_unlock(&priv->lock);
  return ret;
}

#ifdef CONFIG_I2C_RESET
static int tricore_i2c_reset(FAR struct i2c_master_s *dev)
{
  struct tricore_i2c_priv_s *priv = (struct tricore_i2c_priv_s *)dev;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if ((i2c_getreg(priv, TRICORE_I2C_BUSSTAT_OFFSET) & I2C_BUSSTAT_BS_MASK)
      != I2C_BUSSTAT_IDLE)
    {
      tricore_i2c_send_stop(priv);
    }

  tricore_i2c_hw_deinit(priv);
  ret = tricore_i2c_hw_init(priv);

  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif

static int tricore_i2c_setup(FAR struct i2c_master_s *dev)
{
  return OK;
}

static int tricore_i2c_shutdown(FAR struct i2c_master_s *dev)
{
  return OK;
}
