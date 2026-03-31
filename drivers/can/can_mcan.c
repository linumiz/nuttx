/****************************************************************************
 * drivers/can/can_mcan.c
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

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/clock.h>
#include <nuttx/bits.h>

#include <nuttx/can/can_mcan.h>
#include <nuttx/can/can_common.h>
#include "can_mcan_regs.h"

#define CAN_INIT_TIMEOUT_MS  100U

#ifndef ROUND_UP
#  define ROUND_UP(x, y)  (((x) + ((y) - 1)) & ~((y) - 1))
#endif

#define MIN(a, b)  ((a) < (b) ? (a) : (b))

#define CAN_CALC_TDCO(timing, tx_delay, max_tdco) \
    MIN((uint8_t)((timing)->phase_seg1 * (timing)->prescaler), (uint8_t)(max_tdco))

static int can_mcan_calc_timing(struct can_mcan_priv_s *priv,
                                struct can_timing_s *timing,
                                uint32_t bitrate, uint16_t sample_pnt)
{
  /* TODO: implement bit timing calculation for nominal phase */
  UNUSED(priv);
  UNUSED(bitrate);
  UNUSED(sample_pnt);

  timing->sjw        = 1;
  timing->phase_seg1 = 13;
  timing->phase_seg2 = 2;
  timing->prescaler  = 1;

  return 0;
}

#ifdef CONFIG_CAN_FD_MODE
static int can_mcan_calc_timing_data(struct can_mcan_priv_s *priv,
                                     struct can_timing_s *timing,
                                     uint32_t bitrate, uint16_t sample_pnt)
{
  /* TODO: implement bit timing calculation for FD data phase */
  UNUSED(priv);
  UNUSED(bitrate);
  UNUSED(sample_pnt);

  timing->sjw        = 1;
  timing->phase_seg1 = 4;
  timing->phase_seg2 = 1;
  timing->prescaler  = 1;

  return 0;
}
#endif /* CONFIG_CAN_FD_MODE */

/* Transceiver stubs - TODO: call platform PHY ops                    */
static inline int can_transceiver_enable(struct can_mcan_priv_s *priv,
                                         uint32_t mode)
{
  UNUSED(priv);
  UNUSED(mode);
  return 0;
}

static inline int can_transceiver_disable(struct can_mcan_priv_s *priv)
{
  UNUSED(priv);
  return 0;
}

int can_mcan_read_reg(struct can_mcan_priv_s *priv, uint16_t reg,
                      uint32_t *val)
{
  const struct can_mcan_config *config = priv->config;
  int err;

  err = config->ops->read_reg(priv, reg, val);
  if (err != 0)
    {
      canerr("failed to read reg 0x%03x (err %d)", reg, err);
    }

  return err;
}

int can_mcan_write_reg(struct can_mcan_priv_s *priv, uint16_t reg,
                       uint32_t val)
{
  const struct can_mcan_config *config = priv->config;
  int err;

  err = config->ops->write_reg(priv, reg, val);
  if (err != 0)
    {
      canerr("failed to write reg 0x%03x (err %d)", reg, err);
    }

  return err;
}

static int can_mcan_exit_sleep_mode(struct can_mcan_priv_s *priv)
{
  struct can_mcan_data *data = priv->data;
  clock_t start_time;
  uint32_t cccr;
  int err;

  nxmutex_lock(&data->lock);

  err = can_mcan_read_reg(priv, CAN_MCAN_CCCR, &cccr);
  if (err != 0)
    {
      goto unlock;
    }

  cccr &= ~CAN_MCAN_CCCR_CSR;

  err = can_mcan_write_reg(priv, CAN_MCAN_CCCR, cccr);
  if (err != 0)
    {
      goto unlock;
    }

  start_time = clock_systime_ticks();

  err = can_mcan_read_reg(priv, CAN_MCAN_CCCR, &cccr);
  if (err != 0)
    {
      goto unlock;
    }

  while ((cccr & CAN_MCAN_CCCR_CSA) == CAN_MCAN_CCCR_CSA)
    {
      if (clock_systime_ticks() - start_time > MSEC2TICK(CAN_INIT_TIMEOUT_MS))
        {
          cccr |= CAN_MCAN_CCCR_CSR;
          err = can_mcan_write_reg(priv, CAN_MCAN_CCCR, cccr);
          if (err != 0)
            {
              goto unlock;
            }

          err = -EAGAIN;
          goto unlock;
        }

      err = can_mcan_read_reg(priv, CAN_MCAN_CCCR, &cccr);
      if (err != 0)
        {
          goto unlock;
        }
    }

unlock:
  nxmutex_unlock(&data->lock);
  return err;
}

static int can_mcan_enter_init_mode(struct can_mcan_priv_s *priv,
                                    uint32_t timeout_ms)
{
  struct can_mcan_data *data = priv->data;
  clock_t start_time;
  uint32_t cccr;
  int err;

  nxmutex_lock(&data->lock);

  err = can_mcan_read_reg(priv, CAN_MCAN_CCCR, &cccr);
  if (err != 0)
    {
      goto unlock;
    }

  cccr |= CAN_MCAN_CCCR_INIT;

  err = can_mcan_write_reg(priv, CAN_MCAN_CCCR, cccr);
  if (err != 0)
    {
      goto unlock;
    }

  start_time = clock_systime_ticks();

  err = can_mcan_read_reg(priv, CAN_MCAN_CCCR, &cccr);
  if (err != 0)
    {
      goto unlock;
    }

  while ((cccr & CAN_MCAN_CCCR_INIT) == 0u)
    {
      if (clock_systime_ticks() - start_time > MSEC2TICK(timeout_ms))
        {
          cccr &= ~CAN_MCAN_CCCR_INIT;
          err = can_mcan_write_reg(priv, CAN_MCAN_CCCR, cccr);
          if (err != 0)
            {
              goto unlock;
            }

          err = -EAGAIN;
          goto unlock;
        }

      err = can_mcan_read_reg(priv, CAN_MCAN_CCCR, &cccr);
      if (err != 0)
        {
          goto unlock;
        }
    }

unlock:
  nxmutex_unlock(&data->lock);
  return err;
}

static int can_mcan_leave_init_mode(struct can_mcan_priv_s *priv,
                                    uint32_t timeout_ms)
{
  struct can_mcan_data *data = priv->data;
  clock_t start_time;
  uint32_t cccr;
  int err;

  nxmutex_lock(&data->lock);

  err = can_mcan_read_reg(priv, CAN_MCAN_CCCR, &cccr);
  if (err != 0)
    {
      goto unlock;
    }

  cccr &= ~CAN_MCAN_CCCR_INIT;

  err = can_mcan_write_reg(priv, CAN_MCAN_CCCR, cccr);
  if (err != 0)
    {
      goto unlock;
    }

  start_time = clock_systime_ticks();

  err = can_mcan_read_reg(priv, CAN_MCAN_CCCR, &cccr);
  if (err != 0)
    {
      goto unlock;
    }

  while ((cccr & CAN_MCAN_CCCR_INIT) != 0u)
    {
      if (clock_systime_ticks() - start_time > MSEC2TICK(timeout_ms))
        {
          err = -EAGAIN;
          goto unlock;
        }

      err = can_mcan_read_reg(priv, CAN_MCAN_CCCR, &cccr);
      if (err != 0)
        {
          goto unlock;
        }
    }

unlock:
  nxmutex_unlock(&data->lock);
  return err;
}

int can_mcan_set_timing(struct can_mcan_priv_s *priv,
                        const struct can_timing_s *timing)
{
  struct can_mcan_data *data = priv->data;
  uint32_t nbtp = 0u;
  int err;

  if (data->started)
    {
      return -EBUSY;
    }

  nxmutex_lock(&data->lock);

  nbtp |= FIELD_PREP(CAN_MCAN_NBTP_NSJW,   timing->sjw - 1ul)        |
          FIELD_PREP(CAN_MCAN_NBTP_NTSEG1,  timing->phase_seg1 - 1ul) |
          FIELD_PREP(CAN_MCAN_NBTP_NTSEG2,  timing->phase_seg2 - 1ul) |
          FIELD_PREP(CAN_MCAN_NBTP_NBRP,    timing->prescaler - 1ul);

  err = can_mcan_write_reg(priv, CAN_MCAN_NBTP, nbtp);

  nxmutex_unlock(&data->lock);
  return err;
}

#ifdef CONFIG_CAN_FD_MODE
int can_mcan_set_timing_data(struct can_mcan_priv_s *priv,
                             const struct can_timing_s *timing_data)
{
  const uint8_t tdco_max = FIELD_GET(CAN_MCAN_TDCR_TDCO, CAN_MCAN_TDCR_TDCO);
  struct can_mcan_data *data = priv->data;
  uint32_t dbtp = 0u;
  uint8_t tdco;
  int err;

  if (data->started)
    {
      return -EBUSY;
    }

  nxmutex_lock(&data->lock);

  dbtp |= FIELD_PREP(CAN_MCAN_DBTP_DSJW,   timing_data->sjw - 1ul)        |
          FIELD_PREP(CAN_MCAN_DBTP_DTSEG1,  timing_data->phase_seg1 - 1ul) |
          FIELD_PREP(CAN_MCAN_DBTP_DTSEG2,  timing_data->phase_seg2 - 1ul) |
          FIELD_PREP(CAN_MCAN_DBTP_DBRP,    timing_data->prescaler - 1ul);

  if (timing_data->prescaler == 1u || timing_data->prescaler == 2u)
    {
      /* TDC only valid when DBRP = {0,1} */
      dbtp |= CAN_MCAN_DBTP_TDC;

      tdco = CAN_CALC_TDCO(timing_data, 0u, tdco_max);
      caninfo("TDC enabled, TDCO %u", tdco);

      err = can_mcan_write_reg(priv, CAN_MCAN_TDCR,
                               FIELD_PREP(CAN_MCAN_TDCR_TDCO, tdco));
      if (err != 0)
        {
          goto unlock;
        }
    }
  else
    {
      caninfo("TDC disabled, prescaler %u too high", timing_data->prescaler);
    }

  err = can_mcan_write_reg(priv, CAN_MCAN_DBTP, dbtp);

unlock:
  nxmutex_unlock(&data->lock);
  return err;
}
#endif /* CONFIG_CAN_FD_MODE */

int can_mcan_get_capabilities(struct can_mcan_priv_s *priv,
                               can_mode_t *cap)
{
  UNUSED(priv);

  *cap = CAN_MODE_NORMAL | CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY;

#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
  *cap |= CAN_MODE_MANUAL_RECOVERY;
#endif

#ifdef CONFIG_CAN_FD_MODE
  *cap |= CAN_MODE_FD;
#endif

  return 0;
}

int can_mcan_set_mode(struct can_mcan_priv_s *priv, can_mode_t mode)
{
  can_mode_t supported = CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY;
  struct can_mcan_data *data = priv->data;
  uint32_t cccr;
  uint32_t test;
  int err;

#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
  supported |= CAN_MODE_MANUAL_RECOVERY;
#endif

#ifdef CONFIG_CAN_FD_MODE
  supported |= CAN_MODE_FD;
#endif

  if ((mode & ~supported) != 0u)
    {
      canerr("unsupported mode: 0x%08x", mode);
      return -ENOTSUP;
    }

  if (data->started)
    {
      return -EBUSY;
    }

  nxmutex_lock(&data->lock);

  err = can_mcan_read_reg(priv, CAN_MCAN_CCCR, &cccr);
  if (err != 0)
    {
      goto unlock;
    }

  err = can_mcan_read_reg(priv, CAN_MCAN_TEST, &test);
  if (err != 0)
    {
      goto unlock;
    }

  if ((mode & CAN_MODE_LOOPBACK) != 0)
    {
      cccr |= CAN_MCAN_CCCR_TEST;
      test |= CAN_MCAN_TEST_LBCK;
    }
  else
    {
      cccr &= ~CAN_MCAN_CCCR_TEST;
    }

  if ((mode & CAN_MODE_LISTENONLY) != 0)
    {
      cccr |= CAN_MCAN_CCCR_MON;
    }
  else
    {
      cccr &= ~CAN_MCAN_CCCR_MON;
    }

#ifdef CONFIG_CAN_FD_MODE
  if ((mode & CAN_MODE_FD) != 0)
    {
      cccr |= CAN_MCAN_CCCR_FDOE | CAN_MCAN_CCCR_BRSE;
    }
  else
    {
      cccr &= ~(CAN_MCAN_CCCR_FDOE | CAN_MCAN_CCCR_BRSE);
    }
#endif

  err = can_mcan_write_reg(priv, CAN_MCAN_CCCR, cccr);
  if (err != 0)
    {
      goto unlock;
    }

  err = can_mcan_write_reg(priv, CAN_MCAN_TEST, test);
  if (err != 0)
    {
      goto unlock;
    }

  data->mode = mode;

unlock:
  nxmutex_unlock(&data->lock);
  return err;
}

int can_mcan_start(struct can_mcan_priv_s *priv)
{
  const struct can_mcan_config *config = priv->config;
  struct can_mcan_data *data = priv->data;
  int err = 0;

  if (data->started)
    {
      return -EALREADY;
    }

  if (config->phy_present)
    {
      err = can_transceiver_enable(priv, data->mode);
      if (err != 0)
        {
          canerr("failed to enable CAN transceiver (err %d)", err);
          return err;
        }
    }

  err = can_mcan_leave_init_mode(priv, CAN_INIT_TIMEOUT_MS);
  if (err != 0)
    {
      canerr("failed to leave init mode (err %d)", err);

      if (config->phy_present)
        {
          can_transceiver_disable(priv);
        }

      return err;
    }

  data->started = true;
  return 0;
}

int can_mcan_stop(struct can_mcan_priv_s *priv)
{
  const struct can_mcan_config *config = priv->config;
  const struct can_mcan_callbacks *cbs = config->callbacks;
  struct can_mcan_data *data = priv->data;
  can_tx_callback_t tx_cb;
  uint32_t tx_idx;
  int err;

  if (!data->started)
    {
      return -EALREADY;
    }

  /* transmissions auto-stop in init mode */
  err = can_mcan_enter_init_mode(priv, CAN_INIT_TIMEOUT_MS);
  if (err != 0)
    {
      canerr("failed to enter init mode (err %d)", err);
      return -EIO;
    }

  if (config->phy_present)
    {
      err = can_transceiver_disable(priv);
      if (err != 0)
        {
          canerr("failed to disable CAN transceiver (err %d)", err);
          return err;
        }
    }

  can_mcan_enable_configuration_change(priv);

  data->started = false;

  for (tx_idx = 0u; tx_idx < cbs->num_tx; tx_idx++)
    {
      tx_cb = cbs->tx[tx_idx].function;

      if (tx_cb != NULL)
        {
          cbs->tx[tx_idx].function = NULL;
          tx_cb(priv, -ENETDOWN, cbs->tx[tx_idx].user_data);
          nxsem_post(&data->tx_sem);
        }
    }

  return 0;
}

static int can_mcan_read_psr(struct can_mcan_priv_s *priv, uint32_t *val)
{
  /* Reading PSR clears LEC fields - always use this wrapper */
  int err = can_mcan_read_reg(priv, CAN_MCAN_PSR, val);

  if (err != 0)
    {
      return err;
    }

  return 0;
}

int can_mcan_get_state(struct can_mcan_priv_s *priv,
                       enum can_state_e *state,
                       struct can_bus_err_cnt_s *err_cnt)
{
  struct can_mcan_data *data = priv->data;
  uint32_t reg;
  int err;

  if (state != NULL)
    {
      err = can_mcan_read_psr(priv, &reg);
      if (err != 0)
        {
          return err;
        }

      if (!data->started)
        {
          *state = CAN_STATE_STOPPED;
        }
      else if ((reg & CAN_MCAN_PSR_BO) != 0u)
        {
          *state = CAN_STATE_BUS_OFF;
        }
      else if ((reg & CAN_MCAN_PSR_EP) != 0u)
        {
          *state = CAN_STATE_ERROR_PASSIVE;
        }
      else if ((reg & CAN_MCAN_PSR_EW) != 0u)
        {
          *state = CAN_STATE_ERROR_WARNING;
        }
      else
        {
          *state = CAN_STATE_ERROR_ACTIVE;
        }
    }

  if (err_cnt != NULL)
    {
      err = can_mcan_read_reg(priv, CAN_MCAN_ECR, &reg);
      if (err != 0)
        {
          return err;
        }

      err_cnt->tx_err_cnt = FIELD_GET(CAN_MCAN_ECR_TEC, reg);
      err_cnt->rx_err_cnt = FIELD_GET(CAN_MCAN_ECR_REC, reg);
    }

  return 0;
}

void can_mcan_set_state_change_callback(
    struct can_mcan_priv_s *priv,
    can_state_change_callback_t callback,
    void *user_data)
{
  struct can_mcan_data *data = priv->data;

  data->state_change_cb           = callback;
  data->state_change_cb_user_data = user_data;
}

#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
int can_mcan_recover(struct can_mcan_priv_s *priv, uint32_t timeout_ms)
{
  struct can_mcan_data *data = priv->data;

  if (!data->started)
    {
      return -ENETDOWN;
    }

  if ((data->mode & CAN_MODE_MANUAL_RECOVERY) == 0u)
    {
      return -ENOTSUP;
    }

  return can_mcan_leave_init_mode(priv, timeout_ms);
}
#endif /* CONFIG_CAN_MANUAL_RECOVERY_MODE */

void can_mcan_enable_configuration_change(struct can_mcan_priv_s *priv)
{
  struct can_mcan_data *data = priv->data;
  uint32_t cccr;
  int err;

  nxmutex_lock(&data->lock);

  err = can_mcan_read_reg(priv, CAN_MCAN_CCCR, &cccr);
  if (err != 0)
    {
      goto unlock;
    }

  cccr |= CAN_MCAN_CCCR_CCE;

  can_mcan_write_reg(priv, CAN_MCAN_CCCR, cccr);

unlock:
  nxmutex_unlock(&data->lock);
}

int can_mcan_configure_mram(struct can_mcan_priv_s *priv,
                             uintptr_t mrba, uintptr_t mram)
{
  const struct can_mcan_config *config = priv->config;
  uint32_t addr;
  uint32_t reg;
  int err;

  err = can_mcan_exit_sleep_mode(priv);
  if (err != 0)
    {
      canerr("failed to exit sleep mode");
      return -EIO;
    }

  err = can_mcan_enter_init_mode(priv, CAN_INIT_TIMEOUT_MS);
  if (err != 0)
    {
      canerr("failed to enter init mode");
      return -EIO;
    }

  can_mcan_enable_configuration_change(priv);

  addr = mram - mrba + config->mram_offsets[CAN_MCAN_MRAM_CFG_STD_FILTER];
  reg  = (addr & CAN_MCAN_SIDFC_FLSSA) |
         FIELD_PREP(CAN_MCAN_SIDFC_LSS,
                    config->mram_elements[CAN_MCAN_MRAM_CFG_STD_FILTER]);
  err = can_mcan_write_reg(priv, CAN_MCAN_SIDFC, reg);
  if (err != 0)
    {
      return err;
    }

  addr = mram - mrba + config->mram_offsets[CAN_MCAN_MRAM_CFG_EXT_FILTER];
  reg  = (addr & CAN_MCAN_XIDFC_FLESA) |
         FIELD_PREP(CAN_MCAN_XIDFC_LSS,
                    config->mram_elements[CAN_MCAN_MRAM_CFG_EXT_FILTER]);
  err = can_mcan_write_reg(priv, CAN_MCAN_XIDFC, reg);
  if (err != 0)
    {
      return err;
    }

  addr = mram - mrba + config->mram_offsets[CAN_MCAN_MRAM_CFG_RX_FIFO0];
  reg  = (addr & CAN_MCAN_RXF0C_F0SA) |
         FIELD_PREP(CAN_MCAN_RXF0C_F0S,
                    config->mram_elements[CAN_MCAN_MRAM_CFG_RX_FIFO0]);
  err = can_mcan_write_reg(priv, CAN_MCAN_RXF0C, reg);
  if (err != 0)
    {
      return err;
    }

  addr = mram - mrba + config->mram_offsets[CAN_MCAN_MRAM_CFG_RX_FIFO1];
  reg  = (addr & CAN_MCAN_RXF1C_F1SA) |
         FIELD_PREP(CAN_MCAN_RXF1C_F1S,
                    config->mram_elements[CAN_MCAN_MRAM_CFG_RX_FIFO1]);
  err = can_mcan_write_reg(priv, CAN_MCAN_RXF1C, reg);
  if (err != 0)
    {
      return err;
    }

  addr = mram - mrba + config->mram_offsets[CAN_MCAN_MRAM_CFG_RX_BUFFER];
  reg  = (addr & CAN_MCAN_RXBC_RBSA);
  err  = can_mcan_write_reg(priv, CAN_MCAN_RXBC, reg);
  if (err != 0)
    {
      return err;
    }

  addr = mram - mrba + config->mram_offsets[CAN_MCAN_MRAM_CFG_TX_EVENT_FIFO];
  reg  = (addr & CAN_MCAN_TXEFC_EFSA) |
         FIELD_PREP(CAN_MCAN_TXEFC_EFS,
                    config->mram_elements[CAN_MCAN_MRAM_CFG_TX_EVENT_FIFO]);
  err = can_mcan_write_reg(priv, CAN_MCAN_TXEFC, reg);
  if (err != 0)
    {
      return err;
    }

  addr = mram - mrba + config->mram_offsets[CAN_MCAN_MRAM_CFG_TX_BUFFER];
  reg  = (addr & CAN_MCAN_TXBC_TBSA) |
         FIELD_PREP(CAN_MCAN_TXBC_TFQS,
                    config->mram_elements[CAN_MCAN_MRAM_CFG_TX_BUFFER]) |
         CAN_MCAN_TXBC_TFQM;
  err = can_mcan_write_reg(priv, CAN_MCAN_TXBC, reg);
  if (err != 0)
    {
      return err;
    }

  err = can_mcan_write_reg(priv, CAN_MCAN_TXESC, CAN_MCAN_TXESC_TBDS);
  if (err != 0)
    {
      return err;
    }

  err = can_mcan_write_reg(priv, CAN_MCAN_RXESC,
                           CAN_MCAN_RXESC_RBDS |
                           CAN_MCAN_RXESC_F1DS |
                           CAN_MCAN_RXESC_F0DS);
  if (err != 0)
    {
      return err;
    }

  return 0;
}

int can_mcan_send(struct can_mcan_priv_s *priv,
                  const struct can_frame_s *frame,
                  uint32_t timeout_ms,
                  can_tx_callback_t callback,
                  void *user_data)
{
  const struct can_mcan_config *config = priv->config;
  const struct can_mcan_callbacks *cbs = config->callbacks;
  struct can_mcan_data *data = priv->data;
  size_t data_length = can_dlc2bytes(frame->dlc);
  struct can_mcan_tx_buffer_hdr tx_hdr =
    {
      .rtr = (frame->flags & CAN_FRAME_RTR) != 0 ? 1 : 0,
      .xtd = (frame->flags & CAN_FRAME_IDE) != 0 ? 1 : 0,
      .esi = 0,
      .dlc = frame->dlc,
#ifdef CONFIG_CAN_FD_MODE
      .fdf = (frame->flags & CAN_FRAME_FDF) != 0 ? 1 : 0,
      .brs = (frame->flags & CAN_FRAME_BRS) != 0 ? 1 : 0,
#else
      .fdf = 0,
      .brs = 0,
#endif
      .efc = 1,
    };
  uint32_t put_idx = UINT32_MAX;
  uint32_t reg;
  int err;

  caninfo("Sending %zu bytes. Id: 0x%x, IDE: %d RTR: %d FDF: %d BRS: %d",
          data_length, frame->id,
          (frame->flags & CAN_FRAME_IDE) != 0u,
          (frame->flags & CAN_FRAME_RTR) != 0u,
          (frame->flags & CAN_FRAME_FDF) != 0u,
          (frame->flags & CAN_FRAME_BRS) != 0u);

#ifdef CONFIG_CAN_FD_MODE
  if ((frame->flags &
       ~(CAN_FRAME_IDE | CAN_FRAME_RTR | CAN_FRAME_FDF | CAN_FRAME_BRS)) != 0)
    {
      canerr("unsupported frame flags 0x%02x", frame->flags);
      return -ENOTSUP;
    }

  if ((data->mode & CAN_MODE_FD) == 0u &&
      ((frame->flags & (CAN_FRAME_FDF | CAN_FRAME_BRS)) != 0u))
    {
      canerr("FD frame in non-FD mode");
      return -ENOTSUP;
    }
#else
  if ((frame->flags & ~(CAN_FRAME_IDE | CAN_FRAME_RTR)) != 0u)
    {
      canerr("unsupported frame flags 0x%02x", frame->flags);
      return -ENOTSUP;
    }
#endif

  if (data_length > sizeof(frame->data))
    {
      canerr("data length (%zu) > max frame data length (%zu)",
              data_length, sizeof(frame->data));
      return -EINVAL;
    }

  if ((frame->flags & CAN_FRAME_FDF) != 0u)
    {
      if (frame->dlc > CANFD_MAX_DLC)
        {
          canerr("DLC %d too large for CAN FD", frame->dlc);
          return -EINVAL;
        }
    }
  else
    {
      if (frame->dlc > CAN_MAX_DLC)
        {
          canerr("DLC %d too large for CAN", frame->dlc);
          return -EINVAL;
        }
    }

  if (!data->started)
    {
      return -ENETDOWN;
    }

  err = can_mcan_read_psr(priv, &reg);
  if (err != 0)
    {
      return err;
    }

  if ((reg & CAN_MCAN_PSR_BO) != 0)
    {
      return -ENETUNREACH;
    }

  err = nxsem_tickwait_uninterruptible(&data->tx_sem, MSEC2TICK(timeout_ms));
  if (err != 0)
    {
      return -EAGAIN;
    }

  nxmutex_lock(&data->tx_mtx);

  for (int i = 0; i < cbs->num_tx; i++)
    {
      if (cbs->tx[i].function == NULL)
        {
          put_idx = i;
          break;
        }
    }

  tx_hdr.mm = put_idx;

  if ((frame->flags & CAN_FRAME_IDE) != 0)
    {
      tx_hdr.ext_id = frame->id;
    }
  else
    {
      tx_hdr.std_id = frame->id & CAN_STD_ID_MASK;
    }

  err = can_mcan_write_mram(priv,
                             config->mram_offsets[CAN_MCAN_MRAM_CFG_TX_BUFFER] +
                             put_idx * sizeof(struct can_mcan_tx_buffer) +
                             offsetof(struct can_mcan_tx_buffer, hdr),
                             &tx_hdr, sizeof(struct can_mcan_tx_buffer_hdr));
  if (err != 0)
    {
      canerr("failed to write Tx Buffer header (err %d)", err);
      goto err_unlock;
    }

  if ((frame->flags & CAN_FRAME_RTR) == 0 && data_length != 0)
    {
      err = can_mcan_write_mram(priv,
                                config->mram_offsets[CAN_MCAN_MRAM_CFG_TX_BUFFER] +
                                put_idx * sizeof(struct can_mcan_tx_buffer) +
                                offsetof(struct can_mcan_tx_buffer, data_32),
                                &frame->data_32,
                                ROUND_UP(data_length, sizeof(uint32_t)));
      if (err != 0)
        {
          canerr("failed to write Tx Buffer data (err %d)", err);
          goto err_unlock;
        }
    }

  DEBUGASSERT(put_idx < cbs->num_tx);
  cbs->tx[put_idx].function  = callback;
  cbs->tx[put_idx].user_data = user_data;

  err = can_mcan_write_reg(priv, CAN_MCAN_TXBAR, BIT(put_idx));
  if (err != 0)
    {
      cbs->tx[put_idx].function = NULL;
      goto err_unlock;
    }

  nxmutex_unlock(&data->tx_mtx);
  return 0;

err_unlock:
  nxmutex_unlock(&data->tx_mtx);
  nxsem_post(&data->tx_sem);
  return err;
}

int can_mcan_get_max_filters(struct can_mcan_priv_s *priv, bool ide)
{
  const struct can_mcan_config *config = priv->config;
  const struct can_mcan_callbacks *cbs = config->callbacks;

  return ide ? cbs->num_ext : cbs->num_std;
}

int can_mcan_add_rx_filter_std(struct can_mcan_priv_s *priv,
                                can_rx_callback_t callback,
                                void *user_data,
                                const struct can_filter_s *filter)
{
  const struct can_mcan_config *config = priv->config;
  const struct can_mcan_callbacks *cbs = config->callbacks;
  struct can_mcan_data *data = priv->data;
  struct can_mcan_std_filter filter_element =
    {
      .sfid1 = filter->id,
      .sfid2 = filter->mask,
      .sft   = CAN_MCAN_SFT_CLASSIC
    };
  int filter_id = -ENOSPC;
  int err;
  int i;

  nxmutex_lock(&data->lock);

  for (i = 0; i < cbs->num_std; i++)
    {
      if (cbs->std[i].function == NULL)
        {
          filter_id = i;
          break;
        }
    }

  if (filter_id == -ENOSPC)
    {
      canwarn("No free standard filter");
      nxmutex_unlock(&data->lock);
      return -ENOSPC;
    }

  filter_element.sfec = (filter_id & 0x01) ?
                        CAN_MCAN_XFEC_FIFO1 : CAN_MCAN_XFEC_FIFO0;

  err = can_mcan_write_mram(priv,
                             config->mram_offsets[CAN_MCAN_MRAM_CFG_STD_FILTER] +
                             filter_id * sizeof(struct can_mcan_std_filter),
                             &filter_element, sizeof(filter_element));
  if (err != 0)
    {
      canerr("failed to write std filter (err %d)", err);
      nxmutex_unlock(&data->lock);
      return err;
    }

  nxmutex_unlock(&data->lock);

  caninfo("Attached std filter at %d", filter_id);

  DEBUGASSERT(filter_id < cbs->num_std);
  cbs->std[filter_id].function  = callback;
  cbs->std[filter_id].user_data = user_data;

  return filter_id;
}

static int can_mcan_add_rx_filter_ext(struct can_mcan_priv_s *priv,
                                      can_rx_callback_t callback,
                                      void *user_data,
                                      const struct can_filter_s *filter)
{
  const struct can_mcan_config *config = priv->config;
  const struct can_mcan_callbacks *cbs = config->callbacks;
  struct can_mcan_data *data = priv->data;
  struct can_mcan_ext_filter filter_element =
    {
      .efid2 = filter->mask,
      .efid1 = filter->id,
      .eft   = CAN_MCAN_EFT_CLASSIC
    };
  int filter_id = -ENOSPC;
  int err;
  int i;

  nxmutex_lock(&data->lock);

  for (i = 0; i < cbs->num_ext; i++)
    {
      if (cbs->ext[i].function == NULL)
        {
          filter_id = i;
          break;
        }
    }

  if (filter_id == -ENOSPC)
    {
      canwarn("No free extended filter");
      nxmutex_unlock(&data->lock);
      return -ENOSPC;
    }

  /* TODO: proper FIFO balancing */
  filter_element.efec = (filter_id & 0x01) ?
                        CAN_MCAN_XFEC_FIFO1 : CAN_MCAN_XFEC_FIFO0;

  err = can_mcan_write_mram(priv,
                             config->mram_offsets[CAN_MCAN_MRAM_CFG_EXT_FILTER] +
                             filter_id * sizeof(struct can_mcan_ext_filter),
                             &filter_element, sizeof(filter_element));
  if (err != 0)
    {
      canerr("failed to write ext filter (err %d)", err);
      nxmutex_unlock(&data->lock);
      return err;
    }

  nxmutex_unlock(&data->lock);

  caninfo("Attached ext filter at %d", filter_id);

  DEBUGASSERT(filter_id < cbs->num_ext);
  cbs->ext[filter_id].function  = callback;
  cbs->ext[filter_id].user_data = user_data;

  return filter_id;
}

int can_mcan_add_rx_filter(struct can_mcan_priv_s *priv,
                            can_rx_callback_t callback,
                            void *user_data,
                            const struct can_filter_s *filter)
{
  const struct can_mcan_config *config = priv->config;
  const struct can_mcan_callbacks *cbs = config->callbacks;
  int filter_id;

  if ((filter->flags & ~(CAN_FILTER_IDE)) != 0)
    {
      canerr("unsupported filter flags 0x%02x", filter->flags);
      return -ENOTSUP;
    }

  if ((filter->flags & CAN_FILTER_IDE) != 0)
    {
      filter_id = can_mcan_add_rx_filter_ext(priv, callback, user_data,
                                             filter);
      if (filter_id >= 0)
        {
          filter_id += cbs->num_std;
        }
    }
  else
    {
      filter_id = can_mcan_add_rx_filter_std(priv, callback, user_data,
                                             filter);
    }

  return filter_id;
}

void can_mcan_remove_rx_filter(struct can_mcan_priv_s *priv, int filter_id)
{
  const struct can_mcan_config *config = priv->config;
  const struct can_mcan_callbacks *cbs = config->callbacks;
  struct can_mcan_data *data = priv->data;
  int err;

  if (filter_id < 0)
    {
      canerr("filter ID %d out of bounds", filter_id);
      return;
    }

  nxmutex_lock(&data->lock);

  if (filter_id >= cbs->num_std)
    {
      filter_id -= cbs->num_std;

      if (filter_id >= cbs->num_ext)
        {
          canerr("filter ID %d out of bounds", filter_id);
          nxmutex_unlock(&data->lock);
          return;
        }

      cbs->ext[filter_id].function  = NULL;
      cbs->ext[filter_id].user_data = NULL;

      err = can_mcan_clear_mram(priv,
                                config->mram_offsets[CAN_MCAN_MRAM_CFG_EXT_FILTER] +
                                filter_id * sizeof(struct can_mcan_ext_filter),
                                sizeof(struct can_mcan_ext_filter));
      if (err != 0)
        {
          canerr("failed to clear ext filter (err %d)", err);
        }
    }
  else
    {
      cbs->std[filter_id].function  = NULL;
      cbs->std[filter_id].user_data = NULL;

      err = can_mcan_clear_mram(priv,
                                config->mram_offsets[CAN_MCAN_MRAM_CFG_STD_FILTER] +
                                filter_id * sizeof(struct can_mcan_std_filter),
                                sizeof(struct can_mcan_std_filter));
      if (err != 0)
        {
          canerr("failed to clear std filter (err %d)", err);
        }
    }

  nxmutex_unlock(&data->lock);
}

static void can_mcan_state_change_handler(struct can_mcan_priv_s *priv)
{
  const struct can_mcan_config *config = priv->config;
  const struct can_mcan_callbacks *cbs = config->callbacks;
  struct can_mcan_data *data = priv->data;
  const can_state_change_callback_t state_cb = data->state_change_cb;
  void *state_cb_data = data->state_change_cb_user_data;
  struct can_bus_err_cnt_s err_cnt;
  enum can_state_e state;
  can_tx_callback_t tx_cb;
  uint32_t tx_idx;
  uint32_t cccr;
  int err;

  err = can_mcan_get_state(priv, &state, &err_cnt);
  if (err != 0)
    {
      return;
    }

  if (state_cb != NULL)
    {
      state_cb(priv, state, err_cnt, state_cb_data);
    }

  if (state == CAN_STATE_BUS_OFF)
    {
      err = can_mcan_write_reg(priv, CAN_MCAN_TXBCR, CAN_MCAN_TXBCR_CR);
      if (err != 0)
        {
          return;
        }

      for (tx_idx = 0; tx_idx < cbs->num_tx; tx_idx++)
        {
          tx_cb = cbs->tx[tx_idx].function;

          if (tx_cb != NULL)
            {
              cbs->tx[tx_idx].function = NULL;
              tx_cb(priv, -ENETUNREACH, cbs->tx[tx_idx].user_data);
              nxsem_post(&data->tx_sem);
            }
        }

#ifndef CONFIG_CAN_MANUAL_RECOVERY_MODE
      /* auto recovery: clear INIT without lock (we are in ISR) */
      err = can_mcan_read_reg(priv, CAN_MCAN_CCCR, &cccr);
      if (err != 0)
        {
          return;
        }

      cccr &= ~CAN_MCAN_CCCR_INIT;
      can_mcan_write_reg(priv, CAN_MCAN_CCCR, cccr);
#else
      if ((data->mode & CAN_MODE_MANUAL_RECOVERY) == 0u)
        {
          err = can_mcan_read_reg(priv, CAN_MCAN_CCCR, &cccr);
          if (err != 0)
            {
              return;
            }

          cccr &= ~CAN_MCAN_CCCR_INIT;
          can_mcan_write_reg(priv, CAN_MCAN_CCCR, cccr);
        }
#endif
    }
}

static void can_mcan_tx_event_handler(struct can_mcan_priv_s *priv)
{
  const struct can_mcan_config *config = priv->config;
  const struct can_mcan_callbacks *cbs = config->callbacks;
  struct can_mcan_data *data = priv->data;
  struct can_mcan_tx_event_fifo tx_event;
  can_tx_callback_t tx_cb;
  void *user_data;
  uint32_t event_idx;
  uint32_t tx_idx;
  uint32_t txefs;
  int err;

  err = can_mcan_read_reg(priv, CAN_MCAN_TXEFS, &txefs);
  if (err != 0)
    {
      return;
    }

  while ((txefs & CAN_MCAN_TXEFS_EFFL) != 0u)
    {
      event_idx = FIELD_GET(CAN_MCAN_TXEFS_EFGI, txefs);

      err = can_mcan_read_mram(priv,
                               config->mram_offsets[CAN_MCAN_MRAM_CFG_TX_EVENT_FIFO] +
                               event_idx * sizeof(struct can_mcan_tx_event_fifo),
                               &tx_event,
                               sizeof(struct can_mcan_tx_event_fifo));
      if (err != 0)
        {
          canerr("failed to read tx event fifo (err %d)", err);
          return;
        }

      tx_idx = tx_event.mm;

      err = can_mcan_write_reg(priv, CAN_MCAN_TXEFA, event_idx);
      if (err != 0)
        {
          return;
        }

      DEBUGASSERT(tx_idx < cbs->num_tx);
      tx_cb     = cbs->tx[tx_idx].function;
      user_data = cbs->tx[tx_idx].user_data;
      cbs->tx[tx_idx].function = NULL;

      nxsem_post(&data->tx_sem);
      tx_cb(priv, 0, user_data);

      err = can_mcan_read_reg(priv, CAN_MCAN_TXEFS, &txefs);
      if (err != 0)
        {
          return;
        }
    }
}

static void can_mcan_get_message(struct can_mcan_priv_s *priv,
                                  uint16_t fifo_offset,
                                  uint16_t fifo_status_reg,
                                  uint16_t fifo_ack_reg)
{
  const struct can_mcan_config *config = priv->config;
  const struct can_mcan_callbacks *cbs = config->callbacks;
  struct can_mcan_rx_fifo_hdr hdr;
  struct can_frame_s frame;
  can_rx_callback_t cb;
  void *user_data;
  uint32_t get_idx;
  uint32_t filt_idx;
  int data_length;
  uint32_t fifo_status;
  int err;

  memset(&frame, 0, sizeof(frame));

  err = can_mcan_read_reg(priv, fifo_status_reg, &fifo_status);
  if (err != 0)
    {
      return;
    }

  while (FIELD_GET(CAN_MCAN_RXF0S_F0FL, fifo_status) != 0u)
    {
      get_idx = FIELD_GET(CAN_MCAN_RXF0S_F0GI, fifo_status);

      err = can_mcan_read_mram(priv,
                               fifo_offset +
                               get_idx * sizeof(struct can_mcan_rx_fifo) +
                               offsetof(struct can_mcan_rx_fifo, hdr),
                               &hdr,
                               sizeof(struct can_mcan_rx_fifo_hdr));
      if (err != 0)
        {
          canerr("failed to read Rx FIFO header (err %d)", err);
          return;
        }

      frame.dlc = hdr.dlc;

      if (hdr.rtr)
        {
          frame.flags |= CAN_FRAME_RTR;
        }

      if (hdr.fdf)
        {
          frame.flags |= CAN_FRAME_FDF;
        }

      if (hdr.brs)
        {
          frame.flags |= CAN_FRAME_BRS;
        }

      if (hdr.esi)
        {
          frame.flags |= CAN_FRAME_ESI;
        }

#ifdef CONFIG_CAN_RX_TIMESTAMP
      frame.timestamp = hdr.rxts;
#endif

      filt_idx = hdr.fidx;

      if (hdr.xtd)
        {
          frame.id     = hdr.ext_id;
          frame.flags |= CAN_FRAME_IDE;
        }
      else
        {
          frame.id = hdr.std_id;
        }

      data_length = can_dlc2bytes(frame.dlc);
      if (data_length <= (int)sizeof(frame.data))
        {
          if ((frame.flags & CAN_FRAME_RTR) == 0u && data_length != 0)
            {
              err = can_mcan_read_mram(priv,
                                       fifo_offset +
                                       get_idx * sizeof(struct can_mcan_rx_fifo) +
                                       offsetof(struct can_mcan_rx_fifo, data_32),
                                       &frame.data_32,
                                       ROUND_UP(data_length, sizeof(uint32_t)));
              if (err != 0)
                {
                  canerr("failed to read Rx FIFO data (err %d)", err);
                  return;
                }
            }

          if ((frame.flags & CAN_FRAME_IDE) != 0)
            {
              caninfo("Frame on ext filter %d, ID: 0x%x", filt_idx, frame.id);
              DEBUGASSERT(filt_idx < cbs->num_ext);
              cb        = cbs->ext[filt_idx].function;
              user_data = cbs->ext[filt_idx].user_data;
            }
          else
            {
              caninfo("Frame on std filter %d, ID: 0x%x", filt_idx, frame.id);
              DEBUGASSERT(filt_idx < cbs->num_std);
              cb        = cbs->std[filt_idx].function;
              user_data = cbs->std[filt_idx].user_data;
            }

          if (cb != NULL)
            {
              cb(priv, &frame, user_data);
            }
        }
      else
        {
          canerr("Frame too large (%d bytes)", data_length);
        }

      err = can_mcan_write_reg(priv, fifo_ack_reg, get_idx);
      if (err != 0)
        {
          return;
        }

      err = can_mcan_read_reg(priv, fifo_status_reg, &fifo_status);
      if (err != 0)
        {
          return;
        }
    }
}

void can_mcan_line_0_isr(struct can_mcan_priv_s *priv)
{
  const uint32_t events = CAN_MCAN_IR_BO  | CAN_MCAN_IR_EP   | CAN_MCAN_IR_EW  |
                          CAN_MCAN_IR_TEFN | CAN_MCAN_IR_TEFL | CAN_MCAN_IR_ARA |
                          CAN_MCAN_IR_MRAF | CAN_MCAN_IR_PEA  | CAN_MCAN_IR_PED;
  struct can_mcan_data *data = priv->data;
  uint32_t ir;
  int err;

  err = can_mcan_read_reg(priv, CAN_MCAN_IR, &ir);
  if (err != 0)
    {
      return;
    }

  while ((ir & events) != 0u)
    {
      err = can_mcan_write_reg(priv, CAN_MCAN_IR, ir & events);
      if (err != 0)
        {
          return;
        }

      if ((ir & (CAN_MCAN_IR_BO | CAN_MCAN_IR_EP | CAN_MCAN_IR_EW)) != 0u)
        {
          can_mcan_state_change_handler(priv);
        }

      if ((ir & CAN_MCAN_IR_TEFN) != 0u)
        {
          can_mcan_tx_event_handler(priv);
        }

      if ((ir & CAN_MCAN_IR_TEFL) != 0u)
        {
          canerr("TX FIFO element lost");
          nxsem_post(&data->tx_sem);
        }

      if ((ir & CAN_MCAN_IR_ARA) != 0u)
        {
          canerr("Access to reserved address");
        }

      if ((ir & CAN_MCAN_IR_MRAF) != 0u)
        {
          canerr("Message RAM access failure");
        }

      err = can_mcan_read_reg(priv, CAN_MCAN_IR, &ir);
      if (err != 0)
        {
          return;
        }
    }
}

void can_mcan_line_1_isr(struct can_mcan_priv_s *priv)
{
  const struct can_mcan_config *config = priv->config;
  const uint32_t events = CAN_MCAN_IR_RF0N | CAN_MCAN_IR_RF1N |
                          CAN_MCAN_IR_RF0L | CAN_MCAN_IR_RF1L;
  uint32_t ir;
  int err;

  err = can_mcan_read_reg(priv, CAN_MCAN_IR, &ir);
  if (err != 0)
    {
      return;
    }

  if ((ir & CAN_MCAN_IR_PEA) != 0u)
    {
      caninfo("Protocol error arbitration phase: ir=0x%x", ir);
    }

  if ((ir & CAN_MCAN_IR_PED) != 0u)
    {
      caninfo("Protocol error data phase: ir=0x%x", ir);
    }

  while ((ir & events) != 0u)
    {
      err = can_mcan_write_reg(priv, CAN_MCAN_IR, events & ir);
      if (err != 0)
        {
          return;
        }

      if ((ir & CAN_MCAN_IR_RF0N) != 0u)
        {
          can_mcan_get_message(priv,
                               config->mram_offsets[CAN_MCAN_MRAM_CFG_RX_FIFO0],
                               CAN_MCAN_RXF0S, CAN_MCAN_RXF0A);
        }

      if ((ir & CAN_MCAN_IR_RF1N) != 0u)
        {
          can_mcan_get_message(priv,
                               config->mram_offsets[CAN_MCAN_MRAM_CFG_RX_FIFO1],
                               CAN_MCAN_RXF1S, CAN_MCAN_RXF1A);
        }

      if ((ir & CAN_MCAN_IR_RF0L) != 0u)
        {
          canerr("Message lost on FIFO0");
        }

      if ((ir & CAN_MCAN_IR_RF1L) != 0u)
        {
          canerr("Message lost on FIFO1");
        }

      err = can_mcan_read_reg(priv, CAN_MCAN_IR, &ir);
      if (err != 0)
        {
          return;
        }
    }
}

int can_mcan_init(struct can_mcan_priv_s *priv)
{
  const struct can_mcan_config *config = priv->config;
  const struct can_mcan_callbacks *cbs = config->callbacks;
  struct can_mcan_data *data = priv->data;
  struct can_timing_s timing = { 0 };
#ifdef CONFIG_CAN_FD_MODE
  struct can_timing_s timing_data = { 0 };
#endif
  uint32_t reg;
  int err;

  DEBUGASSERT(config->ops->read_reg  != NULL);
  DEBUGASSERT(config->ops->write_reg != NULL);
  DEBUGASSERT(config->ops->read_mram != NULL);
  DEBUGASSERT(config->ops->write_mram != NULL);
  DEBUGASSERT(config->ops->clear_mram != NULL);
  DEBUGASSERT(config->callbacks != NULL);

  DEBUGASSERT(cbs->num_tx  <= config->mram_elements[CAN_MCAN_MRAM_CFG_TX_BUFFER]);
  DEBUGASSERT(cbs->num_std <= config->mram_elements[CAN_MCAN_MRAM_CFG_STD_FILTER]);
  DEBUGASSERT(cbs->num_ext <= config->mram_elements[CAN_MCAN_MRAM_CFG_EXT_FILTER]);

  nxmutex_init(&data->lock);
  nxmutex_init(&data->tx_mtx);
  nxsem_init(&data->tx_sem, 0, cbs->num_tx);

  err = can_mcan_exit_sleep_mode(priv);
  if (err != 0)
    {
      canerr("failed to exit sleep mode");
      return -EIO;
    }

  err = can_mcan_enter_init_mode(priv, CAN_INIT_TIMEOUT_MS);
  if (err != 0)
    {
      canerr("failed to enter init mode");
      return -EIO;
    }

  can_mcan_enable_configuration_change(priv);

#ifdef CONFIG_DEBUG_CAN_INFO
  err = can_mcan_read_reg(priv, CAN_MCAN_CREL, &reg);
  if (err != 0)
    {
      return -EIO;
    }

  caninfo("IP rel: %lu.%lu.%lu %02lu.%lu.%lu\n",
          FIELD_GET(CAN_MCAN_CREL_REL,     reg),
          FIELD_GET(CAN_MCAN_CREL_STEP,    reg),
          FIELD_GET(CAN_MCAN_CREL_SUBSTEP, reg),
          FIELD_GET(CAN_MCAN_CREL_YEAR,    reg),
          FIELD_GET(CAN_MCAN_CREL_MON,     reg),
          FIELD_GET(CAN_MCAN_CREL_DAY,     reg));
#endif

  err = can_mcan_read_reg(priv, CAN_MCAN_CCCR, &reg);
  if (err != 0)
    {
      return err;
    }

  reg &= ~(CAN_MCAN_CCCR_FDOE | CAN_MCAN_CCCR_BRSE |
           CAN_MCAN_CCCR_TEST | CAN_MCAN_CCCR_MON  |
           CAN_MCAN_CCCR_ASM);

  err = can_mcan_write_reg(priv, CAN_MCAN_CCCR, reg);
  if (err != 0)
    {
      return err;
    }

  err = can_mcan_read_reg(priv, CAN_MCAN_TEST, &reg);
  if (err != 0)
    {
      return err;
    }

  reg &= ~CAN_MCAN_TEST_LBCK;

  err = can_mcan_write_reg(priv, CAN_MCAN_TEST, reg);
  if (err != 0)
    {
      return err;
    }

  err = can_mcan_read_reg(priv, CAN_MCAN_GFC, &reg);
  if (err != 0)
    {
      return err;
    }

  reg |= FIELD_PREP(CAN_MCAN_GFC_ANFE, 0x2) |
         FIELD_PREP(CAN_MCAN_GFC_ANFS, 0x2);

#ifndef CONFIG_CAN_ACCEPT_RTR
  reg |= CAN_MCAN_GFC_RRFS | CAN_MCAN_GFC_RRFE;
#endif

  err = can_mcan_write_reg(priv, CAN_MCAN_GFC, reg);
  if (err != 0)
    {
      return err;
    }

  err = can_mcan_calc_timing(priv, &timing,
                             config->bitrate, config->sample_point);
  if (err == -EINVAL)
    {
      canerr("invalid nominal timing params");
      return -EIO;
    }

  caninfo("Presc: %d, TS1: %d, TS2: %d\n",
          timing.prescaler, timing.phase_seg1, timing.phase_seg2);

#ifdef CONFIG_CAN_FD_MODE
  err = can_mcan_calc_timing_data(priv, &timing_data,
                                  config->bitrate_data,
                                  config->sample_point_data);
  if (err == -EINVAL)
    {
      canerr("invalid FD data phase timing params");
      return -EIO;
    }
#endif

  err = can_mcan_set_timing(priv, &timing);
  if (err != 0)
    {
      canerr("failed to set timing (err %d)", err);
      return -ENODEV;
    }

#ifdef CONFIG_CAN_FD_MODE
  err = can_mcan_set_timing_data(priv, &timing_data);
  if (err != 0)
    {
      canerr("failed to set FD data timing (err %d)", err);
      return -ENODEV;
    }
#endif

  reg = CAN_MCAN_IE_BOE  | CAN_MCAN_IE_EWE   | CAN_MCAN_IE_EPE   |
        CAN_MCAN_IE_MRAFE | CAN_MCAN_IE_TEFLE | CAN_MCAN_IE_TEFNE |
        CAN_MCAN_IE_RF0NE | CAN_MCAN_IE_RF1NE |
        CAN_MCAN_IE_RF0LE | CAN_MCAN_IE_RF1LE;

  err = can_mcan_write_reg(priv, CAN_MCAN_IE, reg);
  if (err != 0)
    {
      return err;
    }

  reg = CAN_MCAN_ILS_RF0NL | CAN_MCAN_ILS_RF1NL |
        CAN_MCAN_ILS_RF0LL  | CAN_MCAN_ILS_RF1LL;
  err = can_mcan_write_reg(priv, CAN_MCAN_ILS, reg);
  if (err != 0)
    {
      return err;
    }

  reg = CAN_MCAN_ILE_EINT0 | CAN_MCAN_ILE_EINT1;
  err = can_mcan_write_reg(priv, CAN_MCAN_ILE, reg);
  if (err != 0)
    {
      return err;
    }

  err = can_mcan_write_reg(priv, CAN_MCAN_TXBTIE, CAN_MCAN_TXBTIE_TIE);
  if (err != 0)
    {
      return err;
    }

  return can_mcan_clear_mram(priv, 0, config->mram_size);
}

void can_mcan_rxint(struct can_mcan_priv_s *priv, bool enable)
{
  uint32_t reg;

  if (can_mcan_read_reg(priv, CAN_MCAN_IE, &reg) != 0)
    {
      return;
    }

  if (enable)
    {
      reg |= CAN_MCAN_IE_RF0NE | CAN_MCAN_IE_RF1NE |
             CAN_MCAN_IE_RF0LE | CAN_MCAN_IE_RF1LE;
    }
  else
    {
      reg &= ~(CAN_MCAN_IE_RF0NE | CAN_MCAN_IE_RF1NE |
               CAN_MCAN_IE_RF0LE | CAN_MCAN_IE_RF1LE);
    }

  can_mcan_write_reg(priv, CAN_MCAN_IE, reg);
}

void can_mcan_txint(struct can_mcan_priv_s *priv, bool enable)
{
  uint32_t reg;

  if (can_mcan_read_reg(priv, CAN_MCAN_IE, &reg) != 0)
    {
      return;
    }

  if (enable)
    {
      reg |= CAN_MCAN_IE_TEFNE | CAN_MCAN_IE_TEFLE;
    }
  else
    {
      reg &= ~(CAN_MCAN_IE_TEFNE | CAN_MCAN_IE_TEFLE);
    }

  can_mcan_write_reg(priv, CAN_MCAN_IE, reg);
}

bool can_mcan_txready(struct can_mcan_priv_s *priv)
{
  uint32_t txfqs;

  if (can_mcan_read_reg(priv, CAN_MCAN_TXFQS, &txfqs) != 0)
    {
      return false;
    }

  return (txfqs & CAN_MCAN_TXFQS_TFQF) == 0u;
}

bool can_mcan_txempty(struct can_mcan_priv_s *priv)
{
  uint32_t txbrp;

  if (can_mcan_read_reg(priv, CAN_MCAN_TXBRP, &txbrp) != 0)
    {
      return true;
    }

  return txbrp == 0;
}

int can_mcan_get_timing(struct can_mcan_priv_s *priv,
                        struct can_timing_s *timing)
{
  uint32_t nbtp;
  int err;

  err = can_mcan_read_reg(priv, CAN_MCAN_NBTP, &nbtp);
  if (err != 0)
    {
      return err;
    }

  timing->sjw        = (uint16_t)(FIELD_GET(CAN_MCAN_NBTP_NSJW,   nbtp) + 1);
  timing->phase_seg1 = (uint16_t)(FIELD_GET(CAN_MCAN_NBTP_NTSEG1, nbtp) + 1);
  timing->phase_seg2 = (uint16_t)(FIELD_GET(CAN_MCAN_NBTP_NTSEG2, nbtp) + 1);
  timing->prescaler  = (uint16_t)(FIELD_GET(CAN_MCAN_NBTP_NBRP,   nbtp) + 1);
  timing->prop_seg   = 0;

  return 0;
}

int can_mcan_get_connmodes(struct can_mcan_priv_s *priv,
                            bool *loopback, bool *silent)
{
  uint32_t cccr;
  uint32_t test;
  int err;

  err = can_mcan_read_reg(priv, CAN_MCAN_CCCR, &cccr);
  if (err != 0)
    {
      return err;
    }

  err = can_mcan_read_reg(priv, CAN_MCAN_TEST, &test);
  if (err != 0)
    {
      return err;
    }

  *loopback = ((cccr & CAN_MCAN_CCCR_TEST) != 0) &&
              ((test & CAN_MCAN_TEST_LBCK) != 0);
  *silent   = ((cccr & CAN_MCAN_CCCR_MON) != 0);
  return 0;
}
