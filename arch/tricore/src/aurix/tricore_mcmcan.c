/****************************************************************************
 * arch/tricore/src/aurix/tricore_mcmcan.c
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
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nuttx.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/can/can.h>
#include <nuttx/can/can_common.h>

#include "tricore_internal.h"
#include "tricore_mcmcan.h"

#define MCMCAN_WAIT_FOR(cond, us)              \
  ({                                            \
    int      _r = -ETIMEDOUT;                   \
    uint32_t _i = 0u;                           \
    while (_i < (us))                           \
      {                                         \
        if (cond) { _r = 0; break; }           \
        up_udelay(1u); _i++;                    \
      }                                         \
    _r;                                         \
  })

static void mcmcan_reset(struct can_dev_s *dev);
static int  mcmcan_setup(struct can_dev_s *dev);
static void mcmcan_shutdown(struct can_dev_s *dev);
static void mcmcan_rxint(struct can_dev_s *dev, bool enable);
static void mcmcan_txint(struct can_dev_s *dev, bool enable);
static int  mcmcan_ioctl(struct can_dev_s *dev, int cmd, unsigned long arg);
static int  mcmcan_remoterequest(struct can_dev_s *dev, uint16_t id);
static int  mcmcan_send(struct can_dev_s *dev, struct can_msg_s *msg);
static bool mcmcan_txready(struct can_dev_s *dev);
static bool mcmcan_txempty(struct can_dev_s *dev);

static const struct can_ops_s g_mcmcan_ops =
{
  .co_reset         = mcmcan_reset,
  .co_setup         = mcmcan_setup,
  .co_shutdown      = mcmcan_shutdown,
  .co_rxint         = mcmcan_rxint,
  .co_txint         = mcmcan_txint,
  .co_ioctl         = mcmcan_ioctl,
  .co_remoterequest = mcmcan_remoterequest,
  .co_send          = mcmcan_send,
  .co_txready       = mcmcan_txready,
  .co_txempty       = mcmcan_txempty,
};

static int mcmcan_mcan_read_reg(struct can_mcan_priv_s *priv,
                                 uint16_t reg, uint32_t *val)
{
  struct mcmcan_node_priv_s *np =
      container_of(priv, struct mcmcan_node_priv_s, mcan);
  return can_mcan_sys_read_reg(np->node_base, reg, val);
}

static int mcmcan_mcan_write_reg(struct can_mcan_priv_s *priv,
                                  uint16_t reg, uint32_t val)
{
  struct mcmcan_node_priv_s *np =
      container_of(priv, struct mcmcan_node_priv_s, mcan);
  return can_mcan_sys_write_reg(np->node_base, reg, val);
}

static int mcmcan_mcan_read_mram(struct can_mcan_priv_s *priv,
                                  uint16_t offset, void *dst, size_t len)
{
  struct mcmcan_node_priv_s *np =
      container_of(priv, struct mcmcan_node_priv_s, mcan);
  return can_mcan_sys_read_mram(np->mram_base, offset, dst, len);
}

static int mcmcan_mcan_write_mram(struct can_mcan_priv_s *priv,
                                   uint16_t offset, const void *src, size_t len)
{
  struct mcmcan_node_priv_s *np =
      container_of(priv, struct mcmcan_node_priv_s, mcan);
  return can_mcan_sys_write_mram(np->mram_base, offset, src, len);
}

static int mcmcan_mcan_clear_mram(struct can_mcan_priv_s *priv,
                                   uint16_t offset, size_t len)
{
  struct mcmcan_node_priv_s *np =
      container_of(priv, struct mcmcan_node_priv_s, mcan);
  return can_mcan_sys_clear_mram(np->mram_base, offset, len);
}

static const struct can_mcan_ops g_mcmcan_mcan_ops =
{
  .read_reg   = mcmcan_mcan_read_reg,
  .write_reg  = mcmcan_mcan_write_reg,
  .read_mram  = mcmcan_mcan_read_mram,
  .write_mram = mcmcan_mcan_write_mram,
  .clear_mram = mcmcan_mcan_clear_mram,
};

static void mcmcan_tx_callback(struct can_mcan_priv_s *priv, int error,
                                void *user_data)
{
  struct mcmcan_node_priv_s *np =
      container_of(priv, struct mcmcan_node_priv_s, mcan);
  UNUSED(error);
  UNUSED(user_data);

  can_txdone(np->candev);
}

static void mcmcan_rx_callback(struct can_mcan_priv_s *priv,
                                const struct can_frame_s *frame,
                                void *user_data)
{
  struct mcmcan_node_priv_s *np =
      container_of(priv, struct mcmcan_node_priv_s, mcan);
  struct can_hdr_s hdr;
  int data_len;

  UNUSED(user_data);

  memset(&hdr, 0, sizeof(hdr));
  hdr.ch_dlc = frame->dlc;

#ifdef CONFIG_CAN_EXTID
  if ((frame->flags & CAN_FRAME_IDE) != 0)
    {
      hdr.ch_id    = frame->id;
      hdr.ch_extid = 1;
    }
  else
#endif
    {
      hdr.ch_id = frame->id;
    }

  if ((frame->flags & CAN_FRAME_RTR) != 0) hdr.ch_rtr = 1;

#ifdef CONFIG_CAN_FD
  if ((frame->flags & CAN_FRAME_FDF) != 0) hdr.ch_fdcan = 1;
  if ((frame->flags & CAN_FRAME_BRS) != 0) hdr.ch_brs   = 1;
#endif

  data_len = can_dlc2bytes(frame->dlc);
  if (data_len < 0) data_len = 0;

  can_receive(np->candev, &hdr, (uint8_t *)frame->data);
}

static int mcmcan_line0_isr(int irq, void *context, void *arg)
{
  struct can_dev_s          *dev = arg;
  struct mcmcan_node_priv_s *np  = dev->cd_priv;

  UNUSED(irq);
  UNUSED(context);
  can_mcan_line_0_isr(&np->mcan);

  if (np->irq_line1 == 0)
    can_mcan_line_1_isr(&np->mcan);

  return OK;
}

static int mcmcan_line1_isr(int irq, void *context, void *arg)
{
  struct can_dev_s          *dev = arg;
  struct mcmcan_node_priv_s *np  = dev->cd_priv;

  UNUSED(irq);
  UNUSED(context);
  can_mcan_line_1_isr(&np->mcan);
  return OK;
}

static void mcmcan_reset(struct can_dev_s *dev)
{
  struct mcmcan_node_priv_s *np = dev->cd_priv;
  irqstate_t flags;

  flags = enter_critical_section();
  can_mcan_stop(&np->mcan);
  leave_critical_section(flags);
}

#define MRAM_STD_FILTER_ELEM_SIZE    4
#define MRAM_EXT_FILTER_ELEM_SIZE    8
#define MRAM_RX_FIFO_ELEM_SIZE      72
#define MRAM_TX_EVENT_ELEM_SIZE      8
#define MRAM_TX_BUF_ELEM_SIZE       72

static const uint16_t
g_mram_elem_sizes[CAN_MCAN_MRAM_CFG_NUM_CELLS] =
{
  [CAN_MCAN_MRAM_CFG_OFFSET]        = 0,
  [CAN_MCAN_MRAM_CFG_STD_FILTER]    = MRAM_STD_FILTER_ELEM_SIZE,
  [CAN_MCAN_MRAM_CFG_EXT_FILTER]    = MRAM_EXT_FILTER_ELEM_SIZE,
  [CAN_MCAN_MRAM_CFG_RX_FIFO0]      = MRAM_RX_FIFO_ELEM_SIZE,
  [CAN_MCAN_MRAM_CFG_RX_FIFO1]      = MRAM_RX_FIFO_ELEM_SIZE,
  [CAN_MCAN_MRAM_CFG_RX_BUFFER]     = MRAM_RX_FIFO_ELEM_SIZE,
  [CAN_MCAN_MRAM_CFG_TX_EVENT_FIFO] = MRAM_TX_EVENT_ELEM_SIZE,
  [CAN_MCAN_MRAM_CFG_TX_BUFFER]     = MRAM_TX_BUF_ELEM_SIZE,
};

static void mcmcan_compute_mram_layout(struct can_mcan_config *cfg)
{
  uint16_t offset = 0;
  int i;

  for (i = 0; i < CAN_MCAN_MRAM_CFG_NUM_CELLS; i++)
    {
      cfg->mram_offsets[i] = offset;
      offset += cfg->mram_elements[i] * g_mram_elem_sizes[i];
    }

  cfg->mram_size = (size_t)offset;

  caninfo("MRAM layout: std %u ext %u f0 %u f1 %u rb %u tef %u tb %u "
          "total=%zu\n",
          cfg->mram_offsets[CAN_MCAN_MRAM_CFG_STD_FILTER],
          cfg->mram_offsets[CAN_MCAN_MRAM_CFG_EXT_FILTER],
          cfg->mram_offsets[CAN_MCAN_MRAM_CFG_RX_FIFO0],
          cfg->mram_offsets[CAN_MCAN_MRAM_CFG_RX_FIFO1],
          cfg->mram_offsets[CAN_MCAN_MRAM_CFG_RX_BUFFER],
          cfg->mram_offsets[CAN_MCAN_MRAM_CFG_TX_EVENT_FIFO],
          cfg->mram_offsets[CAN_MCAN_MRAM_CFG_TX_BUFFER],
          cfg->mram_size);
}

static int mcmcan_hw_init_once(struct mcmcan_node_priv_s *np)
{
  struct can_filter_s filt;
  int err;

  mcmcan_compute_mram_layout(&np->mcan_config);

  err = can_mcan_init(&np->mcan);
  if (err != 0)
    {
      canerr("can_mcan_init failed (%d)", err);
      return err;
    }

  err = can_mcan_configure_mram(&np->mcan, 0, np->mram_base);
  if (err != 0)
    {
      canerr("configure_mram failed (%d)", err);
      return err;
    }

  err = irq_attach(np->irq_line0, mcmcan_line0_isr, np->candev);
  if (err < 0)
    {
      canerr("irq_attach line0 failed (%d)", err);
      return err;
    }

  if (np->irq_line1)
    {
      err = irq_attach(np->irq_line1, mcmcan_line1_isr, np->candev);
      if (err < 0)
        {
          canerr("irq_attach line1 failed (%d)", err);
          irq_detach(np->irq_line0);
          return err;
        }
     }

  memset(&filt, 0, sizeof(filt));
  np->rxfilter_std = can_mcan_add_rx_filter(&np->mcan, mcmcan_rx_callback,
                                             NULL, &filt);
  if (np->rxfilter_std < 0)
    {
      canerr("std rx filter failed (%d)", np->rxfilter_std);
      irq_detach(np->irq_line0);
      if (np->irq_line1)
        irq_detach(np->irq_line1);
      return np->rxfilter_std;
    }

#ifdef CONFIG_CAN_EXTID
  filt.flags = CAN_FILTER_IDE;
  np->rxfilter_ext = can_mcan_add_rx_filter(&np->mcan, mcmcan_rx_callback,
                                             NULL, &filt);
  if (np->rxfilter_ext < 0)
    {
      canerr("ext rx filter failed (%d)", np->rxfilter_ext);
      can_mcan_remove_rx_filter(&np->mcan, np->rxfilter_std);
      irq_detach(np->irq_line0);

      if (np->irq_line1)
        irq_detach(np->irq_line1);
      return np->rxfilter_ext;
    }
#endif

  return OK;
}

static int mcmcan_setup(struct can_dev_s *dev)
{
  struct mcmcan_node_priv_s *np = dev->cd_priv;
  int                        err;

  if (!np->initialized)
    {
      err = mcmcan_hw_init_once(np);
      if (err != 0)
        {
          return err;
        }

      np->initialized = true;
    }

  up_enable_irq(np->irq_line0);
  if (np->irq_line1)
    up_enable_irq(np->irq_line1);

  err = can_mcan_start(&np->mcan);
  if (err != 0 && err != -EALREADY)
    {
      canerr("can_mcan_start failed (%d)", err);
      up_disable_irq(np->irq_line0);

      if (np->irq_line1)
        up_disable_irq(np->irq_line1);
      return err;
    }

  return OK;
}

static void mcmcan_shutdown(struct can_dev_s *dev)
{
  struct mcmcan_node_priv_s *np = dev->cd_priv;

  can_mcan_stop(&np->mcan);
  up_disable_irq(np->irq_line0);

  if (np->irq_line1)
    up_disable_irq(np->irq_line1);
}

static void mcmcan_rxint(struct can_dev_s *dev, bool enable)
{
  struct mcmcan_node_priv_s *np = dev->cd_priv;

  can_mcan_rxint(&np->mcan, enable);
}

static void mcmcan_txint(struct can_dev_s *dev, bool enable)
{
  struct mcmcan_node_priv_s *np = dev->cd_priv;

  can_mcan_txint(&np->mcan, enable);
}

static int mcmcan_ioctl(struct can_dev_s *dev, int cmd, unsigned long arg)
{
  struct mcmcan_node_priv_s *np = dev->cd_priv;
  int ret = -ENOTTY;

  switch (cmd)
    {
      case CANIOC_GET_BITTIMING:
        {
          struct canioc_bittiming_s *bt = (struct canioc_bittiming_s *)arg;
          struct can_timing_s timing;

          DEBUGASSERT(bt != NULL);
          ret = can_mcan_get_timing(&np->mcan, &timing);
          if (ret == OK)
            {
              bt->bt_sjw   = (uint8_t)timing.sjw;
              bt->bt_tseg1 = (uint8_t)timing.phase_seg1;
              bt->bt_tseg2 = (uint8_t)timing.phase_seg2;
              bt->bt_baud  = 0;
            }
        }
        break;

      case CANIOC_SET_BITTIMING:
        {
          const struct canioc_bittiming_s *bt =
            (const struct canioc_bittiming_s *)arg;
          struct can_timing_s timing;
          bool was_started;

          DEBUGASSERT(bt != NULL);
          memset(&timing, 0, sizeof(timing));
          timing.sjw        = bt->bt_sjw;
          timing.phase_seg1 = bt->bt_tseg1;
          timing.phase_seg2 = bt->bt_tseg2;
          timing.prescaler  = 1; /* TODO: derive from bt_baud + clock */

          was_started = np->mcan.data->started;
          if (was_started)
            {
              ret = can_mcan_stop(&np->mcan);
              if (ret != 0)
                {
                  break;
                }
            }

          ret = can_mcan_set_timing(&np->mcan, &timing);

          if (was_started && ret == 0)
            {
              ret = can_mcan_start(&np->mcan);
            }
        }
        break;

      case CANIOC_GET_CONNMODES:
        {
          struct canioc_connmodes_s *bm =
            (struct canioc_connmodes_s *)arg;
          bool loopback;
          bool silent;

          DEBUGASSERT(bm != NULL);
          ret = can_mcan_get_connmodes(&np->mcan, &loopback, &silent);
          if (ret == OK)
            {
              bm->bm_loopback = loopback;
              bm->bm_silent   = silent;
            }
        }
        break;

      case CANIOC_SET_CONNMODES:
        {
          const struct canioc_connmodes_s *bm =
            (const struct canioc_connmodes_s *)arg;
          can_mode_t mode = CAN_MODE_NORMAL;
          bool was_started;

          DEBUGASSERT(bm != NULL);
          if (bm->bm_loopback) mode |= CAN_MODE_LOOPBACK;
          if (bm->bm_silent)   mode |= CAN_MODE_LISTENONLY;

         was_started = np->mcan.data->started;
          if (was_started)
            {
              ret = can_mcan_stop(&np->mcan);
              if (ret != 0)
                {
                  break;
                }
            }

          ret = can_mcan_set_mode(&np->mcan, mode);

          if (was_started && ret == 0)
            {
              ret = can_mcan_start(&np->mcan);
            }
        }
        break;

      case CANIOC_ADD_STDFILTER:
        {
          const struct canioc_stdfilter_s *sf =
            (const struct canioc_stdfilter_s *)arg;
          struct can_filter_s f;

          DEBUGASSERT(sf != NULL);
          f.id    = sf->sf_id1;
          f.mask  = sf->sf_id2;
          f.flags = 0;
          ret = can_mcan_add_rx_filter(&np->mcan, mcmcan_rx_callback,
                                        NULL, &f);
        }
        break;

      case CANIOC_DEL_STDFILTER:
        can_mcan_remove_rx_filter(&np->mcan, (int)arg);
        ret = OK;
        break;

#ifdef CONFIG_CAN_EXTID
      case CANIOC_ADD_EXTFILTER:
        {
          const struct canioc_extfilter_s *ef =
            (const struct canioc_extfilter_s *)arg;
          struct can_filter_s f;

          DEBUGASSERT(ef != NULL);
          f.id    = ef->xf_id1;
          f.mask  = ef->xf_id2;
          f.flags = CAN_FILTER_IDE;
          ret = can_mcan_add_rx_filter(&np->mcan, mcmcan_rx_callback,
                                        NULL, &f);
        }
        break;

      case CANIOC_DEL_EXTFILTER:
        can_mcan_remove_rx_filter(&np->mcan, (int)arg);
        ret = OK;
        break;
#endif

#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
      case CANIOC_BUSOFF_RECOVERY:
        ret = can_mcan_recover(&np->mcan, 100);
        break;
#endif

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

static int mcmcan_remoterequest(struct can_dev_s *dev, uint16_t id)
{
  UNUSED(dev);
  UNUSED(id);
  return -ENOSYS;
}

static int mcmcan_send(struct can_dev_s *dev, struct can_msg_s *msg)
{
  struct mcmcan_node_priv_s *np = dev->cd_priv;
  struct can_frame_s         frame;
  int                        data_len;

  memset(&frame, 0, sizeof(frame));

  frame.dlc = msg->cm_hdr.ch_dlc;

#ifdef CONFIG_CAN_EXTID
  if (msg->cm_hdr.ch_extid)
    {
      frame.id     = msg->cm_hdr.ch_id;
      frame.flags |= CAN_FRAME_IDE;
    }
  else
#endif
    {
      frame.id = msg->cm_hdr.ch_id & CAN_STD_ID_MASK;
    }

  if (msg->cm_hdr.ch_rtr)
    {
      frame.flags |= CAN_FRAME_RTR;
    }

#ifdef CONFIG_CAN_FD
  if (msg->cm_hdr.ch_fdcan)  frame.flags |= CAN_FRAME_FDF;
  if (msg->cm_hdr.ch_brs)    frame.flags |= CAN_FRAME_BRS;
#endif

  data_len = can_dlc2bytes(frame.dlc);
  if (data_len > 0 && (frame.flags & CAN_FRAME_RTR) == 0)
    {
      memcpy(frame.data, msg->cm_data, data_len);
    }

  return can_mcan_send(&np->mcan, &frame, 0, mcmcan_tx_callback, NULL);
}

static bool mcmcan_txready(struct can_dev_s *dev)
{
  struct mcmcan_node_priv_s *np = dev->cd_priv;

  return can_mcan_txready(&np->mcan);
}

static bool mcmcan_txempty(struct can_dev_s *dev)
{
  struct mcmcan_node_priv_s *np = dev->cd_priv;

  return can_mcan_txempty(&np->mcan);
}

static void mcmcan_configure_grint(uintptr_t base, uint8_t node)
{
  uint32_t l0 = (uint32_t)(node * 2);
  uint32_t l1 = l0 + 1;

  uint32_t g1 = (l0 << 28) | (l0 << 24) | (l0 << 20) | (l0 << 16) |
                (l0 << 12) | (l0 << 8)  | (l0 << 4)  | l0;

  uint32_t g2 = (l0 << 28) | (l0 << 24) | (l0 << 20) | (l1 << 16) |
                (l1 << 12) | (l1 << 8)  | (l1 << 4)  | l0;

  putreg32(g1, base + MCMCAN_GRINT1(node));
  putreg32(g2, base + MCMCAN_GRINT2(node));
}

static bool aurix_enable_clock(uintptr_t clc_addr, uint32_t timeout_us)
{
  uint32_t i;

#if defined(CONFIG_ARCH_CHIP_FAMILY_TC3XX)
  aurix_cpu_endinit_enable(false);
#endif
  modreg32(0, AURIX_CLC_DISR, clc_addr);
#if defined(CONFIG_ARCH_CHIP_FAMILY_TC3XX)
  aurix_cpu_endinit_enable(true);
#endif

  for (i = 0; i < timeout_us; i++)
    {
      if ((getreg32(clc_addr) & AURIX_CLC_DISS) == 0)
        {
          return true;
        }

      up_udelay(1u);
    }

  return false;
}

int mcmcan_module_init(const struct mcmcan_config *config)
{
  uintptr_t mcr_addr = config->base + MCMCAN_MCR;
  uint32_t mcr;
  uint8_t  i;
  int      err;

  if (!aurix_enable_clock(config->base + MCMCAN_CLC, 1000))
    {
      canerr("clock gate failed");
      return -ETIMEDOUT;
    }

  putreg32(MCMCAN_MCR_CLE | MCMCAN_MCR_CLKSEL, config->base + MCMCAN_MCR);

  err = MCMCAN_WAIT_FOR(
    (getreg32(config->base + MCMCAN_MCR) & MCMCAN_MCR_MRAM_BUSY) == 0, 1000);
  if (err != 0)
   {
     canerr("MRAM not ready");
     return -ETIMEDOUT;
   }

  putreg32(MCMCAN_MCR_CLE | MCMCAN_MCR_CLKSEL | MCMCAN_MCR_INIT_MRAM,
           config->base + MCMCAN_MCR);
  getreg32(config->base + MCMCAN_MCR);

  err = MCMCAN_WAIT_FOR(
    (getreg32(config->base + MCMCAN_MCR) & MCMCAN_MCR_MRAM_BUSY) == 0, 1000);
  if (err != 0)
   {
     canerr("MRAM clear timeout");
     return -ETIMEDOUT;
   }

  putreg32(0, config->base + MCMCAN_MCR);

  mcr = 0;
  for (i = 0; i < 4; i++)
    {
      if (!config->node_pin[i].present) continue;

      mcr |= (0x3 << (i * 2));
      putreg32(config->node_pin[i].npcr_val, config->base + MCMCAN_NPCR(i));
      mcmcan_configure_grint(config->base, i);
    }

  putreg32(MCMCAN_MCR_CLE | MCMCAN_MCR_CLKSEL, config->base + MCMCAN_MCR);
  putreg32(mcr, config->base + MCMCAN_MCR);

  return OK;
}

void mcmcan_node_initialize(struct mcmcan_node_priv_s *np,
                            struct can_dev_s *candev)
{
  np->mcan.config       = &np->mcan_config;
  np->mcan.data         = &np->mcan_data;
  np->mcan_config.ops   = &g_mcmcan_mcan_ops;
  np->candev            = candev;
  np->initialized       = false;
  candev->cd_ops        = &g_mcmcan_ops;
  candev->cd_priv       = np;
}

