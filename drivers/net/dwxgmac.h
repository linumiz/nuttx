/*****************************************************************************
 * drivers/net/dwxgmac.h
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
 *****************************************************************************/

#ifndef __DRIVERS_NET_XGMAC_PRIV_H
#define __DRIVERS_NET_XGMAC_PRIV_H

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/net/netdev.h>
#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------
 * RX Descriptor (Read + Writeback)
 * ------------------------------------------------------------------------- */

struct xgmac_rdes
{
  uint32_t buf1;
#if UINTPTR_MAX > UINT32_MAX
  uint32_t buf1_hi;
#endif
  uint32_t reserved;
  uint32_t status;
};

/* RX status bits */
#define XGMAC_RDES_OWN    (1u << 31)
#define XGMAC_RDES_LD     (1u << 8)
#define XGMAC_RDES_FD     (1u << 9)
#define XGMAC_RDES_ES     (1u << 15)
#define XGMAC_RDES_PL_MASK 0x7fff

static inline bool xgmac_rdes_owned(struct xgmac_rdes *rdes)
{
  return (rdes->status & XGMAC_RDES_OWN) != 0;
}

static inline bool xgmac_rdes_last(struct xgmac_rdes *rdes)
{
  return (rdes->status & XGMAC_RDES_LD) != 0;
}

static inline uint16_t xgmac_rdes_len(struct xgmac_rdes *rdes)
{
  return rdes->status & XGMAC_RDES_PL_MASK;
}

/* -------------------------------------------------------------------------
 * TX Descriptor
 * ------------------------------------------------------------------------- */

struct xgmac_tdes
{
  uint32_t buf1;
#if UINTPTR_MAX > UINT32_MAX
  uint32_t buf1_hi;
#endif
  uint32_t ctl;
  uint32_t status;
};

#define XGMAC_TDES_OWN   (1u << 31)
#define XGMAC_TDES_FS    (1u << 28)
#define XGMAC_TDES_LS    (1u << 29)
#define XGMAC_TDES_CIC   (3u << 16)
#define XGMAC_TDES_LEN(x) ((x) & 0x3fff)

static inline void xgmac_tdes_prepare(struct xgmac_tdes *tdes,
                                      uintptr_t buf,
                                      uint16_t len,
                                      bool checksum)
{
  tdes->buf1 = (uint32_t)buf;
#if UINTPTR_MAX > UINT32_MAX
  tdes->buf1_hi = (uint32_t)(buf >> 32);
#endif
  tdes->ctl =
      XGMAC_TDES_OWN |
      XGMAC_TDES_FS |
      XGMAC_TDES_LS |
      (checksum ? XGMAC_TDES_CIC : 0) |
      XGMAC_TDES_LEN(len);
}

/* -------------------------------------------------------------------------
 * DMA helpers
 * ------------------------------------------------------------------------- */

static inline void xgmac_dma_rx_start(uintptr_t dma, uint32_t ch)
{
  xgmac_write(dma, DMA_CHi_RX_CONTROL(ch),
              xgmac_read(dma, DMA_CHi_RX_CONTROL(ch)) |
              DMA_CHi_RX_CONTROL_SR);
}

static inline void xgmac_dma_tx_start(uintptr_t dma, uint32_t ch)
{
  xgmac_write(dma, DMA_CHi_TX_CONTROL(ch),
              xgmac_read(dma, DMA_CHi_TX_CONTROL(ch)) |
              DMA_CHi_TX_CONTROL_ST);
}

static inline void xgmac_dma_tx_stop(FAR struct xgmac_driver_s *priv,
                                     uint8_t ch)
{
  uintptr_t base = priv->dma_base;

  xgmac_write(base, DMA_CHi_TX_CONTROL(ch),
              xgmac_read(base, DMA_CHi_TX_CONTROL(ch)) &
              ~DMA_CHi_TX_CONTROL_ST);
}

static void xgmac_dma_tx_enable_irq(FAR struct xgmac_driver_s *priv,
                                    uint8_t ch)
{
  uintptr_t base = priv->dma_base;

  xgmac_write(base, DMA_CHi_INT_ENABLE(ch),
              DMA_CHi_INT_TX);
}

static void xgmac_dma_tx_disable_irq(FAR struct xgmac_driver_s *priv,
                                     uint8_t ch)
{
  uintptr_t base = priv->dma_base;

  xgmac_write(base, DMA_CHi_INT_ENABLE(ch), 0);
}

static inline void xgmac_dma_tx_kick(FAR struct xgmac_driver_s *priv,
                                     uint8_t ch)
{
  uintptr_t base = priv->dma_base;

  xgmac_write(base, DMA_CHi_TXDESC_TAIL(ch),
              (uint32_t)&priv->dma_tx[ch].descs[priv->dma_tx[ch].tail]);
}


/* -------------------------------------------------------------------------
 * MAC helpers
 * ------------------------------------------------------------------------- */

static inline void xgmac_mac_enable(uintptr_t mac)
{
  xgmac_write(mac, MAC_CONFIGURATION,
              MAC_CONFIGURATION_TE | MAC_CONFIGURATION_RE);
}

static inline void xgmac_mac_disable(uintptr_t mac)
{
  xgmac_write(mac, MAC_CONFIGURATION,
              xgmac_read(mac, MAC_CONFIGURATION) &
              ~(MAC_CONFIGURATION_TE | MAC_CONFIGURATION_RE));
}

static inline void xgmac_mac_set_link(uintptr_t mac,
                                      uint32_t speed,
                                      bool full)
{
  uint32_t reg = xgmac_read(mac, MAC_CONFIGURATION);

  reg &= ~(MAC_CONFIGURATION_DM |
           MAC_CONFIGURATION_FES |
           MAC_CONFIGURATION_PS);

  if (full)
    reg |= MAC_CONFIGURATION_DM;

  reg |= speed;

  xgmac_write(mac, MAC_CONFIGURATION, reg);
}

/* -------------------------------------------------------------------------
 * MAC address programming (no filters)
 * ------------------------------------------------------------------------- */

static inline void xgmac_set_hwaddr(uintptr_t mac,
                                    const uint8_t addr[6])
{
  xgmac_write(mac, MAC_ADDRESS0_HIGH,
              (addr[5] << 8) | addr[4] | MAC_ADDRESSi_HIGH_AE);

  xgmac_write(mac, MAC_ADDRESS0_LOW,
              (addr[3] << 24) | (addr[2] << 16) |
              (addr[1] << 8)  | addr[0]);
}

#endif /* __DRIVERS_NET_XGMAC_PRIV_H */
