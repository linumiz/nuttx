/****************************************************************************
 * drivers/net/dwxgmac/xgmac_priv.h
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

#ifndef __DRIVERS_NET_DWXGMAC_XGMAC_PRIV_H
#define __DRIVERS_NET_DWXGMAC_XGMAC_PRIV_H

/* The XGMAC variant of the register access of the driver.  Every function
 * here takes the DMA channel or MTL queue number, which are the same: RX
 * and TX queue n are served by DMA channel n.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>

#include <nuttx/arch.h>

#include "dwxgmac.h"
#include "xgmac_reg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Write back descriptor fields */

#define XGMAC_RDES3_PL          GENMASK(13, 0)  /* Packet length */
#define XGMAC_RDES3_ES          BIT(15)         /* Error summary */
#define XGMAC_RDES3_LD          BIT(28)         /* Last descriptor */
#define XGMAC_RDES3_FD          BIT(29)         /* First descriptor */
#define XGMAC_RDES3_CTXT        BIT(30)         /* Context descriptor */
#define XGMAC_RDES3_IOC         BIT(30)         /* Read: interrupt */
#define XGMAC_RDES3_OWN         BIT(31)         /* Owned by the DMA */

#define DWMAC_TDES3_DERR        BIT(27)         /* Write back: error */

/* DMA channel status bits that the interrupt handler serves */

#define DWMAC_CH_STATUS_RX      (XGMAC_DMA_CH_STATUS_RI | \
                                 XGMAC_DMA_CH_STATUS_RBU)
#define DWMAC_CH_STATUS_TX      XGMAC_DMA_CH_STATUS_TI

/* Interrupts enabled on each DMA channel */

#define XGMAC_DMA_CH_IE_COMMON  (XGMAC_DMA_CH_INTERRUPT_ENABLE_NIE | \
                                 XGMAC_DMA_CH_INTERRUPT_ENABLE_FBEE | \
                                 XGMAC_DMA_CH_INTERRUPT_ENABLE_AIE)
#define XGMAC_DMA_CH_IE_RX      (XGMAC_DMA_CH_INTERRUPT_ENABLE_RIE | \
                                 XGMAC_DMA_CH_INTERRUPT_ENABLE_RBUE | \
                                 XGMAC_DMA_CH_INTERRUPT_ENABLE_RSE | \
                                 XGMAC_DMA_CH_IE_COMMON)
#define XGMAC_DMA_CH_IE_TX      (XGMAC_DMA_CH_INTERRUPT_ENABLE_TIE | \
                                 XGMAC_DMA_CH_IE_COMMON)

#define XGMAC_PBL               32      /* DMA burst length */
#define XGMAC_TXQ_ENABLED       2       /* MTL TXQEN: enabled */
#define XGMAC_TXCH_OSP          BIT(4)  /* Operate on second packet */
#define XGMAC_RXQ_EN_DCB        2       /* MAC_RXQ_CTRL0: enabled for DCB */
#define XGMAC_SWR_TIMEOUT       1000    /* Polls of 50 us */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline void dwmac_modreg(uint32_t val, uint32_t mask, uintptr_t reg)
{
  dwmac_putreg((dwmac_getreg(reg) & ~mask) | (val & mask), reg);
}

/* RX descriptors */

static inline bool dwmac_rdes_own(FAR struct dwmac_desc_s *d)
{
  return (d->des3 & XGMAC_RDES3_OWN) != 0;
}

static inline bool dwmac_rdes_good(FAR struct dwmac_desc_s *d)
{
  uint32_t des3 = d->des3;

  return (des3 & (XGMAC_RDES3_LD | XGMAC_RDES3_CTXT | XGMAC_RDES3_ES)) ==
         XGMAC_RDES3_LD;
}

static inline uint16_t dwmac_rdes_length(FAR struct dwmac_desc_s *d)
{
  return FIELD_GET(XGMAC_RDES3_PL, d->des3);
}

static inline void dwmac_rdes_set(FAR struct dwmac_desc_s *d, uintptr_t buf)
{
  d->des0 = (uint32_t)buf;
  d->des1 = 0;
  d->des2 = 0;
  UP_DSB();
  d->des3 = XGMAC_RDES3_IOC | XGMAC_RDES3_OWN;
}

/* DMA */

static inline void dwmac_dma_rx_set_tail(FAR struct dwmac_driver_s *priv,
                                         int ch, FAR void *tail)
{
  dwmac_putreg((uint32_t)(uintptr_t)tail,
               priv->dmabase + XGMAC_DMA_CH_RXDESC_TAIL_LPOINTER(ch));
}

static inline void dwmac_dma_tx_set_tail(FAR struct dwmac_driver_s *priv,
                                         int ch, FAR void *tail)
{
  dwmac_putreg((uint32_t)(uintptr_t)tail,
               priv->dmabase + XGMAC_DMA_CH_TXDESC_TAIL_LPOINTER(ch));
}

static inline void dwmac_dma_rx_init(FAR struct dwmac_driver_s *priv,
                                     int ch)
{
  uintptr_t base = priv->dmabase;
  uint32_t descs = (uint32_t)(uintptr_t)priv->rx[ch].descs;

  dwmac_putreg(0, base + XGMAC_DMA_CH_CONTROL(ch));
  dwmac_putreg(0, base + XGMAC_DMA_CH_RXDESC_LIST_HADDRESS(ch));
  dwmac_putreg(descs, base + XGMAC_DMA_CH_RXDESC_LIST_LADDRESS(ch));
  dwmac_putreg(XGMAC_DMA_CH_RX_CONTROL_RPF |
               FIELD_PREP(XGMAC_DMA_CH_RX_CONTROL_RBSZ, DWMAC_RX_BUFSZ) |
               FIELD_PREP(XGMAC_DMA_CH_RX_CONTROL_RXPBL, XGMAC_PBL),
               base + XGMAC_DMA_CH_RX_CONTROL(ch));
  dwmac_putreg(FIELD_PREP(XGMAC_DMA_CH_RX_CONTROL2_RDRL,
                          DWMAC_NRXDESC - 1),
               base + XGMAC_DMA_CH_RX_CONTROL2(ch));
  dwmac_putreg(descs, base + XGMAC_DMA_CH_RXDESC_TAIL_LPOINTER(ch));
  dwmac_modreg(XGMAC_DMA_CH_IE_RX, XGMAC_DMA_CH_IE_RX,
               base + XGMAC_DMA_CH_INTERRUPT_ENABLE(ch));
}

static inline void dwmac_dma_tx_init(FAR struct dwmac_driver_s *priv,
                                     int ch)
{
  uintptr_t base = priv->dmabase;
  uint32_t descs = (uint32_t)(uintptr_t)priv->tx[ch].descs;

  dwmac_putreg(0, base + XGMAC_DMA_CH_TXDESC_LIST_HADDRESS(ch));
  dwmac_putreg(descs, base + XGMAC_DMA_CH_TXDESC_LIST_LADDRESS(ch));
  dwmac_putreg(FIELD_PREP(XGMAC_DMA_CH_TX_CONTROL2_TDRL,
                          DWMAC_NTXDESC - 1),
               base + XGMAC_DMA_CH_TX_CONTROL2(ch));

  /* Fetch the next frame while the current one is still going out */

  dwmac_modreg(FIELD_PREP(XGMAC_DMA_CH_TX_CONTROL_TXPBL, XGMAC_PBL) |
               XGMAC_TXCH_OSP,
               XGMAC_DMA_CH_TX_CONTROL_TXPBL | XGMAC_TXCH_OSP,
               base + XGMAC_DMA_CH_TX_CONTROL(ch));
  dwmac_putreg(descs, base + XGMAC_DMA_CH_TXDESC_TAIL_LPOINTER(ch));
  dwmac_modreg(XGMAC_DMA_CH_IE_TX, XGMAC_DMA_CH_IE_TX,
               base + XGMAC_DMA_CH_INTERRUPT_ENABLE(ch));
}

static inline void dwmac_dma_rx_enable(FAR struct dwmac_driver_s *priv,
                                       int ch, bool enable)
{
  dwmac_modreg(enable ? XGMAC_DMA_CH_RX_CONTROL_SR : 0,
               XGMAC_DMA_CH_RX_CONTROL_SR,
               priv->dmabase + XGMAC_DMA_CH_RX_CONTROL(ch));
}

static inline void dwmac_dma_tx_enable(FAR struct dwmac_driver_s *priv,
                                       int ch, bool enable)
{
  dwmac_modreg(enable ? XGMAC_DMA_CH_TX_CONTROL_ST : 0,
               XGMAC_DMA_CH_TX_CONTROL_ST,
               priv->dmabase + XGMAC_DMA_CH_TX_CONTROL(ch));
}

static inline void dwmac_dma_irq_disable(FAR struct dwmac_driver_s *priv,
                                         int ch)
{
  dwmac_putreg(0, priv->dmabase + XGMAC_DMA_CH_INTERRUPT_ENABLE(ch));
}

static inline uint32_t dwmac_dma_status(FAR struct dwmac_driver_s *priv)
{
  return dwmac_getreg(priv->dmabase + XGMAC_DMA_INTERRUPT_STATUS) &
         GENMASK(DWMAC_MAXCH - 1, 0);
}

static inline uint32_t dwmac_dma_ch_status(FAR struct dwmac_driver_s *priv,
                                           int ch)
{
  return dwmac_getreg(priv->dmabase + XGMAC_DMA_CH_STATUS(ch));
}

static inline void dwmac_dma_ch_clear(FAR struct dwmac_driver_s *priv,
                                      int ch, uint32_t status)
{
  dwmac_putreg(status, priv->dmabase + XGMAC_DMA_CH_STATUS(ch));
}

static inline void dwmac_txq_flush(FAR struct dwmac_driver_s *priv, int ch)
{
  dwmac_modreg(XGMAC_MTL_TXQX_OPERATION_MODE_FTQ,
               XGMAC_MTL_TXQX_OPERATION_MODE_FTQ,
               priv->base + XGMAC_MTL_TXQX_OPERATION_MODE(ch));
}

/* Reset the DMA and set its bus mode: undefined burst length up to 16
 * beats.  The bus mode is set even if the reset does not complete.
 */

static inline int dwmac_dma_reset(FAR struct dwmac_driver_s *priv)
{
  uintptr_t mode = priv->dmabase + XGMAC_DMA_MODE;
  int ret = -ETIMEDOUT;
  int i;

  dwmac_modreg(XGMAC_DMA_MODE_SWR, XGMAC_DMA_MODE_SWR, mode);

  for (i = 0; i < XGMAC_SWR_TIMEOUT; i++)
    {
      if ((dwmac_getreg(mode) & XGMAC_DMA_MODE_SWR) == 0)
        {
          ret = OK;
          break;
        }

      up_udelay(50);
    }

  dwmac_putreg(XGMAC_DMA_SYSBUS_MODE_UBL | XGMAC_DMA_SYSBUS_MODE_BLEN16,
               priv->dmabase + XGMAC_DMA_SYSBUS_MODE);
  return ret;
}

/* Hardware features: DMA channels, MTL queues and FIFO sizes */

static inline void dwmac_hw_features(FAR struct dwmac_driver_s *priv,
                                     FAR uint32_t *nch, FAR uint32_t *nq,
                                     FAR uint32_t *rxfifo,
                                     FAR uint32_t *txfifo)
{
  uint32_t f0 = dwmac_getreg(priv->base + XGMAC_MAC_HW_FEATURE0);
  uint32_t f1 = dwmac_getreg(priv->base + XGMAC_MAC_HW_FEATURE1);
  uint32_t f2 = dwmac_getreg(priv->base + XGMAC_MAC_HW_FEATURE2);
  uint32_t rxch = FIELD_GET(XGMAC_MAC_HW_FEATURE2_RXCHCNT, f2) + 1;
  uint32_t txch = FIELD_GET(XGMAC_MAC_HW_FEATURE2_TXCHCNT, f2) + 1;
  uint32_t rxq = FIELD_GET(XGMAC_MAC_HW_FEATURE2_RXQCNT, f2) + 1;
  uint32_t txq = FIELD_GET(XGMAC_MAC_HW_FEATURE2_TXQCNT, f2) + 1;

  priv->txcoe = (f0 & XGMAC_MAC_HW_FEATURE0_TXCOESEL) != 0;
  priv->rxcoe = (f0 & XGMAC_MAC_HW_FEATURE0_RXCOESEL) != 0;

  *nch = rxch < txch ? rxch : txch;
  *nq = rxq < txq ? rxq : txq;
  *rxfifo = 128 << FIELD_GET(XGMAC_MAC_HW_FEATURE1_RXFIFOSIZE, f1);
  *txfifo = 128 << FIELD_GET(XGMAC_MAC_HW_FEATURE1_TXFIFOSIZE, f1);
}

/* MAC */

static inline void dwmac_mac_init(FAR struct dwmac_driver_s *priv)
{
  /* Strip the FCS, pass the frames up to the giant packet limit and check
   * the checksums if the MAC can.  Receive all frames, the stack filters.
   */

  dwmac_putreg(XGMAC_MAC_RX_CONFIGURATION_CST |
               XGMAC_MAC_RX_CONFIGURATION_ACS |
               XGMAC_MAC_RX_CONFIGURATION_DCRCC |
               XGMAC_MAC_RX_CONFIGURATION_GPSLCE |
               FIELD_PREP(XGMAC_MAC_RX_CONFIGURATION_GPSL,
                          CONFIG_NET_ETH_PKTSIZE + 4) |
               (priv->rxcoe ? XGMAC_MAC_RX_CONFIGURATION_IPC : 0),
               priv->base + XGMAC_MAC_RX_CONFIGURATION);
  dwmac_putreg(XGMAC_MAC_PACKET_FILTER_RA | XGMAC_MAC_PACKET_FILTER_PM,
               priv->base + XGMAC_MAC_PACKET_FILTER);
  dwmac_putreg(0, priv->base + XGMAC_MAC_EXTENDED_CONFIGURATION);

  /* No MAC and MMC counter interrupts */

  dwmac_putreg(0, priv->base + XGMAC_MAC_INTERRUPT_ENABLE);
  dwmac_putreg(0xffffffff, priv->base + XGMAC_MMC_FPE_RX_INTERRUPT_MASK);
  dwmac_putreg(0xffffffff, priv->base + XGMAC_MMC_FPE_TX_INTERRUPT_MASK);
  dwmac_putreg(0xffffffff, priv->base + XGMAC_MMC_IPC_RX_INTERRUPT_MASK);
}

/* Program the link speed and enable or disable the MAC.  The speed select
 * of MAC_TX_CONFIGURATION is 3 for 1000, 4 for 100 and 7 for 10 Mbps.
 */

static inline void dwmac_mac_link(FAR struct dwmac_driver_s *priv, bool up)
{
  uint32_t ss = priv->speed == 1000 ? 3 : priv->speed == 100 ? 4 : 7;

  if (up)
    {
      dwmac_modreg(FIELD_PREP(XGMAC_MAC_TX_CONFIGURATION_SS, ss),
                   XGMAC_MAC_TX_CONFIGURATION_SS,
                   priv->base + XGMAC_MAC_TX_CONFIGURATION);
      dwmac_modreg(0, XGMAC_MAC_EXTENDED_CONFIGURATION_HD,
                   priv->base + XGMAC_MAC_EXTENDED_CONFIGURATION);
    }

  dwmac_modreg(up ? XGMAC_MAC_RX_CONFIGURATION_RE : 0,
               XGMAC_MAC_RX_CONFIGURATION_RE,
               priv->base + XGMAC_MAC_RX_CONFIGURATION);
  dwmac_modreg(up ? XGMAC_MAC_TX_CONFIGURATION_TE : 0,
               XGMAC_MAC_TX_CONFIGURATION_TE,
               priv->base + XGMAC_MAC_TX_CONFIGURATION);
}

static inline void dwmac_mac_address(FAR struct dwmac_driver_s *priv,
                                     FAR const uint8_t *mac)
{
  dwmac_putreg(((uint32_t)mac[5] << 8) | mac[4] |
               XGMAC_MAC_ADDRESSX_HIGH_AE,
               priv->base + XGMAC_MAC_ADDRESSX_HIGH(0));
  dwmac_putreg(((uint32_t)mac[3] << 24) | ((uint32_t)mac[2] << 16) |
               ((uint32_t)mac[1] << 8) | mac[0],
               priv->base + XGMAC_MAC_ADDRESSX_LOW(0));
}

/* MTL: queue n of each direction on DMA channel n, store and forward.
 * Queue 0 takes multicast and broadcast frames, and an 802.1Q tagged frame
 * goes to the queue of its PCP modulo the number of queues.
 */

static inline void dwmac_mtl_init(FAR struct dwmac_driver_s *priv)
{
  uint32_t map[2] =
  {
    0, 0
  };

  uint32_t prio[2] =
  {
    0, 0
  };

  uint32_t ctrl0 = 0;
  uint32_t mask;
  int pcp;
  int q;

  for (q = 0; q < DWMAC_NRXCH; q++)
    {
      dwmac_putreg(FIELD_PREP(XGMAC_MTL_RXQX_OPERATION_MODE_RQS,
                              (priv->rxqsize + 255) / 256 - 1) |
                   XGMAC_MTL_RXQX_OPERATION_MODE_RSF,
                   priv->base + XGMAC_MTL_RXQX_OPERATION_MODE(q));

      for (mask = 0, pcp = 0; pcp < 8; pcp++)
        {
          if (pcp % DWMAC_NRXCH == q)
            {
              mask |= 1 << pcp;
            }
        }

      ctrl0 |= XGMAC_RXQ_EN_DCB << (2 * q);
      prio[q / 4] |= mask << ((q % 4) * 8);
      map[q / 4] |= (uint32_t)q << ((q % 4) * 8);
    }

  for (q = 0; q < DWMAC_NTXCH; q++)
    {
      dwmac_putreg(FIELD_PREP(XGMAC_MTL_TXQX_OPERATION_MODE_TQS,
                              (priv->txqsize + 255) / 256 - 1) |
                   XGMAC_MTL_TXQX_OPERATION_MODE_TSF |
                   FIELD_PREP(XGMAC_MTL_TXQX_OPERATION_MODE_TXQEN,
                              XGMAC_TXQ_ENABLED),
                   priv->base + XGMAC_MTL_TXQX_OPERATION_MODE(q));
    }

  dwmac_putreg(map[0], priv->base + XGMAC_MTL_RXQ_DMA_MAP0);
  dwmac_putreg(map[1], priv->base + XGMAC_MTL_RXQ_DMA_MAP1);
  dwmac_putreg(ctrl0, priv->base + XGMAC_MAC_RXQ_CTRL0);
  dwmac_putreg(XGMAC_MAC_RXQ_CTRL1_MCBCQEN,
               priv->base + XGMAC_MAC_RXQ_CTRL1);
  dwmac_putreg(prio[0], priv->base + XGMAC_MAC_RXQ_CTRL2);
  dwmac_putreg(prio[1], priv->base + XGMAC_MAC_RXQ_CTRL3);
  dwmac_putreg(0, priv->base + XGMAC_MTL_OPERATION_MODE);
}

static inline void dwmac_mac_interrupt(FAR struct dwmac_driver_s *priv)
{
}

#endif /* __DRIVERS_NET_DWXGMAC_XGMAC_PRIV_H */
