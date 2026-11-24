/****************************************************************************
 * drivers/net/dwxgmac/dwxgmac.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include <arch/barriers.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/dwxgmac.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netdev_lowerhalf.h>

#include "dwxgmac.h"
#include "xgmac_priv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DWMAC_NINTF       CONFIG_NET_DWXGMAC_NINTERFACES

#ifdef DWXGMAC_DMA_SECTION
#  define DWMAC_DMA_SECTION locate_data(DWXGMAC_DMA_SECTION)
#else
#  define DWMAC_DMA_SECTION
#endif

/* Pool buffers that the rings may hold at once.  The receive path takes a
 * replacement buffer before it hands one up, so the pool needs spare
 * buffers beyond the rings, or every refill fails and each frame is
 * dropped.
 */

#define DWMAC_RX_SPARE    8
#define DWMAC_NBUFFERS    (DWMAC_NRXDESC * DWMAC_NRXCH + DWMAC_RX_SPARE + \
                           DWMAC_NTXDESC * DWMAC_NTXCH)

#if CONFIG_IOB_NBUFFERS < DWMAC_NBUFFERS
#  error CONFIG_IOB_NBUFFERS is too small for the DMA rings
#endif

#if CONFIG_IOB_BUFSIZE < CONFIG_NET_LL_GUARDSIZE + DWMAC_RX_BUFSZ
#  error CONFIG_IOB_BUFSIZE must hold the L2 guard and one RX DMA buffer
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct dwmac_desc_s
  g_dwmac_rxdesc[DWMAC_NINTF][DWMAC_NRXCH][DWMAC_NRXDESC]
  aligned_data(64) DWMAC_DMA_SECTION;
static struct dwmac_desc_s
  g_dwmac_txdesc[DWMAC_NINTF][DWMAC_NTXCH][DWMAC_NTXDESC]
  aligned_data(64) DWMAC_DMA_SECTION;

static struct dwmac_driver_s g_dwmac[DWMAC_NINTF];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dwmac_rx_fill
 *
 * Description:
 *   Give every descriptor of an RX ring a buffer and hand the ring to the
 *   DMA.
 *
 ****************************************************************************/

static int dwmac_rx_fill(FAR struct dwmac_driver_s *priv, int ch)
{
  FAR struct dwmac_rxring_s *ring = &priv->rx[ch];
  int i;

  for (i = 0; i < DWMAC_NRXDESC; i++)
    {
      if (ring->pkt[i] == NULL)
        {
          ring->pkt[i] = netpkt_alloc(&priv->dev, NETPKT_RX);
          if (ring->pkt[i] == NULL)
            {
              nerr("ERROR: No RX buffer for channel %d\n", ch);
              return -ENOMEM;
            }
        }

      dwmac_rdes_set(&ring->descs[i],
                     (uintptr_t)netpkt_getdata(&priv->dev, ring->pkt[i]));
    }

  ring->head = 0;
  dwmac_dma_rx_set_tail(priv, ch, &ring->descs[DWMAC_NRXDESC]);
  return OK;
}

/****************************************************************************
 * Name: dwmac_queue_sizes
 *
 * Description:
 *   Share the MTL FIFOs out over the queues and check the queue sizes
 *   against the frame size.
 *
 ****************************************************************************/

static void dwmac_queue_sizes(FAR struct dwmac_driver_s *priv)
{
  uint32_t rxfifo;
  uint32_t txfifo;
  uint32_t nch;
  uint32_t nq;

  dwmac_hw_features(priv, &nch, &nq, &rxfifo, &txfifo);

  ninfo("channels %" PRIu32 ", queues %" PRIu32 ", RX FIFO %" PRIu32
        ", TX FIFO %" PRIu32 "\n", nch, nq, rxfifo, txfifo);

  if (DWMAC_NRXCH > nch || DWMAC_NTXCH > nch ||
      DWMAC_NRXCH > nq || DWMAC_NTXCH > nq)
    {
      nerr("ERROR: More channels configured than the MAC has\n");
    }

  /* The size the MAC reports is wrong on some parts, a queue past the end
   * of the FIFO makes the TX DMA wedge.
   */

  if (priv->txfifo != 0)
    {
      txfifo = priv->txfifo;
    }

  priv->rxqsize = rxfifo / DWMAC_NRXCH;
  priv->txqsize = txfifo / DWMAC_NTXCH;

  if (priv->rxqsize < CONFIG_NET_ETH_PKTSIZE ||
      priv->txqsize < CONFIG_NET_ETH_PKTSIZE)
    {
      nerr("ERROR: MTL queue smaller than one frame\n");
    }

  /* The checksum insertion holds the whole frame in the TX queue until the
   * sum is known, with room for the next one behind it.  With less room
   * the MAC sends the frame on with the checksum field untouched and the
   * peer drops it, so leave the checksums to the network stack.
   */

  if (priv->txcoe && priv->txqsize < 2 * CONFIG_NET_ETH_PKTSIZE)
    {
      priv->txcoe = false;

#ifndef CONFIG_NET_TCP_CHECKSUMS
      nerr("ERROR: TX queue too small for checksum insertion and "
           "CONFIG_NET_TCP_CHECKSUMS is not set\n");
#endif
    }
}

/****************************************************************************
 * Name: dwmac_link
 *
 * Description:
 *   Start or stop the MAC and the TX DMA for the link state.
 *
 ****************************************************************************/

static void dwmac_link(FAR struct dwmac_driver_s *priv, bool up)
{
  irqstate_t flags;
  int ch;

  flags = enter_critical_section();

  for (ch = 0; ch < DWMAC_NTXCH; ch++)
    {
      dwmac_dma_tx_enable(priv, ch, up);
      if (!up)
        {
          dwmac_txq_flush(priv, ch);
        }
    }

  dwmac_mac_link(priv, up);
  leave_critical_section(flags);

  if (up)
    {
      netdev_lower_carrier_on(&priv->dev);
    }
  else
    {
      netdev_lower_carrier_off(&priv->dev);
    }
}

/****************************************************************************
 * Name: dwmac_txch
 *
 * Description:
 *   Pick the TX channel of a frame: an 802.1Q tagged frame goes to the
 *   channel of its PCP modulo the number of channels, as on the RX side,
 *   any other frame to channel 0.
 *
 ****************************************************************************/

static int dwmac_txch(FAR struct dwmac_driver_s *priv, FAR netpkt_t *pkt,
                      unsigned int len)
{
  FAR const uint8_t *frame;

  if (DWMAC_NTXCH == 1 || len < 16)
    {
      return 0;
    }

  frame = netpkt_getdata(&priv->dev, pkt);
  if (frame[12] != 0x81 || frame[13] != 0x00)
    {
      return 0;
    }

  return (frame[14] >> 5) % DWMAC_NTXCH;
}

/****************************************************************************
 * Name: dwmac_transmit
 ****************************************************************************/

static int dwmac_transmit(FAR struct netdev_lowerhalf_s *dev,
                          FAR netpkt_t *pkt)
{
  FAR struct dwmac_driver_s *priv = (FAR struct dwmac_driver_s *)dev;
  FAR struct dwmac_txring_s *ring;
  FAR struct dwmac_desc_s *desc;
  unsigned int len;
  uintptr_t buf;
  uint16_t next;
  int ch;

  len = netpkt_getdatalen(dev, pkt);
  if (len == 0 || len > CONFIG_NET_ETH_PKTSIZE)
    {
      return -EINVAL;
    }

  ch = dwmac_txch(priv, pkt, len);
  ring = &priv->tx[ch];

  /* The upper half calls transmit with the device locked, and the TX
   * completion only advances the head, so the ring needs no lock.  One
   * descriptor stays unused, to tell a full ring from an empty one.  A
   * full ring is busy: the upper half retries after the next txdone.
   */

  next = (ring->tail + 1) % DWMAC_NTXDESC;
  if (next == ring->head)
    {
      return -EBUSY;
    }

  desc = &ring->descs[ring->tail];
  buf = (uintptr_t)netpkt_getdata(dev, pkt);

  desc->des0 = (uint32_t)buf;
#if UINTPTR_MAX > UINT32_MAX
  desc->des1 = (uint32_t)(buf >> 32);
#else
  desc->des1 = 0;
#endif
  desc->des2 = FIELD_PREP(DWMAC_TDES2_B1L, len) | DWMAC_TDES2_IOC;
  UP_DSB();
  desc->des3 = FIELD_PREP(DWMAC_TDES3_FL, len) | DWMAC_TDES3_FD |
               DWMAC_TDES3_LD | DWMAC_TDES3_OWN |
               (priv->txcoe ? DWMAC_TDES3_CIC : 0);
  UP_DSB();

  ring->pkt[ring->tail] = pkt;
  ring->tail = next;
  dwmac_dma_tx_set_tail(priv, ch, &ring->descs[ring->tail]);
  return OK;
}

/****************************************************************************
 * Name: dwmac_receive
 *
 * Description:
 *   Hand up one received frame.  The search goes on from the channel after
 *   the one of the last frame, so that a busy channel does not starve the
 *   others.
 *
 ****************************************************************************/

static FAR netpkt_t *dwmac_receive(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct dwmac_driver_s *priv = (FAR struct dwmac_driver_s *)dev;
  int i;

  for (i = 0; i < DWMAC_NRXCH; i++)
    {
      int ch = (priv->rxch + i) % DWMAC_NRXCH;
      FAR struct dwmac_rxring_s *ring = &priv->rx[ch];

      for (; ; )
        {
          FAR struct dwmac_desc_s *desc = &ring->descs[ring->head];
          FAR netpkt_t *fresh;
          FAR netpkt_t *pkt;
          uint16_t len;
          bool good;

          if (dwmac_rdes_own(desc))
            {
              break;
            }

          good = dwmac_rdes_good(desc);
          len = dwmac_rdes_length(desc);
          pkt = ring->pkt[ring->head];

          /* The descriptor needs a buffer before the DMA can use it again,
           * so the frame is only handed up if a replacement is available.
           */

          fresh = netpkt_alloc(dev, NETPKT_RX);
          if (fresh != NULL)
            {
              ring->pkt[ring->head] = fresh;
            }
          else
            {
              NETDEV_RXDROPPED(&dev->netdev);
              pkt = NULL;
            }

          dwmac_rdes_set(desc, (uintptr_t)netpkt_getdata(dev,
                                                 ring->pkt[ring->head]));
          ring->head = (ring->head + 1) % DWMAC_NRXDESC;
          dwmac_dma_rx_set_tail(priv, ch, &ring->descs[DWMAC_NRXDESC]);

          if (pkt == NULL)
            {
              continue;
            }

          if (!good || len == 0 || len > DWMAC_RX_BUFSZ)
            {
              NETDEV_RXERRORS(&dev->netdev);
              netpkt_free(dev, pkt, NETPKT_RX);
              continue;
            }

          netpkt_setdatalen(dev, pkt, len);
          priv->rxch = (ch + 1) % DWMAC_NRXCH;
          return pkt;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: dwmac_txdone_work
 *
 * Description:
 *   netdev_lower_txdone may block, so it runs on the work queue.
 *
 ****************************************************************************/

static void dwmac_txdone_work(FAR void *arg)
{
  FAR struct dwmac_driver_s *priv = arg;

  netdev_lower_txdone(&priv->dev);
}

/****************************************************************************
 * Name: dwmac_tx_complete
 *
 * Description:
 *   Free the packets of the TX descriptors that the DMA has finished.
 *
 ****************************************************************************/

static void dwmac_tx_complete(FAR struct dwmac_driver_s *priv, int ch)
{
  FAR struct dwmac_txring_s *ring = &priv->tx[ch];
  bool freed = false;

  while (ring->head != ring->tail)
    {
      FAR struct dwmac_desc_s *desc = &ring->descs[ring->head];

      if ((desc->des3 & DWMAC_TDES3_OWN) != 0)
        {
          break;
        }

      if ((desc->des3 & (DWMAC_TDES3_LD | DWMAC_TDES3_DERR)) ==
          (DWMAC_TDES3_LD | DWMAC_TDES3_DERR))
        {
          NETDEV_TXERRORS(&priv->dev.netdev);
        }

      if (ring->pkt[ring->head] != NULL)
        {
          netpkt_free(&priv->dev, ring->pkt[ring->head], NETPKT_TX);
          ring->pkt[ring->head] = NULL;
        }

      ring->head = (ring->head + 1) % DWMAC_NTXDESC;
      freed = true;
    }

  if (freed && work_available(&priv->txdone_work))
    {
      work_queue(HPWORK, &priv->txdone_work, dwmac_txdone_work, priv, 0);
    }
}

/****************************************************************************
 * Name: dwmac_free_buffers
 ****************************************************************************/

static void dwmac_free_buffers(FAR struct dwmac_driver_s *priv)
{
  int ch;
  int i;

  for (ch = 0; ch < DWMAC_NRXCH; ch++)
    {
      for (i = 0; i < DWMAC_NRXDESC; i++)
        {
          if (priv->rx[ch].pkt[i] != NULL)
            {
              netpkt_free(&priv->dev, priv->rx[ch].pkt[i], NETPKT_RX);
              priv->rx[ch].pkt[i] = NULL;
            }
        }
    }

  for (ch = 0; ch < DWMAC_NTXCH; ch++)
    {
      for (i = 0; i < DWMAC_NTXDESC; i++)
        {
          if (priv->tx[ch].pkt[i] != NULL)
            {
              netpkt_free(&priv->dev, priv->tx[ch].pkt[i], NETPKT_TX);
              priv->tx[ch].pkt[i] = NULL;
            }
        }

      priv->tx[ch].head = 0;
      priv->tx[ch].tail = 0;
    }
}

/****************************************************************************
 * Name: dwmac_ifup
 ****************************************************************************/

static int dwmac_ifup(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct dwmac_driver_s *priv = (FAR struct dwmac_driver_s *)dev;
  int ret;
  int ch;

  dwmac_mac_address(priv, dev->netdev.d_mac.ether.ether_addr_octet);

  for (ch = 0; ch < DWMAC_NRXCH; ch++)
    {
      ret = dwmac_rx_fill(priv, ch);
      if (ret < 0)
        {
          dwmac_free_buffers(priv);
          return ret;
        }

      dwmac_dma_rx_enable(priv, ch, true);
    }

  if (priv->irq != 0)
    {
      up_enable_irq(priv->irq);
    }

  priv->ifup = true;
  dwmac_link(priv, priv->link);
  return OK;
}

/****************************************************************************
 * Name: dwmac_ifdown
 ****************************************************************************/

static int dwmac_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct dwmac_driver_s *priv = (FAR struct dwmac_driver_s *)dev;
  int ch;

  if (priv->irq != 0)
    {
      up_disable_irq(priv->irq);
    }

  work_cancel(HPWORK, &priv->txdone_work);

  for (ch = 0; ch < DWMAC_NRXCH; ch++)
    {
      dwmac_dma_rx_enable(priv, ch, false);
    }

  priv->ifup = false;
  dwmac_link(priv, false);
  dwmac_free_buffers(priv);
  return OK;
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct netdev_ops_s g_dwmac_ops =
{
  .ifup     = dwmac_ifup,
  .ifdown   = dwmac_ifdown,
  .transmit = dwmac_transmit,
  .receive  = dwmac_receive,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dwxgmac_interrupt
 *
 * Description:
 *   Serve the DMA channels of an interface.  The status is read until it
 *   is clear, so that an edge triggered interrupt line goes low before the
 *   handler returns and the next completion raises a new edge.
 *
 ****************************************************************************/

int dwxgmac_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct dwmac_driver_s *priv = &g_dwmac[(uintptr_t)arg];
  uint32_t status;

  while ((status = dwmac_dma_status(priv)) != 0)
    {
      while (status != 0)
        {
          int ch = ffs(status) - 1;
          uint32_t chstatus = dwmac_dma_ch_status(priv, ch);

          dwmac_dma_ch_clear(priv, ch, chstatus);

          if (ch < DWMAC_NRXCH && (chstatus & DWMAC_CH_STATUS_RX) != 0)
            {
              netdev_lower_rxready(&priv->dev);
            }

          if (ch < DWMAC_NTXCH && (chstatus & DWMAC_CH_STATUS_TX) != 0)
            {
              dwmac_tx_complete(priv, ch);
            }

          status &= ~(1 << ch);
        }
    }

  dwmac_mac_interrupt(priv);
  return OK;
}

/****************************************************************************
 * Name: dwxgmac_carrier
 ****************************************************************************/

void dwxgmac_carrier(int intf, bool up, int speed)
{
  FAR struct dwmac_driver_s *priv;

  DEBUGASSERT(intf >= 0 && intf < DWMAC_NINTF);
  priv = &g_dwmac[intf];

  priv->link = up;
  if (up)
    {
      priv->speed = speed;
    }

  if (priv->ifup)
    {
      dwmac_link(priv, up);
    }
}

/****************************************************************************
 * Name: dwxgmac_initialize
 ****************************************************************************/

int dwxgmac_initialize(int intf, FAR const struct dwxgmac_config_s *config)
{
  FAR struct dwmac_driver_s *priv;
  FAR uint8_t *mac;
  int ret;
  int ch;

  if (intf < 0 || intf >= DWMAC_NINTF || config == NULL)
    {
      return -EINVAL;
    }

  priv = &g_dwmac[intf];
  memset(priv, 0, sizeof(*priv));

  priv->base = config->base;
  priv->dmabase = config->dmabase != 0 ? config->dmabase : config->base;
  priv->irq = config->irq;
  priv->txfifo = config->txfifo_size;
  priv->speed = config->speed;
  priv->link = config->speed != 0;

  for (ch = 0; ch < DWMAC_NRXCH; ch++)
    {
      priv->rx[ch].descs = g_dwmac_rxdesc[intf][ch];
      memset(priv->rx[ch].descs, 0, sizeof(g_dwmac_rxdesc[intf][ch]));
    }

  for (ch = 0; ch < DWMAC_NTXCH; ch++)
    {
      priv->tx[ch].descs = g_dwmac_txdesc[intf][ch];
      memset(priv->tx[ch].descs, 0, sizeof(g_dwmac_txdesc[intf][ch]));
    }

  if (dwmac_dma_reset(priv) < 0)
    {
      nerr("ERROR: DMA reset did not complete\n");
    }

  for (ch = 0; ch < DWMAC_NTXCH; ch++)
    {
      dwmac_dma_tx_init(priv, ch);
    }

  for (ch = 0; ch < DWMAC_NRXCH; ch++)
    {
      dwmac_dma_rx_init(priv, ch);
    }

  dwmac_queue_sizes(priv);
  dwmac_mtl_init(priv);
  dwmac_mac_init(priv);

  /* A locally administered address until the application sets one */

  mac = priv->dev.netdev.d_mac.ether.ether_addr_octet;
  arc4random_buf(mac, 6);
  mac[0] = (mac[0] & 0xfe) | 0x02;

  priv->dev.ops = &g_dwmac_ops;

  if (priv->irq != 0)
    {
      ret = irq_attach(priv->irq, dwxgmac_interrupt,
                       (FAR void *)(uintptr_t)intf);
      if (ret < 0)
        {
          return ret;
        }
    }

  ret = netdev_lower_register(&priv->dev, NET_LL_ETHERNET);
  if (ret < 0)
    {
      if (priv->irq != 0)
        {
          irq_detach(priv->irq);
        }

      return ret;
    }

  ninfo("XGMAC%d at %" PRIxPTR "\n", intf, priv->base);
  return OK;
}
