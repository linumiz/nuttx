/****************************************************************************
 * drivers/net/dwxgmac.c
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

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>
#include "eth_xgmac_priv.h"
#include <arch/barriers.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev.h>
#include <nuttx/queue.h>
#include <nuttx/random.h>
#include <nuttx/wqueue.h>

#define IRQ_GETH0_DMA_INTR 740
#define ETHWORK HPWORK
struct xgmac_desc_s
{
  uint32_t des0; /* Buffer 1 address (low 32 bits) */
  uint32_t des1; /* Buffer 1 address (high 32 bits) */
  uint32_t des2; /* Control / frame length */
  uint32_t des3; /* Status / ownership */
};

struct eth_dwmac_dma_desc ndma_rx0_descs[CONFIG_ETH_DWMAC_NUM_RX_DMA_DESCRIPTORS] aligned_data(16);
struct eth_dwmac_dma_desc ndma_tx0_descs[CONFIG_ETH_DWMAC_NUM_TX_DMA_DESCRIPTORS] aligned_data(16);

struct eth_dwmac_dma_ch_config n_dma_rx_config[1] = {
        {
                .descs = ndma_rx0_descs,
                .descs_count = CONFIG_ETH_DWMAC_NUM_RX_DMA_DESCRIPTORS,
                .nr = 0,
        },
};

struct eth_dwmac_dma_ch_config n_dma_tx_config[1] = {
        {
                .descs = ndma_tx0_descs,
                .descs_count = CONFIG_ETH_DWMAC_NUM_TX_DMA_DESCRIPTORS,
                .nr = 0,
        },
};

struct eth_dwmac_rx_queue n_mtl_rx_config[1] = {
       {
                .nr = 0,
                .size = ALIGN_UP(CONFIG_NET_ETH_PKTSIZE, 8),
                .priority = 0xFF,
                .threshold = 64,
                .sf = true,
       },
};

struct eth_dwmac_tx_queue n_mtl_tx_config[1] = {
       {
                .nr = 0,
                .size = ALIGN_UP(CONFIG_NET_ETH_PKTSIZE, 8),
                .threshold = 64,
                .sf = true,
       },
};

const struct eth_dwmac_config dwmac_cfg0 = {
        .dma_rx_channel = 1,
        .dma_tx_channel = 1,
        .dma_rx = n_dma_rx_config,
        .dma_tx = n_dma_tx_config,
        .mtl_rx_queues = 1,
        .mtl_tx_queues = 1,
        .mtl_rx = n_mtl_rx_config,
        .mtl_tx = n_mtl_tx_config,
        .is_xgmac = true,
};

struct eth_xgmac_config eth_xgmac_cfg[CONFIG_DWXGMAC_NETHERNET] = {
        {.dma_base = CONFIG_DWXGMAC_DMA_BASE,
         .dwmac = dwmac_cfg0,
	 .ubl = true,
	}
};

struct eth_dwmac_dma_rx_ch_data dma_rx_data[1];
struct eth_dwmac_dma_tx_ch_data dma_tx_data[1];

struct eth_dwmac_data eth_xgmac_data[CONFIG_DWXGMAC_NETHERNET] = {
	{ .dma_rx = dma_rx_data, .dma_tx = dma_tx_data,},
};

static struct xgmac_driver_s g_xgmac[CONFIG_DWXGMAC_NETHERNET];
static uint8_t g_buffer_pool[CONFIG_ETH_DWMAC_NUM_RX_DMA_DESCRIPTORS * CONFIG_NET_ETH_PKTSIZE] aligned_data(16);

int xgmac_send(FAR struct xgmac_driver_s *priv,
                      FAR uint8_t *buf, uint16_t len)
{
  uint8_t dma_ch = 0; // TODO fixme
  struct eth_dwmac_config *cfg = &priv->config->dwmac;
  struct eth_dwmac_data *data = &priv->data->dwmac;
  FAR struct eth_dwmac_dma_ch_config *dma_cfg = &cfg->dma_tx[dma_ch];
  FAR struct eth_dwmac_dma_tx_ch_data *dma_data = &data->dma_tx[dma_ch];
  struct eth_xgmac_tdes_rd *tdes;

  /* Sanity checks */
  if (buf == NULL || len == 0)
    {
      return -EINVAL;
    }

  /* Lock TX channel */
  nxmutex_lock(&dma_data->lock);

  /* Wait for free TX descriptor */
  if (nxsem_wait_uninterruptible(&dma_data->desc_used) < 0)
    {
      nxmutex_unlock(&dma_data->lock);
      return -ETIMEDOUT;
    }

  /* Get TX descriptor */
  tdes = &cfg->dma_tx[dma_ch].descs[dma_data->tail];

  /* Fill descriptor */
  eth_dwmac_tdes_rd_set_buffer(tdes, buf, len);
  eth_dwmac_tdes_rd_set(tdes,
                        true,        /* First segment */
                        true,        /* Last segment */
                        len,
                        priv->txcoe,
                        false);      /* No timestamp for now */

  /* Advance ring index */
  dma_data->tail = (dma_data->tail + 1) % (dma_cfg->descs_count);

  /* Memory barrier before DMA kick */
  UP_DSB(); //TODO fixme !

  /* Notify hardware (TX tail pointer update) */
  eth_dwmac_dma_tx_set_tail(priv, dma_ch);

  nxmutex_unlock(&dma_data->lock);
  return OK;
}

static void xgmac_txpoll(void *arg)
{
  FAR struct xgmac_driver_s *priv = (FAR struct xgmac_driver_s *)arg;
  struct net_driver_s *dev = &priv->dev;

  if (dev->d_len > 0)
    {
      /* Send packet to XGMAC DMA */
      xgmac_send(priv, dev->d_buf, dev->d_len);

      /* Clear buffer after TX */
      dev->d_len = 0;
    }
}

static int dwxgmac_ifup(FAR struct net_driver_s *dev)
{
  FAR struct xgmac_driver_s *priv =
    (FAR struct xgmac_driver_s *)dev;

//  struct phy_link_state state;
  uint8_t dma_ch;

  /* Start RX DMA channels (always running) */
  for (dma_ch = 0; dma_ch < priv->config->dwmac.dma_rx_channel; dma_ch++)
    {
      eth_dwmac_dma_rx_fill_desc(priv, dma_ch);
      eth_dwmac_dma_rx_start(priv, dma_ch);
    }

  /* Start TX DMA channels if required */
//  if (priv->config->slave_mode || priv->config->clocks_stop)
    {
      for (dma_ch = 0; dma_ch < priv->config->dwmac.dma_tx_channel; dma_ch++)
        {
          eth_dwmac_dma_tx_start(priv, dma_ch);
        }
    }

  up_enable_irq(priv->irq);
  up_enable_irq(IRQ_GETH0_DMA_INTR);

  /* Mark interface up */
  priv->ifup = true;

  eth_dwmac_set_link_state(priv, true);
  netdev_carrier_on(dev);

  return OK;
}

static int dwxgmac_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct xgmac_driver_s *priv =
    (FAR struct xgmac_driver_s *)dev;

  uint8_t dma_ch;

  /* Disable interrupts first */
  up_disable_irq(priv->irq);

  /* Stop RX DMA channels */
  for (dma_ch = 0; dma_ch < priv->config->dwmac.dma_rx_channel; dma_ch++)
    {
      eth_dwmac_dma_rx_stop(priv, dma_ch);
      eth_dwmac_dma_rx_disable_irq(priv, dma_ch);
    }

  /* Stop TX DMA channels */
  for (dma_ch = 0; dma_ch < priv->config->dwmac.dma_tx_channel; dma_ch++)
    {
      eth_dwmac_dma_tx_stop(priv, dma_ch);
      eth_dwmac_dma_tx_disable_irq(priv, dma_ch);
    }

  /* Mark interface as down */
  priv->ifup = false;

  eth_dwmac_set_link_state(priv, false);
  /* Notify network stack that carrier is off */
  netdev_carrier_off(dev);

  return OK;
}

static void xgmac_rx_work(FAR void *arg)
{
  struct xgmac_driver_s *priv = arg;
  struct net_driver_s *dev = &priv->dev;
  struct eth_hdr_s *buf = (struct eth_hdr_s *)dev->d_buf;

  net_lock();

  /* IPv4 */
  if (dev->d_len > 0)
    {
#ifdef CONFIG_NET_IPv4
      if (buf->type == HTONS(ETHTYPE_IP))
        {
          NETDEV_RXIPV4(dev);
          ipv4_input(dev);
        }
#endif
#ifdef CONFIG_NET_IPv6
       if (buf->type == HTONS(ETHTYPE_IP6))
        {
          NETDEV_RXIPV6(dev);
          ipv6_input(dev);
        }
#endif
      else
        {
          NETDEV_RXERRORS(dev);
        }
    }

  dev->d_len = 0;
  net_unlock();
}

static void xgmac_dma_rx_process(struct xgmac_driver_s *priv,
                                 uint8_t dma_ch)
{
  struct eth_dwmac_config *cfg = &priv->config->dwmac;
  struct eth_dwmac_data   *data = &priv->data->dwmac;
  struct eth_dwmac_dma_ch_config *dma_cfg = &cfg->dma_rx[dma_ch];
  struct eth_dwmac_dma_rx_ch_data *dma_data = &data->dma_rx[dma_ch];

  struct net_driver_s *dev = &priv->dev;
  void *desc;

  net_lock();
  while (dma_data->head != dma_data->tail)
    {
      desc = &dma_cfg->descs[dma_data->head];

      /* Clear RX interrupt */
      eth_dmwac_dma_rx_clear_irq(priv, dma_ch);

      /* Stop if DMA still owns descriptor */
      if (eth_dwmac_rdes_own(desc))
        {
          break;
        }

      /* Descriptor-level error */
      if (eth_dwmac_rdes_desc_error(desc))
        {
          nerr("xgmac: RX descriptor error\n");
          NETDEV_TXERRORS(dev);
          goto next;
        }

      /* Ignore context descriptors */
      if (eth_dwmac_rdes_context(desc))
        {
          goto next;
        }

      /* Packet-level error */
      if (eth_dwmac_rdes_pkt_error(desc))
        {
          NETDEV_RXERRORS(dev);
          goto next;
        }

      /* First descriptor of frame */
      if (eth_dwmac_rdes_first(desc))
        {
          /* Set RX buffer */
          dev->d_buf = dma_data->data[dma_data->head];
          dev->d_len = 0;
        }

      /* Accumulate length */
      dev->d_len += eth_dwmac_rdes_packet_length(desc);

      /* Last descriptor of frame */
      if (eth_dwmac_rdes_last(desc))
        {
          /* Schedule RX processing (not in ISR) */
          work_queue(ETHWORK, &priv->rxwork, xgmac_rx_work, priv, 0);
        }

next:
      nxsem_post(&dma_data->desc_used);

      dma_data->head = (dma_data->head + 1) % dma_cfg->descs_count;
    }
  net_unlock();
}

static void xgmac_dma_tx_process(struct xgmac_driver_s *priv,
                                 uint8_t dma_ch)
{
  struct eth_dwmac_config *cfg = &priv->config->dwmac;
  struct eth_dwmac_data   *data = &priv->data->dwmac;
  struct eth_dwmac_dma_ch_config *dma_cfg = &cfg->dma_tx[dma_ch];
  struct eth_dwmac_dma_tx_ch_data *dma_data = &data->dma_tx[dma_ch];

  while (dma_data->head != dma_data->tail)
    {
      struct eth_xgmac_tdes_rd *tdes;

      eth_dmwac_dma_tx_clear_irq(priv, dma_ch);

      tdes = &dma_cfg->descs[dma_data->head];

      /* Stop if DMA still owns descriptor */
      if (eth_dwmac_tdes_own(&dma_cfg->descs[dma_data->head]))
        {
          break;
        }

      /* Last descriptor of a frame */
      if (eth_dwmac_tdes_last(&dma_cfg->descs[dma_data->head]))
        {
          if (eth_dwmac_tdes_error(&dma_cfg->descs[dma_data->head]))
            {
              nerr("xgmac: TX DMA error (ch %u)\n", dma_ch);
              NETDEV_TXERRORS(priv->dev);
            }
#ifdef CONFIG_NET_TIMESTAMP
          else
            {
              xgmac_dma_tx_timestamp(priv, dma_ch);
            }
#endif
        }

      /* Advance ring head */
      dma_data->head = (dma_data->head + 1) % dma_cfg->descs_count;

      /* One TX descriptor freed */
      nxsem_post(&dma_data->desc_used);
    }
}

static void xgmac_txavail_work(void *arg)
{
  FAR struct xgmac_driver_s *priv = (FAR struct xgmac_driver_s *)arg;

  net_lock();
  if (priv->ifup)
    {
      devif_poll(&priv->dev, xgmac_txpoll);
    }
  net_unlock();
}

static int dwxgmac_txavail(FAR struct net_driver_s *dev)
{
  FAR struct xgmac_driver_s *priv = (FAR struct xgmac_driver_s *)dev->d_private;
  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */
      work_queue(ETHWORK, &priv->pollwork, xgmac_txavail_work, priv, 0);
    }

  return OK;
}

#define TC4DX_BRIDGE_BASE 0xf901e000
#define TC4DX_BRIDGE_FWDCTRL (TC4DX_BRIDGE_BASE)
#define TC4DX_BRIDGE_PORTCTRL(i) (TC4DX_BRIDGE_BASE + 0xc + (12 * i))
#define TC4DX_BRIDGE_TXQMAP(i) (TC4DX_BRIDGE_BASE + 0x10 + (12 * i))
#define TC4DX_BRIDGE_RXCMAP(i) (TC4DX_BRIDGE_BASE + 0x14 + (12 * i))
#define TC4DX_BRIDGE_INTR_STATUS (TC4DX_BRIDGE_BASE + 0x104)
#define TC4DX_ETH_BASE 0xf9000000
#define TC4DX_ETH_CLC (TC4DX_ETH_BASE)
#define TC4DX_ETH_MACEN (TC4DX_ETH_BASE + 0x18)
#define TC4DX_ETH_RSTCTRL0 (TC4DX_ETH_BASE + 0xc)
#define TC4DX_ETH_RSTCTRL1 (TC4DX_ETH_BASE + 0x10)
#define TC4DX_ETH_RSTSTAT (TC4DX_ETH_BASE + 0x10)
#define TC4DX_ETH_ACCEN_GBL_WRA (TC4DX_ETH_BASE + 0x40)
#define TC4DX_ETH_ACCEN_GBL_WRB (TC4DX_ETH_BASE + 0x44)
#define TC4DX_ETH_ACCEN_CH_WRA(i) (TC4DX_ETH_BASE + 0xA0 + 0x20 * i)
#define TC4DX_ETH_ACCEN_CH_WRB(i) (TC4DX_ETH_BASE + 0xA4 + 0x20 * i)
#define TC4DX_HSPHY_BASE 0xf2000000
#define TC4DX_HSPHY_CLC TC4DX_HSPHY_BASE
#define TC4DX_HSPHY_RSTA (TC4DX_HSPHY_BASE + 0xc)
#define TC4DX_HSPHY_RSTB (TC4DX_HSPHY_BASE + 0x10)
#define TC4DX_HSPHY_RSTSTAT (TC4DX_HSPHY_BASE + 0x14)
#define TC4DX_HSPHY_CMNCFG (TC4DX_HSPHY_BASE + 0x200)
#define TC4DX_HSPHY_PHYCTRL0(i) (TC4DX_HSPHY_BASE + 0x210 + (8 * i))
#define TC4DX_HSPHY_PHYCTRL1(i) (TC4DX_HSPHY_BASE + 0x214 + (8 * i))
#define TC4DX_HSPHY_ETH_BASE(i) (TC4DX_HSPHY_BASE + 0x600 + (4 * i))
#define TC4DX_HSPHY_DLLCFG (TC4DX_HSPHY_BASE + 0x628)
#define IRQ_GETH0_INTR 760

static int xgmac_mac_isr(int irq,
                         FAR void *context,
                         FAR void *arg)
{
  FAR struct xgmac_driver_s *priv = arg;
  struct eth_xgmac_config *cfg = priv->config;
  uint32_t dma_status = 0;
  uint32_t status;

  status = xgmac_read(TC4DX_BRIDGE_INTR_STATUS);

  if (status & BIT(16))
    dma_status = xgmac_read(cfg->dma_base + DMA_INTERRUPT_STATUS);

  while (dma_status)
    {
       uint8_t dma_ch = fls(dma_status) - 1;
       uint32_t dma_ch_status = xgmac_read(cfg->dma_base + DMA_CHi_STATUS(dma_ch));
       if (dma_ch_status & DMA_CHi_STATUS_TI)
         {
           xgmac_dma_tx_process(priv, dma_ch);
         }

       if (dma_ch_status & DMA_CHi_STATUS_RI) 
         {
           xgmac_dma_rx_process(priv, dma_ch);
         }
       dma_ch_status &= ~(1 << dma_ch);
   }

  return OK;
}

static void xgmac_generate_mac(FAR uint8_t *mac)
{
  uint32_t rnd = 0;

  /* Try to get real entropy */
  if (getrandom(&rnd, sizeof(rnd)) != sizeof(rnd))
    {
      /* Fallback if RNG not ready */
      rnd = (uint32_t)clock_systime_ticks();
    }

  /* Locally administered, unicast MAC */
  mac[0] = 0x02;                /* 02 = local, unicast */
  mac[1] = 0x00;
  mac[2] = 0x00;
  mac[3] = (rnd >>  0) & 0xff;
  mac[4] = (rnd >>  8) & 0xff;
  mac[5] = (rnd >> 16) & 0xff;
}

static void xgmac_get_hwaddr(int inst,
                             FAR struct xgmac_driver_s *priv,
                             FAR uint8_t *mac)
{
  uint32_t high;
  uint32_t low;

  /* Read MAC address registers */
  low  = xgmac_read(priv, MAC_ADDRESSX_LOW(inst));
  high = xgmac_read(priv, MAC_ADDRESSX_HIGH(inst));

  mac[0] = (low >>  0) & 0xff;
  mac[1] = (low >>  8) & 0xff;
  mac[2] = (low >> 16) & 0xff;
  mac[3] = (low >> 24) & 0xff;
  mac[4] = (high >>  0) & 0xff;
  mac[5] = (high >>  8) & 0xff;

  /* Validate MAC */
  if (mac[0] == 0 && mac[1] == 0 && mac[2] == 0 &&
      mac[3] == 0 && mac[4] == 0 && mac[5] == 0)
    {
      /* Fallback */
      xgmac_generate_mac(mac);
    }
}

#define PMS_BASE 0xf0240000
#define PMS_VMONP_VDDHSIFCON (PMS_BASE + 0x90c0)
#define PMS_VMONP_VDDHSIFRST  (PMS_BASE + 0x90c4)
#define PMS_VMONP_VDDHSIFSTAT (PMS_BASE + 0x90c8)

#define PMS_VMONP_VDDPHY0CON (PMS_BASE + 0x9114)
#define PMS_VMONP_VDDPHY0RST (PMS_BASE + 0x9118)
#define PMS_VMONP_VDDPHY0STAT (PMS_BASE + 0x911c)

#define PMS_VMONP_VDDPHY1CON (PMS_BASE + 0x9120)
#define PMS_VMONP_VDDPHY1RST (PMS_BASE + 0x9124)

#define PMS_VMONP_VDDPHY2CON (PMS_BASE + 0x912c)
#define PMS_VMONP_VDDPHY2RST (PMS_BASE + 0x9130)

#define PMS_VMONP_VDDPHPHY0RST (PMS_BASE + 0x5c)
#define PMS_VMONP_VDDPHPHY1RST (PMS_BASE + 0x68)
#define PMS_VMONP_VDDPHPHY2RST (PMS_BASE + 0x74)
#define PMS_VMONP_OVE BIT(28)

void init_hsphy(void)
{
  int i;

  // Enable VDDHSIF
  xgmac_modreg(PMS_VMONP_OVE, PMS_VMONP_OVE, PMS_VMONP_VDDHSIFCON);
  for (i = 0; i < 10; i++) {
     if (xgmac_read(PMS_VMONP_VDDHSIFSTAT) & BIT(28) == 0) {
        break;
     }
     usleep(10);
  }

  xgmac_modreg(BIT(25), BIT(24) | BIT(25), PMS_VMONP_VDDHSIFRST);

  // Enable VDDPHY0
  xgmac_modreg(PMS_VMONP_OVE, PMS_VMONP_OVE, PMS_VMONP_VDDPHY0CON);

  // Enable VDDPHY1
  xgmac_modreg(PMS_VMONP_OVE, PMS_VMONP_OVE, PMS_VMONP_VDDPHY1CON);

  // Enable VDDPHY2
  xgmac_modreg(PMS_VMONP_OVE, PMS_VMONP_OVE, PMS_VMONP_VDDPHY2CON);

#if 0 //TODO: needs only for high speed
  // Enable VDDPHPHY0
  xgmac_modreg(BIT(28), BIT(28), PMS_VMONP_VDDPHPHY0RST);

  // Enable VDDPHPHY1
  xgmac_modreg(BIT(28), BIT(28), PMS_VMONP_VDDPHPHY1RST);

  // Enable VDDPHPHY2
  xgmac_modreg(BIT(28), BIT(28), PMS_VMONP_VDDPHPHY2RST);
#endif
}

int xgmac_initialize(int intf)
{
  FAR struct xgmac_driver_s *priv;
  struct eth_xgmac_config *cfg;
  uint8_t dma_ch;
  int ret = 0, i;
  uintptr_t addr;

  DEBUGASSERT(intf < CONFIG_DWXGMAC_NETHERNET);

  priv = &g_xgmac[intf];

  /* Initialize the driver structure */
  memset(priv, 0, sizeof(struct xgmac_driver_s));

  priv->base = CONFIG_DWXGMAC_BASE;
  priv->irq = IRQ_GETH0_INTR;
  priv->data = &eth_xgmac_data[intf];
  priv->config = &eth_xgmac_cfg[intf];

  priv->dev.d_ifup    = dwxgmac_ifup;
  priv->dev.d_ifdown  = dwxgmac_ifdown;
  priv->dev.d_txavail = dwxgmac_txavail;

  priv->dev.d_private = &g_xgmac[intf];

  init_hsphy();

  /* Enable GETH clock */
  xgmac_modreg(0, BIT(0), TC4DX_ETH_CLC);
  for (i = 0; i < 10; i++) {
     if ((xgmac_read(TC4DX_ETH_CLC) & BIT(1)) == 0) {
        break;
     }
     usleep(10);
  }

  if (i == 10)
    return -1;

  xgmac_modreg(0, BIT(0), TC4DX_HSPHY_CLC);
  for (i = 0; i < 10; i++) {
     if ((xgmac_read(TC4DX_HSPHY_CLC) & BIT(1)) == 0) {
        break;
     }
     usleep(10);
  }

  if (i == 10)
    return -1;

  /* Reset HSPHY */
  xgmac_modreg(BIT(31), BIT(31), TC4DX_HSPHY_RSTB);
  xgmac_modreg(BIT(0), BIT(0), TC4DX_HSPHY_RSTA);
  xgmac_modreg(BIT(0), BIT(0), TC4DX_HSPHY_RSTB);

  for (i = 0; i < 10; i++) {
     ret = xgmac_read(TC4DX_HSPHY_RSTSTAT) & BIT(0);
     if (ret == 0) {
        break;
     }
     usleep(10);
  }

  if (ret != 0)
    return -1;

  /* HSPHY power up*/
  xgmac_modreg(0, BIT(8),  TC4DX_HSPHY_PHYCTRL1(intf));
  usleep(25);
  xgmac_modreg(0, BIT(7),  TC4DX_HSPHY_PHYCTRL1(intf));

  /* Enable HSPHY ETH */
  xgmac_modreg(BIT(29), GENMASK(30,28), TC4DX_HSPHY_ETH_BASE(intf)); // rmii
  xgmac_modreg(GENMASK(15,12), GENMASK(15,12), TC4DX_HSPHY_ETH_BASE(intf)); // RXD0D, RXD1D
  xgmac_modreg(GENMASK(23,22), GENMASK(23,22), TC4DX_HSPHY_ETH_BASE(intf)); // REFCLKD
  xgmac_modreg(BIT(9), GENMASK(9,8), TC4DX_HSPHY_ETH_BASE(intf)); // CRSDVC 
  xgmac_modreg(BIT(4), BIT(4), TC4DX_HSPHY_CMNCFG); // RGMII or RMII
  xgmac_modreg(BIT(31) | GENMASK(24, 22), BIT(31) | GENMASK(24, 22), TC4DX_HSPHY_DLLCFG); // 3.3V, pad in clock mode, RX enable, power on

  /* MDIO pin config */
  xgmac_modreg(GENMASK(1,0), GENMASK(1,0), TC4DX_HSPHY_ETH_BASE(intf)); // MDIO pin
  xgmac_modreg(BIT(2), BIT(2), TC4DX_HSPHY_ETH_BASE(intf)); // MDIO enable

  /* Enable GETH MAC */
  xgmac_modreg(BIT(intf), BIT(intf), TC4DX_ETH_MACEN); // TODO mac enabled !!

  /* reset the GETH MAC */
  xgmac_modreg(BIT(0), BIT(0), TC4DX_ETH_RSTCTRL0); // TODO mac enabled !!
  xgmac_modreg(BIT(0), BIT(0), TC4DX_ETH_RSTCTRL1); // TODO mac enabled !!

  for (i = 0; i < 10; i++) {
     ret = xgmac_read(TC4DX_ETH_RSTSTAT) & BIT(0);
     if (ret == 1) {
        break;
     }
     usleep(10);
  }

  if (ret == 0)
    return -1;

  xgmac_modreg(BIT(31), BIT(31), TC4DX_ETH_RSTCTRL1); // TODO mac enabled !!

  /* bridge port setting */ //TODO ports as dma used
  xgmac_modreg(BIT(8), GENMASK(15, 8), TC4DX_BRIDGE_PORTCTRL(intf));
  xgmac_modreg(BIT(24), GENMASK(31, 24), TC4DX_BRIDGE_PORTCTRL(intf));

  /* APU */
  xgmac_write(0xFFFFFFFF, TC4DX_ETH_ACCEN_GBL_WRA);
  xgmac_write(0xFFFFFFFF, TC4DX_ETH_ACCEN_GBL_WRB);

  for (int i = 0; i < 8; i++) {
    xgmac_write(0xFFFFFFFF, TC4DX_ETH_ACCEN_CH_WRA(i));
    xgmac_write(0xFFFFFFFF, TC4DX_ETH_ACCEN_CH_WRB(i));
  }

  /* MAC address */
  xgmac_get_hwaddr(intf, priv, priv->dev.d_mac.ether.ether_addr_octet);
  cfg = &eth_xgmac_cfg[intf];
  /* Initialize TX/RX DMA channels */
  for (dma_ch = 0; dma_ch < cfg->dwmac.dma_tx_channel; dma_ch++) {
          eth_dwmac_dma_tx_data_init(priv, dma_ch);
  }

  addr = (uintptr_t)g_buffer_pool;
  for (dma_ch = 0; dma_ch < cfg->dwmac.dma_rx_channel; dma_ch++) {
          eth_dwmac_dma_rx_data_init(priv, dma_ch, addr);
          addr += CONFIG_NET_ETH_PKTSIZE;
  }

  xgmac_modreg(MAC_INTERRUPT_ENABLE_TSIE, MAC_INTERRUPT_ENABLE_TSIE, priv->base + MAC_INTERRUPT_ENABLE);
  xgmac_write(0xFFFFFFFF, priv->base + MMC_FPE_RX_INTERRUPT_MASK);
  xgmac_write(0xFFFFFFFF, priv->base + MMC_FPE_TX_INTERRUPT_MASK);
  xgmac_write(0xFFFFFFFF, priv->base + MMC_IPC_RX_INTERRUPT_MASK);

  eth_dwmac_dma_init(priv);
  eth_dwmac_mtl_init(priv);
  eth_dwmac_mac_init(priv);

  /* Attach ISR */
  ret = irq_attach(priv->irq, xgmac_mac_isr, priv);
  ret = irq_attach(IRQ_GETH0_DMA_INTR, xgmac_mac_isr, priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Register XGMAC Ethernet device */
  ret = netdev_register(&priv->dev, NET_LL_ETHERNET);
  if (ret < 0)
    {
      goto err_irq;
    }

  return OK;

err_irq:
  irq_detach(priv->irq);

  return ret;
}
