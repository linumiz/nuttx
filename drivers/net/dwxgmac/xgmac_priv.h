/****************************************************************************
 * drivers/net/xgmac_priv.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 *
 ****************************************************************************/

#ifndef XGMAC_PRIV_H
#define XGMAC_PRIV_H

#include "dwxgmac.h"
#include "dwxgmac_reg.h"
#include <nuttx/bits.h>

/****************************************************************************
 * Helper macros
 ****************************************************************************/

#define LSB_GET(value)            ((value) & -(value))
#define FIELD_PREP(mask, value)   (((value) * LSB_GET(mask)) & (mask))
#define FIELD_GET(mask, value)    (((value) & (mask)) / LSB_GET(mask))

#ifndef ALIGN_UP
#  define ALIGN_UP(x, a)          (((x) + (a) - 1) & ~((a) - 1))
#endif

#ifndef aligned_data
#  define aligned_data(n) __attribute__((aligned(n)))
#endif

/****************************************************************************
 * XGMAC-specific config / data
 ****************************************************************************/

struct eth_xgmac_config
{
  const struct eth_dwmac_config dwmac;
  uint8_t blen;
  bool aal;
  bool ubl;
};

struct eth_xgmac_data
{
  struct eth_dwmac_data dwmac;
};

struct xgmac_driver_s
{
  struct net_driver_s dev;
  uintptr_t base;
  int irq;

  struct eth_xgmac_config *config;
  struct eth_xgmac_data *data;

  struct work_s rxwork;
  struct work_s pollwork;

  bool ifup;
  bool txcoe;
};

struct eth_xgmac_rdes_rd
{
  struct { uint32_t buf1ap; }         rdes0;
  struct { uint32_t buf1ap_high; }    rdes1;
  struct { uint32_t buf2ap; }         rdes2;
  struct
  {
    uint32_t buf2ap : 30;
    uint32_t ioc    : 1;
    uint32_t own    : 1;
  } rdes3;
};

struct eth_xgmac_rdes_wb
{
  union
  {
    struct { uint32_t ovt : 16; uint32_t ivt : 16; } rdes0;
    uint32_t rdes0_raw;
  };
  struct { uint32_t rssh_frpli; } rdes1;
  struct
  {
    uint32_t avtcp   : 1;
    uint32_t avtdp   : 1;
    uint32_t hl      : 8;
    uint32_t frspsm  : 1;
    uint32_t tnp     : 1;
    uint32_t eld     : 1;
    uint32_t ios     : 1;
    uint32_t rpng    : 1;
    uint32_t vf      : 1;
    uint32_t saf     : 1;
    uint32_t hfdaf   : 2;
    uint32_t madrm   : 8;
    uint32_t l3fm    : 1;
    uint32_t l4fm    : 1;
    uint32_t l3l4fm  : 3;
  } rdes2;
  struct
  {
    uint32_t pl      : 14;
    uint32_t frpsl   : 1;
    uint32_t es      : 1;
    uint32_t et_lt   : 4;
    uint32_t l3l4t   : 4;
    uint32_t etm_ncp : 1;
    uint32_t isp     : 1;
    uint32_t rsv     : 1;
    uint32_t cda     : 1;
    uint32_t ld      : 1;
    uint32_t fd      : 1;
    uint32_t ctxt    : 1;
    uint32_t own     : 1;
  } rdes3;
};

/****************************************************************************
 * TX descriptor — Read format
 ****************************************************************************/

struct eth_xgmac_tdes_rd
{
  struct { uint32_t buf1ap; }  tdes0;
  struct { uint32_t buf2ap; }  tdes1;
  struct
  {
    uint32_t b1l  : 14;
    uint32_t vtir : 2;
    uint32_t b2l  : 14;
    uint32_t ttse : 1;
    uint32_t ioc  : 1;
  } tdes2;
  struct
  {
    uint32_t fl       : 15;
    uint32_t tpl      : 1;
    uint32_t cic      : 2;
    uint32_t tse      : 1;
    uint32_t slotnum  : 4;
    uint32_t saic     : 3;
    uint32_t cpc      : 2;
    uint32_t ld       : 1;
    uint32_t fd       : 1;
    uint32_t ctxt     : 1;
    uint32_t own      : 1;
  } tdes3;
};

/****************************************************************************
 * TX descriptor — Write-back format
 ****************************************************************************/

struct eth_xgmac_tdes_wb
{
  uint32_t tdes0;
  uint32_t tdes1;
  uint32_t tdes2;
  struct
  {
    uint32_t reserved : 27;
    uint32_t derr     : 1;
    uint32_t ld       : 1;
    uint32_t fd       : 1;
    uint32_t ctxt     : 1;
    uint32_t own      : 1;
  } tdes3;
};

/****************************************************************************
 * RX descriptor inline accessors
 ****************************************************************************/

static inline bool eth_dwmac_rdes_own(void *d)
{
  return ((struct eth_xgmac_rdes_rd *)d)->rdes3.own == 1;
}

static inline bool eth_dwmac_rdes_first(void *d)
{
  return ((struct eth_xgmac_rdes_wb *)d)->rdes3.fd == 1;
}

static inline bool eth_dwmac_rdes_last(void *d)
{
  return ((struct eth_xgmac_rdes_wb *)d)->rdes3.ld == 1;
}

static inline bool eth_dwmac_rdes_context(void *d)
{
  return ((struct eth_xgmac_rdes_wb *)d)->rdes3.ctxt == 1;
}

static inline bool eth_dwmac_rdes_pkt_error(void *d)
{
  struct eth_xgmac_rdes_wb *wb = d;
  return (wb->rdes3.ld == 1) && (wb->rdes3.es == 1);
}

static inline bool eth_dwmac_rdes_desc_error(void *d)
{
  struct eth_xgmac_rdes_wb *wb = d;
  return wb->rdes3.fd == 1 && wb->rdes3.ld == 1 && wb->rdes3.ctxt == 1;
}

static inline uint16_t eth_dwmac_rdes_packet_length(void *d)
{
  return ((struct eth_xgmac_rdes_wb *)d)->rdes3.pl;
}

static inline void eth_dwmac_rdes_rd_set(void *d, uintptr_t buf)
{
  struct eth_xgmac_rdes_rd *rdes = d;
  rdes->rdes0.buf1ap = (uint32_t)buf;
  rdes->rdes1.buf1ap_high = 0;
  rdes->rdes2.buf2ap = 0;
  /* Set IOC + OWN */
  *(volatile uint32_t *)&rdes->rdes3 = BIT(30) | BIT(31);
}

static inline void eth_dwmac_tdes_rd_set_buffer(struct eth_xgmac_tdes_rd *t,
                                                 uint8_t *buf, uint16_t len)
{
  t->tdes0.buf1ap = (uint32_t)(uintptr_t)buf;
  t->tdes1.buf2ap = 0;
  t->tdes2.b1l = len;
  t->tdes2.b2l = 0;
}

static inline void eth_dwmac_tdes_rd_set(struct eth_xgmac_tdes_rd *t,
                                         bool fd, bool ld, uint16_t len,
                                         bool txcoe, bool ts)
{
  t->tdes2.ioc = ld ? 1 : 0;
  t->tdes2.ttse = ts ? 1 : 0;

  uint32_t d3 = 0;
  d3 |= (len & 0x7FFF) |  BIT(31);
  txcoe ? d3 |= (3u << 16) : d3;
  ld ? d3 |= BIT(28) : d3;
  fd ? d3 |= BIT(29) : d3;

  *(volatile uint32_t *)&t->tdes3 = d3;
}

static inline bool eth_dwmac_tdes_own(void *d)
{
  return ((struct eth_xgmac_tdes_wb *)d)->tdes3.own;
}

static inline bool eth_dwmac_tdes_last(void *d)
{
  return ((struct eth_xgmac_tdes_wb *)d)->tdes3.ld;
}

static inline bool eth_dwmac_tdes_error(void *d)
{
  return ((struct eth_xgmac_tdes_wb *)d)->tdes3.derr;
}

static inline void eth_dwmac_dma_rx_set_tail(const struct xgmac_driver_s *priv,
                                             uint8_t dma_ch)
{
  const struct eth_xgmac_config *cfg = priv->config;
  struct eth_dwmac_data *data = &priv->data->dwmac;
  uint8_t hw_ch = cfg->dwmac.dma_rx[dma_ch].nr;
  void *tail = &cfg->dwmac.dma_rx[dma_ch].descs[data->dma_rx[dma_ch].tail];

  xgmac_write((uint32_t)(uintptr_t)tail,
              priv->base + DMA_CHi_RXDESC_TAIL_LPOINTER(hw_ch));
}

static inline void eth_dwmac_dma_rx_init(const struct xgmac_driver_s *priv,
                                         uint8_t dma_ch)
{
  const struct eth_xgmac_config *cfg = priv->config;
  uint8_t hw_ch = cfg->dwmac.dma_rx[dma_ch].nr;
  const struct eth_dwmac_dma_ch_config *dcfg = &cfg->dwmac.dma_rx[dma_ch];

  xgmac_write((uint32_t)(uintptr_t)dcfg->descs,
              priv->base + DMA_CHi_RXDESC_LIST_LADDRESS(hw_ch));
  xgmac_write(FIELD_PREP(DMA_CHi_RX_CONTROL_RBSZ,
              CONFIG_NET_ETH_PKTSIZE / 8),
              priv->base + DMA_CHi_RX_CONTROL(hw_ch));
  xgmac_write(FIELD_PREP(DMA_CHi_RX_CONTROL2_RDRL,
              dcfg->descs_count - 1),
              priv->base + DMA_CHi_RX_CONTROL2(hw_ch));
  xgmac_write((uint32_t)(uintptr_t)dcfg->descs,
              priv->base + DMA_CHi_RXDESC_TAIL_LPOINTER(hw_ch));
}

static inline void eth_dwmac_dma_rx_start(const struct xgmac_driver_s *priv,
                                          uint8_t dma_ch)
{
  const struct eth_xgmac_config *cfg = priv->config;
  uint8_t hw_ch = cfg->dwmac.dma_rx[dma_ch].nr;

  xgmac_modreg(DMA_CHi_RX_CONTROL_SR, DMA_CHi_RX_CONTROL_SR,
               priv->base + DMA_CHi_RX_CONTROL(hw_ch));
}

static inline void eth_dwmac_dma_rx_stop(const struct xgmac_driver_s *priv,
                                         uint8_t dma_ch)
{
  const struct eth_xgmac_config *cfg = priv->config;
  uint8_t hw_ch = cfg->dwmac.dma_rx[dma_ch].nr;

  xgmac_modreg(0, DMA_CHi_RX_CONTROL_SR,
               priv->base + DMA_CHi_RX_CONTROL(hw_ch));
}

static inline void eth_dwmac_dma_rx_irq_enable(const struct xgmac_driver_s *priv,
                                               uint8_t dma_ch)
{
  const struct eth_xgmac_config *cfg = priv->config;
  uint8_t hw_ch = cfg->dwmac.dma_rx[dma_ch].nr;

  xgmac_modreg(DMA_CHi_INTERRUPT_ENABLE_RIE |
               DMA_CHi_INTERRUPT_ENABLE_NIE |
               DMA_CHi_INTERRUPT_ENABLE_FBEE |
               DMA_CHi_INTERRUPT_ENABLE_AIE,
               DMA_CHi_INTERRUPT_ENABLE_RIE |
               DMA_CHi_INTERRUPT_ENABLE_NIE |
               DMA_CHi_INTERRUPT_ENABLE_FBEE |
               DMA_CHi_INTERRUPT_ENABLE_AIE,
               priv->base + DMA_CHi_INTERRUPT_ENABLE(hw_ch));
}

static inline void eth_dwmac_dma_rx_disable_irq(const struct xgmac_driver_s *priv,
                                                uint8_t dma_ch)
{
  const struct eth_xgmac_config *cfg = priv->config;
  uint8_t hw_ch = cfg->dwmac.dma_rx[dma_ch].nr;

  xgmac_modreg(0,
               DMA_CHi_INTERRUPT_ENABLE_RIE |
               DMA_CHi_INTERRUPT_ENABLE_NIE |
               DMA_CHi_INTERRUPT_ENABLE_FBEE |
               DMA_CHi_INTERRUPT_ENABLE_AIE,
               priv->base + DMA_CHi_INTERRUPT_ENABLE(hw_ch));
}

static inline void eth_dwmac_dma_rx_clear_irq(const struct xgmac_driver_s *priv,
                                              uint8_t dma_ch)
{
  const struct eth_xgmac_config *cfg = priv->config;
  uint8_t hw_ch = cfg->dwmac.dma_rx[dma_ch].nr;

  xgmac_write(DMA_CHi_STATUS_RI | DMA_CHi_STATUS_NIS,
              priv->base + DMA_CHi_STATUS(hw_ch));
}

static inline void eth_dwmac_dma_tx_set_tail(const struct xgmac_driver_s *priv,
                                             uint8_t dma_ch)
{
  const struct eth_xgmac_config *cfg = priv->config;
  struct eth_dwmac_data *data = &priv->data->dwmac;
  uint8_t hw_ch = cfg->dwmac.dma_tx[dma_ch].nr;
  void *tail = &cfg->dwmac.dma_tx[dma_ch].descs[data->dma_tx[dma_ch].tail];

  xgmac_write((uint32_t)(uintptr_t)tail,
              priv->base + DMA_CHi_TXDESC_TAIL_LPOINTER(hw_ch));
}

static inline void eth_dwmac_dma_tx_init(const struct xgmac_driver_s *priv,
                                         uint8_t dma_ch)
{
  const struct eth_xgmac_config *cfg = priv->config;
  uint8_t hw_ch = cfg->dwmac.dma_tx[dma_ch].nr;
  const struct eth_dwmac_dma_ch_config *dcfg = &cfg->dwmac.dma_tx[dma_ch];

  xgmac_write((uint32_t)(uintptr_t)dcfg->descs,
              priv->base + DMA_CHi_TXDESC_LIST_LADDRESS(hw_ch));
  xgmac_write(FIELD_PREP(DMA_CHi_TX_CONTROL2_TDRL,
              dcfg->descs_count - 1),
              priv->base + DMA_CHi_TX_CONTROL2(hw_ch));
  xgmac_write((uint32_t)(uintptr_t)dcfg->descs,
              priv->base + DMA_CHi_TXDESC_TAIL_LPOINTER(hw_ch));
}

static inline void eth_dwmac_dma_tx_start(const struct xgmac_driver_s *priv,
                                          uint8_t dma_ch)
{
  const struct eth_xgmac_config *cfg = priv->config;
  uint8_t hw_ch = cfg->dwmac.dma_tx[dma_ch].nr;

  xgmac_modreg(DMA_CHi_TX_CONTROL_ST, DMA_CHi_TX_CONTROL_ST,
               priv->base + DMA_CHi_TX_CONTROL(hw_ch));
}

static inline void eth_dwmac_dma_tx_stop(const struct xgmac_driver_s *priv,
                                         uint8_t dma_ch)
{
  const struct eth_xgmac_config *cfg = priv->config;
  uint8_t hw_ch = cfg->dwmac.dma_tx[dma_ch].nr;

  xgmac_modreg(0, DMA_CHi_TX_CONTROL_ST,
               priv->base + DMA_CHi_TX_CONTROL(hw_ch));
}

static inline void eth_dwmac_dma_tx_irq_enable(const struct xgmac_driver_s *priv,
                                               uint8_t dma_ch)
{
  const struct eth_xgmac_config *cfg = priv->config;
  uint8_t hw_ch = cfg->dwmac.dma_tx[dma_ch].nr;

  xgmac_modreg(DMA_CHi_INTERRUPT_ENABLE_TIE |
               DMA_CHi_INTERRUPT_ENABLE_NIE |
               DMA_CHi_INTERRUPT_ENABLE_FBEE |
               DMA_CHi_INTERRUPT_ENABLE_AIE,
               DMA_CHi_INTERRUPT_ENABLE_TIE |
               DMA_CHi_INTERRUPT_ENABLE_NIE |
               DMA_CHi_INTERRUPT_ENABLE_FBEE |
               DMA_CHi_INTERRUPT_ENABLE_AIE,
               priv->base + DMA_CHi_INTERRUPT_ENABLE(hw_ch));
}

static inline void eth_dwmac_dma_tx_disable_irq(const struct xgmac_driver_s *priv,
                                                uint8_t dma_ch)
{
  const struct eth_xgmac_config *cfg = priv->config;
  uint8_t hw_ch = cfg->dwmac.dma_tx[dma_ch].nr;

  xgmac_modreg(0,
               DMA_CHi_INTERRUPT_ENABLE_TIE |
               DMA_CHi_INTERRUPT_ENABLE_NIE |
               DMA_CHi_INTERRUPT_ENABLE_FBEE |
               DMA_CHi_INTERRUPT_ENABLE_AIE,
               priv->base + DMA_CHi_INTERRUPT_ENABLE(hw_ch));
}

static inline void eth_dwmac_dma_tx_clear_irq(const struct xgmac_driver_s *priv,
                                              uint8_t dma_ch)
{
  const struct eth_xgmac_config *cfg = priv->config;
  uint8_t hw_ch = cfg->dwmac.dma_tx[dma_ch].nr;

  xgmac_write(DMA_CHi_STATUS_TI | DMA_CHi_STATUS_TBU | DMA_CHi_STATUS_NIS,
              priv->base + DMA_CHi_STATUS(hw_ch));
}

static inline void eth_dwmac_dma_tx_queue_flush(const struct xgmac_driver_s *priv,
                                                uint8_t dma_ch)
{
  const struct eth_xgmac_config *cfg = priv->config;

  xgmac_modreg(MTL_TXQX_OPERATION_MODE_TQS, MTL_TXQX_OPERATION_MODE_TQS,
               priv->base + MTL_TXQX_OPERATION_MODE(cfg->dwmac.dma_tx[dma_ch].nr));
}

static inline void eth_dwmac_dma_set_sysbus(const struct xgmac_driver_s *priv)
{
  const struct eth_xgmac_config *cfg = priv->config;

  xgmac_write((cfg->aal ? DMA_SYSBUS_MODE_AAL : 0) |
              (cfg->ubl ? DMA_SYSBUS_MODE_UBL : 0) |
              ((uint32_t)cfg->blen << 1),
              priv->base + DMA_SYSBUS_MODE);
}

static inline void eth_dwmac_mac_config(const struct xgmac_driver_s *priv)
{
  xgmac_write(MAC_RX_CONFIGURATION_IPC |
              MAC_RX_CONFIGURATION_CST |
              MAC_RX_CONFIGURATION_ACS,
              priv->base + MAC_RX_CONFIGURATION);

  xgmac_write(MAC_PACKET_FILTER_PM, priv->base + MAC_PACKET_FILTER);
  xgmac_write(0, priv->base + MAC_EXTENDED_CONFIGURATION);
}

static inline void eth_dwmac_mac_set_link(const struct xgmac_driver_s *priv,
                                          uint32_t ss, bool fd)
{
  xgmac_modreg(FIELD_PREP(MAC_TX_CONFIGURATION_SS, ss),
               MAC_TX_CONFIGURATION_SS,
               priv->base + MAC_TX_CONFIGURATION);
  xgmac_modreg(fd ? 0 : MAC_EXTENDED_CONFIGURATION_HD,
               MAC_EXTENDED_CONFIGURATION_HD,
               priv->base + MAC_EXTENDED_CONFIGURATION);
}

static inline bool eth_dwmac_mac_rx_state(const struct xgmac_driver_s *priv)
{
  return (xgmac_read(priv->base + MAC_RX_CONFIGURATION) &
         MAC_RX_CONFIGURATION_RE) != 0;
}

static inline void eth_dwmac_mac_rx_set_state(const struct xgmac_driver_s *priv,
                                              bool up)
{
  xgmac_modreg(up ? MAC_RX_CONFIGURATION_RE : 0,
               MAC_RX_CONFIGURATION_RE,
               priv->base + MAC_RX_CONFIGURATION);
}

static inline bool eth_dwmac_mac_tx_state(const struct xgmac_driver_s *priv)
{
  return (xgmac_read(priv->base + MAC_TX_CONFIGURATION) &
         MAC_TX_CONFIGURATION_TE) != 0;
}

static inline void eth_dwmac_mac_tx_set_state(const struct xgmac_driver_s *priv,
                                              bool up)
{
  xgmac_modreg(up ? MAC_TX_CONFIGURATION_TE : 0,
               MAC_TX_CONFIGURATION_TE,
               priv->base + MAC_TX_CONFIGURATION);
}

static inline void eth_dwmac_set_mac_addr(const struct xgmac_driver_s *priv,
                                          uint8_t nr, uint8_t *mac)
{
  uint32_t hi = ((uint32_t)mac[5] << 8) | mac[4];
  uint32_t lo = ((uint32_t)mac[3] << 24) | ((uint32_t)mac[2] << 16) |
                ((uint32_t)mac[1] << 8) | mac[0];

  xgmac_write(hi | MAC_ADDRESSX_HIGH_AE, priv->base + MAC_ADDRESSX_HIGH(nr));
  xgmac_write(lo, priv->base + MAC_ADDRESSX_LOW(nr));
}

static inline void eth_dwmac_mtl_set_mode(const struct xgmac_driver_s *priv,
                                          uint8_t schalg)
{
  xgmac_write(FIELD_PREP(MTL_OPERATION_MODE_ETSALG, schalg),
              priv->base + MTL_OPERATION_MODE);
}

static inline void eth_dwmac_mtl_rxq_init(const struct xgmac_driver_s *priv,
                                          uint8_t queue)
{
  const struct eth_dwmac_rx_queue *qcfg = &priv->config->dwmac.mtl_rx[queue];

  xgmac_write(FIELD_PREP(MTL_RXQX_OPERATION_MODE_RQS, qcfg->size / 256) |
              (qcfg->sf ? MTL_RXQX_OPERATION_MODE_RSF : 0),
              priv->base + MTL_RXQX_OPERATION_MODE(qcfg->nr));
}

static inline void eth_dwmac_mtl_txq_init(const struct xgmac_driver_s *priv,
                                          uint8_t queue)
{
  const struct eth_dwmac_tx_queue *qcfg = &priv->config->dwmac.mtl_tx[queue];

  xgmac_write(FIELD_PREP(MTL_TXQX_OPERATION_MODE_TQS, 7) |
              (qcfg->sf ? MTL_TXQX_OPERATION_MODE_TSF
                        : FIELD_PREP(MTL_TXQX_OPERATION_MODE_TTC, qcfg->threshold)) |
              FIELD_PREP(GENMASK(3, 2), 0x2),
              priv->base + MTL_TXQX_OPERATION_MODE(queue));
}

static inline void eth_dwmac_mtl_set_rxq_ctrl(const struct xgmac_driver_s *priv)
{
  const struct eth_xgmac_config *cfg = priv->config;
  uint8_t queue;
  uint32_t rxq_ctrl0 = 0;
  uint32_t rxq_dma_map[2] = {0, 0};

  for (queue = 0; queue < cfg->dwmac.mtl_rx_queues; queue++)
    {
      const struct eth_dwmac_rx_queue *qcfg = &cfg->dwmac.mtl_rx[queue];

      rxq_ctrl0 |= (0x2u << (2 * qcfg->nr));

      rxq_dma_map[qcfg->nr / 4] |= (qcfg->dma_channel << ((qcfg->nr % 4) * 8));
    }

  xgmac_write(rxq_dma_map[0], priv->base + MTL_RXQ_DMA_MAP0);
  xgmac_write(rxq_dma_map[1], priv->base + MTL_RXQ_DMA_MAP1);
  xgmac_write(rxq_ctrl0, priv->base + MAC_RXQ_CTRL0);
}

#endif /* XGMAC_PRIV_H */
