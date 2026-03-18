/****************************************************************************
 * drivers/net/qos_priv.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 *
 ****************************************************************************/

#ifndef QOS_PRIV_H
#define QOS_PRIV_H

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/bits.h>
#include <nuttx/net/netdev.h>
#include <nuttx/wqueue.h>

#include "dwxgmac.h"
#include "qos_reg.h"

/****************************************************************************
 * Helper macros (mirrors xgmac_priv.h)
 ****************************************************************************/

#define LSB_GET(value)            ((value) & -(value))
#define FIELD_PREP(mask, value)   (((value) * LSB_GET(mask)) & (mask))
#define FIELD_GET(mask, value)    (((value) & (mask)) / LSB_GET(mask))

#ifndef ALIGN_UP
#  define ALIGN_UP(x, a) (((x) + (a) - 1) & ~((a) - 1))
#endif

/****************************************************************************
 * Config / data structs
 ****************************************************************************/

struct eth_qos_rdes_rd
{
  struct { uint32_t buf1ap; }      rdes0;
  struct { uint32_t buf1ap_high; } rdes1;
  struct { uint32_t buf2ap; }      rdes2;
  struct
  {
    uint32_t        : 24;
    uint32_t buf1v  : 1;
    uint32_t buf2v  : 1;
    uint32_t        : 4;
    uint32_t ioc    : 1;
    uint32_t own    : 1;
  } rdes3;
};

struct eth_qos_rdes_wb
{
  struct
  {
    uint32_t ovt : 16;
    uint32_t ivt : 16;
  } rdes0;
  struct
  {
    uint32_t pt   : 3;
    uint32_t iphe : 1;
    uint32_t ipv4 : 1;
    uint32_t ipv6 : 1;
    uint32_t ipcb : 1;
    uint32_t ipce : 1;
    uint32_t pmt  : 4;
    uint32_t pft  : 1;
    uint32_t pv   : 1;
    uint32_t tsa  : 1;
    uint32_t td   : 1;
    uint32_t opc  : 16;
  } rdes1;
  struct
  {
    uint32_t hl     : 10;
    uint32_t arpnr  : 1;
    uint32_t ios    : 3;
    uint32_t its    : 1;
    uint32_t ots    : 1;
    uint32_t saf    : 1;
    uint32_t daf    : 1;
    uint32_t hf     : 1;
    uint32_t madrm  : 8;
    uint32_t l3fm   : 1;
    uint32_t l4fm   : 1;
    uint32_t l3l4fm : 3;
  } rdes2;
  struct
  {
    uint32_t pl   : 15;
    uint32_t es   : 1;
    uint32_t lt   : 3;
    uint32_t de   : 1;
    uint32_t re   : 1;
    uint32_t oe   : 1;
    uint32_t rwt  : 1;
    uint32_t gp   : 1;
    uint32_t ce   : 1;
    uint32_t rs0v : 1;
    uint32_t rs1v : 1;
    uint32_t rs2v : 1;
    uint32_t ld   : 1;
    uint32_t fd   : 1;
    uint32_t ctxt : 1;
    uint32_t own  : 1;
  } rdes3;
};

struct eth_qos_rdes_ctx
{
  uint32_t rdes0;
  uint32_t rdes1;
  uint32_t rdes2;
  struct
  {
    uint32_t     : 29;
    uint32_t de  : 1;
    uint32_t ctxt: 1;
    uint32_t own : 1;
  } rdes3;
};

struct eth_qos_tdes_rd
{
  struct { uint32_t buf1ap; } tdes0;
  struct { uint32_t buf2ap; } tdes1;
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
    uint32_t fl      : 15;
    uint32_t tpl     : 1;
    uint32_t cic     : 2;
    uint32_t tse     : 1;
    uint32_t slotnum : 4;
    uint32_t saic    : 3;
    uint32_t cpc     : 2;
    uint32_t ld      : 1;
    uint32_t fd      : 1;
    uint32_t ctxt    : 1;
    uint32_t own     : 1;
  } tdes3;
};

struct eth_qos_tdes_wb
{
  struct { uint32_t ttsl; } tdes0;
  struct { uint32_t ttsh; } tdes1;
  struct { uint32_t      : 32; } tdes2;
  struct
  {
    uint32_t ihe  : 1;
    uint32_t db   : 1;
    uint32_t uf   : 1;
    uint32_t ed   : 1;
    uint32_t cc   : 4;
    uint32_t ec   : 1;
    uint32_t lc   : 1;
    uint32_t nc   : 1;
    uint32_t loc  : 1;
    uint32_t pce  : 1;
    uint32_t ff   : 1;
    uint32_t jt   : 1;
    uint32_t es   : 1;
    uint32_t      : 1;
    uint32_t ttss : 1;
    uint32_t      : 5;
    uint32_t derr : 1;
    uint32_t      : 4;
    uint32_t ld   : 1;
    uint32_t fd   : 1;
    uint32_t ctxt : 1;
    uint32_t own  : 1;
  } tdes3;
};

static inline bool eth_dwmac_rdes_own(void *d)
{
  return ((struct eth_qos_rdes_rd *)d)->rdes3.own == 1;
}

static inline bool eth_dwmac_rdes_first(void *d)
{
  return ((struct eth_qos_rdes_wb *)d)->rdes3.fd == 1;
}

static inline bool eth_dwmac_rdes_last(void *d)
{
  return ((struct eth_qos_rdes_wb *)d)->rdes3.ld == 1;
}

static inline bool eth_dwmac_rdes_context(void *d)
{
  return ((struct eth_qos_rdes_wb *)d)->rdes3.ctxt == 1;
}

static inline bool eth_dwmac_rdes_pkt_error(void *d)
{
  struct eth_qos_rdes_wb *wb = d;
  return (wb->rdes3.ld == 1) && (wb->rdes3.es == 1);
}

static inline bool eth_dwmac_rdes_desc_error(void *d)
{
  struct eth_qos_rdes_wb *wb = d;
  return wb->rdes3.fd == 1 && wb->rdes3.ctxt == 1;
}

static inline uint16_t eth_dwmac_rdes_packet_length(void *d)
{
  return ((struct eth_qos_rdes_wb *)d)->rdes3.pl;
}

static inline void eth_dwmac_rdes_rd_set(void *d, uintptr_t buf)
{
  struct eth_qos_rdes_rd *rdes = d;
  rdes->rdes0.buf1ap      = (uint32_t)buf;
#if UINTPTR_MAX == UINT64_MAX
  rdes->rdes1.buf1ap_high = (uint32_t)(buf >> 32);
#else
  rdes->rdes1.buf1ap_high = 0;
#endif
  rdes->rdes2.buf2ap = 0;

  *(volatile uint32_t *)&rdes->rdes3 = BIT(24) | BIT(30) | BIT(31);
}

static inline void eth_dwmac_tdes_rd_set_buffer(struct eth_qos_tdes_rd *t,
                                                uint8_t *buf, uint16_t len)
{
  t->tdes0.buf1ap = (uint32_t)(uintptr_t)buf;
#if UINTPTR_MAX == UINT64_MAX
  t->tdes1.buf2ap = (uint32_t)((uintptr_t)buf >> 32u);
#else
  t->tdes1.buf2ap = 0;
#endif
  t->tdes2.b1l = len;
  t->tdes2.b2l = 0;
}

static inline void eth_dwmac_tdes_rd_set(struct eth_qos_tdes_rd *t,
                                         bool fd, bool ld, uint16_t len,
                                         bool txcoe, bool ts)
{
  t->tdes2.ioc  = ld ? 1 : 0;
  t->tdes2.ttse = ts ? 1 : 0;

  uint32_t d3 = 0;
  d3 |= (len & 0x7fffu) | BIT(31);
  if (txcoe) d3 |= (3u << 16);
  if (ld)    d3 |= BIT(28);
  if (fd)    d3 |= BIT(29);

  *(volatile uint32_t *)&t->tdes3 = d3;
}

static inline bool eth_dwmac_tdes_own(void *d)
{
  return ((struct eth_qos_tdes_rd *)d)->tdes3.own;
}

static inline bool eth_dwmac_tdes_last(void *d)
{
  return ((struct eth_qos_tdes_wb *)d)->tdes3.ld;
}

static inline bool eth_dwmac_tdes_error(void *d)
{
  return ((struct eth_qos_tdes_wb *)d)->tdes3.derr;
}

static inline bool eth_dwmac_tdes_has_ts(void *d)
{
  return ((struct eth_qos_tdes_wb *)d)->tdes3.ld &&
         ((struct eth_qos_tdes_wb *)d)->tdes3.ttss;
}

/****************************************************************************
 * DMA RX channel helpers
 ****************************************************************************/

static inline void eth_dwmac_dma_rx_set_tail(const struct xgmac_driver_s *priv,
                                             uint8_t dma_ch)
{
  const struct eth_xgmac_config *cfg = priv->config;
  struct eth_dwmac_data *data = &priv->data->dwmac;
  uint8_t hw_ch = cfg->dwmac.dma_rx[dma_ch].nr;
  void *tail = &cfg->dwmac.dma_rx[dma_ch].descs[data->dma_rx[dma_ch].tail];

  xgmac_write((uint32_t)(uintptr_t)tail,
                priv->base + DMA_CHi_RXDESC_TAIL_POINTER(hw_ch));
}

static inline void eth_dwmac_dma_rx_init(const struct xgmac_driver_s *priv,
                                         uint8_t dma_ch)
{
  const struct eth_xgmac_config *cfg = priv->config;
  uint8_t hw_ch = cfg->dwmac.dma_rx[dma_ch].nr;
  const struct eth_dwmac_dma_ch_config *dcfg = &cfg->dwmac.dma_rx[dma_ch];
  uintptr_t dma = priv->base;

#if UINTPTR_MAX == UINT64_MAX
  xgmac_write((uint32_t)((uintptr_t)dcfg->descs >> 32),
                dma + DMA_CHi_RXDESC_LIST_HADDRESS(hw_ch));
#endif
  xgmac_write((uint32_t)(uintptr_t)dcfg->descs,
                dma + DMA_CHi_RXDESC_LIST_ADDRESS(hw_ch));
  xgmac_write(FIELD_PREP(DMA_CHi_RX_CONTROL_RBSZ, CONFIG_NET_ETH_PKTSIZE),
                dma + DMA_CHi_RX_CONTROL(hw_ch));
  xgmac_write(FIELD_PREP(DMA_CHi_RXDESC_RING_LENGTH_RDRL,
                dcfg->descs_count - 1),
                dma + DMA_CHi_RXDESC_RING_LENGTH(hw_ch));
  xgmac_write((uint32_t)(uintptr_t)dcfg->descs,
                dma + DMA_CHi_RXDESC_TAIL_POINTER(hw_ch));
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
                 DMA_CHi_INTERRUPT_ENABLE_NIE  |
                 DMA_CHi_INTERRUPT_ENABLE_FBEE |
                 DMA_CHi_INTERRUPT_ENABLE_CDEE |
                 DMA_CHi_INTERRUPT_ENABLE_AIE,
                 DMA_CHi_INTERRUPT_ENABLE_RIE |
                 DMA_CHi_INTERRUPT_ENABLE_NIE  |
                 DMA_CHi_INTERRUPT_ENABLE_FBEE |
                 DMA_CHi_INTERRUPT_ENABLE_CDEE |
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
                 DMA_CHi_INTERRUPT_ENABLE_NIE  |
                 DMA_CHi_INTERRUPT_ENABLE_FBEE |
                 DMA_CHi_INTERRUPT_ENABLE_CDEE |
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
                priv->base + DMA_CHi_TXDESC_TAIL_POINTER(hw_ch));
}

static inline void eth_dwmac_dma_tx_init(const struct xgmac_driver_s *priv,
                                         uint8_t dma_ch)
{
  const struct eth_xgmac_config *cfg = priv->config;
  uint8_t hw_ch = cfg->dwmac.dma_tx[dma_ch].nr;
  const struct eth_dwmac_dma_ch_config *dcfg = &cfg->dwmac.dma_tx[dma_ch];
  uintptr_t dma = priv->base;

#if UINTPTR_MAX == UINT64_MAX
  xgmac_write((uint32_t)((uintptr_t)dcfg->descs >> 32),
                dma + DMA_CHi_TXDESC_LIST_HADDRESS(hw_ch));
#endif
  xgmac_write((uint32_t)(uintptr_t)dcfg->descs,
                dma + DMA_CHi_TXDESC_LIST_ADDRESS(hw_ch));
  xgmac_write(FIELD_PREP(DMA_CHi_TXDESC_RING_LENGTH_TDRL,
                dcfg->descs_count - 1),
                dma + DMA_CHi_TXDESC_RING_LENGTH(hw_ch));
  xgmac_write((uint32_t)(uintptr_t)dcfg->descs,
                dma + DMA_CHi_TXDESC_TAIL_POINTER(hw_ch));
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
                 DMA_CHi_INTERRUPT_ENABLE_NIE  |
                 DMA_CHi_INTERRUPT_ENABLE_FBEE |
                 DMA_CHi_INTERRUPT_ENABLE_CDEE |
                 DMA_CHi_INTERRUPT_ENABLE_AIE,
                 DMA_CHi_INTERRUPT_ENABLE_TIE |
                 DMA_CHi_INTERRUPT_ENABLE_NIE  |
                 DMA_CHi_INTERRUPT_ENABLE_FBEE |
                 DMA_CHi_INTERRUPT_ENABLE_CDEE |
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
                 DMA_CHi_INTERRUPT_ENABLE_NIE  |
                 DMA_CHi_INTERRUPT_ENABLE_FBEE |
                 DMA_CHi_INTERRUPT_ENABLE_CDEE |
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
  uintptr_t reg = priv->base +
                  MTL_TXQi_OPERATION_MODE(cfg->dwmac.dma_tx[dma_ch].nr);

  xgmac_modreg(MTL_TXQi_OPERATION_MODE_TQS,
                 MTL_TXQi_OPERATION_MODE_TQS, reg);
}

static inline void eth_dwmac_dma_set_sysbus(const struct xgmac_driver_s *priv)
{
  const struct eth_xgmac_config *cfg = priv->config;

  xgmac_write((cfg->aal ? DMA_SYSBUS_MODE_AAL : 0) |
                (cfg->ubl  ? DMA_SYSBUS_MODE_FB  : 0) |
#if UINTPTR_MAX == UINT64_MAX
                DMA_SYSBUS_MODE_EAME |
#endif
                ((uint32_t)cfg->blen << 1),
                priv->base + DMA_SYSBUS_MODE);
}

static inline void eth_dwmac_mac_config(const struct xgmac_driver_s *priv)
{
  xgmac_write(MAC_CONFIGURATION_IPC |
                MAC_CONFIGURATION_CST |
                MAC_CONFIGURATION_ACS,
                priv->base + MAC_CONFIGURATION);

  xgmac_write(MAC_PACKET_FILTER_PM, priv->base + MAC_PACKET_FILTER);
  xgmac_write(0, priv->base + MAC_EXT_CONFIGURATION);
}

static inline void eth_dwmac_mac_set_link(const struct xgmac_driver_s *priv,
                                          uint32_t ss, bool fd)
{
  uint32_t val = xgmac_read(priv->base + MAC_CONFIGURATION);

  val &= ~(MAC_CONFIGURATION_DM | MAC_CONFIGURATION_FES | MAC_CONFIGURATION_PS);
  val |= (fd ? MAC_CONFIGURATION_DM : 0) |
         FIELD_PREP(MAC_CONFIGURATION_FES | MAC_CONFIGURATION_PS, ss);

  xgmac_write(val, priv->base + MAC_CONFIGURATION);
}

static inline bool eth_dwmac_mac_rx_state(const struct xgmac_driver_s *priv)
{
  return (xgmac_read(priv->base + MAC_CONFIGURATION) &
          MAC_CONFIGURATION_RE) != 0;
}

static inline void eth_dwmac_mac_rx_set_state(const struct xgmac_driver_s *priv,
                                              bool up)
{
  xgmac_modreg(up ? MAC_CONFIGURATION_RE : 0,
                 MAC_CONFIGURATION_RE,
                 priv->base + MAC_CONFIGURATION);
}

static inline bool eth_dwmac_mac_tx_state(const struct xgmac_driver_s *priv)
{
  return (xgmac_read(priv->base + MAC_CONFIGURATION) &
          MAC_CONFIGURATION_TE) != 0;
}

static inline void eth_dwmac_mac_tx_set_state(const struct xgmac_driver_s *priv,
                                              bool up)
{
  xgmac_modreg(up ? MAC_CONFIGURATION_TE : 0,
                 MAC_CONFIGURATION_TE,
                 priv->base + MAC_CONFIGURATION);
}

static inline void eth_dwmac_set_mac_addr(const struct xgmac_driver_s *priv,
                                          uint8_t nr, uint8_t *mac)
{
  uint32_t hi = ((uint32_t)mac[5] << 8) | mac[4];
  uint32_t lo = ((uint32_t)mac[3] << 24) | ((uint32_t)mac[2] << 16) |
                ((uint32_t)mac[1] << 8)  | mac[0];

  xgmac_write(hi | MAC_ADDRESSi_HIGH_AE, priv->base + MAC_ADDRESSi_HIGH(nr));
  xgmac_write(lo,                        priv->base + MAC_ADDRESSi_LOW(nr));
}

static inline void eth_dwmac_mtl_set_mode(const struct xgmac_driver_s *priv,
                                          uint8_t schalg)
{
  xgmac_write(FIELD_PREP(MTL_OPERATION_MODE_SCHALG, schalg),
                priv->base + MTL_OPERATION_MODE);
}

static inline void eth_dwmac_mtl_rxq_init(const struct xgmac_driver_s *priv,
                                          uint8_t queue)
{
  const struct eth_dwmac_rx_queue *qcfg = &priv->config->dwmac.mtl_rx[queue];

  xgmac_write(FIELD_PREP(MTL_RXQi_OPERATION_MODE_RQS, qcfg->size / 256) |
                (qcfg->sf ? MTL_RXQi_OPERATION_MODE_RSF
                           : FIELD_PREP(MTL_RXQi_OPERATION_MODE_RTC,
                                        qcfg->threshold)),
                priv->base + MTL_RXQi_OPERATION_MODE(qcfg->nr));
}

static inline void eth_dwmac_mtl_txq_init(const struct xgmac_driver_s *priv,
                                          uint8_t queue)
{
  const struct eth_dwmac_tx_queue *qcfg = &priv->config->dwmac.mtl_tx[queue];

  xgmac_write(FIELD_PREP(MTL_TXQi_OPERATION_MODE_TQS,
                            (qcfg->size + 255) / 256 - 1) |
                (qcfg->sf ? MTL_TXQi_OPERATION_MODE_TSF
                           : FIELD_PREP(MTL_TXQi_OPERATION_MODE_TTC,
                                        qcfg->threshold)) |
                MTL_TXQi_OPERATION_MODE_TXQEN,
                priv->base + MTL_TXQi_OPERATION_MODE(queue));
}

static inline void eth_dwmac_mtl_set_rxq_ctrl(const struct xgmac_driver_s *priv)
{
  const struct eth_xgmac_config *cfg = priv->config;
  uint8_t queue;
  uint32_t rxq_ctrl0 = 0;
  uint32_t rxq_ctrl1 = 0;
  uint32_t rxq_ctrl2[2] = {0, 0};
  uint32_t rxq_dma_map[2] = {0, 0};

  for (queue = 0; queue < cfg->dwmac.mtl_rx_queues; queue++)
    {
      const struct eth_dwmac_rx_queue *qcfg = &cfg->dwmac.mtl_rx[queue];

      rxq_ctrl0 |= ((qcfg->av_queue ? 0x1u : 0x2u) << (2 * qcfg->nr));
      rxq_ctrl2[qcfg->nr / 4] |= (qcfg->priority << (qcfg->nr % 4) * 8);

      if (qcfg->route_multi_broad)
        {
          rxq_ctrl1 |= MAC_RXQ_CTRL1_MCBCQEN |
                       FIELD_PREP(MAC_RXQ_CTRL1_MCBCQ, qcfg->nr);
        }

      rxq_dma_map[qcfg->nr / 4] |=
        (qcfg->dma_channel << (qcfg->nr % 4) * 8) |
        (qcfg->dynamic_dma_channel << ((qcfg->nr % 4) * 8 + 7));
    }

  xgmac_write(rxq_dma_map[0], priv->base + MTL_RXQ_DMA_MAP0);
  xgmac_write(rxq_dma_map[1], priv->base + MTL_RXQ_DMA_MAP1);
  xgmac_write(rxq_ctrl0,      priv->base + MAC_RXQ_CTRL0);
  xgmac_write(rxq_ctrl1,      priv->base + MAC_RXQ_CTRL1);
  xgmac_write(rxq_ctrl2[0],   priv->base + MAC_RXQ_CTRL2);
  xgmac_write(rxq_ctrl2[1],   priv->base + MAC_RXQ_CTRL3);
}

#endif /* QOS_PRIV_H */
