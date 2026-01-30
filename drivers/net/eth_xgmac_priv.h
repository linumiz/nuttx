/*
 * Copyright 2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ETH_XGMAC_PRIH_H
#define ETH_XGMAC_PRIH_H

#include "eth_dwmac.h"
#include "eth_xgmac_reg.h"
#include <nuttx/bits.h>

#define LSB_GET(value) ((value) & -(value))
#define FIELD_PREP(mask, value) (((value) * LSB_GET(mask)) & (mask))


struct eth_xgmac_config {
	const struct eth_dwmac_config dwmac;

	/* Register Address */
	/** MAC Base Address */
	uintptr_t dma_base;

	/** AXI Burst Length enable bit field */
	uint8_t blen;

	/** Address aligned beats */
	bool aal;
	/** Undefined burst length */
	bool ubl;
};

struct eth_xgmac_data {
	struct eth_dwmac_data dwmac;

#if defined(CONFIG_NET_PKT_TIMESTAMP)
	sys_slist_t ptp_pkts;
	struct k_sem ptp_pkts_count;
#endif
};

struct eth_xgmac_rdes_rd {
	struct eth_xgmac_rdes_rd0 {
		uint32_t buf1ap;
	} rdes0;
	struct eth_xgmac_rdes_rd1 {
		uint32_t buf1ap_high;
	} rdes1;
	struct eth_xgmac_rdes_rd2 {
		uint32_t buf2ap;
	} rdes2;
	struct eth_xgmac_rdes_rd3 {
		uint32_t buf2ap: 30;
		uint32_t ioc: 1;
		uint32_t own: 1;
	} rdes3;
};

struct eth_xgmac_rdes_wb {
	union {
		struct eth_xgmac_rdes_wb0_tnp {
			uint32_t ol2l3: 3;
			uint32_t: 5;
			uint32_t vnid: 24;
		} rdes0_tnp;
		struct eth_xgmac_rdes_wb0 {
			uint32_t ovt: 16;
			uint32_t ivt: 16;
		} rdes0;
	};
	struct eth_xgmac_rdes_wb1 {
		uint32_t rssh_frpli;
	} rdes1;
	struct eth_xgmac_rdes_wb2 {
		uint32_t avtcp: 1;
		uint32_t avtdp: 1;
		uint32_t hl: 8;
		uint32_t frspsm: 1;
		uint32_t tnp: 1;
		uint32_t eld: 1;
		uint32_t ios: 1;
		uint32_t rpng: 1;
		uint32_t vf: 1;
		uint32_t saf: 1;
		uint32_t hfdaf: 2;
		uint32_t madrm: 8;
		uint32_t l3fm: 1;
		uint32_t l4fm: 1;
		uint32_t l3l4fm: 3;
	} rdes2;
	struct eth_xgmac_rdes_wb3 {
		uint32_t pl: 14;
		uint32_t frpsl: 1;
		uint32_t es: 1;
		uint32_t et_lt: 4;
		uint32_t l3l4t: 4;
		uint32_t etm_ncp: 1;
		uint32_t isp: 1;
		uint32_t rsv: 1;
		uint32_t cda: 1;
		uint32_t ld: 1;
		uint32_t fd: 1;
		uint32_t ctxt: 1;
		uint32_t own: 1;
	} rdes3;
};

struct eth_xgmac_rdes_ctx {
	uint32_t rdes0;
	uint32_t rdes1;
	uint32_t rdes2;
	struct eth_xgmac_rdes_ctx3 {
		uint32_t pmt: 4;
		uint32_t tsa: 1;
		uint32_t: 1;
		uint32_t tsd: 1;
		uint32_t: 23;
		uint32_t ctxt: 1;
		uint32_t own: 1;
	} rdes3;
};

struct xgmac_driver_s {
  struct net_driver_s dev;
  uintptr_t base;  /* XGMAC register base */
  int       irq;    /* IRQ number */
  int       irq_rx;    /* IRQ number */
  FAR struct phy_device_s *phy;

  struct eth_xgmac_config *config;
  struct eth_xgmac_data *data;

  FAR uint8_t *rx_buf[CONFIG_ETH_DWMAC_NUM_RX_DMA_DESCRIPTORS];

  struct work_s rxwork;        /* RX deferred work */
  struct work_s pollwork;      /* For deferring poll work to the work queue */

  spinlock_t lock;                /* Protect TX/RX indices */

  bool ifup;
  bool txcoe;

  uint8_t blen;
  bool aal;
  bool ubl;
};

static inline bool eth_dwmac_rdes_own(struct eth_xgmac_rdes_rd *rdes)
{
	return rdes->rdes3.own == 1;
}

static inline bool eth_dwmac_rdes_first(struct eth_xgmac_rdes_wb *rdes)
{
	return rdes->rdes3.fd == 1;
}

static inline bool eth_dwmac_rdes_last(struct eth_xgmac_rdes_wb *rdes)
{
	return rdes->rdes3.ld == 1;
}

static inline bool eth_dwmac_rdes_context(struct eth_xgmac_rdes_wb *rdes)
{
	return rdes->rdes3.ctxt == 1;
}

static inline bool eth_dwmac_rdes_pkt_error(struct eth_xgmac_rdes_wb *rdes)
{
	return (rdes->rdes3.ld == 1) & (rdes->rdes3.es == 1);
}

static inline bool eth_dwmac_rdes_pkt_finish(struct eth_xgmac_rdes_wb *rdes)
{
	return rdes->rdes3.ctxt == 1 ||
	       rdes->rdes3.ld == 1;
}

static inline bool eth_dwmac_rdes_desc_error(struct eth_xgmac_rdes_wb *rdes)
{
	return rdes->rdes3.fd == 1 && rdes->rdes3.ld == 1 && rdes->rdes3.ctxt == 1;
}

static inline uint16_t eth_dwmac_rdes_packet_length(struct eth_xgmac_rdes_wb *rdes)
{
	return rdes->rdes3.pl;
}

static inline bool eth_dwmac_rdes_vlan_valid(struct eth_xgmac_rdes_wb *rdes)
{
	/* Only check c-tag and double tagged with inner c-tag*/
	return rdes->rdes3.et_lt == 9 || rdes->rdes3.et_lt == 10 || rdes->rdes3.et_lt == 12;
}

static inline uint16_t eth_dwmac_rdes_get_tci(struct eth_xgmac_rdes_wb *rdes)
{
	return rdes->rdes3.et_lt == 9 ? rdes->rdes0.ovt : rdes->rdes0.ivt;
}

#if defined(CONFIG_NET_PKT_TIMESTAMP)
static inline bool eth_dwmac_rdes_ts_valid(struct eth_xgmac_rdes_ctx *rdes)
{
	return rdes->rdes3.tsd == 0 && rdes->rdes3.tsa == 1;
}

static inline void eth_dwmac_rdes_get_ts(struct eth_xgmac_rdes_ctx *rdes, struct net_ptp_time *ts)
{
	ts->nanosecond = rdes->rdes0;
	ts->second = rdes->rdes1;
}
#endif

static inline void eth_dwmac_rdes_rd_set(struct eth_xgmac_rdes_rd *rdes, uintptr_t data)
{
	rdes->rdes0.buf1ap = (uint32_t)data;
#if UINTPTR_MAX == UINT64_MAX
	rdes->rdes1.buf2ap = (uint32_t)((uint32_t)data >> 32u);
#endif
	rdes->rdes3 = (struct eth_xgmac_rdes_rd3){.ioc = 1, .own = 1};
}

static inline void eth_dwmac_dma_rx_set_tail(const struct xgmac_driver_s *priv, uint8_t dma_ch)
{
	const struct eth_xgmac_config *cfg = priv->config;
	struct eth_dwmac_data *data = priv->data;
	void *const tail_ptr = &cfg->dwmac.dma_rx[dma_ch].descs[data->dma_rx[dma_ch].tail];

	xgmac_write((uint32_t)tail_ptr,
		 cfg->dma_base + DMA_CHi_RXDESC_TAIL_LPOINTER(cfg->dwmac.dma_tx[dma_ch].nr));
}

static inline void eth_dwmac_dma_rx_init(const struct xgmac_driver_s *priv, uint8_t dma_ch)
{
	const struct eth_xgmac_config *cfg = priv->config;

#if UINTPTR_MAX != UINT32_MAX
	xgmac_write((uint32_t)((uint32_t)(cfg->dwmac.dma_rx[dma_ch].descs) >> 32),
		 cfg->dma_base + DMA_CHi_RXDESC_LIST_HADDRESS(cfg->dwmac.dma_tx[dma_ch].nr));
#endif
	xgmac_write((uint32_t)cfg->dwmac.dma_rx[dma_ch].descs,
		 cfg->dma_base + DMA_CHi_RXDESC_LIST_LADDRESS(cfg->dwmac.dma_tx[dma_ch].nr));
	xgmac_write(FIELD_PREP(DMA_CHi_RX_CONTROL_RBSZ, CONFIG_IOB_BUFSIZE / 8),
		 cfg->dma_base + DMA_CHi_RX_CONTROL(cfg->dwmac.dma_tx[dma_ch].nr));
	xgmac_write(FIELD_PREP(DMA_CHi_RX_CONTROL2_RDRL, cfg->dwmac.dma_rx[dma_ch].descs_count - 1),
		 cfg->dma_base + DMA_CHi_RX_CONTROL2(cfg->dwmac.dma_tx[dma_ch].nr));
	xgmac_write((uint32_t)cfg->dwmac.dma_rx[dma_ch].descs,
		 cfg->dma_base + DMA_CHi_RXDESC_TAIL_LPOINTER(cfg->dwmac.dma_tx[dma_ch].nr));
}

static inline void eth_dwmac_dma_rx_start(const struct xgmac_driver_s *priv, uint8_t dma_ch)
{
	const struct eth_xgmac_config *cfg = priv->config;

	xgmac_modreg(DMA_CHi_RX_CONTROL_SR, DMA_CHi_RX_CONTROL_SR,
		     cfg->dma_base + DMA_CHi_RX_CONTROL(cfg->dwmac.dma_tx[dma_ch].nr));
}

static inline void eth_dwmac_dma_rx_stop(const struct xgmac_driver_s *priv, uint8_t dma_ch)
{
	const struct eth_xgmac_config *cfg = priv->config;

	xgmac_modreg(0, DMA_CHi_RX_CONTROL_SR,
		     cfg->dma_base + DMA_CHi_RX_CONTROL(cfg->dwmac.dma_tx[dma_ch].nr));
}

static inline void eth_dwmac_dma_rx_irq_enable(const struct xgmac_driver_s *priv, uint8_t dma_ch)
{
	const struct eth_xgmac_config *cfg = priv->config;

	xgmac_modreg(DMA_CHi_INTERRUPT_ENABLE_NIE | DMA_CHi_INTERRUPT_ENABLE_FBEE |
		     DMA_CHi_INTERRUPT_ENABLE_CDEE | DMA_CHi_INTERRUPT_ENABLE_AIE,
		     GENMASK(15, 12),
		     cfg->dma_base + DMA_CHi_INTERRUPT_ENABLE(cfg->dwmac.dma_tx[dma_ch].nr));
}

static inline void eth_dwmac_dma_rx_disable_irq(const struct xgmac_driver_s *priv, uint8_t dma_ch)
{
	const struct eth_xgmac_config *cfg = priv->config;

	xgmac_modreg(0,
		     DMA_CHi_INTERRUPT_ENABLE_NIE | DMA_CHi_INTERRUPT_ENABLE_FBEE |
		     DMA_CHi_INTERRUPT_ENABLE_CDEE | DMA_CHi_INTERRUPT_ENABLE_AIE,
		     cfg->dma_base + DMA_CHi_INTERRUPT_ENABLE(cfg->dwmac.dma_tx[dma_ch].nr));
}

static inline void eth_dmwac_dma_rx_clear_irq(const struct xgmac_driver_s *priv, uint8_t dma_ch)
{
	const struct eth_xgmac_config *cfg = priv->config;
	xgmac_write(DMA_CHi_STATUS_RI,
		    cfg->dma_base + DMA_CHi_STATUS(cfg->dwmac.dma_rx[dma_ch].nr));
}

struct eth_xgmac_tdes_rd {
	struct eth_xgmac_tdes_rd0 {
		uint32_t buf1ap;
	} tdes0;
	struct eth_xgmac_tdes_rd1 {
		uint32_t buf2ap;
	} tdes1;
	struct eth_xgmac_tdes_rd2 {
		uint32_t b1l: 14;
		uint32_t vtir: 2;
		uint32_t b2l: 14;
		uint32_t ttse: 1;
		uint32_t ioc: 1;
	} tdes2;
	struct eth_xgmac_tdes_rd3 {
		uint32_t fl: 15;
		uint32_t tpl: 1;
		uint32_t cic: 2;
		uint32_t tse: 1;
		uint32_t slotnum: 4;
		uint32_t saic: 3;
		uint32_t cpc: 2;
		uint32_t ld: 1;
		uint32_t fd: 1;
		uint32_t ctxt: 1;
		uint32_t own: 1;
	} tdes3;
};

struct eth_xgmac_tdes_wb {
	struct eth_xgmac_tdes_wb0 {
		uint32_t: 32;
	} tdes0;
	struct eth_xgmac_tdes_wb1 {
		uint32_t: 32;
	} tdes1;
	struct eth_xgmac_tdes_wb2 {
		uint32_t: 32;
	} tdes2;
	struct eth_xgmac_tdes_wb3 {
		uint32_t: 27;
		uint32_t derr: 1;
		uint32_t ld: 1;
		uint32_t fd: 1;
		uint32_t ctxt: 1;
		uint32_t own: 1;
	} tdes3;
};

static inline void eth_dwmac_tdes_rd_set(struct eth_xgmac_tdes_rd *tdes, bool fd, bool ld,
					 uint16_t len, bool txcoe, bool ts)
{
	tdes->tdes3 = (struct eth_xgmac_tdes_rd3){
		.fd = fd, .ld = ld, .own = 1, .fl = len, .cic = txcoe ? 0x3 : 0};
	tdes->tdes2.ioc = ld;
	tdes->tdes2.ttse = ts;
}

static inline void eth_dwmac_tdes_rd_set_buffer(struct eth_xgmac_tdes_rd *tdes, uint8_t *buf, uint16_t len)
{
	tdes->tdes2.b1l = len;
	tdes->tdes0.buf1ap = (uint32_t)(buf);
#if UINTPTR_MAX == UINT64_MAX
	tdes->tdes1.buf2ap = (uint32_t)(buf >> 32u);
#endif
}

static inline bool eth_dwmac_tdes_own(void *tdes)
{
	return ((struct eth_xgmac_tdes_rd *)tdes)->tdes3.own;
}

static inline bool eth_dwmac_tdes_last(void *tdes)
{
	return ((struct eth_xgmac_tdes_wb *)tdes)->tdes3.ld;
}

static inline bool eth_dwmac_tdes_error(void *tdes)
{
	return ((struct eth_xgmac_tdes_wb *)tdes)->tdes3.derr;
}

static inline void eth_dwmac_dma_set_sysbus(const struct xgmac_driver_s *priv)
{
	const struct eth_xgmac_config *cfg = priv->config;

	xgmac_write((UINTPTR_MAX != UINT32_MAX ? DMA_SYSBUS_MODE_EAME : 0) |
			    (cfg->aal ? DMA_SYSBUS_MODE_AAL : 0) |
			    (cfg->ubl ? DMA_SYSBUS_MODE_UBL : 0) | cfg->blen << 1,
		    cfg->dma_base + DMA_SYSBUS_MODE);
}

static inline void eth_dwmac_dma_ts_handle(const struct xgmac_driver_s *priv, uint8_t dma_ch)
{
}

static inline void eth_dwmac_dma_tx_set_tail(const struct xgmac_driver_s *priv, uint8_t dma_ch)
{
	const struct eth_xgmac_config *cfg = priv->config;
	struct eth_dwmac_data *data = priv->data;
	void *const tail_ptr = &cfg->dwmac.dma_tx[dma_ch].descs[data->dma_tx[dma_ch].tail];

	xgmac_write((uint32_t)tail_ptr,
		 cfg->dma_base + DMA_CHi_TXDESC_TAIL_LPOINTER(cfg->dwmac.dma_tx[dma_ch].nr));
}

static inline void eth_dwmac_dma_tx_queue_flush(const struct xgmac_driver_s *priv, uint8_t dma_ch)
{
	const struct eth_xgmac_config *cfg = priv->config;
	xgmac_modreg(MTL_TXQX_OPERATION_MODE_TQS, MTL_TXQX_OPERATION_MODE_TQS,
		     priv->base + MTL_TXQX_OPERATION_MODE(cfg->dwmac.dma_tx[dma_ch].nr));
}

static inline void eth_dwmac_dma_tx_init(const struct xgmac_driver_s *priv, uint8_t dma_ch)
{
	const struct eth_xgmac_config *cfg = priv->config;

#if UINTPTR_MAX != UINT32_MAX
	xgmac_write((uint32_t)((uint32_t)(cfg->dwmac.dma_tx[dma_ch].descs) >> 32),
		    cfg->dma_base + DMA_CHi_TXDESC_LIST_HADDRESS(cfg->dwmac.dma_tx[dma_ch].nr));
#endif
	xgmac_write((uint32_t)cfg->dwmac.dma_tx[dma_ch].descs,
		    cfg->dma_base + DMA_CHi_TXDESC_LIST_LADDRESS(cfg->dwmac.dma_tx[dma_ch].nr));
	xgmac_write(FIELD_PREP(DMA_CHi_TX_CONTROL2_TDRL, cfg->dwmac.dma_tx[dma_ch].descs_count - 1),
		    cfg->dma_base + DMA_CHi_TX_CONTROL2(cfg->dwmac.dma_tx[dma_ch].nr));
	xgmac_write((uint32_t)cfg->dwmac.dma_tx[dma_ch].descs,
		    cfg->dma_base + DMA_CHi_TXDESC_TAIL_LPOINTER(cfg->dwmac.dma_tx[dma_ch].nr));
}

static inline void eth_dwmac_dma_tx_start(const struct xgmac_driver_s *priv, uint8_t dma_ch)
{
	const struct eth_xgmac_config *cfg = priv->config;

	xgmac_modreg(DMA_CHi_TX_CONTROL_ST | BIT(16), DMA_CHi_TX_CONTROL_ST | BIT(16),
		     cfg->dma_base + DMA_CHi_TX_CONTROL(cfg->dwmac.dma_tx[dma_ch].nr));
}

static inline void eth_dwmac_dma_tx_stop(const struct xgmac_driver_s *priv, uint8_t dma_ch)
{
	const struct eth_xgmac_config *cfg = priv->config;
	xgmac_modreg(0, DMA_CHi_TX_CONTROL_ST,
		     cfg->dma_base + DMA_CHi_TX_CONTROL(cfg->dwmac.dma_tx[dma_ch].nr));
}

static inline void eth_dwmac_dma_tx_irq_enable(const struct xgmac_driver_s *priv, uint8_t dma_ch)
{
	const struct eth_xgmac_config *cfg = priv->config;

	xgmac_modreg(DMA_CHi_INTERRUPT_ENABLE_NIE | DMA_CHi_INTERRUPT_ENABLE_FBEE |
		     DMA_CHi_INTERRUPT_ENABLE_CDEE | DMA_CHi_INTERRUPT_ENABLE_AIE,
		     GENMASK(15, 12),
		     cfg->dma_base + DMA_CHi_INTERRUPT_ENABLE(cfg->dwmac.dma_tx[dma_ch].nr));
}

static inline void eth_dwmac_dma_tx_disable_irq(const struct xgmac_driver_s *priv, uint8_t dma_ch)
{
	const struct eth_xgmac_config *cfg = priv->config;

	xgmac_modreg(0,
		     DMA_CHi_INTERRUPT_ENABLE_NIE | DMA_CHi_INTERRUPT_ENABLE_FBEE |
		     DMA_CHi_INTERRUPT_ENABLE_CDEE | DMA_CHi_INTERRUPT_ENABLE_AIE,
		     cfg->dma_base + DMA_CHi_INTERRUPT_ENABLE(cfg->dwmac.dma_tx[dma_ch].nr));
}

static inline void eth_dmwac_dma_tx_clear_irq(const struct xgmac_driver_s *priv, uint8_t dma_ch)
{
	const struct eth_xgmac_config *cfg = priv->config;
	xgmac_write(DMA_CHi_STATUS_TI | DMA_CHi_STATUS_TBU,
		    cfg->dma_base + DMA_CHi_STATUS(cfg->dwmac.dma_tx[dma_ch].nr));
}

static inline void eth_dwmac_mac_config(const struct xgmac_driver_s *priv)
{
	const struct eth_xgmac_config *cfg = priv->config;

	xgmac_write(MAC_RX_CONFIGURATION_IPC | MAC_RX_CONFIGURATION_CST | MAC_RX_CONFIGURATION_ACS |
			    (cfg->dwmac.loopback ? MAC_RX_CONFIGURATION_LM : 0),
		    priv->base + MAC_RX_CONFIGURATION);
	xgmac_write(MAC_PACKET_FILTER_PM, priv->base + MAC_PACKET_FILTER);
	xgmac_write((cfg->dwmac.da_duplication ? MAC_EXTENDED_CONFIGURATION_DDS : 0),
		    priv->base + MAC_EXTENDED_CONFIGURATION);

	/* TODO: Handle VLAN */
	/* TODO: Handle L3, L4 */
#if 0
	/* Insert default filter all vlan filter */
	if (p->feature3 & MAC_HW_FEATURE3_NRVF) {
		dwmac_vlan_write_ext_filter(dev, 0, true, 0x3ff);
	} else {
		REG_WRITE(MAC_VLAN_TAG, MAC_VLAN_TAG_CTRL_EVLRXS |
						FIELD_PREP(MAC_VLAN_TAG_CTRL_EVLS, 3) |
						FIELD_PREP(MAC_VLAN_TAG_CTRL_VL, 0x3ff));
	}
#endif
}

static inline void eth_dwmac_mac_set_link(const struct xgmac_driver_s *priv, uint32_t ss, bool fd)
{
	xgmac_modreg(FIELD_PREP(MAC_TX_CONFIGURATION_SS, ss), MAC_TX_CONFIGURATION_SS,
		     priv->base + MAC_TX_CONFIGURATION);
	xgmac_modreg((fd ? 0 : MAC_EXTENDED_CONFIGURATION_HD), MAC_EXTENDED_CONFIGURATION_HD,
		     priv->base + MAC_EXTENDED_CONFIGURATION);
}

static inline bool eth_dwmac_mac_rx_state(const struct xgmac_driver_s *priv)
{
	return (xgmac_read(priv->base + MAC_RX_CONFIGURATION) & MAC_RX_CONFIGURATION_RE) != 0;
}

static inline void eth_dwmac_mac_rx_set_state(const struct xgmac_driver_s *priv, bool up)
{
	xgmac_modreg((up ? MAC_RX_CONFIGURATION_RE : 0), MAC_RX_CONFIGURATION_RE,
		     priv->base + MAC_RX_CONFIGURATION);
}

static inline bool eth_dwmac_mac_tx_state(const struct xgmac_driver_s *priv)
{
	return (xgmac_read(priv->base + MAC_TX_CONFIGURATION) & MAC_TX_CONFIGURATION_TE) != 0;
}

static inline void eth_dwmac_mac_tx_set_state(const struct xgmac_driver_s *priv, bool up)
{
	xgmac_modreg((up ? MAC_TX_CONFIGURATION_TE : 0), MAC_TX_CONFIGURATION_TE,
		     priv->base + MAC_TX_CONFIGURATION);
}
#if 0
static inline uint8_t eth_dwmac_mac_get_speed(enum phy_link_speed link_speed)
{
	switch (link_speed) {
	case LINK_HALF_10BASE_T:
	case LINK_FULL_10BASE_T:
		return 0x7;
	case LINK_HALF_100BASE_T:
	case LINK_FULL_100BASE_T:
		return 0x4;
	case LINK_HALF_1000BASE_T:
	case LINK_FULL_1000BASE_T:
		return 0x3;
	case LINK_FULL_2500BASE_T:
		return 0x2;
	case LINK_FULL_5000BASE_T:
		return 0x5;
	}
}

static inline int eth_dwmac_mac_reserve_tx_ts(const struct xgmac_driver_s *priv, struct net_pkt *pkt,
					      k_timeout_t timeout)
{
	struct eth_xgmac_data *data = priv->data;

#if defined(CONFIG_NET_PKT_TIMESTAMP)
	if (k_sem_take(&data->ptp_pkts_count, timeout) != 0) {
		return -ETIMEDOUT;
	}

	net_pkt_slist_put(&data->ptp_pkts, pkt);
#endif
	return 0;
}

static inline void eth_dwmac_mac_wait_idle(const struct xgmac_driver_s *priv)
{
	const struct eth_xgmac_config *cfg = priv->config;
	WAIT_FOR(xgmac_read(cfg->dma_base + MAC_DEBUG) == 0, 1000, k_busy_wait(100));
}
#endif

static inline void eth_dwmac_mtl_set_mode(const struct xgmac_driver_s *priv, uint8_t schalg)
{
	const struct eth_xgmac_config *cfg = priv->config;
	xgmac_write(FIELD_PREP(MTL_OPERATION_MODE_ETSALG, schalg), priv->base + MTL_OPERATION_MODE);
}

static inline void eth_dwmac_mtl_rxq_init(const struct xgmac_driver_s *priv, uint8_t queue)
{
	const struct eth_xgmac_config *cfg = priv->config;
	const struct eth_dwmac_rx_queue *qcfg = &cfg->dwmac.mtl_rx[queue];

	xgmac_write(FIELD_PREP(MTL_RXQX_OPERATION_MODE_RQS, qcfg->size / 256) |
			    (qcfg->sf ? MTL_RXQX_OPERATION_MODE_RSF
				      : (qcfg->threshold << MTL_RXQX_OPERATION_MODE_RTC)),
		    priv->base + MTL_RXQX_OPERATION_MODE(qcfg->nr));
}

static inline void eth_dwmac_mtl_set_rxq_ctrl(const struct xgmac_driver_s *priv)
{
	const struct eth_xgmac_config *cfg = priv->config;
	uint8_t queue;
	uint32_t rxq_ctrl0 = 0;
	uint32_t rxq_ctrl1 = 0;
	uint32_t rxq_ctrl2[2] = {0};
	uint32_t rxq_dma_map[2] = {0};
	for (queue = 0; queue < cfg->dwmac.mtl_rx_queues; queue++) {
		const struct eth_dwmac_rx_queue *qcfg = &cfg->dwmac.mtl_rx[queue];
		rxq_ctrl0 |= ((qcfg->av_queue ? 0x1 : 0x2) << (2 * qcfg->nr));
		rxq_ctrl2[qcfg->nr / 4] |= (qcfg->priority << (qcfg->nr % 4) * 8);
		if (qcfg->route_multi_broad) {
			rxq_ctrl1 |=
				MAC_RXQ_CTRL1_MCBCQEN | FIELD_PREP(MAC_RXQ_CTRL1_MCBCQ, qcfg->nr);
		}
		rxq_dma_map[qcfg->nr / 4] |=
			(qcfg->dma_channel << (qcfg->nr % 4) * 8) |
			(qcfg->dynamic_dma_channel << ((qcfg->nr % 4) * 8 + 7));
	}
	xgmac_write(rxq_dma_map[0], priv->base + MTL_RXQ_DMA_MAP0);
	xgmac_write(rxq_dma_map[1], priv->base + MTL_RXQ_DMA_MAP1);
	xgmac_write(rxq_ctrl0, priv->base + MAC_RXQ_CTRL0);
	xgmac_write(rxq_ctrl1, priv->base + MAC_RXQ_CTRL1);
	xgmac_write(rxq_ctrl2[0], priv->base + MAC_RXQ_CTRL2);
	xgmac_write(rxq_ctrl2[1], priv->base + MAC_RXQ_CTRL3);
}

static inline void eth_dwmac_mtl_txq_init(const struct xgmac_driver_s *priv, uint8_t queue)
{
	const struct eth_xgmac_config *cfg = priv->config;
	const struct eth_dwmac_tx_queue *qcfg = &cfg->dwmac.mtl_tx[queue];

	xgmac_write(FIELD_PREP(MTL_TXQX_OPERATION_MODE_TQS, 7) | /*(qcfg->size + 255) / 256)  | */
			    (qcfg->sf ? MTL_TXQX_OPERATION_MODE_TSF
				      : FIELD_PREP(MTL_TXQX_OPERATION_MODE_TTC, qcfg->threshold)) |
			    /*MTL_TXQX_OPERATION_MODE_TXQEN*/ BIT(3),
		    priv->base + MTL_TXQX_OPERATION_MODE(queue));
}

static inline void eth_dwmac_set_mac_addr(const struct xgmac_driver_s *priv, uint8_t nr, uint8_t *mac_addr)
{
	uint32_t reg_val;

	reg_val = (mac_addr[5] << 8) | mac_addr[4];
	xgmac_write(reg_val | MAC_ADDRESSX_HIGH_AE, priv->base + MAC_ADDRESSX_HIGH(nr));
	reg_val = (mac_addr[3] << 24) | (mac_addr[2] << 16) | (mac_addr[1] << 8) | mac_addr[0];
	xgmac_write(reg_val, priv->base + MAC_ADDRESSX_LOW(nr));
}

static inline uint16_t eth_dwmac_ts_get_high(const struct xgmac_driver_s *priv)
{
	return xgmac_read(priv->base + MAC_SYSTEM_TIME_HIGHER_WORD_SECONDS);
}

#endif
