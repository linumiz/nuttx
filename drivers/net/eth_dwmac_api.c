/*
 * Copyright 2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include "eth_dwmac.h"

#include "eth_xgmac_priv.h"

void eth_dwmac_dma_init(struct xgmac_driver_s *priv)
{
	const struct eth_dwmac_config *cfg = &priv->config->dwmac;
	uint32_t irq_enable[8] = {};
	uint8_t ch;

	if (!cfg->slave_mode) {
		eth_dwmac_dma_set_sysbus(priv);
	}

	for (ch = 0; ch < cfg->dma_tx_channel; ch++) {
		eth_dwmac_dma_tx_init(priv, ch);
		eth_dwmac_dma_tx_irq_enable(priv, ch);
	}

	for (ch = 0; ch < cfg->dma_rx_channel; ch++) {
		eth_dwmac_dma_rx_init(priv, ch);
		eth_dwmac_dma_rx_irq_enable(priv, ch);
	}
}

void eth_dwmac_dma_rx_data_init(struct xgmac_driver_s *priv, uint8_t dma_ch, uint8_t *buf)
{
	const struct eth_dwmac_config *cfg = &priv->config->dwmac;
	struct eth_dwmac_data *data = &priv->data->dwmac;
	const struct eth_dwmac_dma_ch_config *dma_cfg = &cfg->dma_rx[dma_ch];
	struct eth_dwmac_dma_rx_ch_data *dma_data = &data->dma_rx[dma_ch];

#if CONFIG_MMU
	dma_data->descs_phys = tx_descs_phys + sizeof(struct dwmac_dma_desc) * dma_ch * NB_TX_DESCS;
#endif
	dma_data->head = 0;
	dma_data->tail = 0;
	dma_data->data = buf; //TODO data pool should not bind to buf
	dma_data->length = 0;
	nxsem_init(&dma_data->desc_used, dma_cfg->descs_count - 1, dma_cfg->descs_count - 1);
//	sys_slist_init(&dma_data->frags);
}

void eth_dwmac_dma_rx_fill_desc(struct xgmac_driver_s *priv, uint8_t dma_ch)
{
	const struct eth_dwmac_config *cfg = priv->config;
	struct eth_dwmac_data *data = priv->data;
	const struct eth_dwmac_dma_ch_config *dma_cfg = &cfg->dma_rx[dma_ch];
	struct eth_dwmac_dma_rx_ch_data *dma_data = &data->dma_rx[dma_ch];
	const uint32_t desc_free;
	uint32_t descs;
	int ret;

	nxsem_get_value(&dma_data->desc_used, &desc_free);
	for (descs = 0; descs < desc_free; descs++) {
		ret = nxsem_trywait(&dma_data->desc_used);
		if (ret) {
			break;
		}

		eth_dwmac_rdes_rd_set((void *)&dma_cfg->descs[dma_data->tail], dma_data->data);

		dma_data->tail = (dma_data->tail + 1) % (dma_cfg->descs_count + 1);
	}

	/* Update tail pointer for new descriptors */
	if (descs != 0) {
		eth_dwmac_dma_rx_set_tail(priv, dma_ch);
	}
}

#if 0
static void eth_dwmac_dma_rx_process(struct xgmac_driver_s *priv, uint8_t dma_ch)
{
	const struct eth_dwmac_config *cfg = priv->config;
	struct eth_dwmac_data *data = priv->data;
	const struct eth_dwmac_dma_ch_config *dma_cfg = &cfg->dma_rx[dma_ch];
	struct eth_dwmac_dma_rx_ch_data *dma_data = &data->dma_rx[dma_ch];
	struct net_pkt *pkt = dma_data->packet;
	struct net_buf *frag;
	void *desc;
	int ret;

	while (dma_data->head != dma_data->tail) {
		desc = &dma_cfg->descs[dma_data->head];
		eth_dmwac_dma_rx_clear_irq(priv, dma_ch);
		/* stop here if hardware still owns it */
		if (eth_dwmac_rdes_own(desc)) {
			break;
		}

		/* a packet's first descriptor: */
		if (eth_dwmac_rdes_first(desc)) {
			__ASSERT(pkt == NULL, "Packet should be received before new packet");
			pkt = net_pkt_rx_alloc_on_iface(data->iface, K_NO_WAIT);
			if (!pkt) {
				LOG_DBG("%s: failed to alloc rx packet", priv->name);
				eth_stats_update_errors_rx(data->iface);
			}
		}

		/* retrieve current fragment */
		frag = net_buf_slist_get(&dma_data->frags);

		/* Check for valid packet */
		if (!pkt) {
			net_buf_unref(frag);
			goto next;
		}

		/* Check for descriptor error */
		if (eth_dwmac_rdes_desc_error(desc)) {
			net_buf_unref(frag);
			if (pkt) {
				net_pkt_unref(pkt);
				pkt = NULL;
			}
			LOG_ERR("%s: Descriptor Definition Error", priv->name);
			goto next;
		}

		/* Handle buffer fragment */
		if (eth_dwmac_rdes_context(desc)) {
			net_buf_unref(frag);
		} else {
			net_buf_add(frag, !eth_dwmac_rdes_last(desc)
						  ? CONFIG_NET_BUF_DATA_SIZE
						  : eth_dwmac_rdes_packet_length(desc) -
							    net_pkt_get_len(pkt));
			net_pkt_frag_add(pkt, frag);
			sys_cache_data_invd_range(frag->data, frag->size);
		}

#if defined(CONFIG_NET_PKT_TIMESTAMP)
		/* Handle timestamp from context descriptor */
		if (eth_dwmac_rdes_context(desc) && eth_dwmac_rdes_ts_valid(desc)) {
			struct net_ptp_time ts;
			eth_dwmac_rdes_get_ts(desc, &ts);
			ts._sec.high = eth_dwmac_ts_get_high(priv);
			net_pkt_set_timestamp(pkt, &ts);
		}
#endif

#if defined(CONFIG_NET_VLAN)
		if (eth_dwmac_rdes_last(desc) && eth_dwmac_rdes_vlan_valid(desc)) {
			net_pkt_set_vlan_tci(pkt, eth_dwmac_rdes_get_tci(desc));
			net_pkt_set_priority(pkt, net_vlan2priority(net_pkt_vlan_priority(pkt)));
		}
#endif

		/* Packet reception error */
		if (eth_dwmac_rdes_pkt_error(desc)) {
			eth_stats_update_errors_rx(p->iface);
			net_pkt_unref(pkt);
			pkt = NULL;
			goto next;
		}

		if (eth_dwmac_rdes_pkt_finish(desc)) {
			ret = net_recv_data(data->iface, pkt);
			if ((ret < 0)) {
				LOG_ERR("%s: Failed to receive packet: %d", priv->name, ret);
				net_pkt_unref(pkt);
			}
			pkt = NULL;
			goto next;
		}

next:
		k_sem_give(&dma_data->desc_used);
		dma_data->head = (dma_data->head + 1) % dma_cfg->descs_count;
	}

	/* Store partly processed packet*/
	dma_data->packet = pkt;
}

static void eth_dwmac_dma_tx_process(struct xgmac_driver_s *priv, uint8_t dma_ch)
{
	const struct eth_dwmac_config *cfg = &priv->config->dwmac;
	struct eth_dwmac_data *data = &priv->data->dwmac;
	struct net_buf *frag;
	const struct eth_dwmac_dma_ch_config *dma_cfg = &cfg->dma_tx[dma_ch];
	struct eth_dwmac_dma_tx_ch_data *dma_data = &data->dma_tx[dma_ch];

	while (dma_data->head != dma_data->tail) {
		eth_dmwac_dma_tx_clear_irq(priv, dma_ch);
		/* stop here if hardware still owns it */
		if (eth_dwmac_tdes_own((void *)&dma_cfg->descs[dma_data->head])) {
			break;
		}

		/* We are done using the buf */
		frag = net_buf_slist_get(&dma_data->frags);
		net_buf_unref(frag);

		/* last packet descriptor: */
		if (eth_dwmac_tdes_last((void *)&dma_cfg->descs[dma_data->head])) {
			/* log any errors */
			if (eth_dwmac_tdes_error((void *)&dma_cfg->descs[dma_data->head])) {
				LOG_ERR("%s: dma error while transmission", priv->name);
				eth_stats_update_errors_tx(data->iface);
			}
#if IS_ENABLED(CONFIG_NET_PKT_TIMESTAMP)
			else {
				eth_dwmac_dma_ts_handle(priv, dma_ch);
			}
#endif
		}

		dma_data->head = (dma_data->head + 1) % dma_cfg->descs_count;
		k_sem_give(&dma_data->desc_used);
	}
}
#endif

void eth_dwmac_dma_tx_data_init(struct xgmac_driver_s *priv, uint8_t dma_ch)
{
	const struct eth_dwmac_config *cfg = &priv->config->dwmac;
	struct eth_dwmac_data *data = &priv->data->dwmac;
	const struct eth_dwmac_dma_ch_config *dma_cfg = &cfg->dma_tx[dma_ch];
	struct eth_dwmac_dma_tx_ch_data *dma_data = &data->dma_tx[dma_ch];
#if CONFIG_MMU
	dma_data->descs_phys = tx_descs_phys + sizeof(struct dwmac_dma_desc) * dma_ch * NB_TX_DESCS;
#endif
	dma_data->head = 0;
	dma_data->tail = 0;
	nxmutex_init(&dma_data->lock);
	nxsem_init(&dma_data->desc_used, dma_cfg->descs_count - 1,
		   dma_cfg->descs_count - 1);
//	sys_slist_init(&dma_data->frags);
}

void eth_dwmac_mac_init(struct xgmac_driver_s *priv)
{
	eth_dwmac_mac_config(priv);
}

void eth_dwmac_mtl_init(struct xgmac_driver_s *priv)
{
	const struct eth_dwmac_config *cfg = &priv->config->dwmac;
	uint32_t queue;

	for (queue = 0; queue < cfg->mtl_rx_queues; queue++) {
		eth_dwmac_mtl_rxq_init(priv, queue);
	}

	for (queue = 0; queue < cfg->mtl_tx_queues; queue++) {
		eth_dwmac_mtl_txq_init(priv, queue);
	}

	eth_dwmac_mtl_set_rxq_ctrl(priv);

	/* set up MTL */
	eth_dwmac_mtl_set_mode(priv, 0);
}

void eth_dwmac_set_link_state(struct xgmac_driver_s *priv, bool is_link_up)
{
	const struct eth_dwmac_config *cfg = priv->config;
	uint8_t dma_ch;

	/* Lock against unwanted changes */
        irqstate_t flags = enter_critical_section();

	/* Check if link state changed already. */
	if (eth_dwmac_mac_rx_state(priv) == is_link_up ||
	    eth_dwmac_mac_tx_state(priv) == is_link_up) {
                leave_critical_section(flags);
		return;
	}

	if (is_link_up) {
		/*  Program new phy state*/
		eth_dwmac_mac_set_link(priv, BIT(2), true); //TODO fix link speed 100 base
	}

	/* Operate TX DMA if clock doesn't stop */
	if (!cfg->clocks_stop) {
		if (is_link_up) {
			for (dma_ch = 0; dma_ch < cfg->dma_tx_channel; dma_ch++) {
				eth_dwmac_dma_tx_start(priv, dma_ch);
			}
		} else {
			/* Disable DMA*/
			for (dma_ch = 0; dma_ch < cfg->dma_tx_channel; dma_ch++) {
				eth_dwmac_dma_tx_stop(priv, dma_ch);
			}
			/*  Wait for queue emtpy or flush */
			for (dma_ch = 0; dma_ch < cfg->dma_tx_channel; dma_ch++) {
				eth_dwmac_dma_tx_queue_flush(priv, dma_ch);
			}
		}
	}
	/* Disable or enable receiver*/
	eth_dwmac_mac_rx_set_state(priv, is_link_up);
	/* Disable or enable transmitter and set mac configuration */
	eth_dwmac_mac_tx_set_state(priv, is_link_up);
        leave_critical_section(flags);
}

#if 0
static void eth_dwmac_link_state_changed(const struct privice *phy_priv,
					 struct phy_link_state *link_state, void *user_data)
{
	struct xgmac_driver_s *priv = user_data;
	const struct eth_dwmac_config *cfg = priv->config;
	struct eth_dwmac_data *data = priv->data;
	uint8_t dma_ch;

	if (link_state->is_up) {
		if (cfg->clocks_stop) {
			eth_dwmac_mac_wait_idle(priv);
		} else {
		}
		eth_dwmac_set_link_state(priv, link_state);
		net_if_carrier_on(data->iface);
		LOG_DBG("%s: Link up", priv->name);
	} else {
		eth_dwmac_set_link_state(priv, link_state);
		net_if_carrier_off(data->iface);
		LOG_DBG("%s: Link down", priv->name);
	}
}

void eth_dwmac_iface_init(struct net_if *iface)
{
	struct xgmac_driver_s *priv = net_if_get_privice(iface);
	const struct eth_dwmac_config *cfg = priv->config;
	struct eth_dwmac_data *data = priv->data;

	data->iface = iface;
	data->started = false;

	net_if_set_link_addr(iface, data->mac_addr, sizeof(data->mac_addr), NET_LINK_ETHERNET);
	if (!cfg->slave_mode) {
		struct eth_dwmac_mac_filter filter = {.addr = {0},
						      cfg->da_duplication ? (1 << cfg->dma_rx->nr)
									  : cfg->dma_rx->nr,
						      0,
						      true,
						      false,
						      true};
		memcpy(filter.addr, data->mac_addr, 6);
		eth_dwmac_mac_set_filter(priv, 0, &filter);
	}

	ethernet_init(iface);

	if (cfg->phy) {
		net_if_carrier_off(iface);
		phy_link_callback_set(cfg->phy, &eth_dwmac_link_state_changed, (void *)priv);
	} else {
		net_if_carrier_on(iface);
	}
}

int eth_dwmac_start(struct xgmac_driver_s *priv)
{
	const struct eth_dwmac_config *cfg = priv->config;
	struct eth_dwmac_data *data = priv->data;
	struct phy_link_state state;
	uint8_t dma_ch;

	/* Check if the privice is initialized successfully. */
	if (!privice_is_ready(priv)) {
		return -EIO;
	}

	/* Start RX DMA operation this is always running. */
	for (dma_ch = 0; dma_ch < cfg->dma_rx_channel; dma_ch++) {
		eth_dwmac_dma_rx_fill_desc(priv, dma_ch);
		eth_dwmac_dma_rx_start(priv, dma_ch);
	}
	/* Start Tx operation only if clock stops or slave mode */
	if (cfg->slave_mode || cfg->clocks_stop) {
		for (dma_ch = 0; dma_ch < cfg->dma_tx_channel; dma_ch++) {
			eth_dwmac_dma_tx_start(priv, dma_ch);
		}
	}

	data->started = true;

	/* Check for link status with phy */
	if (cfg->phy) {
		phy_get_link_state(cfg->phy, &state);

		/* If link is up, start initial operation. Afterwards operation is
		 * started and stoped through the link state callback according
		 * to clock requirements. */
		if (state.is_up) {
			net_if_carrier_on(data->iface);
			eth_dwmac_set_link_state(priv, &state);
		}
	}

	LOG_DBG("%s: privice started", priv->name);

	return 0;
}

static int eth_dwmac_send(struct xgmac_driver_s *priv, struct net_pkt *pkt)
{
	const struct eth_dwmac_config *cfg = priv->config;
	struct eth_dwmac_data *data = priv->data;
	uint8_t dma_ch = 0;
	const struct eth_dwmac_dma_ch_config *dma_cfg = &cfg->dma_tx[dma_ch];
	struct eth_dwmac_dma_tx_ch_data *dma_data = &data->dma_tx[dma_ch];
	struct net_buf *prev_frag;
	struct net_buf *frag;
	const size_t pkt_len = net_pkt_get_len(pkt);
	void *tdes;
	size_t frag_count = 0;

	if (!pkt || !pkt->frags) {
		LOG_ERR("%s: cannot TX, invalid argument", priv->name);
		return -EINVAL;
	}

	if (pkt_len == 0) {
		LOG_ERR("%s cannot TX, zero packet length", priv->name);
		// UPDATE_ETH_STATS_TX_ERROR_PKT_CNT(priv_data, 1u);
		return -EINVAL;
	}

	/* Lock DMA Channel */
	k_mutex_lock(&dma_data->lock, K_FOREVER);

#if IS_ENABLED(CONFIG_NET_PKT_TIMESTAMP)
	if (net_pkt_is_tx_timestamping(pkt) || net_pkt_is_ptp(pkt)) {
		/* Reserve tx timestamp if needed */
		if (eth_dwmac_mac_reserve_tx_ts(priv, dma_ch, pkt, K_MSEC(1000)) != 0) {
			k_mutex_unlock(&dma_data->lock);
			return -ETIMEDOUT;
		}
		net_pkt_ref(pkt);
	}
#endif

	/* Map packet fragments */
	frag = pkt->frags;
	prev_frag = SYS_SLIST_CONTAINER(sys_slist_peek_tail(&dma_data->frags), prev_frag, node);

	do {
		/* Reserve a free descriptor  */
		if (k_sem_take(&dma_data->desc_used, K_MSEC(1000)) != 0) {
			LOG_ERR("%s: DMA CH%d: Timeout waiting for free TX", priv->name, dma_ch);
			goto abort_frag;
		}

		tdes = &cfg->dma_tx[dma_ch].descs[dma_data->tail];
		eth_dwmac_tdes_rd_set_frag(tdes, frag);
		eth_dwmac_tdes_rd_set(
			tdes, frag_count == 0, !frag->frags, pkt_len, data->txcoe_available,
			COND_CODE_1(IS_ENABLED(CONFIG_NET_PKT_TIMESTAMP), (pkt->tx_timestamping || pkt->ptp_pkt), (0)));
		sys_cache_data_flush_range(frag->data, frag->len);
		frag_count++;
		/* Reference frags for DMA use */
		net_buf_slist_put(&data->dma_tx[dma_ch].frags, net_buf_ref(frag));

		dma_data->tail = (dma_data->tail + 1) % (dma_cfg->descs_count);
	} while ((frag = frag->frags));

	barrier_dsync_fence_full();

	/* Notify the hardware */
	eth_dwmac_dma_tx_set_tail(priv, dma_ch);

	/* Unlock DMA Channel */
	k_mutex_unlock(&dma_data->lock);

	return 0;

abort_frag:
	/* Reverse packet fragment mapping */
	dma_data->tail =
		(dma_data->tail - frag_count + dma_cfg->descs_count) % (dma_cfg->descs_count);

	struct net_buf *frag_to_free = pkt->frags;
	while (frag_to_free != frag) {
		k_sem_give(&dma_data->desc_used);
		net_buf_unref(frag_to_free);
		net_buf_slist_remove(&dma_data->frags, prev_frag, frag_to_free);
		frag_to_free = frag_to_free->frags;
	}

	/* Unmap packet */
	net_pkt_unref(pkt);

	/* Unlock DMA Channel */
	k_mutex_unlock(&dma_data->lock);

	return -ETIMEDOUT;
}

#if CONFIG_PTP_CLOCK
const struct privice *eth_dwmac_get_ptp_clock(struct xgmac_driver_s *priv)
{
	const struct eth_dwmac_config *cfg = priv->config;

	return cfg->ptp_clock;
}
#endif

static const struct privice *eth_dwmac_get_phy(struct xgmac_driver_s *priv)
{
	return ((const struct eth_dwmac_config *)priv)->phy;
}
#endif
