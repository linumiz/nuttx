#ifndef ETHERNET_ETH_DWMAC_H
#define ETHERNET_ETH_DWMAC_H

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/net/netdev.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

struct eth_dwmac_dma_desc {
	uint32_t des0;
	uint32_t des1;
	uint32_t des2;
	uint32_t des3;
};

struct eth_dwmac_dma_tx_ch_data {
	/** Descriptor used semphore */
	sem_t desc_used;
	/** Head index */
	uint16_t head;
	/** Tail index */
	uint16_t tail;

#if CONFIG_MMU
	/** Physical address to descriptor ring */
	uintptr_t *descs_phys;
#endif
	/** Channel mutext */
	mutex_t lock;
};

struct eth_dwmac_dma_rx_ch_data {
	/** Descriptor used semphore */
	sem_t desc_used;
	/** Head index */
	uint16_t head;
	/** Tail index */
	uint16_t tail;
#if CONFIG_MMU
	/** Physical address to descriptor ring */
	uintptr_t *descs_phys;
#endif
	/** Currently processed packet */
//	struct net_pkt *packet;
        uint8_t  *data;
        uint16_t length;
};

struct eth_dwmac_dma_ch_config {
	/** Pointer to descriptor ring */
	struct eth_dwmac_dma_desc *descs;
	/** Descriptor count */
	uint16_t descs_count;
	/** DMA Channel Number */
	uint8_t nr;
	/** Operate on second frame flag */
	bool osf;
};

struct eth_dwmac_rx_queue {
	uint16_t size;
	uint8_t nr;
	uint8_t priority;
	uint8_t dma_channel;
	uint8_t threshold;
	bool sf;
	bool dynamic_dma_channel;
	bool av_queue;

	bool route_multi_broad;
};

struct eth_dwmac_tx_queue {
	uint16_t size;
	uint8_t nr;
	uint8_t priority;
	uint8_t threshold;
	bool sf;
};

struct eth_dwmac_config {
	/** MAC Base address */
//	uintptr_t dma_base;
	/* Device nodes */
	/** Phy deivce */
	const struct device *phy;
#if CONFIG_PTP_CLOCK
	/** PTP Clock Device */
	const struct device *ptp_clock;
#endif

	/** XGMAC Type */
	bool is_xgmac;
	/** Interface stops RX and TX clocks during link changes */
	bool clocks_stop;
	/** Slave mode */
	bool slave_mode;
	/** Destination address based duplication */
	bool da_duplication;
	/** MAC loopback */
	bool loopback;
	/** Drop TX Status in MTL */
	bool drop_tx_status;

	/** Number of RX DMA Channels used */
	uint8_t dma_rx_channel;
	/** Number of TX DMA Channel used */
	uint8_t dma_tx_channel;
	uint8_t mtl_rx_queues;
	uint8_t mtl_tx_queues;
	/** RX DMA Channel configurations */
	struct eth_dwmac_dma_ch_config *dma_rx;
	/** TX DMA Channel configurations  */
	struct eth_dwmac_dma_ch_config *dma_tx;
	struct eth_dwmac_rx_queue *mtl_rx;
	struct eth_dwmac_tx_queue *mtl_tx;
};

struct eth_dwmac_data {
	struct net_if *iface;

	struct eth_dwmac_dma_rx_ch_data *dma_rx;
	struct eth_dwmac_dma_tx_ch_data *dma_tx;

#if CONFIG_DWMAC_TARGET_TIME_CALLBACK
	void (*ts_callback)();
	void *ts_callback_data;
#endif

	uint8_t mac_addr[6];

	bool started;
	bool txcoe_available;
	bool rxcoe_available;
};
#endif
