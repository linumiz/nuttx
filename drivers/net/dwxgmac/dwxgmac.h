/****************************************************************************
 * drivers/net/dwxgmac.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 *
 ****************************************************************************/

#ifndef ETHERNET_DWXGMAC_H
#define ETHERNET_DWXGMAC_H

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/net/netdev.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

struct eth_dwmac_dma_desc
{
  uint32_t des0;
  uint32_t des1;
  uint32_t des2;
  uint32_t des3;
};

struct eth_dwmac_dma_tx_ch_data
{
  sem_t desc_used;
  uint16_t head;
  uint16_t tail;
  mutex_t lock;
};

struct eth_dwmac_dma_rx_ch_data
{
  sem_t desc_used;
  uint16_t head;
  uint16_t tail;
  uint8_t *data[CONFIG_DWXGMAC_NUM_RX_DMA_DESCRIPTORS];
};

struct eth_dwmac_dma_ch_config
{
  struct eth_dwmac_dma_desc *descs;
  uint16_t descs_count;
  uint8_t nr;
  bool osf;
};

struct eth_dwmac_rx_queue
{
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

struct eth_dwmac_tx_queue
{
  uint16_t size;
  uint8_t nr;
  uint8_t priority;
  uint8_t threshold;
  bool sf;
};

struct eth_dwmac_config
{
  bool is_xgmac;
  bool clocks_stop;
  bool slave_mode;
  bool da_duplication;
  bool loopback;
  bool drop_tx_status;

  uint8_t dma_rx_channel;
  uint8_t dma_tx_channel;
  uint8_t mtl_rx_queues;
  uint8_t mtl_tx_queues;

  struct eth_dwmac_dma_ch_config *dma_rx;
  struct eth_dwmac_dma_ch_config *dma_tx;
  struct eth_dwmac_rx_queue *mtl_rx;
  struct eth_dwmac_tx_queue *mtl_tx;
};

struct eth_dwmac_data
{
  struct eth_dwmac_dma_rx_ch_data *dma_rx;
  struct eth_dwmac_dma_tx_ch_data *dma_tx;

  uint8_t mac_addr[6];
  bool started;
  bool txcoe_available;
  bool rxcoe_available;
};

#endif /* ETHERNET_DWXGMAC_H */
