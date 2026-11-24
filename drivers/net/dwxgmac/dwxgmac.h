/****************************************************************************
 * drivers/net/dwxgmac/dwxgmac.h
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

#ifndef __DRIVERS_NET_DWXGMAC_DWXGMAC_H
#define __DRIVERS_NET_DWXGMAC_DWXGMAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/bits.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/netdev_lowerhalf.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DWMAC_NRXCH         CONFIG_NET_DWXGMAC_NRXCHANNELS
#define DWMAC_NTXCH         CONFIG_NET_DWXGMAC_NTXCHANNELS
#define DWMAC_NRXDESC       CONFIG_NET_DWXGMAC_NRXDESC
#define DWMAC_NTXDESC       CONFIG_NET_DWXGMAC_NTXDESC
#define DWMAC_MAXCH         8

/* Size of one RX DMA buffer.  The DMA counts the 4 octet FCS against it
 * even with CRC stripping enabled.  A frame that does not fit spills into
 * a second descriptor and is dropped.
 */

#define DWMAC_RX_BUFSZ      ((CONFIG_NET_ETH_PKTSIZE + 4 + 7) & ~7)

/* TDES3 of a read (ring) descriptor, the same in all variants */

#define DWMAC_TDES3_FL      GENMASK(14, 0)  /* Frame length */
#define DWMAC_TDES3_CIC     GENMASK(17, 16) /* Checksum insertion */
#define DWMAC_TDES3_LD      BIT(28)         /* Last descriptor */
#define DWMAC_TDES3_FD      BIT(29)         /* First descriptor */
#define DWMAC_TDES3_OWN     BIT(31)         /* Owned by the DMA */

#define DWMAC_TDES2_B1L     GENMASK(13, 0)  /* Buffer 1 length */
#define DWMAC_TDES2_IOC     BIT(31)         /* Interrupt on completion */

/* Register access */

#define dwmac_getreg(a)     (*(FAR volatile uint32_t *)(a))
#define dwmac_putreg(v, a)  (*(FAR volatile uint32_t *)(a) = (v))

/* Field of a register, with m a constant mask */

#define DWMAC_LSB(m)        ((m) & -(m))
#define FIELD_PREP(m, v)    (((uint32_t)(v) * DWMAC_LSB(m)) & (m))
#define FIELD_GET(m, v)     (((v) & (m)) / DWMAC_LSB(m))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* DMA descriptor, read and write back formats share the four words */

struct dwmac_desc_s
{
  uint32_t des0;
  uint32_t des1;
  uint32_t des2;
  uint32_t des3;
};

/* RX and TX ring of one DMA channel */

struct dwmac_rxring_s
{
  FAR struct dwmac_desc_s *descs;      /* DMA_NRXDESC descriptors */
  FAR netpkt_t *pkt[DWMAC_NRXDESC];    /* Buffer of each descriptor */
  uint16_t head;                       /* Next descriptor to receive */
};

struct dwmac_txring_s
{
  FAR struct dwmac_desc_s *descs;      /* DWMAC_NTXDESC descriptors */
  FAR netpkt_t *pkt[DWMAC_NTXDESC];    /* Packet of each descriptor */
  uint16_t head;                       /* Oldest descriptor in flight */
  uint16_t tail;                       /* Next free descriptor */
};

/* The driver state of one interface */

struct dwmac_driver_s
{
  struct netdev_lowerhalf_s dev;       /* Must be first */
  uintptr_t base;                      /* MAC registers */
  uintptr_t dmabase;                   /* DMA registers */
  int irq;                             /* DMA interrupt or 0 */
  uint32_t txfifo;                     /* Configured TX FIFO size or 0 */
  uint32_t rxqsize;                    /* MTL RX queue size */
  uint32_t txqsize;                    /* MTL TX queue size */
  int speed;                           /* Link speed in Mbps */
  bool link;                           /* PHY link up */
  bool ifup;                           /* Interface up */
  bool txcoe;                          /* TX checksum insertion */
  bool rxcoe;                          /* RX checksum offload */
  uint8_t rxch;                        /* RX channel to look at first */
  struct work_s txdone_work;
  struct dwmac_rxring_s rx[DWMAC_NRXCH];
  struct dwmac_txring_s tx[DWMAC_NTXCH];
};

#endif /* __DRIVERS_NET_DWXGMAC_DWXGMAC_H */
