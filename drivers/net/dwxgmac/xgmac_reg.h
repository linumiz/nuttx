/****************************************************************************
 * drivers/net/dwxgmac/xgmac_reg.h
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

#ifndef __DRIVERS_NET_DWXGMAC_XGMAC_REG_H
#define __DRIVERS_NET_DWXGMAC_XGMAC_REG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/bits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Synopsys DesignWare XGMAC register offsets and fields used by the
 * driver.  The MAC, MTL and MMC offsets are relative to the MAC base, the
 * DMA offsets to the DMA base, which is the MAC base on most parts.
 */

#define XGMAC_MAC_TX_CONFIGURATION          0x0
#define XGMAC_MAC_TX_CONFIGURATION_TE       BIT(0)
#define XGMAC_MAC_TX_CONFIGURATION_SS       GENMASK(31, 29)

#define XGMAC_MAC_RX_CONFIGURATION          0x4
#define XGMAC_MAC_RX_CONFIGURATION_RE       BIT(0)
#define XGMAC_MAC_RX_CONFIGURATION_ACS      BIT(1)
#define XGMAC_MAC_RX_CONFIGURATION_CST      BIT(2)
#define XGMAC_MAC_RX_CONFIGURATION_DCRCC    BIT(3)
#define XGMAC_MAC_RX_CONFIGURATION_GPSLCE   BIT(6)
#define XGMAC_MAC_RX_CONFIGURATION_IPC      BIT(9)
#define XGMAC_MAC_RX_CONFIGURATION_LM       BIT(10)
#define XGMAC_MAC_RX_CONFIGURATION_GPSL     GENMASK(29, 16)

#define XGMAC_MAC_PACKET_FILTER             0x8
#define XGMAC_MAC_PACKET_FILTER_PM          BIT(4)
#define XGMAC_MAC_PACKET_FILTER_RA          BIT(31)

#define XGMAC_MAC_RXQ_CTRL0                 0xA0

#define XGMAC_MAC_RXQ_CTRL1                 0xA4
#define XGMAC_MAC_RXQ_CTRL1_MCBCQ           GENMASK(11, 8)
#define XGMAC_MAC_RXQ_CTRL1_MCBCQEN         BIT(15)

#define XGMAC_MAC_RXQ_CTRL2                 0xA8

#define XGMAC_MAC_RXQ_CTRL3                 0xAC

#define XGMAC_MAC_INTERRUPT_ENABLE          0xB4

#define XGMAC_MAC_HW_FEATURE0               0x11C
#define XGMAC_MAC_HW_FEATURE0_TXCOESEL      BIT(14)
#define XGMAC_MAC_HW_FEATURE0_RXCOESEL      BIT(16)

#define XGMAC_MAC_HW_FEATURE1               0x120
#define XGMAC_MAC_HW_FEATURE1_RXFIFOSIZE    GENMASK(4, 0)
#define XGMAC_MAC_HW_FEATURE1_TXFIFOSIZE    GENMASK(10, 6)

#define XGMAC_MAC_HW_FEATURE2               0x124
#define XGMAC_MAC_HW_FEATURE2_RXQCNT        GENMASK(3, 0)
#define XGMAC_MAC_HW_FEATURE2_TXQCNT        GENMASK(9, 6)
#define XGMAC_MAC_HW_FEATURE2_RXCHCNT       GENMASK(15, 12)
#define XGMAC_MAC_HW_FEATURE2_TXCHCNT       GENMASK(21, 18)

#define XGMAC_MAC_EXTENDED_CONFIGURATION    0x140
#define XGMAC_MAC_EXTENDED_CONFIGURATION_HD BIT(24)

#define XGMAC_MAC_ADDRESSX_HIGH(i)          (0x300 + (i) * 0x8)

#define XGMAC_MAC_ADDRESSX_LOW(i)           (0x304 + (i) * 0x8)
#define XGMAC_MAC_ADDRESSX_HIGH_AE          BIT(31)

#define XGMAC_MMC_FPE_TX_INTERRUPT_MASK     0xA04

#define XGMAC_MMC_FPE_RX_INTERRUPT_MASK     0xA24

#define XGMAC_MMC_IPC_RX_INTERRUPT_MASK     0xA5C

#define XGMAC_MTL_OPERATION_MODE            0x1000
#define XGMAC_MTL_OPERATION_MODE_ETSALG     GENMASK(6, 5)

#define XGMAC_MTL_RXQ_DMA_MAP0              0x1030

#define XGMAC_MTL_RXQ_DMA_MAP1              0x1034

#define XGMAC_MTL_TXQX_OPERATION_MODE(i)    (0x1100 + (i) * 0x80)
#define XGMAC_MTL_TXQX_OPERATION_MODE_FTQ   BIT(0)
#define XGMAC_MTL_TXQX_OPERATION_MODE_TSF   BIT(1)
#define XGMAC_MTL_TXQX_OPERATION_MODE_TXQEN GENMASK(3, 2)
#define XGMAC_MTL_TXQX_OPERATION_MODE_TTC   GENMASK(6, 4)
#define XGMAC_MTL_TXQX_OPERATION_MODE_TQS   GENMASK(22, 16)

#define XGMAC_MTL_RXQX_OPERATION_MODE(i)    (0x1140 + (i) * 0x80)
#define XGMAC_MTL_RXQX_OPERATION_MODE_RSF   BIT(5)
#define XGMAC_MTL_RXQX_OPERATION_MODE_RQS   GENMASK(22, 16)

#define XGMAC_DMA_MODE                      0xF000
#define XGMAC_DMA_MODE_SWR                  BIT(0)

#define XGMAC_DMA_SYSBUS_MODE               0xF004
#define XGMAC_DMA_SYSBUS_MODE_UBL           BIT(0)
#define XGMAC_DMA_SYSBUS_MODE_BLEN16        BIT(3)
#define XGMAC_DMA_SYSBUS_MODE_AAL           BIT(12)

#define XGMAC_DMA_INTERRUPT_STATUS          0xF008

#define XGMAC_DMA_CH_CONTROL(i)             (0xF100 + (i) * 0x80)

#define XGMAC_DMA_CH_TX_CONTROL(i)          (0xF104 + (i) * 0x80)
#define XGMAC_DMA_CH_TX_CONTROL_ST          BIT(0)
#define XGMAC_DMA_CH_TX_CONTROL_TXPBL       GENMASK(21, 16)

#define XGMAC_DMA_CH_RX_CONTROL(i)          (0xF108 + (i) * 0x80)
#define XGMAC_DMA_CH_RX_CONTROL_SR          BIT(0)
#define XGMAC_DMA_CH_RX_CONTROL_RBSZ        GENMASK(14, 1)
#define XGMAC_DMA_CH_RX_CONTROL_RXPBL       GENMASK(21, 16)
#define XGMAC_DMA_CH_RX_CONTROL_RPF         BIT(31)

#define XGMAC_DMA_CH_TXDESC_LIST_LADDRESS(i) (0xF114 + (i) * 0x80)

#define XGMAC_DMA_CH_TXDESC_LIST_HADDRESS(i) (0xF118 + (i) * 0x80)

#define XGMAC_DMA_CH_RXDESC_LIST_LADDRESS(i) (0xF11C + (i) * 0x80)

#define XGMAC_DMA_CH_RXDESC_LIST_HADDRESS(i) (0xF120 + (i) * 0x80)

#define XGMAC_DMA_CH_TXDESC_TAIL_LPOINTER(i) (0xF124 + (i) * 0x80)

#define XGMAC_DMA_CH_RXDESC_TAIL_LPOINTER(i) (0xF12C + (i) * 0x80)

#define XGMAC_DMA_CH_TX_CONTROL2(i)         (0xF130 + (i) * 0x80)
#define XGMAC_DMA_CH_TX_CONTROL2_TDRL       GENMASK(15, 0)

#define XGMAC_DMA_CH_RX_CONTROL2(i)         (0xF134 + (i) * 0x80)
#define XGMAC_DMA_CH_RX_CONTROL2_RDRL       GENMASK(15, 0)

#define XGMAC_DMA_CH_INTERRUPT_ENABLE(i)    (0xF138 + (i) * 0x80)
#define XGMAC_DMA_CH_INTERRUPT_ENABLE_TIE   BIT(0)
#define XGMAC_DMA_CH_INTERRUPT_ENABLE_RIE   BIT(6)
#define XGMAC_DMA_CH_INTERRUPT_ENABLE_RBUE  BIT(7)
#define XGMAC_DMA_CH_INTERRUPT_ENABLE_RSE   BIT(8)
#define XGMAC_DMA_CH_INTERRUPT_ENABLE_FBEE  BIT(12)
#define XGMAC_DMA_CH_INTERRUPT_ENABLE_AIE   BIT(14)
#define XGMAC_DMA_CH_INTERRUPT_ENABLE_NIE   BIT(15)

#define XGMAC_DMA_CH_STATUS(i)              (0xF160 + (i) * 0x80)
#define XGMAC_DMA_CH_STATUS_TI              BIT(0)
#define XGMAC_DMA_CH_STATUS_TBU             BIT(2)
#define XGMAC_DMA_CH_STATUS_RI              BIT(6)
#define XGMAC_DMA_CH_STATUS_RBU             BIT(7)
#define XGMAC_DMA_CH_STATUS_NIS             BIT(15)

#endif /* __DRIVERS_NET_DWXGMAC_XGMAC_REG_H */
