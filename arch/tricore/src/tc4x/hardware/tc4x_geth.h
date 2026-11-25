/****************************************************************************
 * arch/tricore/src/tc4x/hardware/tc4x_geth.h
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

#ifndef __ARCH_TRICORE_SRC_TC4X_HARDWARE_TC4X_GETH_H
#define __ARCH_TRICORE_SRC_TC4X_HARDWARE_TC4X_GETH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/bits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GETH module: wrapper, the XGMAC of each port and the bridge */

#define TC4X_GETH_BASE             0xf9000000
#define TC4X_GETH_CLC              (TC4X_GETH_BASE + 0x000)
#define TC4X_GETH_RST_CTRLA        (TC4X_GETH_BASE + 0x00c)
#define TC4X_GETH_RST_CTRLB        (TC4X_GETH_BASE + 0x010)
#define TC4X_GETH_RST_STAT         (TC4X_GETH_BASE + 0x014)
#define TC4X_GETH_MACEN            (TC4X_GETH_BASE + 0x018)
#define TC4X_GETH_ACCEN_GBL_WRA    (TC4X_GETH_BASE + 0x040)
#define TC4X_GETH_ACCEN_GBL_WRB    (TC4X_GETH_BASE + 0x044)
#define TC4X_GETH_ACCEN_MAC_WRA(p) (TC4X_GETH_BASE + 0x060 + (p) * 0x20)
#define TC4X_GETH_ACCEN_MAC_WRB(p) (TC4X_GETH_BASE + 0x064 + (p) * 0x20)
#define TC4X_GETH_ACCEN_CH_WRA(n)  (TC4X_GETH_BASE + 0x0a0 + (n) * 0x20)
#define TC4X_GETH_ACCEN_CH_WRB(n)  (TC4X_GETH_BASE + 0x0a4 + (n) * 0x20)

#define TC4X_GETH_MAC_BASE(p)      (TC4X_GETH_BASE + 0x10000 + (p) * 0x2000)
#define TC4X_GETH_DMA_BASE         TC4X_GETH_MAC_BASE(0)
#define TC4X_GETH_NDMACH           8

#define TC4X_GETH_BRIDGE_BASE      (TC4X_GETH_BASE + 0x1e000)
#define TC4X_GETH_BRIDGE_FLUSH     (TC4X_GETH_BRIDGE_BASE + 0x004)
#define TC4X_GETH_BRIDGE_PORT0     (TC4X_GETH_BRIDGE_BASE + 0x00c)
#define TC4X_GETH_BRIDGE_DMASEL    (TC4X_GETH_BRIDGE_BASE + 0x110)

/* Interrupts of DMA channel n: SRC_GETH0_DMAn for TX and
 * SRC_GETH0_DMA(8 + n) for RX
 */

#define TC4X_GETH_TX_IRQ(n)        TRICORE_SRC2IRQ(0xf4432b90 + (n) * 4)
#define TC4X_GETH_RX_IRQ(n)        TRICORE_SRC2IRQ(0xf4432bb0 + (n) * 4)

#define TC4X_GETH_CLC_DISR         BIT(0)  /* Disable request */
#define TC4X_GETH_CLC_DISS         BIT(1)  /* Disable status */
#define TC4X_GETH_RST_KRST         BIT(0)  /* Kernel reset */
#define TC4X_GETH_RST_STATCLR      BIT(31) /* Clear the reset status */

/* Bridge: all queues and DMA channels of port 0 */

#define TC4X_GETH_BRIDGE_PORT0_ALL 0xff00ff00

/* HSPHY: MII mode and input selection of each GETH port */

#define TC4X_HSPHY_BASE            0xf2000000
#define TC4X_HSPHY_CLC             (TC4X_HSPHY_BASE + 0x000)
#define TC4X_HSPHY_RST_CTRLA       (TC4X_HSPHY_BASE + 0x00c)
#define TC4X_HSPHY_RST_CTRLB       (TC4X_HSPHY_BASE + 0x010)
#define TC4X_HSPHY_RST_STAT        (TC4X_HSPHY_BASE + 0x014)
#define TC4X_HSPHY_CMNCFG          (TC4X_HSPHY_BASE + 0x200)
#define TC4X_HSPHY_PHYCTRL1(p)     (TC4X_HSPHY_BASE + 0x214 + (p) * 8)
#define TC4X_HSPHY_ETH(p)          (TC4X_HSPHY_BASE + 0x600 + (p) * 4)

#define TC4X_HSPHY_CMNCFG_FSR      BIT(4)
#define TC4X_HSPHY_PHYCTRL1_RST    BIT(7)  /* PHY reset */
#define TC4X_HSPHY_PHYCTRL1_PWRDWN BIT(8)  /* PHY power down */

#define TC4X_HSPHY_ETH_MDIO        GENMASK(1, 0)   /* MDIO input */
#define TC4X_HSPHY_ETH_MDIOEN      BIT(2)
#define TC4X_HSPHY_ETH_CRSDV       GENMASK(9, 8)   /* CRS_DV input */
#define TC4X_HSPHY_ETH_RXD0        GENMASK(13, 12) /* RXD0 input */
#define TC4X_HSPHY_ETH_RXD1        GENMASK(15, 14) /* RXD1 input */
#define TC4X_HSPHY_ETH_REFCLK      GENMASK(23, 22) /* REFCLK input */
#define TC4X_HSPHY_ETH_MODE        GENMASK(30, 28) /* Interface mode */
#define TC4X_HSPHY_ETH_MODE_RMII   (2 << 28)

/* LMU0 access protection: PROTRGN selects the region that the ACCENCFG
 * registers configure.  TAG IDs 48 to 55 are the GETH DMA masters, bits
 * 16 to 23 of the WRB register of TAG IDs 32 to 63.
 */

#define TC4X_LMU0_PROTRGN          0xfb000070
#define TC4X_LMU0_ACCENCFG_WRB     0xfb000304

#define TC4X_LMU_PROTRGN_STATE     GENMASK(2, 0)
#define TC4X_LMU_PROTRGN_SEL       GENMASK(11, 8)
#define TC4X_LMU_PROT_INIT         0
#define TC4X_LMU_PROT_CONFIG       1
#define TC4X_LMU_WRB_GETH          GENMASK(23, 16)

/* PMS supply monitors of the HSIF and PHY supplies */

#define TC4X_PMS_VDDHSIFCON        0xf02490c0
#define TC4X_PMS_VDDHSIFRST        0xf02490c4
#define TC4X_PMS_VDDHSIFSTAT       0xf02490c8
#define TC4X_PMS_VDDPHY0CON        0xf0249114
#define TC4X_PMS_VDDPHY0STAT       0xf024911c
#define TC4X_PMS_VDDPHY1CON        0xf0249120
#define TC4X_PMS_VDDPHY2CON        0xf024912c

#define TC4X_PMS_VMON_OVE          BIT(28) /* Overvoltage monitor enable */
#define TC4X_PMS_VMON_RESETOFF     BIT(24)
#define TC4X_PMS_VMON_RESETOFF_P   BIT(25) /* RESETOFF write enable */
#define TC4X_PMS_VMON_RESULT       GENMASK(15, 8)
#define TC4X_PMS_VMON_RESETVAL     GENMASK(7, 0)

#endif /* __ARCH_TRICORE_SRC_TC4X_HARDWARE_TC4X_GETH_H */
