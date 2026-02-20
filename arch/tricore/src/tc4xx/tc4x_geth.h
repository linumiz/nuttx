/****************************************************************************
 * drivers/net/tc4x_geth.h
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

#ifndef ETHERNET_TC4X_GETH_H
#define ETHERNET_TC4X_GETH_H

/****************************************************************************
 * GETH registers
 ****************************************************************************/

#define TC4XX_ETH_BASE              CONFIG_TRICORE_GETH_BASE
#define TC4XX_ETH_CLC               (TC4XX_ETH_BASE + 0x000)
#define TC4XX_ETH_RST_CTRLA         (TC4XX_ETH_BASE + 0x00C)
#define TC4XX_ETH_RST_CTRLB         (TC4XX_ETH_BASE + 0x010)
#define TC4XX_ETH_RST_STAT          (TC4XX_ETH_BASE + 0x014)
#define TC4XX_ETH_MACEN             (TC4XX_ETH_BASE + 0x018)
#define TC4XX_ETH_ACCEN_GBL_WRA     (TC4XX_ETH_BASE + 0x040)
#define TC4XX_ETH_ACCEN_GBL_WRB     (TC4XX_ETH_BASE + 0x044)
#define TC4XX_ETH_ACCEN_MAC0_WRA    (TC4XX_ETH_BASE + 0x060)
#define TC4XX_ETH_ACCEN_MAC0_WRB    (TC4XX_ETH_BASE + 0x064)
#define TC4XX_ETH_ACCEN_CH_WRA(i)   (TC4XX_ETH_BASE + 0x0A0 + 0x20 * (i))
#define TC4XX_ETH_ACCEN_CH_WRB(i)   (TC4XX_ETH_BASE + 0x0A4 + 0x20 * (i))

/****************************************************************************
 * Bridge registers
 ****************************************************************************/

#define TC4XX_BRIDGE_BASE            (TC4XX_ETH_BASE + 0x1E000)
#define TC4XX_BRIDGE_FWDCTRL         (TC4XX_BRIDGE_BASE + 0x000)
#define TC4XX_BRIDGE_PORT0_CTRL      (TC4XX_BRIDGE_BASE + 0x00C)
#define TC4XX_BRIDGE_TXQ_MAP_P0      (TC4XX_BRIDGE_BASE + 0x010)
#define TC4XX_BRIDGE_RXC_MAP_P0      (TC4XX_BRIDGE_BASE + 0x014)
#define TC4XX_BRIDGE_PORT1_CTRL      (TC4XX_BRIDGE_BASE + 0x018)
#define TC4XX_BRIDGE_INTR_STATUS     (TC4XX_BRIDGE_BASE + 0x104)

/****************************************************************************
 * HSPHY registers
 ****************************************************************************/

#define TC4XX_HSPHY_BASE             0xF2000000u
#define TC4XX_HSPHY_CLC              (TC4XX_HSPHY_BASE + 0x000)
#define TC4XX_HSPHY_RSTA             (TC4XX_HSPHY_BASE + 0x00C)
#define TC4XX_HSPHY_RSTB             (TC4XX_HSPHY_BASE + 0x010)
#define TC4XX_HSPHY_RSTSTAT          (TC4XX_HSPHY_BASE + 0x014)
#define TC4XX_HSPHY_CMNCFG           (TC4XX_HSPHY_BASE + 0x200)
#define TC4XX_HSPHY_PHYCTRL0(i)      (TC4XX_HSPHY_BASE + 0x210 + (8 * (i)))
#define TC4XX_HSPHY_PHYCTRL1(i)      (TC4XX_HSPHY_BASE + 0x214 + (8 * (i)))
#define TC4XX_HSPHY_ETH(i)           (TC4XX_HSPHY_BASE + 0x600 + (4 * (i)))
#define TC4XX_HSPHY_DLLCFG           (TC4XX_HSPHY_BASE + 0x628)

/****************************************************************************
 * PMS VMON registers
 ****************************************************************************/

#define PMS_VMONP_VDDPHPHY0CON   0xF024909Cu
#define PMS_VMONP_VDDPHPHY0RST   0xF02490A0u
#define PMS_VMONP_VDDPHPHY0STAT  0xF02490A4u
#define PMS_VMONP_VDDPHPHY1CON   0xF02490A8u
#define PMS_VMONP_VDDPHPHY1RST   0xF02490ACu
#define PMS_VMONP_VDDPHPHY1STAT  0xF02490B0u
#define PMS_VMONP_VDDPHPHY2CON   0xF02490B4u
#define PMS_VMONP_VDDPHPHY2RST   0xF02490B8u
#define PMS_VMONP_VDDPHPHY2STAT  0xF02490BCu
#define PMS_VMONP_VDDHSIFCON     0xF02490C0u
#define PMS_VMONP_VDDHSIFRST     0xF02490C4u
#define PMS_VMONP_VDDHSIFSTAT    0xF02490C8u
#define PMS_VMONP_VDDPHY0CON     0xF0249114u
#define PMS_VMONP_VDDPHY0RST     0xF0249118u
#define PMS_VMONP_VDDPHY0STAT    0xF024911Cu
#define PMS_VMONP_VDDPHY1CON     0xF0249120u
#define PMS_VMONP_VDDPHY1RST     0xF0249124u
#define PMS_VMONP_VDDPHY2CON     0xF024912Cu
#define PMS_VMONP_VDDPHY2RST     0xF0249130u

/* PMS VMON register bits */
#define PMS_VMONP_OVE            BIT(28)
#define PMS_VMONP_RESETOFF       BIT(24)
#define PMS_VMONP_RESETOFF_P     BIT(25)

#endif /* ETHERNET_TC4X_GETH_H */
