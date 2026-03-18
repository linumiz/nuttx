/****************************************************************************
 * arch/tricore/src/tc3xx/tc3x_geth.h
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

#ifndef ETHERNET_TC3X_GETH_H
#define ETHERNET_TC3X_GETH_H

/****************************************************************************
 * GETH registers
 ****************************************************************************/

#define TC3XX_ETH_BASE              CONFIG_TRICORE_GETH_BASE
#define TC3XX_ETH_CLC               (TC3XX_ETH_BASE + 0x000)
#define TC3XX_ETH_GPCTL             (TC3XX_ETH_BASE + 0x008)
#define TC3XX_ETH_ACCEN0            (TC3XX_ETH_BASE + 0x00C)
#define TC3XX_ETH_ACCEN1            (TC3XX_ETH_BASE + 0x010)
#define TC3XX_ETH_RST_CTRLA         (TC3XX_ETH_BASE + 0x014)
#define TC3XX_ETH_RST_CTRLB         (TC3XX_ETH_BASE + 0x018)
#define TC3XX_ETH_RST_STAT          (TC3XX_ETH_BASE + 0x01C)
#define TC3XX_ETH_ACCEN_CH_WRA(i)   (TC3XX_ETH_BASE + 0x020 + 0x8 * (i))
#define TC3XX_ETH_ACCEN_CH_WRB(i)   (TC3XX_ETH_BASE + 0x024 + 0x8 * (i))


#define TC3XX_ETH_PHY_MODE_MASK     GENMASK(24, 22)
#define TC3XX_ETH_PHY_MODE(mode)    (mode << 22)

#endif /* ETHERNET_TC3X_GETH_H */
