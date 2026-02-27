/****************************************************************************
 * arch/tricore/src/tc4xx/hardware/tc4xx_i2c.h
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

#ifndef __ARCH_TRICORE_SRC_TC4XX_HARDWARE_TC4XX_I2C_H
#define __ARCH_TRICORE_SRC_TC4XX_HARDWARE_TC4XX_I2C_H

#define TC4XX_I2C0_BASE         0xF44C0000
#define TC4XX_I2C1_BASE         0xF44E0000
#define TC4XX_I2C2_BASE         0xF4500000

#define TC4XX_I2C_NUM_MODULES   3

#define TC4XX_I2C_CLC_OFFSET        0x10000
#define TC4XX_I2C_ID_OFFSET         0x10008
#define TC4XX_I2C_RST_CTRLA_OFFSET  0x1000C
#define TC4XX_I2C_RST_CTRLB_OFFSET  0x10010
#define TC4XX_I2C_RST_STAT_OFFSET   0x10014
#define TC4XX_I2C_GPCTL_OFFSET      0x10060

#define I2C_RST_CTRLA_KRST          BIT(0)

#define I2C_RST_CTRLB_KRST          BIT(0)
#define I2C_RST_CTRLB_STATCLR       BIT(31)

#endif /* __ARCH_TRICORE_SRC_TC4XX_HARDWARE_TC4XX_I2C_H */
