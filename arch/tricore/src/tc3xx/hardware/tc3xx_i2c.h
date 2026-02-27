/****************************************************************************
 * arch/tricore/src/tc3xx/hardware/tc3xx_i2c.h
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

#ifndef __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_I2C_H
#define __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_I2C_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TC3XX_I2C0_BASE         0xF00C0000
#define TC3XX_I2C1_BASE         0xF00E0000

#define TC3XX_I2C_NUM_MODULES   2

#define TC3XX_I2C_CLC_OFFSET        0x10000
#define TC3XX_I2C_MODID_OFFSET      0x10004
#define TC3XX_I2C_GPCTL_OFFSET      0x10008
#define TC3XX_I2C_ACCEN0_OFFSET     0x1000C
#define TC3XX_I2C_ACCEN1_OFFSET     0x10010
#define TC3XX_I2C_KRST0_OFFSET      0x10014
#define TC3XX_I2C_KRST1_OFFSET      0x10018
#define TC3XX_I2C_KRSTCLR_OFFSET    0x1001C

#define I2C_KRST0_RST               BIT(0)
#define I2C_KRST0_RSTSTAT           BIT(1)

#define I2C_KRST1_RST               BIT(0)

#define I2C_KRSTCLR_CLR             BIT(0)

#define I2C_ACCEN0_DEFAULT          0xffffffff

#endif /* __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_I2C_H */
