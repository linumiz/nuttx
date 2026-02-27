/****************************************************************************
 * arch/tricore/src/tc4xx/tc4xx_i2c.h
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

#ifndef __ARCH_TRICORE_SRC_TC4XX_TC4XX_I2C_H
#define __ARCH_TRICORE_SRC_TC4XX_TC4XX_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_TC4XX_I2C

/****************************************************************************
 * Name: tc4xx_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus.
 *
 * Input Parameters:
 *   port - I2C bus number (0, 1, or 2)
 *
 * Returned Value:
 *   Valid i2c_master_s pointer on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct i2c_master_s *tc4xx_i2cbus_initialize(int bus);

/****************************************************************************
 * Name: tc4xx_i2cbus_uninitialize
 *
 * Description:
 *   Shut down an I2C bus.
 *
 ****************************************************************************/

int tc4xx_i2cbus_uninitialize(FAR struct i2c_master_s *dev);

/****************************************************************************
 * Name: tc4xx_i2c_bringup
 *
 * Description:
 *   Board-level I2C initialization: init controllers, register /dev/i2cN.
 *   Call from board_app_initialize() or board_late_initialize().
 *
 ****************************************************************************/

int tc4xx_i2c_bringup(void);

#endif /* CONFIG_TC4XX_I2C */
#endif /* __ARCH_TRICORE_SRC_TC4XX_TC4XX_I2C_H */
