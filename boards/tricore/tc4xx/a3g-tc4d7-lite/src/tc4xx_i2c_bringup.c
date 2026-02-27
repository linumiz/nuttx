/****************************************************************************
 * boards/tricore/tc4xx/a3g-tc4d7-lite/src/tc4xx_i2c_bringup.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include "tc4xx_i2c.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int tc4xx_i2c_bringup(void)
{
  FAR struct i2c_master_s *i2c;
  int ret = OK;

#ifdef CONFIG_TC4XX_I2C0
  i2c = tc4xx_i2cbus_initialize(0);
  if (i2c == NULL)
    {
      i2cerr("Failed to initialize I2C0\n");
      ret = -ENODEV;
    }
  else
    {
#ifdef CONFIG_I2C_DRIVER
      i2c_register(i2c, 0);
#endif
    }
#endif

#ifdef CONFIG_TC4XX_I2C1
  i2c = tc4xx_i2cbus_initialize(1);
  if (i2c == NULL)
    {
      i2cerr("Failed to initialize I2C1\n");
      ret = -ENODEV;
    }
  else
    {
#ifdef CONFIG_I2C_DRIVER
      i2c_register(i2c, 1);
#endif
    }
#endif

#ifdef CONFIG_TC4XX_I2C2
  i2c = tc4xx_i2cbus_initialize(2);
  if (i2c == NULL)
    {
      i2cerr("Failed to initialize I2C2\n");
      ret = -ENODEV;
    }
  else
    {
#ifdef CONFIG_I2C_DRIVER
      i2c_register(i2c, 2);
#endif
    }
#endif

  return ret;
}
