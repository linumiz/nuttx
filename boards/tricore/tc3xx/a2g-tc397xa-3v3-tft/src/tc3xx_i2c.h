/****************************************************************************
 * arch/tricore/src/tc3xx/tc3xx_i2c.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.
 *
 ****************************************************************************/

#ifndef __ARCH_TRICORE_SRC_TC3XX_TC3XX_I2C_H
#define __ARCH_TRICORE_SRC_TC3XX_TC3XX_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_TC3XX_I2C

/****************************************************************************
 * Name: tc3xx_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus.
 *
 * Input Parameters:
 *   port - I2C bus number (0 or 1)
 *
 * Returned Value:
 *   Valid i2c_master_s pointer on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct i2c_master_s *tc3xx_i2cbus_initialize(int bus);

/****************************************************************************
 * Name: tc3xx_i2c_bringup
 *
 * Description:
 *   Board-level I2C initialization: init controllers, register /dev/i2cN.
 *   Call from board_app_initialize() or board_late_initialize().
 *
 ****************************************************************************/

int tc3xx_i2c_bringup(void);

#endif /* CONFIG_TC3XX_I2C */
#endif /* __ARCH_TRICORE_SRC_TC3XX_TC3XX_I2C_H */
