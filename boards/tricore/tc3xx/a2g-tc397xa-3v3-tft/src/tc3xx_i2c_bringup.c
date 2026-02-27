/****************************************************************************
 * boards/tricore/tc3xx/a2g-tc397xa-3v3-tft/src/tc3xx_i2c_bringup.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>

#include "tc3xx_i2c.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int tc3xx_i2c_bringup(void)
{
  FAR struct i2c_master_s *i2c;
  int ret = OK;

#ifdef CONFIG_TC3XX_I2C0
  i2c = tc3xx_i2cbus_initialize(0);
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

#ifdef CONFIG_TC3XX_I2C1
  i2c = tc3xx_i2cbus_initialize(1);
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

  return ret;
}
