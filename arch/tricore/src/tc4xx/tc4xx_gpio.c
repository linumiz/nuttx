/****************************************************************************
 * arch/tricore/src/tc4xx/tc4xx_gpio.c
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include "tc4xx_gpio.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_aurix_gpio_lock = SP_UNLOCKED;
/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aurix_config_gpio
 *
 * Description:
 *   Configure a GPIO pin based on pin-encoded description of the pin.
 *
 ****************************************************************************/

int __attribute__((optimize("O0"))) aurix_config_gpio(gpio_pinset_t pinset)
{
  irqstate_t flags;
  int ret;
  uintptr_t regaddr;
  uint32_t regval = 0;
  uint32_t padflag;
  flags = spin_lock_irqsave(&g_aurix_gpio_lock);

  /* Configure based upon the pin mode */

  switch (pinset & GPIO_MODE_MASK)
    {
      case GPIO_INPUT:
        {
          regaddr = TC4X_PORTn_CCR(GPIO_PORT(pinset), GPIO_PIN(pinset));
          regval = pinset & (GPIO_INPUT_LEVEL_MASK | GPIO_PAD_CONFIG_MASK);
          modreg32(regval, (GPIO_INPUT_LEVEL_MASK | GPIO_PAD_CONFIG_MASK | BIT(0)), regaddr);
        }
        break;

      case GPIO_OUTPUT:
        {
          regaddr = TC4X_PORTn_CCR(GPIO_PORT(pinset), GPIO_PIN(pinset));
          modreg32(BIT(0), (GPIO_PAD_CONFIG_MASK | BIT(0)), regaddr);
        }
        break;

      case GPIO_PERIPH_OWN_PAD:
        {

	  regaddr = TC4X_PORTn_PCRSEL(GPIO_PORT(pinset));
          regval = BIT(GPIO_PIN(pinset));
          modreg32(regval, regval, regaddr);
        }
        /* fall through */

      case GPIO_PERIPH:
        {
          /* Configure the pin as a peripheral */
          regaddr = TC4X_PORTn_CCR(GPIO_PORT(pinset), GPIO_PIN(pinset));
          regval = (pinset & GPIO_PAD_CONFIG_MASK) | BIT(0);
          modreg32(regval, (GPIO_PAD_CONFIG_MASK | BIT(0)), regaddr);
        }
        break;

      default:
        ret = -EINVAL;
        break;
    }

  spin_unlock_irqrestore(&g_aurix_gpio_lock, flags);
  return ret;
}

/****************************************************************************
 * Name: aurix_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void aurix_gpio_write(gpio_pinset_t pinset, bool value)
{
  irqstate_t flags;
  uintptr_t regaddr;
  uint32_t regval;
  uint32_t mask;

  regaddr = TC4X_PORTn_OMR(GPIO_PORT(pinset));
  flags = spin_lock_irqsave(&g_aurix_gpio_lock);
  regval = BIT(GPIO_PIN(pinset));
  mask = BIT(GPIO_PIN(pinset));

  if (value == 0)
   {
     regval <<= TC4X_PORT_OMR_CLR_SHIFT;
     mask <<= TC4X_PORT_OMR_CLR_SHIFT;
   }

  modreg32(regval, mask, regaddr);
  spin_unlock_irqrestore(&g_aurix_gpio_lock, flags);
}

/****************************************************************************
 * Name: aurix_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool aurix_gpio_read(gpio_pinset_t pinset)
{
  irqstate_t flags;
  uintptr_t regaddr;
  bool value;

  regaddr = TC4X_PORTn_IN(GPIO_PORT(pinset));
  flags = spin_lock_irqsave(&g_aurix_gpio_lock);
  value = !!(getreg32(regaddr) & BIT(GPIO_PIN(pinset)));
  spin_unlock_irqrestore(&g_aurix_gpio_lock, flags);

  return value;
}
