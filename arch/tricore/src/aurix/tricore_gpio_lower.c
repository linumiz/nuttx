/****************************************************************************
 * arch/tricore/src/aurix/tricore_gpio_lower.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/ioexpander/gpio.h>

#include "tricore_gpio.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct aurix_gpio_lower_s
{
  struct gpio_dev_s gpio;
  gpio_pinset_t pinset;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  aurix_gpio_lower_read(FAR struct gpio_dev_s *dev,
                                   FAR bool *value);
static int  aurix_gpio_lower_write(FAR struct gpio_dev_s *dev,
                                    bool value);
static int  aurix_gpio_lower_setpintype(FAR struct gpio_dev_s *dev,
                                         enum gpio_pintype_e pintype);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gpio_operations_s g_aurix_gpio_ops =
{
  .go_read       = aurix_gpio_lower_read,
  .go_write      = aurix_gpio_lower_write,
  .go_attach     = NULL,
  .go_enable     = NULL,
  .go_setpintype = aurix_gpio_lower_setpintype,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int aurix_gpio_lower_read(FAR struct gpio_dev_s *dev,
                                  FAR bool *value)
{
  FAR struct aurix_gpio_lower_s *priv =
    (FAR struct aurix_gpio_lower_s *)dev;

  DEBUGASSERT(priv != NULL && value != NULL);
  *value = aurix_gpio_read(priv->pinset);

  return OK;
}

static int aurix_gpio_lower_write(FAR struct gpio_dev_s *dev,
                                   bool value)
{
  FAR struct aurix_gpio_lower_s *priv =
    (FAR struct aurix_gpio_lower_s *)dev;

  DEBUGASSERT(priv != NULL);
  aurix_gpio_write(priv->pinset, value);

  return OK;
}

static int aurix_gpio_lower_setpintype(FAR struct gpio_dev_s *dev,
                                        enum gpio_pintype_e pintype)
{
  FAR struct aurix_gpio_lower_s *priv =
    (FAR struct aurix_gpio_lower_s *)dev;
  gpio_pinset_t newset;

  DEBUGASSERT(priv != NULL);

  newset = priv->pinset & ~(GPIO_MODE_MASK | GPIO_FUNCALT_MASK |
                             GPIO_OPEN_DRAIN | GPIO_OUTPUT_HIGH);

  switch (pintype)
    {
      case GPIO_INPUT_PIN:
        newset |= GPIO_INPUT | GPIO_PULL_NONE;
        break;

      case GPIO_INPUT_PIN_PULLUP:
        newset |= GPIO_INPUT | GPIO_PULL_UP;
        break;

      case GPIO_INPUT_PIN_PULLDOWN:
        newset |= GPIO_INPUT | GPIO_PULL_DOWN;
        break;

      case GPIO_OUTPUT_PIN:
        newset |= GPIO_OUTPUT | GPIO_ALT0;
        break;

      case GPIO_OUTPUT_PIN_OPENDRAIN:
        newset |= GPIO_OUTPUT | GPIO_ALT0 | GPIO_OPEN_DRAIN;
        break;

      default:
        return -EINVAL;
    }

  priv->pinset = newset;
  return aurix_config_gpio(newset);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int aurix_gpio_lower_register(gpio_pinset_t pinset, int minor)
{
  FAR struct aurix_gpio_lower_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->pinset = pinset;

  switch (GPIO_GET_MODE(pinset))
    {
      case GPIO_INPUT:
        if (GPIO_GET_FUNCALT(pinset) == 2)
          {
            priv->gpio.gp_pintype = GPIO_INPUT_PIN_PULLUP;
          }
        else if (GPIO_GET_FUNCALT(pinset) == 1)
          {
            priv->gpio.gp_pintype = GPIO_INPUT_PIN_PULLDOWN;
          }
        else
          {
            priv->gpio.gp_pintype = GPIO_INPUT_PIN;
          }
        break;

      case GPIO_OUTPUT:
        if (GPIO_IS_OPENDRAIN(pinset))
          {
            priv->gpio.gp_pintype = GPIO_OUTPUT_PIN_OPENDRAIN;
          }
        else
          {
            priv->gpio.gp_pintype = GPIO_OUTPUT_PIN;
          }
        break;

      default:

        priv->gpio.gp_pintype = GPIO_OUTPUT_PIN;
        break;
    }

  priv->gpio.gp_ops = &g_aurix_gpio_ops;

  ret = aurix_config_gpio(pinset);
  if (ret < 0)
    {
      kmm_free(priv);
      return ret;
    }

  ret = gpio_pin_register(&priv->gpio, minor);
  if (ret < 0)
    {
      kmm_free(priv);
      return ret;
    }

  return OK;
}
