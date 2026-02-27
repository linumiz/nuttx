/****************************************************************************
 * boards/tricore/tc3xx/a2g-tc397xa-3v3-tft/src/tc397_appinit.c
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
#include <debug.h>
#include <errno.h>
#include "tricore_gpio.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void board_aurix_setup_serial_pin(void)
{
  gpio_pinset_t txpin = AURIX_GPIO(14, 0, GPIO_PERIPH, GPIO_ALT2);
  gpio_pinset_t rxpin = AURIX_GPIO(14, 1, GPIO_INPUT, GPIO_PULL_UP);

  aurix_config_gpio(txpin);
  aurix_config_gpio(rxpin);
}

int board_app_initialize(uintptr_t arg)
{
  bool ledon = true;
  uint8_t i = 10;
  int ret = OK;
  gpio_pinset_t led1 = AURIX_GPIO(GPIO_PORT33, GPIO_PIN1, GPIO_OUTPUT, GPIO_ALT0);

  aurix_config_gpio(led1);
  while (i--) {
    aurix_gpio_write(led1, (ledon = !ledon));
    usleep(500 * 1000);
  }

  return ret;
}
