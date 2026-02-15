/****************************************************************************
 * arch/tricore/src/tc4xx/tc4xx_asclin_spi.h
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

#ifndef __ARCH_TRICORE_SRC_TC4XX_TC4XX_ASCLIN_SPI_H
#define __ARCH_TRICORE_SRC_TC4XX_TC4XX_ASCLIN_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>

#include "tc4xx_gpio.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tc4xx_asclin_spibus_initialize
 *
 * Description:
 *   Initialize an ASCLIN module in SPI master mode and return a reference
 *   to the SPI interface.
 *
 *   The ASCLIN IP supports UART/LIN/SPI modes.  This driver configures the
 *   selected ASCLIN instance for synchronous SPI master operation using
 *   FRAMECON.MODE = SPI.
 *
 *   Chip select is managed via GPIO, not the hardware ASLSO signal.  This
 *   gives full flexibility for any CS pin assignment.
 *
 * Input Parameters:
 *   bus - SPI bus number (maps to CONFIG_TC4XX_ASCLINn_SPI)
 *
 * Returned Value:
 *   A pointer to the SPI bus interface on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct spi_dev_s *tc4xx_asclin_spibus_initialize(int bus);

/****************************************************************************
 * Name: tc4xx_asclin_spi_select
 *
 * Description:
 *   Board-level chip select callback.  Weak default implementation
 *   provided by the driver uses the GPIO CS pin from the config.
 *   Boards may override this for custom CS logic.
 *
 ****************************************************************************/

void tc4xx_asclin_spi_select(FAR struct spi_dev_s *dev, uint32_t devid,
                             bool selected);

#endif /* __ARCH_TRICORE_SRC_TC4XX_TC4XX_ASCLIN_SPI_H */
