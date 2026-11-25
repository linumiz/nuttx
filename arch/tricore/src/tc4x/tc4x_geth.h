/****************************************************************************
 * arch/tricore/src/tc4x/tc4x_geth.h
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

#ifndef __ARCH_TRICORE_SRC_TC4X_TC4X_GETH_H
#define __ARCH_TRICORE_SRC_TC4X_TC4X_GETH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/tc4x_geth.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Input selection of the RMII signals of a GETH port, 0 to 3 for the
 * alternate inputs A to D that the board routes the signal to.
 */

struct tc4x_geth_rmii_s
{
  uint8_t crsdv;
  uint8_t rxd0;
  uint8_t rxd1;
  uint8_t refclk;
  uint8_t mdio;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tc4x_geth_initialize
 *
 * Description:
 *   Power, clock and reset the GETH and its HSPHY, set up an RMII port and
 *   register it as dwxgmac interface 0.  The board configures the RMII
 *   pins before and the PHY after, reporting its link with
 *   dwxgmac_carrier.  Its MDIO bus is on TC4X_GETH_MAC_BASE(port).
 *
 * Input Parameters:
 *   port - GETH port, 0 for now
 *   rmii - Input selection of the RMII signals
 *
 * Returned Value:
 *   OK on success, a negated errno value on failure.
 *
 ****************************************************************************/

int tc4x_geth_initialize(int port, FAR const struct tc4x_geth_rmii_s *rmii);

#endif /* __ARCH_TRICORE_SRC_TC4X_TC4X_GETH_H */
