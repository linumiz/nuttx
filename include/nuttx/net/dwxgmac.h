/****************************************************************************
 * include/nuttx/net/dwxgmac.h
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

#ifndef __INCLUDE_NUTTX_NET_DWXGMAC_H
#define __INCLUDE_NUTTX_NET_DWXGMAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/net/mdio.h>

#ifdef CONFIG_NET_DWXGMAC

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* One Synopsys DesignWare XGMAC, as the chip code describes it to
 * dwxgmac_initialize.
 */

struct dwxgmac_config_s
{
  uintptr_t base;         /* Base address of the MAC registers */
  uintptr_t dmabase;      /* Base address the DMA register offsets are
                           * relative to, 0 when it is the MAC base */
  int       irq;          /* DMA interrupt, or 0 when the chip code
                           * attaches the interrupt and calls
                           * dwxgmac_interrupt */
  uint32_t  txfifo_size;  /* Transmit FIFO size in bytes, 0 to read it
                           * from MAC_HW_FEATURE1 */
  int       speed;        /* Link speed at initialization in Mbps (10,
                           * 100 or 1000), 0 to keep the carrier off
                           * until dwxgmac_carrier */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: dwxgmac_initialize
 *
 * Description:
 *   Initialize the MAC and its DMA and register the network device.  The
 *   chip code calls it once the MAC is clocked and out of reset.
 *
 * Input Parameters:
 *   intf   - Interface number, 0 to CONFIG_NET_DWXGMAC_NINTERFACES - 1
 *   config - Description of the MAC
 *
 * Returned Value:
 *   OK on success, a negated errno value on failure.
 *
 ****************************************************************************/

int dwxgmac_initialize(int intf, FAR const struct dwxgmac_config_s *config);

/****************************************************************************
 * Name: dwxgmac_interrupt
 *
 * Description:
 *   Handle the DMA interrupt of an interface.  The driver attaches it
 *   itself when the interface has an interrupt number; otherwise the chip
 *   code calls it from its own handler, with the interface number cast to
 *   a pointer as the argument.
 *
 ****************************************************************************/

int dwxgmac_interrupt(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Name: dwxgmac_carrier
 *
 * Description:
 *   Report the link state of an interface, as the board reads it from the
 *   PHY.
 *
 * Input Parameters:
 *   intf  - Interface number
 *   up    - True if the link is up
 *   speed - Link speed in Mbps: 10, 100 or 1000
 *
 ****************************************************************************/

void dwxgmac_carrier(int intf, bool up, int speed);

/****************************************************************************
 * Name: dwxgmac_mdio_initialize
 *
 * Description:
 *   Register the MDIO bus of a MAC.
 *
 * Input Parameters:
 *   bus  - MDIO bus number, 0 to CONFIG_NET_DWXGMAC_NINTERFACES - 1
 *   base - Base address of the MAC registers
 *
 * Returned Value:
 *   The MDIO bus, or NULL on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_DWXGMAC_MDIO
FAR struct mdio_bus_s *dwxgmac_mdio_initialize(int bus, uintptr_t base);
#endif

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_DWXGMAC */
#endif /* __INCLUDE_NUTTX_NET_DWXGMAC_H */
