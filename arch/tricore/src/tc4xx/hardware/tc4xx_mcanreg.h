/****************************************************************************
 * arch/tricore/src/tc4xx/hardware/tc4xx_mcanreg.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_TRICORE_SRC_TC4XX_MCMCAN_H
#define __ARCH_TRICORE_SRC_TC4XX_MCMCAN_H

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/can/can.h>
#include <nuttx/can/can_mcan.h>

#define MCMCAN_CLC         0x00
#define MCMCAN_MCR         0x70
#define MCMCAN_GRINT1(n)   (0x12C + n * 0x400)
#define MCMCAN_GRINT2(n)   (0x130 + n * 0x400)
#define MCMCAN_NPCR(n)     (0x14C + n * 0x400)

#endif /* __ARCH_TRICORE_SRC_TC4XX_MCMCAN_H */
