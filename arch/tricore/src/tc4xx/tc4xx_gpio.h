/****************************************************************************
 * arch/tricore/src/tc4xx/tc4xx_gpio.h
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

#ifndef __ARCH/TRICORE/SRC/TC4XX/TC4XX_GPIO_H
#define __ARCH/TRICORE/SRC/TC4XX/TC4XX_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/bits.h>
#include <nuttx/config.h>

#include "tricore_internal.h"

#define AURIX_PORT_BASE			(0xF003A000)

#define TC4X_PORT_ADDR(n)		(AURIX_PORT_BASE + (n * 0x400))

#define TC4X_PORTn_ID(n)		(TC4X_PORT_ADDR(n) + 0x00)
#define TC4X_PORTn_WKUP_EN(n)		(TC4X_PORT_ADDR(n) + 0x10)
#define TC4X_PORTn_WKUP_STATUS(n)	(TC4X_PORT_ADDR(n) + 0x14)
#define TC4X_PORTn_WKUP_STATUS_CLR(n)	(TC4X_PORT_ADDR(n) + 0x1c)
#define TC4X_PORTn_OUT(n)		(TC4X_PORT_ADDR(n) + 0x20)
#define TC4X_PORTn_IN(n)		(TC4X_PORT_ADDR(n) + 0x24)
#define TC4X_PORTn_HWSEL(n)		(TC4X_PORT_ADDR(n) + 0x28)
#define TC4X_PORTn_PCRSEL(n)		(TC4X_PORT_ADDR(n) + 0x34)
#define TC4X_PORTn_OMR(n)		(TC4X_PORT_ADDR(n) + 0x3c)
#define TC4X_PORTn_CCR(n, pin)		(TC4X_PORT_ADDR(n) + 0x304 + (pin * 0x10))
#define TC4X_PORTn_LDO(n)		(TC4X_PORT_ADDR(n) + 0x190)

#define TC4X_PORT_CFG_CTRL_DIR BIT(0)
#define TC4X_PORT_OMR_CLR_SHIFT	(16)

/* GPIO Port Number
 *
 */
#define GPIO_PORT_SHIFT		(14)      /* Bits 21-14: Pin mode */
#define GPIO_PORT_MASK		GENMASK(21, 14)
#define GPIO_PORT0		0
#define GPIO_PORT1		1
#define GPIO_PORT2		2
#define GPIO_PORT3		3
#define GPIO_PORT4		4
#define GPIO_PORT5		5
#define GPIO_PORT6		6
#define GPIO_PORT7		7
#define GPIO_PORT8		8
#define GPIO_PORT9		9
#define GPIO_PORT10		10
#define GPIO_PORT11		11
#define GPIO_PORT12		12
#define GPIO_PORT13		13
#define GPIO_PORT14		14
#define GPIO_PORT15		15
#define GPIO_PORT16		16
#define GPIO_PORT17		17
#define GPIO_PORT18		18
#define GPIO_PORT19		19
#define GPIO_PORT20		20
#define GPIO_PORT21		21
#define GPIO_PORT22		22
#define GPIO_PORT23		23
#define GPIO_PORT24		24
#define GPIO_PORT25		25
#define GPIO_PORT26		26
#define GPIO_PORT27		27
#define GPIO_PORT28		28
#define GPIO_PORT29		29
#define GPIO_PORT30		30
#define GPIO_PORT31		31
#define GPIO_PORT32		32
#define GPIO_PORT33		33
#define GPIO_PORT34		34
#define GPIO_PORT35		35
#define GPIO_PORT36		36
#define GPIO_PORT37		37
#define GPIO_PORT38		38
#define GPIO_PORT39		39


/* GPIO input level:
 */

#define GPIO_INPUT_LEVEL_SHIFT        (12)      /* Bits 13-12: Pad input level */
#define GPIO_INPUT_LEVEL_MASK         GENMASK(13, 12)
#define AURIX_PAD_IN_LEVEL_AL         (0u << GPIO_INPUT_LEVEL_SHIFT)
#define AURIX_PAD_IN_LEVEL_TLL_5V     (2u << GPIO_INPUT_LEVEL_SHIFT)
#define AURIX_PAD_IN_LEVEL_TLL_3V3    (3u << GPIO_INPUT_LEVEL_SHIFT)

/* Input/Output Selection:
 */

#define GPIO_MODE_SHIFT        (8)      /* Bits 9-8: Pin mode */
#define GPIO_MODE_MASK         GENMASK(9, 8)
#  define GPIO_INPUT           (0u << GPIO_MODE_SHIFT) /* GPIO input */
#  define GPIO_OUTPUT          (1u << GPIO_MODE_SHIFT) /* GPIO output */
#  define GPIO_PERIPH          (2u << GPIO_MODE_SHIFT) /* Peripheral */
#  define GPIO_PERIPH_OWN_PAD  (3u << GPIO_MODE_SHIFT) /* Peripheral directly control pad */


/* PAD Configurations Selection:
 */

#define GPIO_PAD_CONFIG_SHIFT           (4)      /* Bits 7-4: Pad config */
#define GPIO_PAD_CONFIG_MASK            GENMASK(7, 4)
#define GPIO_PAD_CONFIG_IN_TRISTATE	(0x0)
#define GPIO_PAD_CONFIG_IN_PULLDOWN	(0x1)
#define GPIO_PAD_CONFIG_IN_PULLUP	(0x2)

#define GPIO_PAD_CONFIG_OUT_GPIO	(0x0)
#define GPIO_PAD_CONFIG_OUT_ALT01	(0x1)
#define GPIO_PAD_CONFIG_OUT_ALT02	(0x2)
#define GPIO_PAD_CONFIG_OUT_ALT03	(0x3)
#define GPIO_PAD_CONFIG_OUT_ALT04	(0x4)
#define GPIO_PAD_CONFIG_OUT_ALT05	(0x5)
#define GPIO_PAD_CONFIG_OUT_ALT06	(0x6)
#define GPIO_PAD_CONFIG_OUT_ALT07	(0x7)
#define GPIO_PAD_CONFIG_OUT_ALT08	(0x8)
#define GPIO_PAD_CONFIG_OUT_ALT09	(0x9)
#define GPIO_PAD_CONFIG_OUT_ALT10	(0xA)
#define GPIO_PAD_CONFIG_OUT_ALT11	(0xB)
#define GPIO_PAD_CONFIG_OUT_ALT12	(0xC)
#define GPIO_PAD_CONFIG_OUT_ALT13	(0xD)
#define GPIO_PAD_CONFIG_OUT_ALT14	(0xE)
#define GPIO_PAD_CONFIG_OUT_ALT15	(0xF)

/* GPIO Pin Number:
 *
 */
#define GPIO_PIN_SHIFT       (0)    /* Bits 3-0: GPIO pin number */
#define GPIO_PIN_MASK        GENMASK(3, 0)  /* Bits 3-0: GPIO pin number */
#define GPIO_PIN0            (0)  /* Pin  0 */
#define GPIO_PIN1            (1)  /* Pin  1 */
#define GPIO_PIN2            (2)  /* Pin  2 */
#define GPIO_PIN3            (3)  /* Pin  3 */
#define GPIO_PIN4            (4)  /* Pin  4 */
#define GPIO_PIN5            (5)  /* Pin  5 */
#define GPIO_PIN6            (6)  /* Pin  6 */
#define GPIO_PIN7            (7)  /* Pin  7 */
#define GPIO_PIN8            (8)  /* Pin  8 */
#define GPIO_PIN9            (9)  /* Pin  9 */
#define GPIO_PIN10           (10) /* Pin 10 */
#define GPIO_PIN11           (11) /* Pin 11 */
#define GPIO_PIN12           (12) /* Pin 12 */
#define GPIO_PIN13           (13) /* Pin 13 */
#define GPIO_PIN14           (14) /* Pin 14 */
#define GPIO_PIN15           (15) /* Pin 15 */

#define GPIO_PAD_CFG(port, pin, mode, cfg)	((cfg << GPIO_PAD_CONFIG_SHIFT) | mode | (port << GPIO_PORT_SHIFT) | (pin & 0xF))
#define GPIO_PIN(x)		(x & 0xF)
#define GPIO_PORT(x)		((x & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT)

typedef uint32_t gpio_pinset_t;

bool aurix_gpio_read(gpio_pinset_t pinset);
void aurix_gpio_write(gpio_pinset_t pinset, bool value);
int aurix_config_gpio(gpio_pinset_t pinset);

#endif /* __ARCH/TRICORE/SRC/TC4XX/TC4XX_GPIO_H */
