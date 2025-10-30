/****************************************************************************
 * arch/tricore/src/common/tricore_irq.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include "tricore_irq.h"
#include "tricore_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irq_enable(void)
{
	__asm__ __volatile__ ("enable" : : : "memory");
}

void up_irqinitialize(void)
{
	__asm__ __volatile__ ("enable" : : : "memory");
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
	SRCR reg = {.U = getreg32(IFX_IR_GET_SRC(irq))};
	reg.B.SRE = 0;
	putreg32(reg.U, IFX_IR_GET_SRC(irq));
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
	SRCR reg = {.U = getreg32(IFX_IR_GET_SRC(irq))};

	reg.B.CLRR = 1,
	reg.B.IOVCLR = 1,
	reg.B.SRE = 1;
	if (reg.B.SRPN == 0x0)
		reg.B.SRPN = TRICORE_DEFAULT_IR_PRIO;
	if (reg.B.TOS == 0xf)
		reg.B.TOS = TRICORE_DEFAULT_IR_TOS;

	putreg32(reg.U, IFX_IR_GET_SRC(irq));
}

#ifdef CONFIG_ARCH_HAVE_IRQTRIGGER

/****************************************************************************
 * Name: up_trigger_irq
 *
 * Description:
 *   Trigger an IRQ by software.
 *
 ****************************************************************************/

void up_trigger_irq(int irq, cpu_set_t cpuset)
{
	// GPRS - mbox
	(void) cpuset;
	SRCR reg = {.U = getreg32(IFX_IR_GET_SRC(irq))};

	reg.B.SETR = 1;

	putreg32(reg.U, IFX_IR_GET_SRC(irq));
}

#endif

/****************************************************************************
 * Name: up_affinity_irq
 *
 * Description:
 *   Set an IRQ affinity by software.
 *
 ****************************************************************************/

void up_affinity_irq(int irq, cpu_set_t cpuset)
{
	SRCR reg = {.U = getreg32(IFX_IR_GET_SRC(irq))};

	up_disable_irq(irq);
	// FIXME ffs(cpuset)
	reg.B.TOS = cpuset;

	putreg32(reg.U, IFX_IR_GET_SRC(irq));
	up_enable_irq(irq);
}

int up_prioritize_irq(int irq, int priority)
{
	SRCR reg = {.U = getreg32(IFX_IR_GET_SRC(irq))};

	reg.B.SRPN = priority;

	putreg32(reg.U, IFX_IR_GET_SRC(irq));
}
