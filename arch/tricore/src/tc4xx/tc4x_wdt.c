/****************************************************************************
 * arch/tricore/src/tc4xx/tc4x_wdt.c
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

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/bits.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/timers/watchdog.h>

#include "tricore_internal.h"
#include "hardware/tc4x_wdt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TC4X_WDT_FSPB   CONFIG_TC4X_WDT_FSPB

#define CTRLA_LCK       BIT(0)
#define CTRLA_PW_MASK   (0x7F << 1)

#define CTRLB_DR        BIT(0)

/* Register addresses */
#define WDT_CPU0_CTRLA  0xF000003C
#define WDT_CPU0_CTRLB  0xF0000040
#define WDT_SYS_CTRLA   0xF00001A8
#define WDT_SYS_CTRLB   0xF00001AC

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_TC4X_WDT
struct tc4x_wdt_lowerhalf_s
{
  const struct watchdog_ops_s *ops;
  uintptr_t  base;
  uint32_t   timeout;
  uint16_t   reload;
  uint8_t    ifsr;
  bool       started;
};

static int tc4x_wdt_start(struct watchdog_lowerhalf_s *lower);
static int tc4x_wdt_stop(struct watchdog_lowerhalf_s *lower);
static int tc4x_wdt_keepalive(struct watchdog_lowerhalf_s *lower);
static int tc4x_wdt_getstatus(struct watchdog_lowerhalf_s *lower,
                              struct watchdog_status_s *status);
static int tc4x_wdt_settimeout(struct watchdog_lowerhalf_s *lower,
                               uint32_t timeout);

static const struct watchdog_ops_s g_tc4x_wdt_ops =
{
  .start      = tc4x_wdt_start,
  .stop       = tc4x_wdt_stop,
  .keepalive  = tc4x_wdt_keepalive,
  .getstatus  = tc4x_wdt_getstatus,
  .settimeout = tc4x_wdt_settimeout,
  .capture    = NULL,
  .ioctl      = NULL,
};

static const uintptr_t g_wdt_base[TC4X_WDT_COUNT] =
{
  TC4X_WDTCPU0_BASE,
  TC4X_WDTCPU1_BASE,
  TC4X_WDTCPU2_BASE,
  TC4X_WDTCPU3_BASE,
  TC4X_WDTCPU4_BASE,
  TC4X_WDTCPU5_BASE,
  TC4X_WDTSYS_BASE,
};

static const uint32_t g_wdt_dividers[] =
{
  WDT_DIVIDER_16384,
  WDT_DIVIDER_256,
  WDT_DIVIDER_64,
};

#endif

static void tc4x_wdt_disable(uintptr_t ctrla_addr, uintptr_t ctrlb_addr)
{
  uint32_t ctrla;

  /* Unlock: read CTRLA, invert PW, clear LCK */
  ctrla = getreg32(ctrla_addr);
  if (ctrla & CTRLA_LCK)
    {
      ctrla &= ~CTRLA_LCK;
      ctrla ^= CTRLA_PW_MASK;
      putreg32(ctrla, ctrla_addr);
    }

  /* Set disable request */
  putreg32(CTRLB_DR, ctrlb_addr);

  /* Lock to apply */
  ctrla = getreg32(ctrla_addr);
  ctrla |= CTRLA_LCK;
  putreg32(ctrla, ctrla_addr);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void tricore_wdt_disable(void)
{
  tc4x_wdt_disable(WDT_SYS_CTRLA, WDT_SYS_CTRLB);
  tc4x_wdt_disable(WDT_CPU0_CTRLA, WDT_CPU0_CTRLB);
}

#ifdef CONFIG_TC4X_WDT

static inline uint32_t tc4x_wdt_getreg(struct tc4x_wdt_lowerhalf_s *priv,
                                        uint32_t offset)
{
  return getreg32(priv->base + offset);
}

static inline void tc4x_wdt_putreg(struct tc4x_wdt_lowerhalf_s *priv,
                                    uint32_t offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

static uint32_t tc4x_wdt_dump_stat(struct tc4x_wdt_lowerhalf_s *priv,
                                    const char *tag)
{
  uint32_t stat = tc4x_wdt_getreg(priv, TC4X_WDT_STAT_OFFSET);

  wdinfo("[%s] STAT=0x%08lx D=%d TOM=%d OE=%d AE=%d IFS=%d "
         "TIM=0x%04lx\n",
         tag,
         (unsigned long)stat,
         (int)(stat & WDT_STAT_D ? 1 : 0),
         (int)(stat & WDT_STAT_TOM ? 1 : 0),
         (int)(stat & WDT_STAT_OE ? 1 : 0),
         (int)(stat & WDT_STAT_AE ? 1 : 0),
         (int)((stat & WDT_STAT_IFS_MASK) >> WDT_STAT_IFS_SHIFT),
         (unsigned long)((stat & WDT_STAT_TIM_MASK) >> WDT_STAT_TIM_SHIFT));

  return stat;
}

static void tc4x_wdt_unlock(struct tc4x_wdt_lowerhalf_s *priv)
{
  uint32_t ctrla;

  ctrla = tc4x_wdt_getreg(priv, TC4X_WDT_CTRLA_OFFSET);

  if (ctrla & WDT_CTRLA_LCK)
    {
      ctrla ^= WDT_CTRLA_PW_LOW_MASK;
      ctrla &= ~WDT_CTRLA_LCK;

      tc4x_wdt_putreg(priv, TC4X_WDT_CTRLA_OFFSET, ctrla);
    }
}

static void tc4x_wdt_lock(struct tc4x_wdt_lowerhalf_s *priv)
{
  uint32_t ctrla;

  ctrla = tc4x_wdt_getreg(priv, TC4X_WDT_CTRLA_OFFSET);
  ctrla |= WDT_CTRLA_LCK;

  tc4x_wdt_putreg(priv, TC4X_WDT_CTRLA_OFFSET, ctrla);
}

static void tc4x_wdt_configure(struct tc4x_wdt_lowerhalf_s *priv,
                                bool enable)
{
  uint32_t ctrlb = 0;

  if (!enable)
    {
      ctrlb |= WDT_CTRLB_DR;
    }

  ctrlb |= ((uint32_t)priv->ifsr << WDT_CTRLB_IFSR_SHIFT) &
            WDT_CTRLB_IFSR_MASK;
  ctrlb |= ((uint32_t)priv->reload << WDT_CTRLB_TIMR_SHIFT) &
            WDT_CTRLB_TIMR_MASK;

  tc4x_wdt_putreg(priv, TC4X_WDT_CTRLB_OFFSET, ctrlb);
}

static int tc4x_wdt_calc_timeout(uint32_t timeout_ms, uint16_t *reload,
                                  uint8_t *ifsr)
{
  uint64_t ticks;
  int i;

  static const uint8_t try_ifsr[3] =
  {
    WDT_IFS_DIV64, WDT_IFS_DIV256, WDT_IFS_DIV16384
  };

  for (i = 0; i < 3; i++)
    {
      uint32_t div = g_wdt_dividers[try_ifsr[i]];

      ticks = (uint64_t)timeout_ms * (TC4X_WDT_FSPB / 1000) / div;

      if (ticks > 0 && ticks <= WDT_TIMER_MAX)
        {
          *reload = (uint16_t)(WDT_TIMER_MAX - ticks);
          *ifsr   = try_ifsr[i];
          return OK;
        }
    }

  return -ERANGE;
}

static uint32_t tc4x_wdt_calc_timeleft(struct tc4x_wdt_lowerhalf_s *priv)
{
  uint32_t stat;
  uint32_t tim;
  uint32_t ifs;
  uint32_t divider;
  uint64_t remaining;

  stat = tc4x_wdt_getreg(priv, TC4X_WDT_STAT_OFFSET);
  tim  = (stat & WDT_STAT_TIM_MASK) >> WDT_STAT_TIM_SHIFT;
  ifs  = (stat & WDT_STAT_IFS_MASK) >> WDT_STAT_IFS_SHIFT;

  if (ifs > 2)
    {
      return 0;
    }

  divider = g_wdt_dividers[ifs];
  remaining = (uint64_t)(WDT_TIMER_MAX - tim) * divider * 1000 / TC4X_WDT_FSPB;

  return (uint32_t)remaining;
}

static int tc4x_wdt_start(struct watchdog_lowerhalf_s *lower)
{
  struct tc4x_wdt_lowerhalf_s *priv =
    (struct tc4x_wdt_lowerhalf_s *)lower;
  irqstate_t flags;

  wdinfo("Entry\n");

  flags = enter_critical_section();

  tc4x_wdt_unlock(priv);
  tc4x_wdt_configure(priv, true);
  tc4x_wdt_lock(priv);
  priv->started = true;

  tc4x_wdt_dump_stat(priv, "start");

  leave_critical_section(flags);

  return OK;
}

static int tc4x_wdt_stop(struct watchdog_lowerhalf_s *lower)
{
  struct tc4x_wdt_lowerhalf_s *priv =
    (struct tc4x_wdt_lowerhalf_s *)lower;
  irqstate_t flags;

  wdinfo("Entry\n");

  priv->started = false;

  flags = enter_critical_section();

  tc4x_wdt_unlock(priv);
  tc4x_wdt_configure(priv, false);
  tc4x_wdt_lock(priv);

  tc4x_wdt_dump_stat(priv, "stop");

  leave_critical_section(flags);

  return OK;
}

static int tc4x_wdt_keepalive(struct watchdog_lowerhalf_s *lower)
{
  struct tc4x_wdt_lowerhalf_s *priv =
    (struct tc4x_wdt_lowerhalf_s *)lower;
  irqstate_t flags;

  flags = enter_critical_section();

  tc4x_wdt_unlock(priv);
  tc4x_wdt_lock(priv);

  leave_critical_section(flags);

  return OK;
}

static int tc4x_wdt_getstatus(struct watchdog_lowerhalf_s *lower,
                               struct watchdog_status_s *status)
{
  struct tc4x_wdt_lowerhalf_s *priv =
    (struct tc4x_wdt_lowerhalf_s *)lower;
  uint32_t stat;

  DEBUGASSERT(status != NULL);

  stat = tc4x_wdt_getreg(priv, TC4X_WDT_STAT_OFFSET);

  status->flags = 0;
  if (!(stat & WDT_STAT_D))
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

  status->flags |= WDFLAGS_RESET;
  status->timeout  = priv->timeout;
  status->timeleft = tc4x_wdt_calc_timeleft(priv);

  return OK;
}

static int tc4x_wdt_settimeout(struct watchdog_lowerhalf_s *lower,
                                uint32_t timeout)
{
  struct tc4x_wdt_lowerhalf_s *priv =
    (struct tc4x_wdt_lowerhalf_s *)lower;
  uint16_t reload;
  uint8_t  ifsr;
  irqstate_t flags;
  int ret;

  wdinfo("timeout=%lu\n", (unsigned long)timeout);

  ret = tc4x_wdt_calc_timeout(timeout, &reload, &ifsr);
  if (ret < 0)
    {
      wderr("Timeout %lu ms out of range\n", (unsigned long)timeout);
      return ret;
    }

  wdinfo("Calculated: reload=0x%04x ifsr=%d\n", reload, ifsr);

  flags = enter_critical_section();

  priv->reload  = reload;
  priv->ifsr    = ifsr;
  priv->timeout = timeout;

  if (priv->started)
    {
      tc4x_wdt_unlock(priv);
      tc4x_wdt_configure(priv, true);
      tc4x_wdt_lock(priv);

      tc4x_wdt_dump_stat(priv, "settimeout-live");
    }

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int tc4x_wdt_initialize(const char *devpath, int wdtid)
{
  struct tc4x_wdt_lowerhalf_s *priv;
  irqstate_t flags;

  DEBUGASSERT(devpath != NULL);
  DEBUGASSERT(wdtid >= 0 && wdtid < TC4X_WDT_COUNT);

  wdinfo("devpath=%s wdtid=%d base=0x%08lx\n",
         devpath, wdtid, (unsigned long)g_wdt_base[wdtid]);

  priv = kmm_zalloc(sizeof(struct tc4x_wdt_lowerhalf_s));
  if (priv == NULL)
    {
      wderr("Failed to allocate driver structure\n");
      return -ENOMEM;
    }

  priv->ops     = &g_tc4x_wdt_ops;
  priv->base    = g_wdt_base[wdtid];
  priv->started = false;

  priv->reload  = 0;
  priv->ifsr    = WDT_IFS_DIV16384;
  priv->timeout = (uint32_t)((uint64_t)WDT_TIMER_MAX * WDT_DIVIDER_16384 * 1000 /
                              TC4X_WDT_FSPB);

  flags = enter_critical_section();

  tc4x_wdt_dump_stat(priv, "pre-init");

  tc4x_wdt_unlock(priv);

  tc4x_wdt_dump_stat(priv, "unlocked");

  tc4x_wdt_configure(priv, false);
  tc4x_wdt_lock(priv);

  tc4x_wdt_dump_stat(priv, "post-init-disabled");

  leave_critical_section(flags);

  if (watchdog_register(devpath,
                        (struct watchdog_lowerhalf_s *)priv) == NULL)
    {
      wderr("watchdog_register failed: %s\n", devpath);
      kmm_free(priv);
      return -ENODEV;
    }

  return OK;
}

#endif /* CONFIG_TC4X_WDT */
