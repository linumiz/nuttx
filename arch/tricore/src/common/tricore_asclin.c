/****************************************************************************
 * arch/tricore/src/common/tricore_asclin.c
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

#include <stdbool.h>
#include <stdint.h>

#include "tricore_asclin.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static float tricore_asclin_abs(float value)
{
  return value < 0.0f ? -value : value;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_asclin_brg
 *
 * Description:
 *   Find the ASCLIN prescaler, oversampling and fractional divider for a
 *   baud rate, with the search of the iLLD IfxAsclin_setBitTiming, so that
 *   the same baud rate gives the same register values.
 *
 * Input Parameters:
 *   fa           - ASCLIN clock frequency in Hz
 *   prescaler    - Current prescaler (BITCON.PRESCALER + 1)
 *   baud         - Baud rate
 *   oversampling - Wanted oversampling, 4 to 16
 *   brg          - Returned setting
 *
 ****************************************************************************/

void tricore_asclin_brg(float fa, uint32_t prescaler, uint32_t baud,
                        uint32_t oversampling,
                        struct tricore_asclin_brg_s *brg)
{
  float fpd = fa / prescaler;
  float baudrate = baud;
  float fovs;
  float relerror;
  float newrelerror;
  float limit;
  float f;
  uint32_t ovs = oversampling < 4 ? 4 : oversampling;
  uint32_t d;
  uint32_t n;
  uint32_t dbest;
  uint32_t nbest;
  uint32_t addl;
  uint32_t addh;
  uint32_t addlmin;
  uint32_t count;
  bool done = false;

  fovs = baudrate * ovs;

  if (baudrate <= 6000)
    {
      limit = 0.01f * fovs;
    }
  else if (baudrate < 100000)
    {
      limit = 0.001f * fovs;
    }
  else if (baudrate < 1000000)
    {
      limit = 0.0001f * fovs;
    }
  else if (baudrate < 8330000)
    {
      limit = 0.00001f * fovs;
    }
  else
    {
      limit = 0.000001f * fovs;
    }

  /* Integer divider, lowering the oversampling or the prescaler if the
   * clock is too slow, raising them if the divider is too large
   */

  d = (uint32_t)(fpd / fovs);
  n = 1;

  while (d < 1)
    {
      if (ovs >= 5)
        {
          ovs--;
          fovs = baudrate * ovs;
          d = (uint32_t)(fpd / fovs);
        }
      else
        {
          prescaler = 1;
          ovs = 16;
          fpd = fa / prescaler;
          fovs = baudrate * ovs;
          d = (uint32_t)(fpd / fovs);
        }
    }

  if (d >> 12)
    {
      if (((uint32_t)(fpd / (baudrate * 16))) >> 12)
        {
          while (d >> 12)
            {
              prescaler++;
              if (prescaler >> 12)
                {
                  prescaler = 0xfff;
                  break;
                }

              fpd = fa / prescaler;
              d = (uint32_t)(fpd / fovs);
            }
        }
      else
        {
          while (d >> 12)
            {
              ovs++;
              if (ovs > 16)
                {
                  ovs = 16;
                  break;
                }

              fovs = baudrate * ovs;
              d = (uint32_t)(fpd / fovs);
            }
        }
    }

  /* Search the fraction n / d closest to the wanted baud rate */

  dbest = d;
  nbest = n;
  addlmin = 0;
  f = (fpd * n) / d;
  relerror = tricore_asclin_abs(fovs - f);
  if (relerror <= limit)
    {
      done = true;
    }

  for (n = 2; !done && n <= 0xfff; n++)
    {
      if (n == 2)
        {
          addl = 1;
          addh = 1;
        }
      else
        {
          addl = (addlmin * n) / nbest;
          addh = addl + 1;
        }

      for (count = addl; count <= addh && nbest <= dbest; count++)
        {
          f = (fpd * n) / (n * d + count);
          newrelerror = tricore_asclin_abs(fovs - f);
          if (relerror > newrelerror)
            {
              relerror = newrelerror;
              nbest = n;
              dbest = n * d + count;
              if (dbest > 4095)
                {
                  dbest = 4095;
                }

              addlmin = count;
            }

          if (relerror <= limit)
            {
              done = true;
              break;
            }
        }
    }

  brg->prescaler    = prescaler;
  brg->oversampling = ovs;
  brg->numerator    = nbest;
  brg->denominator  = dbest;
}
