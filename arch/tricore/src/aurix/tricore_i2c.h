/****************************************************************************
 * arch/tricore/src/common/tricore_i2c.h
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

#ifndef __ARCH_TRICORE_SRC_COMMON_TRICORE_I2C_H
#define __ARCH_TRICORE_SRC_COMMON_TRICORE_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <nuttx/bits.h>
#include <nuttx/mutex.h>
#include <nuttx/i2c/i2c_master.h>

#include "tricore_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TRICORE_I2C_CLC1_OFFSET       0x00
#define TRICORE_I2C_ID1_OFFSET        0x08
#define TRICORE_I2C_RUNCTRL_OFFSET    0x10
#define TRICORE_I2C_ENDDCTRL_OFFSET   0x14
#define TRICORE_I2C_FDIVCFG_OFFSET    0x18
#define TRICORE_I2C_FDIVHIGHCFG_OFFSET 0x1C
#define TRICORE_I2C_ADDRCFG_OFFSET    0x20
#define TRICORE_I2C_BUSSTAT_OFFSET    0x24
#define TRICORE_I2C_FIFOCFG_OFFSET    0x28
#define TRICORE_I2C_MRPSCTRL_OFFSET   0x2C
#define TRICORE_I2C_RPSSTAT_OFFSET    0x30
#define TRICORE_I2C_TPSCTRL_OFFSET    0x34
#define TRICORE_I2C_FFSSTAT_OFFSET    0x38
#define TRICORE_I2C_TIMCFG_OFFSET     0x40

#define TRICORE_I2C_ERRIRQSM_OFFSET   0x60
#define TRICORE_I2C_ERRIRQSS_OFFSET   0x64
#define TRICORE_I2C_ERRIRQSC_OFFSET   0x68

#define TRICORE_I2C_PIRQSM_OFFSET     0x70
#define TRICORE_I2C_PIRQSS_OFFSET     0x74
#define TRICORE_I2C_PIRQSC_OFFSET     0x78

#define TRICORE_I2C_RIS_OFFSET        0x80
#define TRICORE_I2C_IMSC_OFFSET       0x84
#define TRICORE_I2C_MIS_OFFSET        0x88
#define TRICORE_I2C_ICR_OFFSET        0x8C
#define TRICORE_I2C_ISR_OFFSET        0x90

#define TRICORE_I2C_TXD_OFFSET        0x8000
#define TRICORE_I2C_RXD_OFFSET        0xC000

#define I2C_CLC1_DISR               BIT(0)
#define I2C_CLC1_DISS               BIT(1)
#define I2C_CLC1_SPEN               BIT(2)
#define I2C_CLC1_EDIS               BIT(3)
#define I2C_CLC1_SBWE               BIT(4)
#define I2C_CLC1_FSOE               BIT(5)
#define I2C_CLC1_RMC_SHIFT          8
#define I2C_CLC1_RMC_MASK           (0xff << I2C_CLC1_RMC_SHIFT)

#define I2C_CLC_DISR                BIT(0)
#define I2C_CLC_DISS                BIT(1)
#define I2C_CLC_EDIS                BIT(3)

#define I2C_RUNCTRL_RUN             BIT(0)

#define I2C_ENDDCTRL_SETRSC         BIT(0)
#define I2C_ENDDCTRL_SETEND         BIT(1)

#define I2C_FDIVCFG_DEC_SHIFT       0
#define I2C_FDIVCFG_DEC_MASK        (0x7ff << I2C_FDIVCFG_DEC_SHIFT)
#define I2C_FDIVCFG_INC_SHIFT       16
#define I2C_FDIVCFG_INC_MASK        (0xff << I2C_FDIVCFG_INC_SHIFT)

#define I2C_FDIVHIGHCFG_DEC_SHIFT   0
#define I2C_FDIVHIGHCFG_DEC_MASK    (0x7ff << I2C_FDIVHIGHCFG_DEC_SHIFT)
#define I2C_FDIVHIGHCFG_INC_SHIFT   16
#define I2C_FDIVHIGHCFG_INC_MASK    (0xff << I2C_FDIVHIGHCFG_INC_SHIFT)

#define I2C_ADDRCFG_ADR_SHIFT       0
#define I2C_ADDRCFG_ADR_MASK        (0x3ff << I2C_ADDRCFG_ADR_SHIFT)
#define I2C_ADDRCFG_TBAM            BIT(16)
#define I2C_ADDRCFG_GCE             BIT(17)
#define I2C_ADDRCFG_MCE             BIT(18)
#define I2C_ADDRCFG_MNS             BIT(19)
#define I2C_ADDRCFG_SONA            BIT(20)
#define I2C_ADDRCFG_SOPE            BIT(21)

#define I2C_BUSSTAT_BS_SHIFT        0
#define I2C_BUSSTAT_BS_MASK         (0x3 << I2C_BUSSTAT_BS_SHIFT)
#define I2C_BUSSTAT_RNW             BIT(2)

#define I2C_BUSSTAT_IDLE            0
#define I2C_BUSSTAT_STARTED         1
#define I2C_BUSSTAT_BUSYMASTER      2
#define I2C_BUSSTAT_REMOTESLAVE     3

#define I2C_FIFOCFG_RXBS_SHIFT      0
#define I2C_FIFOCFG_RXBS_MASK       (0x3 << I2C_FIFOCFG_RXBS_SHIFT)
#define I2C_FIFOCFG_TXBS_SHIFT      4
#define I2C_FIFOCFG_TXBS_MASK       (0x3 << I2C_FIFOCFG_TXBS_SHIFT)
#define I2C_FIFOCFG_RXFA_SHIFT      8
#define I2C_FIFOCFG_RXFA_MASK       (0x3 << I2C_FIFOCFG_RXFA_SHIFT)
#define I2C_FIFOCFG_TXFA_SHIFT      12
#define I2C_FIFOCFG_TXFA_MASK       (0x3 << I2C_FIFOCFG_TXFA_SHIFT)
#define I2C_FIFOCFG_RXFC            BIT(16)
#define I2C_FIFOCFG_TXFC            BIT(17)
#define I2C_FIFOCFG_CRBC            BIT(18)

#define I2C_MRPSCTRL_MRPS_SHIFT     0
#define I2C_MRPSCTRL_MRPS_MASK      (0x3fff << I2C_MRPSCTRL_MRPS_SHIFT)

#define I2C_TPSCTRL_TPS_SHIFT       0
#define I2C_TPSCTRL_TPS_MASK        (0x3fff << I2C_TPSCTRL_TPS_SHIFT)

#define I2C_FFSSTAT_FFS_SHIFT       0
#define I2C_FFSSTAT_FFS_MASK        (0x3f << I2C_FFSSTAT_FFS_SHIFT)

#define I2C_TIMCFG_SDA_DEL_HD_DAT_SHIFT    0
#define I2C_TIMCFG_SDA_DEL_HD_DAT_MASK     (0x3f << 0)
#define I2C_TIMCFG_HS_SDA_DEL_HD_DAT_SHIFT 6
#define I2C_TIMCFG_HS_SDA_DEL_HD_DAT_MASK  (0x7 << 6)
#define I2C_TIMCFG_SCL_DEL_HD_STA_SHIFT    9
#define I2C_TIMCFG_SCL_DEL_HD_STA_MASK     (0x7 << 9)
#define I2C_TIMCFG_EN_SCL_LOW_LEN          BIT(14)
#define I2C_TIMCFG_FS_SCL_LOW              BIT(15)
#define I2C_TIMCFG_HS_SDA_DEL_SHIFT        16
#define I2C_TIMCFG_HS_SDA_DEL_MASK         (0x1f << 16)
#define I2C_TIMCFG_SCL_LOW_LEN_SHIFT       24
#define I2C_TIMCFG_SCL_LOW_LEN_MASK        (0xff << 24)

#define I2C_ERRIRQ_RXF_UFL          BIT(0)
#define I2C_ERRIRQ_RXF_OFL          BIT(1)
#define I2C_ERRIRQ_TXF_UFL          BIT(2)
#define I2C_ERRIRQ_TXF_OFL          BIT(3)
#define I2C_ERRIRQ_ALL              0x0f

#define I2C_PIRQ_AM                 BIT(0)
#define I2C_PIRQ_GC                 BIT(1)
#define I2C_PIRQ_MC                 BIT(2)
#define I2C_PIRQ_AL                 BIT(3)
#define I2C_PIRQ_NACK               BIT(4)
#define I2C_PIRQ_TX_END             BIT(5)
#define I2C_PIRQ_RX                 BIT(6)
#define I2C_PIRQ_ALL                0x7f

#define I2C_INT_LSREQ               BIT(0)
#define I2C_INT_SREQ                BIT(1)
#define I2C_INT_LBREQ               BIT(2)
#define I2C_INT_BREQ                BIT(3)
#define I2C_INT_ERR                 BIT(4)
#define I2C_INT_P                   BIT(5)
#define I2C_INT_DTR_ALL             (I2C_INT_LSREQ | I2C_INT_SREQ | \
                                     I2C_INT_LBREQ | I2C_INT_BREQ)

#define I2C_GPCTL_PISEL_SHIFT       0
#define I2C_GPCTL_PISEL_MASK        (0x7 << I2C_GPCTL_PISEL_SHIFT)

#define I2C_TIMEOUT_US              100000
#define I2C_CLK_TIMEOUT_US          10000

#define I2C_POLL_INTERVAL_US        1

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

struct tricore_i2c_config_s
{
  uintptr_t       base;
  uint32_t        frequency;
  uint32_t        clk_freq;
  uint8_t         bus;
  uint8_t         pisel;

  uint32_t        clc_offset;
  uint32_t        gpctl_offset;
};

struct tricore_i2c_priv_s
{
  const struct i2c_ops_s              *ops;
  const struct tricore_i2c_config_s   *config;
  mutex_t                              lock;
  uint32_t                             frequency;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int tricore_i2c_set_rate(struct tricore_i2c_priv_s *priv,
                          uint32_t rate);

int  tricore_i2c_hw_init(struct tricore_i2c_priv_s *priv);
void tricore_i2c_hw_deinit(struct tricore_i2c_priv_s *priv);

static inline uint32_t i2c_getreg(struct tricore_i2c_priv_s *priv,
                                  uint32_t offset)
{
  return getreg32(priv->config->base + offset);
}

static inline void i2c_putreg(struct tricore_i2c_priv_s *priv,
                              uint32_t offset, uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

static inline void i2c_modifyreg(struct tricore_i2c_priv_s *priv,
                                 uint32_t offset,
                                 uint32_t clearbits, uint32_t setbits)
{
  uintptr_t addr = priv->config->base + offset;

  modreg32(setbits, setbits | clearbits, addr);
}

extern const struct i2c_ops_s g_tricore_i2c_ops;

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_TRICORE_SRC_COMMON_TRICORE_I2C_H */
