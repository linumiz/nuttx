/****************************************************************************
 * arch/tricore/src/aurix/tricore_mcmcan.c
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

#ifndef __DRIVERS_CAN_CAN_MCAN_H
#define __DRIVERS_CAN_CAN_MCAN_H

#include <nuttx/config.h>

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include <nuttx/arch.h>
#include <nuttx/bits.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#ifndef FIELD_GET
#  define FIELD_GET(mask, reg) \
     (((uint32_t)(reg) & (uint32_t)(mask)) >> (__builtin_ctzl(mask)))
#  define FIELD_PREP(mask, val) \
     (((uint32_t)(val) << (__builtin_ctzl(mask))) & (uint32_t)(mask))
#endif

typedef uint32_t can_mode_t;

#define CAN_MODE_NORMAL          0u
#define CAN_MODE_LOOPBACK        BIT(0)
#define CAN_MODE_LISTENONLY      BIT(1)
#define CAN_MODE_FD              BIT(2)
#define CAN_MODE_MANUAL_RECOVERY BIT(3)

#define CAN_FRAME_IDE  BIT(0)  /* extended ID */
#define CAN_FRAME_RTR  BIT(1)  /* remote frame */
#define CAN_FRAME_FDF  BIT(2)  /* FD frame */
#define CAN_FRAME_BRS  BIT(3)  /* bit rate switch */
#define CAN_FRAME_ESI  BIT(4)  /* error state indicator */

#define CAN_FILTER_IDE  BIT(0)

#define CAN_STD_ID_MASK  0x7ffu
#define CAN_MAX_DLC      8u
#define CANFD_MAX_DLC    15u

enum can_state_e
{
  CAN_STATE_ERROR_ACTIVE  = 0,
  CAN_STATE_ERROR_WARNING = 1,
  CAN_STATE_ERROR_PASSIVE = 2,
  CAN_STATE_BUS_OFF       = 3,
  CAN_STATE_STOPPED       = 4
};

struct can_bus_err_cnt_s
{
  uint8_t tx_err_cnt;
  uint8_t rx_err_cnt;
};

struct can_frame_s
{
  uint32_t id;
  uint8_t  dlc;
  uint8_t  flags;
  union
  {
    uint8_t  data[64];
    uint32_t data_32[16];
  };
#ifdef CONFIG_CAN_RX_TIMESTAMP
  uint16_t timestamp;
#endif
};

struct can_filter_s
{
  uint32_t id;
  uint32_t mask;
  uint8_t  flags;
};

struct can_timing_s
{
  uint16_t sjw;
  uint16_t prop_seg;
  uint16_t phase_seg1;
  uint16_t phase_seg2;
  uint16_t prescaler;
};

/* Timing range macros - used when registering timing constraints */
#define CAN_MCAN_TIMING_MIN_INITIALIZER \
  { .sjw = 1, .prop_seg = 0, .phase_seg1 = 2,   .phase_seg2 = 2,   .prescaler = 1   }
#define CAN_MCAN_TIMING_MAX_INITIALIZER \
  { .sjw = 128, .prop_seg = 0, .phase_seg1 = 256, .phase_seg2 = 128, .prescaler = 512 }
#define CAN_MCAN_TIMING_DATA_MIN_INITIALIZER \
  { .sjw = 1, .prop_seg = 0, .phase_seg1 = 1,  .phase_seg2 = 1,  .prescaler = 1  }
#define CAN_MCAN_TIMING_DATA_MAX_INITIALIZER \
  { .sjw = 16, .prop_seg = 0, .phase_seg1 = 32, .phase_seg2 = 16, .prescaler = 32 }

/* Message RAM layout indexes (needed to populate can_mcan_config)    */
#define CAN_MCAN_MRAM_CFG_OFFSET        0
#define CAN_MCAN_MRAM_CFG_STD_FILTER    1
#define CAN_MCAN_MRAM_CFG_EXT_FILTER    2
#define CAN_MCAN_MRAM_CFG_RX_FIFO0      3
#define CAN_MCAN_MRAM_CFG_RX_FIFO1      4
#define CAN_MCAN_MRAM_CFG_RX_BUFFER     5
#define CAN_MCAN_MRAM_CFG_TX_EVENT_FIFO 6
#define CAN_MCAN_MRAM_CFG_TX_BUFFER     7
#define CAN_MCAN_MRAM_CFG_NUM_CELLS     8

struct can_mcan_priv_s;

typedef void (*can_tx_callback_t)(struct can_mcan_priv_s *priv,
                                  int error, void *user_data);

typedef void (*can_rx_callback_t)(struct can_mcan_priv_s *priv,
                                  const struct can_frame_s *frame,
                                  void *user_data);

typedef void (*can_state_change_callback_t)(struct can_mcan_priv_s *priv,
                                            enum can_state_e state,
                                            struct can_bus_err_cnt_s err_cnt,
                                            void *user_data);

typedef int (*can_mcan_read_reg_t)(struct can_mcan_priv_s *priv,
                                   uint16_t reg, uint32_t *val);
typedef int (*can_mcan_write_reg_t)(struct can_mcan_priv_s *priv,
                                    uint16_t reg, uint32_t val);
typedef int (*can_mcan_read_mram_t)(struct can_mcan_priv_s *priv,
                                    uint16_t offset, void *dst, size_t len);
typedef int (*can_mcan_write_mram_t)(struct can_mcan_priv_s *priv,
                                     uint16_t offset, const void *src,
                                     size_t len);
typedef int (*can_mcan_clear_mram_t)(struct can_mcan_priv_s *priv,
                                     uint16_t offset, size_t len);

struct can_mcan_ops
{
  can_mcan_read_reg_t   read_reg;
  can_mcan_write_reg_t  write_reg;
  can_mcan_read_mram_t  read_mram;
  can_mcan_write_mram_t write_mram;
  can_mcan_clear_mram_t clear_mram;
};

struct can_mcan_tx_callback
{
  can_tx_callback_t function;
  void             *user_data;
};

struct can_mcan_rx_callback
{
  can_rx_callback_t function;
  void             *user_data;
};

struct can_mcan_callbacks
{
  struct can_mcan_tx_callback *tx;
  struct can_mcan_rx_callback *std;
  struct can_mcan_rx_callback *ext;
  uint8_t num_tx;
  uint8_t num_std;
  uint8_t num_ext;
};

#define CAN_MCAN_CALLBACKS_DEFINE(_name, _tx, _std, _ext)          \
  static struct can_mcan_tx_callback _name##_tx_cbs[(_tx)];        \
  static struct can_mcan_rx_callback _name##_std_cbs[(_std)];      \
  static struct can_mcan_rx_callback _name##_ext_cbs[(_ext)];      \
  static const struct can_mcan_callbacks _name = {                 \
    .tx = _name##_tx_cbs, .std = _name##_std_cbs,                  \
    .ext = _name##_ext_cbs,                                        \
    .num_tx = (_tx), .num_std = (_std), .num_ext = (_ext),         \
  }

struct can_mcan_config
{
  const struct can_mcan_ops       *ops;
  const struct can_mcan_callbacks *callbacks;

  uint32_t bitrate;        /* nominal bitrate, Hz */
  uint16_t sample_point;   /* nominal sample point, per mille */
#ifdef CONFIG_CAN_FD_MODE
  uint32_t bitrate_data;
  uint16_t sample_point_data;
#endif

  bool     phy_present;

  uint16_t mram_elements[CAN_MCAN_MRAM_CFG_NUM_CELLS];
  uint16_t mram_offsets[CAN_MCAN_MRAM_CFG_NUM_CELLS];
  size_t   mram_size;

#ifdef CONFIG_CAN_RX_TIMESTAMP
  uint8_t  timestamp_prescaler;
#endif
};

struct can_mcan_data
{
  mutex_t    lock;
  mutex_t    tx_mtx;
  sem_t      tx_sem;

  can_mode_t mode;
  bool       started;

  can_state_change_callback_t state_change_cb;
  void                       *state_change_cb_user_data;
} __attribute__((aligned(4)));

/* Handle - SoC driver embeds this in its priv struct.
 * Recover with container_of(priv, soc_priv_t, mcan).
 */

struct can_mcan_priv_s
{
  const struct can_mcan_config *config;
  struct can_mcan_data         *data;
};

static inline int can_mcan_sys_read_reg(uintptr_t base, uint16_t reg,
                                         uint32_t *val)
{
  *val = getreg32(base + reg);
  return 0;
}

static inline int can_mcan_sys_write_reg(uintptr_t base, uint16_t reg,
                                          uint32_t val)
{
  putreg32(val, base + reg);
  return 0;
}

static inline int can_mcan_sys_read_mram(uintptr_t base, uint16_t offset,
                                          void *dst, size_t len)
{
  volatile uint32_t *src32 = (volatile uint32_t *)(base + offset);
  uint32_t          *dst32 = (uint32_t *)dst;
  size_t             len32 = len / sizeof(uint32_t);

  DEBUGASSERT((base % 4) == 0 && (offset % 4) == 0);
  DEBUGASSERT(((uintptr_t)dst % 4) == 0 && (len % 4) == 0);

#ifdef CONFIG_DCACHE
  up_invalidate_dcache(base + offset, base + offset + len);
#endif
  while (len32-- > 0) *dst32++ = *src32++;
  return 0;
}

static inline int can_mcan_sys_write_mram(uintptr_t base, uint16_t offset,
                                           const void *src, size_t len)
{
  volatile uint32_t *dst32 = (volatile uint32_t *)(base + offset);
  const uint32_t    *src32 = (const uint32_t *)src;
  size_t             len32 = len / sizeof(uint32_t);

  DEBUGASSERT((base % 4) == 0 && (offset % 4) == 0);
  DEBUGASSERT(((uintptr_t)src % 4) == 0 && (len % 4) == 0);

  while (len32-- > 0) *dst32++ = *src32++;
#ifdef CONFIG_DCACHE
  up_clean_dcache(base + offset, base + offset + len);
#endif
  return 0;
}

static inline int can_mcan_sys_clear_mram(uintptr_t base, uint16_t offset,
                                           size_t len)
{
  volatile uint32_t *dst32 = (volatile uint32_t *)(base + offset);
  size_t             len32 = len / sizeof(uint32_t);

  DEBUGASSERT((base % 4) == 0 && (offset % 4) == 0 && (len % 4) == 0);

  while (len32-- > 0) *dst32++ = 0;
#ifdef CONFIG_DCACHE
  up_clean_dcache(base + offset, base + offset + len);
#endif
  return 0;
}

static inline int can_mcan_read_mram(struct can_mcan_priv_s *priv,
                                      uint16_t offset, void *dst, size_t len)
{
  return priv->config->ops->read_mram(priv, offset, dst, len);
}

static inline int can_mcan_write_mram(struct can_mcan_priv_s *priv,
                                       uint16_t offset, const void *src,
                                       size_t len)
{
  return priv->config->ops->write_mram(priv, offset, src, len);
}

static inline int can_mcan_clear_mram(struct can_mcan_priv_s *priv,
                                       uint16_t offset, size_t len)
{
  return priv->config->ops->clear_mram(priv, offset, len);
}

/* Public API */

int  can_mcan_read_reg(struct can_mcan_priv_s *priv, uint16_t reg,
                       uint32_t *val);
int  can_mcan_write_reg(struct can_mcan_priv_s *priv, uint16_t reg,
                        uint32_t val);
void can_mcan_enable_configuration_change(struct can_mcan_priv_s *priv);

int  can_mcan_configure_mram(struct can_mcan_priv_s *priv, uintptr_t mrba,
                              uintptr_t mram);
int  can_mcan_init(struct can_mcan_priv_s *priv);

void can_mcan_line_0_isr(struct can_mcan_priv_s *priv);
void can_mcan_line_1_isr(struct can_mcan_priv_s *priv);

int  can_mcan_get_capabilities(struct can_mcan_priv_s *priv, can_mode_t *cap);
int  can_mcan_start(struct can_mcan_priv_s *priv);
int  can_mcan_stop(struct can_mcan_priv_s *priv);
int  can_mcan_set_mode(struct can_mcan_priv_s *priv, can_mode_t mode);

int  can_mcan_set_timing(struct can_mcan_priv_s *priv,
                         const struct can_timing_s *timing);
#ifdef CONFIG_CAN_FD_MODE
int  can_mcan_set_timing_data(struct can_mcan_priv_s *priv,
                               const struct can_timing_s *timing_data);
#endif

int  can_mcan_get_state(struct can_mcan_priv_s *priv,
                        enum can_state_e *state,
                        struct can_bus_err_cnt_s *err_cnt);
void can_mcan_set_state_change_callback(struct can_mcan_priv_s *priv,
                                        can_state_change_callback_t cb,
                                        void *user_data);

void can_mcan_rxint(struct can_mcan_priv_s *priv, bool enable);
void can_mcan_txint(struct can_mcan_priv_s *priv, bool enable);

bool can_mcan_txready(struct can_mcan_priv_s *priv);
bool can_mcan_txempty(struct can_mcan_priv_s *priv);

int  can_mcan_get_timing(struct can_mcan_priv_s *priv,
                         struct can_timing_s *timing);

int  can_mcan_get_connmodes(struct can_mcan_priv_s *priv,
                             bool *loopback, bool *silent);

#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
int  can_mcan_recover(struct can_mcan_priv_s *priv, uint32_t timeout_ms);
#endif

int  can_mcan_send(struct can_mcan_priv_s *priv,
                   const struct can_frame_s *frame, uint32_t timeout_ms,
                   can_tx_callback_t callback, void *user_data);

int  can_mcan_get_max_filters(struct can_mcan_priv_s *priv, bool ide);
int  can_mcan_add_rx_filter(struct can_mcan_priv_s *priv,
                             can_rx_callback_t callback, void *user_data,
                             const struct can_filter_s *filter);
void can_mcan_remove_rx_filter(struct can_mcan_priv_s *priv, int filter_id);

#endif /* __DRIVERS_CAN_CAN_MCAN_H */
