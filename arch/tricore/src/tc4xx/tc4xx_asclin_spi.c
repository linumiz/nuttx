/****************************************************************************
 * arch/tricore/src/tc4xx/tc4xx_asclin_spi.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>

#include "tricore_internal.h"
#include "tricore_asclin.h"
#include "tc4xx_gpio.h"
#include "hardware/tc4xx_asclin_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Additional ASCLIN register bits not in the UART header but needed for SPI.
 * FLAGS.OMT = Operation Mode Transition (bit 11) - set on mode switch
 * FLAGS.TC  = Transfer Complete (bit 17) - already defined
 * FLAGSCLEAR.OMTC = bit 11
 * FLAGSSET.SWTRGS = Software Trigger Set (bit 12)
 */

#ifndef ASCLIN_FLAGS_OMT
#  define ASCLIN_FLAGS_OMT            BIT(11)
#endif

#ifndef ASCLIN_FLAGSCLEAR_OMTC
#  define ASCLIN_FLAGSCLEAR_OMTC      BIT(11)
#endif

#ifndef ASCLIN_FLAGSSET_SWTRGS
#  define ASCLIN_FLAGSSET_SWTRGS      BIT(12)
#endif

#ifndef ASCLIN_FLAGSCLEAR_SWTRGC
#  define ASCLIN_FLAGSCLEAR_SWTRGC    BIT(12)
#endif

#ifndef ASCLIN_FLAGSCLEAR_TWRQC
#  define ASCLIN_FLAGSCLEAR_TWRQC     BIT(13)
#endif

#ifndef ASCLIN_FLAGSCLEAR_TRRQC
#  define ASCLIN_FLAGSCLEAR_TRRQC     BIT(15)
#endif

/* FIFO inlet/outlet width for 16-bit (2-byte) transfers.
 * INW/OUTW encodings: 0=0 bytes, 1=1 byte, 2=2 bytes, 3=4 bytes
 * The base header only defines INW_1/OUTW_1.
 */

#ifndef ASCLIN_TXFIFOCON_INW_2
#  define ASCLIN_TXFIFOCON_INW_2    (2u << ASCLIN_TXFIFOCON_INW_SHIFT)
#endif

#ifndef ASCLIN_RXFIFOCON_OUTW_2
#  define ASCLIN_RXFIFOCON_OUTW_2   (2u << ASCLIN_RXFIFOCON_OUTW_SHIFT)
#endif

/* DATCON.TRGSRC encodings */

#define ASCLIN_DATCON_TRGSRC_NONE   (0u << ASCLIN_DATCON_TRGSRC_SHIFT)
#define ASCLIN_DATCON_TRGSRC_SW     (3u << ASCLIN_DATCON_TRGSRC_SHIFT)

/* Default ASCLIN clock frequency */

#ifndef TC4X_ASCLINF_FREQUENCY
#  define TC4X_ASCLINF_FREQUENCY    200000000u
#endif

#ifndef TC4X_ASCLIN_CLK_SRC
#  define TC4X_ASCLIN_CLK_SRC       ASCLIN_CSR_CLKSEL_FASCLINF
#endif

/* Polling timeout */

#define SPI_POLL_TIMEOUT            1000000u

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Per-bus hardware configuration */

struct tc4xx_spi_config_s
{
  uintptr_t     base;            /* ASCLIN register base address */
  uint32_t      fclk;            /* Input clock frequency (Hz) */
  uint8_t       clk_src;         /* CSR.CLKSEL value */
  uint8_t       bus;             /* SPI bus number (for spi_register) */
  gpio_pinset_t mosi_pin;        /* MOSI (ATX) pin configuration */
  gpio_pinset_t miso_pin;        /* MISO (ARX) pin configuration */
  gpio_pinset_t sclk_pin;        /* SCLK (ASCLKO) pin configuration */
  gpio_pinset_t cs_pin;          /* CS pin (GPIO-controlled) */
  uint8_t       rx_alti;         /* IOCR.ALTI for RX input select */
  bool          loopback;        /* Enable internal loopback for testing */
};

/* Per-bus runtime state */

struct tc4xx_spi_priv_s
{
  struct spi_dev_s              spidev;
  const struct tc4xx_spi_config_s *config;
  mutex_t                       lock;
  uint32_t                      frequency;
  uint32_t                      actual_freq;
  enum spi_mode_e               mode;
  int                           nbits;
  bool                          initialized;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int      tc4xx_spi_lock(FAR struct spi_dev_s *dev, bool lock);
static void     tc4xx_spi_select(FAR struct spi_dev_s *dev,
                                 uint32_t devid, bool selected);
static uint32_t tc4xx_spi_setfrequency(FAR struct spi_dev_s *dev,
                                       uint32_t frequency);
static void     tc4xx_spi_setmode(FAR struct spi_dev_s *dev,
                                  enum spi_mode_e mode);
static void     tc4xx_spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint8_t  tc4xx_spi_status(FAR struct spi_dev_s *dev,
                                 uint32_t devid);
static uint32_t tc4xx_spi_send(FAR struct spi_dev_s *dev, uint32_t wd);
#ifdef CONFIG_SPI_EXCHANGE
static void     tc4xx_spi_exchange(FAR struct spi_dev_s *dev,
                                   FAR const void *txbuffer,
                                   FAR void *rxbuffer, size_t nwords);
#else
static void     tc4xx_spi_sndblock(FAR struct spi_dev_s *dev,
                                   FAR const void *buffer, size_t nwords);
static void     tc4xx_spi_recvblock(FAR struct spi_dev_s *dev,
                                    FAR void *buffer, size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_asclin_spi_ops =
{
  .lock         = tc4xx_spi_lock,
  .select       = tc4xx_spi_select,
  .setfrequency = tc4xx_spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay     = NULL,
#endif
  .setmode      = tc4xx_spi_setmode,
  .setbits      = tc4xx_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures   = NULL,
#endif
  .status       = tc4xx_spi_status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = NULL,
#endif
  .send         = tc4xx_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = tc4xx_spi_exchange,
#else
  .sndblock     = tc4xx_spi_sndblock,
  .recvblock    = tc4xx_spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger      = NULL,
#endif
  .registercallback = NULL,
};

/* -----------------------------------------------------------------------
 * ASCLIN SPI bus instance(s)
 * -----------------------------------------------------------------------
 * Default pin mapping uses ASCLIN2 on mikroBUS-adjacent pins.
 * The user must verify pin assignments against the actual board schematic
 * and pinmux tables.  Loopback mode is enabled by default for safe
 * initial bring-up (no external wiring required).
 *
 * For real hardware:
 *   - Check pinmux.pdf for ASCLINx_ATX, ASCLINx_ASCLK, ASCLINx_ASLSO
 *     output alternate functions on chosen port pins.
 *   - Check pinmux.pdf for ASCLINx_ARXy input function on chosen MISO pin.
 *   - Set loopback = false.
 * -----------------------------------------------------------------------
 */

#ifdef CONFIG_TC4XX_ASCLIN2_SPI

static const struct tc4xx_spi_config_s g_spi0_config =
{
  .base      = TRICORE_ASCLIN_BASE(2),
  .fclk      = TC4X_ASCLINF_FREQUENCY,
  .clk_src   = TC4X_ASCLIN_CLK_SRC,
  .bus        = 0,

  /* Pin mapping: user must adjust to match board wiring.
   * Defaults use P15.0 (ATX/MOSI), P15.1 (ARX/MISO), P15.2 (ASCLK),
   * P14.7 (GPIO CS).  Verify against pinmux.pdf Table for Port 15.
   */

  .mosi_pin  = AURIX_GPIO(15, 0, GPIO_PERIPH, GPIO_ALT2),
  .miso_pin  = AURIX_GPIO(15, 1, GPIO_INPUT, GPIO_PULL_UP),
  .sclk_pin  = AURIX_GPIO(15, 2, GPIO_PERIPH, GPIO_ALT2),
  //.cs_pin    = AURIX_GPIO(14, 7, GPIO_OUTPUT, GPIO_PAD_CONFIG_OUT_GPIO),
  .rx_alti   = ASCLIN_IOCR_ALTI_A,

  /* Enable loopback for initial bring-up.  Set to false for real HW. */

  .loopback  = true,
};

static struct tc4xx_spi_priv_s g_spi0_priv =
{
  .spidev      = { .ops = &g_asclin_spi_ops },
  .config      = &g_spi0_config,
  .lock        = NXMUTEX_INITIALIZER,
  .frequency   = 1000000u,
  .actual_freq = 0u,
  .mode        = SPIDEV_MODE0,
  .nbits       = 8,
  .initialized = false,
};

#endif /* CONFIG_TC4XX_ASCLIN2_SPI */

/****************************************************************************
 * Private Functions - Register Access
 ****************************************************************************/

static inline uint32_t spi_getreg(const struct tc4xx_spi_config_s *cfg,
                                  uint32_t offset)
{
  return getreg32(cfg->base + offset);
}

static inline void spi_putreg(const struct tc4xx_spi_config_s *cfg,
                              uint32_t offset, uint32_t value)
{
  putreg32(value, cfg->base + offset);
}

static inline void spi_modreg(const struct tc4xx_spi_config_s *cfg,
                              uint32_t offset,
                              uint32_t setbits, uint32_t clrbits)
{
  modreg32(setbits, setbits | clrbits, cfg->base + offset);
}

/****************************************************************************
 * Name: spi_wait_clk
 *
 * Description:
 *   Wait for CSR.CON to reach expected state after clock selection change.
 *
 ****************************************************************************/

static int spi_wait_clk(const struct tc4xx_spi_config_s *cfg, bool on)
{
  uint32_t timeout = SPI_POLL_TIMEOUT;

  while (timeout--)
    {
      uint32_t csr = spi_getreg(cfg, ASCLIN_CSR_OFFSET);
      if (!!(csr & ASCLIN_CSR_CON) == on)
        {
          return OK;
        }
    }

  spierr("ERROR: clock wait timeout (expected %s)\n", on ? "ON" : "OFF");
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: spi_set_clk
 *
 * Description:
 *   Set ASCLIN clock source and wait for it to take effect.
 *
 ****************************************************************************/

static int spi_set_clk(const struct tc4xx_spi_config_s *cfg,
                       uint8_t clksel)
{
  spi_putreg(cfg, ASCLIN_CSR_OFFSET, clksel);
  return spi_wait_clk(cfg, clksel != ASCLIN_CSR_CLKSEL_NONE);
}

/****************************************************************************
 * Name: spi_compute_baudrate
 *
 * Description:
 *   Compute BRG numerator/denominator and BITCON prescaler/oversampling
 *   to achieve the requested SPI clock frequency.
 *
 *   For SPI mode with symmetric clock:
 *     oversampling should be even
 *     samplepoint = oversampling / 2
 *
 *   fPD  = fclk / (prescaler + 1)
 *   fOVS = fPD * NUM / DEN
 *   fSCLK = fOVS / (oversampling + 1)
 *
 *   Target: NUM/DEN = freq * (oversampling + 1) * (prescaler + 1) / fclk
 *
 * Returns actual frequency achieved.
 *
 ****************************************************************************/

static uint32_t spi_compute_baudrate(const struct tc4xx_spi_config_s *cfg,
                                     uint32_t freq)
{
  uint32_t prescaler;
  uint32_t oversampling;
  uint32_t samplepoint;
  uint32_t fpd;
  uint32_t target_fovs;
  uint32_t best_num = 1;
  uint32_t best_den = 1;
  uint32_t best_err = UINT32_MAX;
  uint32_t actual;
  uint32_t bitcon;
  uint32_t brg;

  /* Use oversampling = 16 for symmetric SCLK (even), samplepoint = 8
   * This gives SCLK with 50% duty cycle.
   */

  oversampling = 15u;  /* register value: actual oversampling = value + 1 */
  samplepoint  = 7u;   /* register value: first half = value + 1 = 8 ticks */

  /* Find prescaler that keeps fPD in a useful range.
   * fPD = fclk / (prescaler + 1)
   * We want fPD such that NUM/DEN ratio stays within 12-bit range.
   */

  prescaler = 0;

  /* target_fovs = freq * (oversampling + 1) */

  target_fovs = freq * (oversampling + 1);

  /* Try to find a prescaler that works well */

  for (prescaler = 0; prescaler < 4096; prescaler++)
    {
      fpd = cfg->fclk / (prescaler + 1);

      /* We need NUM/DEN = target_fovs / fpd
       * Both NUM and DEN must fit in 12 bits (0..4095)
       * DEN=0 is not allowed (means bypass, not what we want)
       */

      if (fpd == 0)
        {
          continue;
        }

      /* Simple search: try DEN from 1 to 4095, compute NUM */

      uint32_t den;
      uint32_t num;

      for (den = 1; den <= 4095; den++)
        {
          uint64_t num64 = ((uint64_t)target_fovs * den + (fpd / 2)) / fpd;
          if (num64 == 0 || num64 > 4095)
            {
              continue;
            }

          num = (uint32_t)num64;

          /* Compute actual frequency */

          uint64_t fovs = ((uint64_t)fpd * num) / den;
          actual = (uint32_t)(fovs / (oversampling + 1));

          uint32_t err;
          if (actual > freq)
            {
              err = actual - freq;
            }
          else
            {
              err = freq - actual;
            }

          if (err < best_err)
            {
              best_err = err;
              best_num = num;
              best_den = den;

              if (err == 0)
                {
                  goto found;
                }
            }
        }

      /* If we found a good enough match with this prescaler, stop */

      if (best_err < (freq / 1000))
        {
          break;
        }
    }

found:

  /* Program BITCON: prescaler, oversampling, samplepoint, single sample */

  bitcon = (prescaler & 0xFFFu)
         | ((oversampling & 0xFu) << ASCLIN_BITCON_OVERSAMPLING_SHIFT)
         | ((samplepoint & 0xFu) << ASCLIN_BITCON_SAMPLEPOINT_SHIFT);

  spi_putreg(cfg, ASCLIN_BITCON_OFFSET, bitcon);

  /* Program BRG: numerator and denominator */

  brg = (best_den & 0xFFFu)
      | ((best_num & 0xFFFu) << 16u);

  spi_putreg(cfg, ASCLIN_BRG_OFFSET, brg);

  /* Compute actual achieved frequency */

  fpd = cfg->fclk / (prescaler + 1);
  actual = (uint32_t)(((uint64_t)fpd * best_num) /
                      ((uint64_t)best_den * (oversampling + 1)));

  spiinfo("SPI baud: req=%lu actual=%lu pre=%lu num=%lu den=%lu ovs=%lu\n",
          (unsigned long)freq, (unsigned long)actual,
          (unsigned long)prescaler, (unsigned long)best_num,
          (unsigned long)best_den, (unsigned long)(oversampling + 1));

  return actual;
}

/****************************************************************************
 * Name: spi_hw_init
 *
 * Description:
 *   Full ASCLIN SPI initialization sequence.
 *
 *   1. Enable module clock (CLC)
 *   2. Configure GPIO pins
 *   3. Turn off operating clock (CSR.CLKSEL = 0) for register writes
 *   4. Set FRAMECON.MODE = SPI
 *   5. Configure bit timing (BITCON) and baud rate (BRG)
 *   6. Configure data length (DATCON)
 *   7. Configure IOCR (CPOL, SPOL, loopback, RX input select)
 *   8. Configure TX/RX FIFOs
 *   9. Clear all flags
 *  10. Turn on operating clock
 *
 ****************************************************************************/

static int spi_hw_init(struct tc4xx_spi_priv_s *priv)
{
  const struct tc4xx_spi_config_s *cfg = priv->config;
  uint32_t reg;
  int ret;

  /* Step 1: Enable module clock */

  spi_putreg(cfg, ASCLIN_CLC_OFFSET, 0u);

  uint32_t timeout = SPI_POLL_TIMEOUT;
  while (timeout--)
    {
      if (!(spi_getreg(cfg, ASCLIN_CLC_OFFSET) & ASCLIN_CLC_DISS))
        {
          break;
        }
    }

  if (timeout == 0)
    {
      spierr("ERROR: ASCLIN module clock enable timeout\n");
      return -ETIMEDOUT;
    }

  /* Step 2: Configure GPIO pins */

  if (!cfg->loopback)
    {
      aurix_config_gpio(cfg->mosi_pin);
      aurix_config_gpio(cfg->miso_pin);
      aurix_config_gpio(cfg->sclk_pin);
    }

  /* CS pin: configure as GPIO output, drive inactive (high) */

  aurix_config_gpio(cfg->cs_pin);
  aurix_gpio_write(cfg->cs_pin, true);

  /* Step 3: Disable operating clock for register configuration */

  ret = spi_set_clk(cfg, ASCLIN_CSR_CLKSEL_NONE);
  if (ret < 0)
    {
      spierr("ERROR: clock disable failed\n");
      return ret;
    }

  /* Step 4: FRAMECON - SPI mode, MSB first, no parity
   * LEAD = 1 (1 bit time leading delay before first SCLK edge)
   * STOP = 1 (1 bit time trailing delay after last SCLK edge) - "trail"
   * IDLE = 1 (1 bit time idle between frames)
   */

  reg = ASCLIN_FRAMECON_MODE_SPI
      | ASCLIN_FRAMECON_MSB
      | (1u << ASCLIN_FRAMECON_LEAD_SHIFT)
      | (1u << ASCLIN_FRAMECON_STOP_SHIFT)
      | (1u << ASCLIN_FRAMECON_IDLE_SHIFT);

  spi_putreg(cfg, ASCLIN_FRAMECON_OFFSET, reg);

  /* Step 5: Configure bit timing and baud rate */

  priv->actual_freq = spi_compute_baudrate(cfg, priv->frequency);

  /* Step 6: DATCON - data length, no trigger initially */

  reg = ASCLIN_DATCON_DATLEN(priv->nbits);
  spi_putreg(cfg, ASCLIN_DATCON_OFFSET, reg);

  /* Step 7: IOCR - clock polarity, slave select polarity, input select */

  reg = (cfg->rx_alti & ASCLIN_IOCR_ALTI_MASK);

  /* SPOL = 1 means SLSO idle is high (active low CS) */

  reg |= ASCLIN_IOCR_SPOL;

  /* Set CPOL based on current SPI mode */

  if (priv->mode == SPIDEV_MODE2 || priv->mode == SPIDEV_MODE3)
    {
      reg |= ASCLIN_IOCR_CPOL;
    }

  /* Enable loopback if configured */

  if (cfg->loopback)
    {
      reg |= ASCLIN_IOCR_LB;
    }

  spi_putreg(cfg, ASCLIN_IOCR_OFFSET, reg);

  /* Step 8: Configure FIFOs
   * TX: flush, enable outlet, combined mode
   * RX: flush, enable inlet
   * Inlet/outlet width must match the transfer word size:
   *   nbits <= 8:  INW_1 / OUTW_1 (1 byte per FIFO access)
   *   nbits > 8:   INW_2 / OUTW_2 (2 bytes per FIFO access)
   */

  reg = ASCLIN_TXFIFOCON_FLUSH
      | ASCLIN_TXFIFOCON_ENO
      | ASCLIN_TXFIFOCON_FM_COMBINED
      | ((priv->nbits > 8) ? ASCLIN_TXFIFOCON_INW_2
                           : ASCLIN_TXFIFOCON_INW_1);

  spi_putreg(cfg, ASCLIN_TXFIFOCON_OFFSET, reg);

  reg = ASCLIN_RXFIFOCON_FLUSH
      | ASCLIN_RXFIFOCON_ENI
      | ((priv->nbits > 8) ? ASCLIN_RXFIFOCON_OUTW_2
                           : ASCLIN_RXFIFOCON_OUTW_1);

  spi_putreg(cfg, ASCLIN_RXFIFOCON_OFFSET, reg);

  /* Step 9: Clear all flags, disable all flag interrupts */

  spi_putreg(cfg, ASCLIN_FLAGSCLEAR_OFFSET, ASCLIN_FLAGSCLEAR_ALL);
  spi_putreg(cfg, ASCLIN_FLAGSENABLE_OFFSET, 0u);

  /* Step 10: Enable operating clock */

  ret = spi_set_clk(cfg, cfg->clk_src);
  if (ret < 0)
    {
      spierr("ERROR: clock enable failed\n");
      return ret;
    }

  priv->initialized = true;
  spiinfo("ASCLIN SPI bus %d initialized (base=0x%08lx loopback=%d)\n",
          cfg->bus, (unsigned long)cfg->base, cfg->loopback);

  return OK;
}

/****************************************************************************
 * Name: spi_exchange_word
 *
 * Description:
 *   Exchange a single SPI word (polled).  Writes TX data, waits for
 *   transfer complete, reads RX data.
 *
 *   ASCLIN SPI transfer sequence:
 *   1. Flush TX/RX FIFOs
 *   2. Clear TC flag
 *   3. Write data to TXDATA
 *   4. Poll for FLAGS.TC = 1 (transfer complete)
 *   5. Read RXDATA
 *   6. Clear TC flag
 *
 ****************************************************************************/

static uint32_t spi_exchange_word(struct tc4xx_spi_priv_s *priv,
                                  uint32_t txword)
{
  const struct tc4xx_spi_config_s *cfg = priv->config;
  uint32_t rxword;
  uint32_t timeout;

  /* Flush both FIFOs */

  spi_modreg(cfg, ASCLIN_TXFIFOCON_OFFSET,
             ASCLIN_TXFIFOCON_FLUSH, ASCLIN_TXFIFOCON_FLUSH);
  spi_modreg(cfg, ASCLIN_RXFIFOCON_OFFSET,
             ASCLIN_RXFIFOCON_FLUSH, ASCLIN_RXFIFOCON_FLUSH);

  /* Clear TC and all relevant flags */

  spi_putreg(cfg, ASCLIN_FLAGSCLEAR_OFFSET,
             ASCLIN_FLAGSCLEAR_TCC | ASCLIN_FLAGSCLEAR_RRC |
             ASCLIN_FLAGSCLEAR_THC | ASCLIN_FLAGSCLEAR_TRC);

  /* Write TX data */

  spi_putreg(cfg, ASCLIN_TXDATA_OFFSET, txword);

  /* Poll for transfer complete (FLAGS.TC = 1) */

  timeout = SPI_POLL_TIMEOUT;
  while (timeout--)
    {
      uint32_t flags = spi_getreg(cfg, ASCLIN_FLAGS_OFFSET);
      if (flags & ASCLIN_FLAGS_TC)
        {
          break;
        }
    }

  if (timeout == 0)
    {
      spierr("ERROR: SPI transfer timeout\n");
      return 0;
    }

  /* Read received data */

  rxword = spi_getreg(cfg, ASCLIN_RXDATA_OFFSET);

  /* Clear TC flag */

  spi_putreg(cfg, ASCLIN_FLAGSCLEAR_OFFSET, ASCLIN_FLAGSCLEAR_TCC);

  return rxword;
}

/****************************************************************************
 * Name: spi_reconfigure
 *
 * Description:
 *   Reconfigure the ASCLIN for changed mode/bits.  Requires clock off,
 *   modify registers, clock on sequence.
 *
 ****************************************************************************/

static void spi_reconfigure(struct tc4xx_spi_priv_s *priv)
{
  const struct tc4xx_spi_config_s *cfg = priv->config;
  uint32_t iocr;
  uint32_t datcon;
  uint32_t framecon;

  /* Must disable clock to change config registers */

  spi_set_clk(cfg, ASCLIN_CSR_CLKSEL_NONE);

  /* Update IOCR for clock polarity */

  iocr = spi_getreg(cfg, ASCLIN_IOCR_OFFSET);
  iocr &= ~ASCLIN_IOCR_CPOL;

  if (priv->mode == SPIDEV_MODE2 || priv->mode == SPIDEV_MODE3)
    {
      iocr |= ASCLIN_IOCR_CPOL;
    }

  spi_putreg(cfg, ASCLIN_IOCR_OFFSET, iocr);

  /* Update FRAMECON for clock phase (CPHA).
   *
   * ASCLIN SPI is defined as: shift on first edge, sample on second edge.
   * This maps to CPOL=0,CPHA=0 (MODE0) or CPOL=1,CPHA=0 (MODE2) natively.
   *
   * For CPHA=1 (MODE1/MODE3), we adjust the LEAD delay to achieve the
   * phase shift effect.  LEAD=0 with the SPI mode gives CPHA=1 behavior.
   *
   * NOTE: ASCLIN SPI always shifts first, then latches.  For true CPHA=1,
   * the lead delay effectively shifts the data alignment.
   */

  framecon = spi_getreg(cfg, ASCLIN_FRAMECON_OFFSET);
  framecon &= ~ASCLIN_FRAMECON_LEAD_MASK;

  if (priv->mode == SPIDEV_MODE1 || priv->mode == SPIDEV_MODE3)
    {
      /* CPHA=1: reduce lead delay to 0 */

      framecon |= (0u << ASCLIN_FRAMECON_LEAD_SHIFT);
    }
  else
    {
      /* CPHA=0: standard lead delay of 1 */

      framecon |= (1u << ASCLIN_FRAMECON_LEAD_SHIFT);
    }

  spi_putreg(cfg, ASCLIN_FRAMECON_OFFSET, framecon);

  /* Update DATCON for data length */

  datcon = ASCLIN_DATCON_DATLEN(priv->nbits);
  spi_putreg(cfg, ASCLIN_DATCON_OFFSET, datcon);

  /* Update baud rate */

  priv->actual_freq = spi_compute_baudrate(cfg, priv->frequency);

  /* Reconfigure FIFO widths to match current nbits, then flush.
   * Must fully rewrite TXFIFOCON/RXFIFOCON since FLUSH is W-only.
   */

  {
    uint32_t txfifo;
    uint32_t rxfifo;

    txfifo = ASCLIN_TXFIFOCON_FLUSH
           | ASCLIN_TXFIFOCON_ENO
           | ASCLIN_TXFIFOCON_FM_COMBINED
           | ((priv->nbits > 8) ? ASCLIN_TXFIFOCON_INW_2
                                : ASCLIN_TXFIFOCON_INW_1);

    spi_putreg(cfg, ASCLIN_TXFIFOCON_OFFSET, txfifo);

    rxfifo = ASCLIN_RXFIFOCON_FLUSH
           | ASCLIN_RXFIFOCON_ENI
           | ((priv->nbits > 8) ? ASCLIN_RXFIFOCON_OUTW_2
                                : ASCLIN_RXFIFOCON_OUTW_1);

    spi_putreg(cfg, ASCLIN_RXFIFOCON_OFFSET, rxfifo);
  }

  /* Clear flags */

  spi_putreg(cfg, ASCLIN_FLAGSCLEAR_OFFSET, ASCLIN_FLAGSCLEAR_ALL);

  /* Re-enable clock */

  spi_set_clk(cfg, cfg->clk_src);
}

/****************************************************************************
 * Private Functions - SPI ops implementation
 ****************************************************************************/

static int tc4xx_spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  struct tc4xx_spi_priv_s *priv = (struct tc4xx_spi_priv_s *)dev;

  if (lock)
    {
      return nxmutex_lock(&priv->lock);
    }
  else
    {
      return nxmutex_unlock(&priv->lock);
    }
}

/****************************************************************************
 * Name: tc4xx_spi_select
 *
 * Description:
 *   Assert or de-assert CS via GPIO.  Active low by default.
 *
 ****************************************************************************/

static void tc4xx_spi_select(FAR struct spi_dev_s *dev, uint32_t devid,
                             bool selected)
{
  struct tc4xx_spi_priv_s *priv = (struct tc4xx_spi_priv_s *)dev;

  spiinfo("devid=%lu selected=%d\n", (unsigned long)devid, selected);

  /* Active low: selected=true means drive CS low */

  aurix_gpio_write(priv->config->cs_pin, !selected);
}

/****************************************************************************
 * Name: tc4xx_spi_setfrequency
 *
 * Description:
 *   Set the SPI clock frequency.
 *
 ****************************************************************************/

static uint32_t tc4xx_spi_setfrequency(FAR struct spi_dev_s *dev,
                                       uint32_t frequency)
{
  struct tc4xx_spi_priv_s *priv = (struct tc4xx_spi_priv_s *)dev;

  if (priv->frequency == frequency && priv->actual_freq != 0)
    {
      return priv->actual_freq;
    }

  priv->frequency = frequency;
  spi_reconfigure(priv);

  spiinfo("frequency=%lu actual=%lu\n",
          (unsigned long)frequency, (unsigned long)priv->actual_freq);

  return priv->actual_freq;
}

/****************************************************************************
 * Name: tc4xx_spi_setmode
 *
 * Description:
 *   Set the SPI mode (CPOL/CPHA).
 *
 *   ASCLIN SPI natively does shift-first, sample-second.  Combined with
 *   IOCR.CPOL this gives:
 *     MODE0: CPOL=0, shift on rising, sample on falling
 *     MODE2: CPOL=1, shift on falling, sample on rising
 *
 *   For MODE1/MODE3 (CPHA=1), we adjust the LEAD delay.
 *
 ****************************************************************************/

static void tc4xx_spi_setmode(FAR struct spi_dev_s *dev,
                              enum spi_mode_e mode)
{
  struct tc4xx_spi_priv_s *priv = (struct tc4xx_spi_priv_s *)dev;

  if (priv->mode == mode)
    {
      return;
    }

  priv->mode = mode;
  spi_reconfigure(priv);

  spiinfo("mode=%d\n", mode);
}

/****************************************************************************
 * Name: tc4xx_spi_setbits
 *
 * Description:
 *   Set the number of bits per word (2..16).
 *
 ****************************************************************************/

static void tc4xx_spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  struct tc4xx_spi_priv_s *priv = (struct tc4xx_spi_priv_s *)dev;

  if (nbits < 2 || nbits > 16)
    {
      spierr("ERROR: invalid nbits=%d (must be 2..16)\n", nbits);
      return;
    }

  if (priv->nbits == nbits)
    {
      return;
    }

  priv->nbits = nbits;
  spi_reconfigure(priv);

  spiinfo("nbits=%d\n", nbits);
}

/****************************************************************************
 * Name: tc4xx_spi_status
 *
 * Description:
 *   Return SPI status.
 *
 ****************************************************************************/

static uint8_t tc4xx_spi_status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return SPI_STATUS_PRESENT;
}

/****************************************************************************
 * Name: tc4xx_spi_send
 *
 * Description:
 *   Exchange one word on SPI.  Returns the received word.
 *
 ****************************************************************************/

static uint32_t tc4xx_spi_send(FAR struct spi_dev_s *dev, uint32_t wd)
{
  struct tc4xx_spi_priv_s *priv = (struct tc4xx_spi_priv_s *)dev;
  return spi_exchange_word(priv, wd);
}

/****************************************************************************
 * Name: tc4xx_spi_exchange
 *
 * Description:
 *   Exchange a block of data.  Full duplex: simultaneously sends txbuffer
 *   and receives into rxbuffer.  Either buffer may be NULL.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_EXCHANGE
static void tc4xx_spi_exchange(FAR struct spi_dev_s *dev,
                               FAR const void *txbuffer,
                               FAR void *rxbuffer, size_t nwords)
{
  struct tc4xx_spi_priv_s *priv = (struct tc4xx_spi_priv_s *)dev;
  FAR const uint8_t *txp8 = (FAR const uint8_t *)txbuffer;
  FAR const uint16_t *txp16 = (FAR const uint16_t *)txbuffer;
  FAR uint8_t *rxp8 = (FAR uint8_t *)rxbuffer;
  FAR uint16_t *rxp16 = (FAR uint16_t *)rxbuffer;
  size_t i;

  spiinfo("exchange nwords=%zu nbits=%d\n", nwords, priv->nbits);

  for (i = 0; i < nwords; i++)
    {
      uint32_t txword;
      uint32_t rxword;

      /* Get TX word */

      if (txbuffer != NULL)
        {
          if (priv->nbits <= 8)
            {
              txword = (uint32_t)txp8[i];
            }
          else
            {
              txword = (uint32_t)txp16[i];
            }
        }
      else
        {
          txword = 0xffu; /* Send 0xFF when no TX data (standard SPI idle) */
        }

      /* Exchange */

      rxword = spi_exchange_word(priv, txword);

      /* Store RX word */

      if (rxbuffer != NULL)
        {
          if (priv->nbits <= 8)
            {
              rxp8[i] = (uint8_t)(rxword & 0xffu);
            }
          else
            {
              rxp16[i] = (uint16_t)(rxword & 0xffffu);
            }
        }
    }
}
#else

static void tc4xx_spi_sndblock(FAR struct spi_dev_s *dev,
                               FAR const void *buffer, size_t nwords)
{
  tc4xx_spi_exchange(dev, buffer, NULL, nwords);
}

static void tc4xx_spi_recvblock(FAR struct spi_dev_s *dev,
                                FAR void *buffer, size_t nwords)
{
  tc4xx_spi_exchange(dev, NULL, buffer, nwords);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc4xx_asclin_spibus_initialize
 ****************************************************************************/

FAR struct spi_dev_s *tc4xx_asclin_spibus_initialize(int bus)
{
  struct tc4xx_spi_priv_s *priv = NULL;
  int ret;

  switch (bus)
    {
#ifdef CONFIG_TC4XX_ASCLIN2_SPI
      case 0:
        priv = &g_spi0_priv;
        break;
#endif
      default:
        spierr("ERROR: unsupported SPI bus %d\n", bus);
        return NULL;
    }

  if (priv->initialized)
    {
      return &priv->spidev;
    }

  ret = spi_hw_init(priv);
  if (ret < 0)
    {
      spierr("ERROR: SPI HW init failed: %d\n", ret);
      return NULL;
    }

  return &priv->spidev;
}
