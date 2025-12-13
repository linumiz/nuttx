#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "tricore_internal.h"
#include "tc4xx_gpio.h"

#pragma GCC push_options
#pragma GCC optimize ("O0")

#ifndef ASCLIN_TXFIFOCON_FILL
#  define ASCLIN_TXFIFOCON_FILL(val) \
    (((val) & ASCLIN_TXFIFOCON_FILL_MASK) >> ASCLIN_TXFIFOCON_FILL_SHIFT)
#endif

#define TC4X_ASCLIN0_FREQUENCY   80000000u

#define TC4X_ASCLIN0_BASE        0xF46C0000u
#define TC4X_ASCLIN_STRIDE       0x00000200u

#define TC4X_ASCLIN_BASE(n)  (TC4X_ASCLIN0_BASE + ((uint32_t)(n) * TC4X_ASCLIN_STRIDE))
#define REGADDR(off)   (TC4X_ASCLIN0_BASE + (off))

#define ASCLIN_CLC_OFFSET           0x0000u
#define ASCLIN_IOCR_OFFSET          0x0100u
#define ASCLIN_TXFIFOCON_OFFSET     0x0104u
#define ASCLIN_RXFIFOCON_OFFSET     0x0108u
#define ASCLIN_BITCON_OFFSET        0x010Cu
#define ASCLIN_FRAMECON_OFFSET      0x0110u
#define ASCLIN_DATCON_OFFSET        0x0114u
#define ASCLIN_BRG_OFFSET           0x0118u
#define ASCLIN_BRD_OFFSET           0x011Cu
#define ASCLIN_FLAGS_OFFSET         0x012Cu
#define ASCLIN_FLAGSSET_OFFSET      0x0130u
#define ASCLIN_FLAGSCLEAR_OFFSET    0x0134u
#define ASCLIN_FLAGSENABLE_OFFSET   0x0138u
#define ASCLIN_CSR_OFFSET           0x013Cu
#define ASCLIN_TXDATA0_OFFSET       0x0140u
#define ASCLIN_RXDATA0_OFFSET       0x0160u

#define ASCLIN_CLC_DISR          (1u << 0)
#define ASCLIN_CLC_DISS          (1u << 1)

#define ASCLIN_TXFIFOCON_FILL_SHIFT   16u
#define ASCLIN_TXFIFOCON_FILL_MASK    (0x1Fu << ASCLIN_TXFIFOCON_FILL_SHIFT)
#define ASCLIN_TXFIFOCON_FILL(val)    (((val) & ASCLIN_TXFIFOCON_FILL_MASK) >> ASCLIN_TXFIFOCON_FILL_SHIFT)

#define ASCLIN_FIFO_DEPTH            16u

static void c4x_uart0_pinmux(void)
{
  gpio_pinset_t tx;
  gpio_pinset_t rx;

  /* P14.0: peripheral, ALT2, ASCLIN0_TX */
  tx = GPIO_PAD_CFG(GPIO_PORT14,
                    GPIO_PIN0,
                    GPIO_PERIPH,
                    GPIO_PAD_CONFIG_OUT_ALT02);
  aurix_config_gpio(tx);
}

static void tc4x_asclin0_set_baud(uint32_t baud)
{
  uint32_t addr;
  uint32_t val;

  addr = REGADDR(ASCLIN_BRG_OFFSET);
  val  = 0x200D9;
  putreg32(val, addr);

  addr = REGADDR(ASCLIN_BITCON_OFFSET);
  val  = 0x880F0000;
  putreg32(val, addr);
}

void tc4x_asclin0_init(uint32_t baud)
{
  uint32_t addr;
  uint32_t val;

  addr = REGADDR(ASCLIN_CLC_OFFSET);
  val  = getreg32(addr);
  val &= ~ASCLIN_CLC_DISR;
  putreg32(val, addr);

  while (getreg32(addr) & ASCLIN_CLC_DISS)
    {
      /* wait until enabled */
    }

  c4x_uart0_pinmux();

  addr = REGADDR(ASCLIN_CSR_OFFSET);
  putreg32(0u, addr);

  addr = REGADDR(ASCLIN_FRAMECON_OFFSET);
  putreg32(0u, addr);

  tc4x_asclin0_set_baud(baud);

  addr = REGADDR(ASCLIN_FRAMECON_OFFSET);
  putreg32(0x10200, addr);

  addr = REGADDR(ASCLIN_DATCON_OFFSET);
  putreg32(0x7, addr);

  addr = REGADDR(ASCLIN_TXFIFOCON_OFFSET);
  putreg32(0x42, addr);

  addr = REGADDR(ASCLIN_RXFIFOCON_OFFSET);
  putreg32(0x42, addr);

  addr = REGADDR(ASCLIN_CSR_OFFSET);
  putreg32(0x2, addr);

  return;
}

static void tc4x_asclin0_putc_raw(char ch)
{
  uint32_t addr;
  uint32_t val;

  addr = REGADDR(ASCLIN_TXFIFOCON_OFFSET);
  do
    {
      val = getreg32(addr);
    }
  while (ASCLIN_TXFIFOCON_FILL(val) >= ASCLIN_FIFO_DEPTH);

  addr = REGADDR(ASCLIN_TXDATA0_OFFSET);
  putreg32((uint32_t)(uint8_t)ch, addr);
}

void tc4x_asclin0_puts(const char *s)
{
  while (*s != '\0')
    {
      char ch = *s++;

      if (ch == '\n')
        {
          tc4x_asclin0_putc_raw('\r');
        }

      tc4x_asclin0_putc_raw(ch);
    }
}

void up_putc(int ch)
{
  if (ch == '\n')
    {
      tc4x_asclin0_putc_raw('\r');
    }

  tc4x_asclin0_putc_raw(ch);
  return;
}

void up_lowputc(int ch)
{
  up_putc(ch);
}
#pragma GCC pop_options
