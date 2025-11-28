/****************************************************************************
 * Arch:  TriCore AURIX TC4Dx
 * File:  tc4xx_serial.c
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <nuttx/irq.h>

#include "tricore_internal.h"
#include "tc4xx_gpio.h"

/*-------------------------------------------------------------------------*/
/* Configuration                                                           */
/*-------------------------------------------------------------------------*/

/* Use ASCLIN0 as console by default. */

#define TC4X_CONSOLE_ASCLIN_INDEX   0u   /* ASCLIN0 */

/* ASCLIN module base address and spacing */

#define TC4X_ASCLIN0_BASE           0xF46C0000u
#define TC4X_ASCLIN_STRIDE          0x00000200u

#define TC4X_ASCLIN_BASE(n) \
  (TC4X_ASCLIN0_BASE + ((uint32_t)(n) * TC4X_ASCLIN_STRIDE))

/* Register offsets (derived from ASCLIN0_* macros) */

#define ASCLIN_CLC_OFFSET           0x0000u
#define ASCLIN_OCS_OFFSET           0x0004u
#define ASCLIN_ID_OFFSET            0x0008u
#define ASCLIN_RST_CTRLA_OFFSET     0x000Cu
#define ASCLIN_RST_CTRLB_OFFSET     0x0010u
#define ASCLIN_RST_STAT_OFFSET      0x0014u
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

/* Simple register access macro */

#define ASCLIN_REG(base, off)   (*(volatile uint32_t *)((base) + (off)))

/* Individual registers */

#define ASCLIN_CLC(base)        ASCLIN_REG((base), ASCLIN_CLC_OFFSET)
#define ASCLIN_IOCR(base)       ASCLIN_REG((base), ASCLIN_IOCR_OFFSET)
#define ASCLIN_TXFIFOCON(base)  ASCLIN_REG((base), ASCLIN_TXFIFOCON_OFFSET)
#define ASCLIN_RXFIFOCON(base)  ASCLIN_REG((base), ASCLIN_RXFIFOCON_OFFSET)
#define ASCLIN_BITCON(base)     ASCLIN_REG((base), ASCLIN_BITCON_OFFSET)
#define ASCLIN_FRAMECON(base)   ASCLIN_REG((base), ASCLIN_FRAMECON_OFFSET)
#define ASCLIN_DATCON(base)     ASCLIN_REG((base), ASCLIN_DATCON_OFFSET)
#define ASCLIN_BRG(base)        ASCLIN_REG((base), ASCLIN_BRG_OFFSET)
#define ASCLIN_BRD(base)        ASCLIN_REG((base), ASCLIN_BRD_OFFSET)
#define ASCLIN_FLAGS(base)      ASCLIN_REG((base), ASCLIN_FLAGS_OFFSET)
#define ASCLIN_FLAGSCLEAR(base) ASCLIN_REG((base), ASCLIN_FLAGSCLEAR_OFFSET)
#define ASCLIN_FLAGSENABLE(base) ASCLIN_REG((base), ASCLIN_FLAGSENABLE_OFFSET)
#define ASCLIN_CSR(base)        ASCLIN_REG((base), ASCLIN_CSR_OFFSET)
#define ASCLIN_TXDATA0(base)    ASCLIN_REG((base), ASCLIN_TXDATA0_OFFSET)
#define ASCLIN_RXDATA0(base)    ASCLIN_REG((base), ASCLIN_RXDATA0_OFFSET)

/*-------------------------------------------------------------------------*/
/* Bit definitions							   */
/*-------------------------------------------------------------------------*/

/* CLC bits (standard for AURIX peripherals) */

#define ASCLIN_CLC_DISR          (1u << 0)  /* Disable request */
#define ASCLIN_CLC_DISS          (1u << 1)  /* Disable status */

/* Tx/Rx FIFO control bits */

#define ASCLIN_TXFIFOCON_FLUSH   (1u << 0)
#define ASCLIN_TXFIFOCON_ENO     (1u << 1)  /* Enable output */

#define ASCLIN_RXFIFOCON_FLUSH   (1u << 0)
#define ASCLIN_RXFIFOCON_ENI     (1u << 1)  /* Enable input */

/* FILL fields: bits [20:16], mask 0x1F */

#define ASCLIN_TXFIFOCON_FILL_SHIFT   16u
#define ASCLIN_TXFIFOCON_FILL_MASK    (0x1Fu << ASCLIN_TXFIFOCON_FILL_SHIFT)

#define ASCLIN_RXFIFOCON_FILL_SHIFT   16u
#define ASCLIN_RXFIFOCON_FILL_MASK    (0x1Fu << ASCLIN_RXFIFOCON_FILL_SHIFT)

#define ASCLIN_TXFIFOCON_FILL(v) \
  (((v) & ASCLIN_TXFIFOCON_FILL_MASK) >> ASCLIN_TXFIFOCON_FILL_SHIFT)

#define ASCLIN_RXFIFOCON_FILL(v) \
  (((v) & ASCLIN_RXFIFOCON_FILL_MASK) >> ASCLIN_RXFIFOCON_FILL_SHIFT)

/* FIFO depth – **verify for your device**. 16 is typical. */
#define ASCLIN_FIFO_DEPTH        16u

/*-------------------------------------------------------------------------*/
/* Frame / data / bit timing defaults                                      */
/*-------------------------------------------------------------------------*/

#define DEFAULT_FRAMECON_8N1    0u  /* TODO: set proper 8N1 ASC mode bits */
#define DEFAULT_DATCON_8BIT     0u  /* TODO: set proper data length/parity */
#define DEFAULT_BITCON          0u  /* TODO: prescaler/oversampling */
#define DEFAULT_BRG             0u  /* TODO: numerator/denominator/etc */
#define DEFAULT_BRD             0u  /* Usually derived from BRG */

/*-------------------------------------------------------------------------*/
/* Low-level ASCLIN helpers (init / configure / poll I/O)                  */
/*-------------------------------------------------------------------------*/

static inline uintptr_t tricore_asclin_base(unsigned int index)
{
  return (uintptr_t)TC4X_ASCLIN_BASE(index);
}

/* Basic module enable + FIFO clean. Does NOT do full baud config. */

static void tricore_asclin_init(unsigned int index)
{
  uintptr_t base = tricore_asclin_base(index);

  /* Enable module clock */
  ASCLIN_CLC(base) &= ~ASCLIN_CLC_DISR;
  while (ASCLIN_CLC(base) & ASCLIN_CLC_DISS)
    {
      /* wait until module is enabled */
    }

  /* Flush FIFOs and enable them */
  ASCLIN_TXFIFOCON(base) |= ASCLIN_TXFIFOCON_FLUSH;
  ASCLIN_RXFIFOCON(base) |= ASCLIN_RXFIFOCON_FLUSH;

  ASCLIN_TXFIFOCON(base) |= ASCLIN_TXFIFOCON_ENO;
  ASCLIN_RXFIFOCON(base) |= ASCLIN_RXFIFOCON_ENI;

  /* Optional: clear any stale flags */
  ASCLIN_FLAGSCLEAR(base) = 0xFFFFFFFFu;
}

static void tricore_asclin_configure(unsigned int index,
                                     uint32_t framecon,
                                     uint32_t datcon,
                                     uint32_t bitcon,
                                     uint32_t brg,
                                     uint32_t brd)
{
  uintptr_t base = tricore_asclin_base(index);

  ASCLIN_FRAMECON(base) = framecon;
  ASCLIN_DATCON(base)   = datcon;
  ASCLIN_BITCON(base)   = bitcon;
  ASCLIN_BRG(base)      = brg;
  ASCLIN_BRD(base)      = brd;
}

static void tricore_asclin_configure_default(unsigned int index)
{
  tricore_asclin_configure(index,
                           DEFAULT_FRAMECON_8N1,
                           DEFAULT_DATCON_8BIT,
                           DEFAULT_BITCON,
                           DEFAULT_BRG,
                           DEFAULT_BRD);
}

static void tricore_asclin_poll_out(unsigned int index, uint8_t ch)
{
  uintptr_t base = tricore_asclin_base(index);

  /* Wait for space in TX FIFO */
  while (ASCLIN_TXFIFOCON(base) &&
         ASCLIN_TXFIFOCON_FILL(ASCLIN_TXFIFOCON(base)) >= ASCLIN_FIFO_DEPTH)
    {
      /* busy wait */
    }

  /* Write data (lower 8 bits are used) */
  ASCLIN_TXDATA0(base) = (uint32_t)ch;
}

/* Poll-in: non-blocking receive of one byte.
 * Returns:
 *   0  on success, *ch contains data
 *  -1  if no data available
 */

static int tricore_asclin_poll_in(unsigned int index, uint8_t *ch)
{
  uintptr_t base = tricore_asclin_base(index);

  if (ASCLIN_RXFIFOCON_FILL(ASCLIN_RXFIFOCON(base)) == 0u)
    {
      return -1; /* no data */
    }

  /* Read byte */
  *ch = (uint8_t)ASCLIN_RXDATA0(base);
  return 0;
}

/*-------------------------------------------------------------------------*/
/* UART0 pinmux (P14.0 / P14.1)                                            */
/*-------------------------------------------------------------------------*/

/* Configure:
 *   P14.0 -> ASCLIN0_ATX_F  (TX, ALT2, peripheral output)
 *   P14.1 -> ASCLIN0_ARXA_F (RX, peripheral input)
 *
 * NOTE:
 *  - For RX, this uses INPUT/TRISTATE.
 */

static void tc4x_uart0_pinmux(void)
{
  gpio_pinset_t tx;
  gpio_pinset_t rx;

  /* P14.0: peripheral, ALT2, ASCLIN0_TX */
  tx = GPIO_PAD_CFG(GPIO_PORT14,
                    GPIO_PIN0,
                    GPIO_PERIPH,
                    GPIO_PAD_CONFIG_OUT_ALT02);
  aurix_config_gpio(tx);

  /* P14.1: peripheral RX, input, tristate */
  rx = GPIO_PAD_CFG(GPIO_PORT14,
                    GPIO_PIN1,
                    GPIO_INPUT,
                    GPIO_PAD_CONFIG_IN_TRISTATE);
  aurix_config_gpio(rx);
}

/*-------------------------------------------------------------------------*/
/* NuttX serial driver integration                                         */
/*-------------------------------------------------------------------------*/

#ifdef USE_SERIALDRIVER

/* Minimal private state for one ASCLIN port */

struct tc4x_uart_s
{
  uintptr_t base;
};

/* Console device instance – ASCLIN0 by default */

static struct tc4x_uart_s g_tc4x_console_priv =
{
  .base = (uintptr_t)TC4X_ASCLIN_BASE(TC4X_CONSOLE_ASCLIN_INDEX),
};

/* Forward declarations of NuttX ops */

static int  tc4x_setup(FAR struct uart_dev_s *dev);
static void tc4x_shutdown(FAR struct uart_dev_s *dev);
static int  tc4x_attach(FAR struct uart_dev_s *dev);
static void tc4x_detach(FAR struct uart_dev_s *dev);
static int  tc4x_receive(FAR struct uart_dev_s *dev, FAR uint32_t *status);
static void tc4x_rxint(FAR struct uart_dev_s *dev, bool enable);
static bool tc4x_rxavailable(FAR struct uart_dev_s *dev);
static void tc4x_send(FAR struct uart_dev_s *dev, int ch);
static void tc4x_txint(FAR struct uart_dev_s *dev, bool enable);
static bool tc4x_txready(FAR struct uart_dev_s *dev);
static bool tc4x_txempty(FAR struct uart_dev_s *dev);

/* UART operations table */

static const struct uart_ops_s g_tc4x_uart_ops =
{
  .setup       = tc4x_setup,
  .shutdown    = tc4x_shutdown,
  .attach      = tc4x_attach,
  .detach      = tc4x_detach,
  .receive     = tc4x_receive,
  .ioctl       = NULL,
  .rxint       = tc4x_rxint,
  .rxavailable = tc4x_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol = NULL,
#endif
  .send        = tc4x_send,
  .txint       = tc4x_txint,
  .txready     = tc4x_txready,
  .txempty     = tc4x_txempty,
};

/* NuttX UART device wrapper */

static uart_dev_t g_tc4x_console_dev =
{
  .isconsole = true,
  .recv =
  {
    .size   = 0,            /* no HW RX buffer */
    .buffer = NULL,
  },
  .xmit =
  {
    .size   = 0,            /* no HW TX buffer */
    .buffer = NULL,
  },
  .ops  = &g_tc4x_uart_ops,
  .priv = &g_tc4x_console_priv,
};

/*-----------------------------------------------------------------------*/
/* UART ops implementation                                              */
/*-----------------------------------------------------------------------*/

static int tc4x_setup(FAR struct uart_dev_s *dev)
{
  FAR struct tc4x_uart_s *priv = (FAR struct tc4x_uart_s *)dev->priv;

  /* Pinmux for UART0 (P14.0 / P14.1) */
  tc4x_uart0_pinmux();

  /* Basic init + default configuration */
  tricore_asclin_init(TC4X_CONSOLE_ASCLIN_INDEX);
  tricore_asclin_configure_default(TC4X_CONSOLE_ASCLIN_INDEX);

  (void)priv;

  return OK;
}

static void tc4x_shutdown(FAR struct uart_dev_s *dev)
{
  FAR struct tc4x_uart_s *priv = (FAR struct tc4x_uart_s *)dev->priv;
  uintptr_t base = priv->base;

  ASCLIN_CLC(base) |= ASCLIN_CLC_DISR;
}

static int tc4x_attach(FAR struct uart_dev_s *dev)
{
  /* No interrupts yet */
  return OK;
}

static void tc4x_detach(FAR struct uart_dev_s *dev)
{
  /* polling driver */
}

/* Receive one character and return status.
 */

static int tc4x_receive(FAR struct uart_dev_s *dev, FAR uint32_t *status)
{
  uint8_t ch;

  if (tricore_asclin_poll_in(TC4X_CONSOLE_ASCLIN_INDEX, &ch) < 0)
    {
      *status = 0;
      return 0;
    }

  *status = 0;
  return (int)ch;
}

static void tc4x_rxint(FAR struct uart_dev_s *dev, bool enable)
{
  /* No interrupts implemented */
  (void)dev;
  (void)enable;
}

static bool tc4x_rxavailable(FAR struct uart_dev_s *dev)
{
  uintptr_t base = ((struct tc4x_uart_s *)dev->priv)->base;
  uint32_t val = ASCLIN_RXFIFOCON(base);

  return ASCLIN_RXFIFOCON_FILL(val) > 0u;
}

static void tc4x_send(FAR struct uart_dev_s *dev, int ch)
{
  tricore_asclin_poll_out(TC4X_CONSOLE_ASCLIN_INDEX, (uint8_t)ch);
}

static void tc4x_txint(FAR struct uart_dev_s *dev, bool enable)
{
  /* No interrupts implemented */
  (void)dev;
  (void)enable;
}

static bool tc4x_txready(FAR struct uart_dev_s *dev)
{
  uintptr_t base = ((struct tc4x_uart_s *)dev->priv)->base;
  uint32_t val = ASCLIN_TXFIFOCON(base);

  return ASCLIN_TXFIFOCON_FILL(val) < ASCLIN_FIFO_DEPTH;
}

static bool tc4x_txempty(FAR struct uart_dev_s *dev)
{
  return tc4x_txready(dev);
}

/*-------------------------------------------------------------------------*/
/* Public interface for arch init                                          */
/*-------------------------------------------------------------------------*/

/* Called from up_initialize() */

void tricore_serialinit(void)
{
//#ifdef CONFIG_UART_CONSOLE
  /* Register console device */
  (void)uart_register("/dev/console", &g_tc4x_console_dev);
  (void)uart_register("/dev/ttyS0", &g_tc4x_console_dev);
//#endif
}

/* Low-level arch debug putc().
 */

void up_putc(int ch)
{
  if (ch == '\n')
    {
      tricore_asclin_poll_out(TC4X_CONSOLE_ASCLIN_INDEX, '\r');
    }

  tricore_asclin_poll_out(TC4X_CONSOLE_ASCLIN_INDEX, (uint8_t)ch);
  return;
}

#endif /* USE_SERIALDRIVER */
