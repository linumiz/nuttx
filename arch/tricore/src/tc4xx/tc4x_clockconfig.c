/* tc4x_clockconfig.c
 *
 * Aurix TC4x system / peripheral clock tree init
 *
 * - Configures external oscillator (fosc)
 * - Configures SYSPLL (~500 MHz) and PERPLL (~160 MHz)
 * - Programs CCU dividers so each domain runs as close as possible
 *   to its configured TARGET_HZ (see tc4x_clock.h) without exceeding it
 * - Selects SYS clock from SYSPLL and PER clock from PERPLL
 *
 */

#include <nuttx/config.h>
#include <nuttx/bits.h>
#include <stdint.h>
#include <nuttx/arch.h>
#include "tricore_internal.h"

#include <nuttx/arch.h>
#include "hardware/tc4x_clock.h"

/* --------------------------------------------------------------------------
 * Helpers
 * -------------------------------------------------------------------------- */

#ifndef DIV_ROUND_UP
#  define DIV_ROUND_UP(n, d)  (((n) + (d) - 1u) / (d))
#endif

static inline void tc4x_busywait(unsigned int loops)
{
  volatile unsigned int i;
  for (i = 0; i < loops; i++)
    {
      __asm__ __volatile__("nop");
    }
}

/* Convenience: field prep without needing ffs() since we know SHIFT. */
#define FIELD_PREP(mask, shift, val)   (((uint32_t)(val) << (shift)) & (mask))

/* Shortcuts for register addresses */
#define REGADDR(off)   (TC4X_CLOCK_BASE + (off))

static inline void tc4x_ccu_wait_unlocked(void)
{
  /* Busy-wait until CCUSTAT.LCK == 0 */
  while (getreg32(REGADDR(TC4X_CLOCK_CCUSTAT_OFFSET)) & TC4X_CCUSTAT_LCK)
    {
      __asm__ __volatile__("nop");
    }
}

/* --------------------------------------------------------------------------
 * Oscillator init (external crystal / clock)
 * -------------------------------------------------------------------------- */

void tc4x_osc_init(void)
{
  uint32_t addr;
  uint32_t val;

  /* OSCCON: MODE = external clock, INSEL = XTAL input */
  addr = REGADDR(TC4X_CLOCK_OSCCON_OFFSET);
  val  = getreg32(addr);

  val &= ~(TC4X_OSCCON_MODE_MASK | TC4X_OSCCON_INSEL_MASK);
  val |= TC4X_OSCCON_MODE_EXTCLK;
  val |= TC4X_OSCCON_INSEL_XTAL;

  tc4x_ccu_wait_unlocked();
  putreg32(val, addr);

  /* OSCMON1: OSCVAL = fosc_MHz - 15 */
  {
    uint32_t mhz = TC4X_FOSC_HZ / 1000000u;
    uint32_t oscval = 0;

    if (mhz > 15u)
      {
        oscval = mhz - 15u;
      }

    val  = FIELD_PREP(TC4X_OSCMON1_OSCVAL_MASK,
                      TC4X_OSCMON1_OSCVAL_SHIFT,
                      oscval);

    /* Enable monitors if you like; here we keep it minimal */
    tc4x_ccu_wait_unlocked();
    putreg32(val, REGADDR(TC4X_CLOCK_OSCMON1_OFFSET));
  }

  /* Give oscillator some time to stabilise */
  tc4x_busywait(100000);
}

/* --------------------------------------------------------------------------
 * SYSPLL init: fVCO = 20 MHz * 50 / 2 = 500 MHz, fpll0 = 500 MHz
 * -------------------------------------------------------------------------- */

void tc4x_syspll_init(void)
{
  uint32_t addr;
  uint32_t val;

  /* SYSPLLCON0: NDIV, PDIV, power up, restart lock detect */
  addr = REGADDR(TC4X_CLOCK_SYSPLLCON0_OFFSET);

  val  = 0;
  val |= FIELD_PREP(TC4X_SYSPLLCON0_NDIV_MASK,
                    TC4X_SYSPLLCON0_NDIV_SHIFT,
                    TC4X_SYSPLL_NDIV - 1u);
  val |= FIELD_PREP(TC4X_SYSPLLCON0_PDIV_MASK,
                    TC4X_SYSPLLCON0_PDIV_SHIFT,
                    TC4X_SYSPLL_PDIV - 1u);
  val |= TC4X_SYSPLLCON0_PLLPWR;
  val |= TC4X_SYSPLLCON0_RESLD;

  tc4x_ccu_wait_unlocked();
  putreg32(val, addr);

  /* Wait for SYSPLL power status */
  addr = REGADDR(TC4X_CLOCK_SYSPLLSTAT_OFFSET);
  while ((getreg32(addr) & TC4X_PLLSTAT_PWRSTAT) == 0u)
    {
      /* spin */
    }

  /* SYSPLLCON1: K2DIV / K2PREDIV (output divider) */
  addr = REGADDR(TC4X_CLOCK_SYSPLLCON1_OFFSET);

  val  = 0;
  val |= FIELD_PREP(TC4X_SYSPLLCON1_K2DIV_MASK,
                    TC4X_SYSPLLCON1_K2DIV_SHIFT,
                    TC4X_SYSPLL_K2DIV - 1u);
  val |= FIELD_PREP(TC4X_SYSPLLCON1_K2PREDIV_MASK,
                    TC4X_SYSPLLCON1_K2PREDIV_SHIFT,
                    TC4X_SYSPLL_K2PREDIV - 1u);

  tc4x_ccu_wait_unlocked();
  putreg32(val, addr);

  /* Wait for SYSPLL lock */
  addr = REGADDR(TC4X_CLOCK_SYSPLLSTAT_OFFSET);
  while ((getreg32(addr) & TC4X_PLLSTAT_PLLLOCK) == 0u)
    {
      /* spin */
    }
}

/* --------------------------------------------------------------------------
 * PERPLL init: fVCO = 20 MHz * 40 / 1 = 800 MHz, fpll1 = 160 MHz
 * -------------------------------------------------------------------------- */

void tc4x_perpll_init(void)
{
  uint32_t addr;
  uint32_t val;

  /* PERPLLCON0: NDIV, PDIV, power up, restart lock detect */
  addr = REGADDR(TC4X_CLOCK_PERPLLCON0_OFFSET);

  val  = 0;
  val |= FIELD_PREP(TC4X_PERPLLCON0_NDIV_MASK,
                    TC4X_PERPLLCON0_NDIV_SHIFT,
                    TC4X_PERPLL_NDIV - 1u);
  val |= FIELD_PREP(TC4X_PERPLLCON0_PDIV_MASK,
                    TC4X_PERPLLCON0_PDIV_SHIFT,
                    TC4X_PERPLL_PDIV - 1u);
  val |= TC4X_PERPLLCON0_PLLPWR;
  val |= TC4X_PERPLLCON0_RESLD;

  tc4x_ccu_wait_unlocked();
  putreg32(val, addr);

  /* Wait for PERPLL power status */
  addr = REGADDR(TC4X_CLOCK_PERPLLSTAT_OFFSET);
  while ((getreg32(addr) & TC4X_PLLSTAT_PWRSTAT) == 0u)
    {
      /* spin */
    }

  /* PERPLLCON1: K2DIV / K2PREDIV (output divider) */
  addr = REGADDR(TC4X_CLOCK_PERPLLCON1_OFFSET);

  val  = 0;
  val |= FIELD_PREP(TC4X_PERPLLCON1_K2DIV_MASK,
                    TC4X_PERPLLCON1_K2DIV_SHIFT,
                    TC4X_PERPLL_K2DIV - 1u);
  val |= FIELD_PREP(TC4X_PERPLLCON1_K2PREDIV_MASK,
                    TC4X_PERPLLCON1_K2PREDIV_SHIFT,
                    TC4X_PERPLL_K2PREDIV - 1u);

  tc4x_ccu_wait_unlocked();
  putreg32(val, addr);

  /* Wait for PERPLL lock */
  addr = REGADDR(TC4X_CLOCK_PERPLLSTAT_OFFSET);
  while ((getreg32(addr) & TC4X_PLLSTAT_PLLLOCK) == 0u)
    {
      /* spin */
    }
}

/* --------------------------------------------------------------------------
 * CCU: program dividers from PLL outputs to domains (SRI/SPB/CPB/...)
 * -------------------------------------------------------------------------- */

void tc4x_ccu_set_dividers(void)
{
  uint64_t sys_vco;
  uint32_t fsource0;
  uint64_t per_vco;
  uint32_t fsource1;

  uint32_t fsri_div;
  uint32_t fspb_div;
  uint32_t ftpb_div;
  uint32_t fstm_div;
  uint32_t fleth_div;
  uint32_t ffsi_div;
  uint32_t fgeth_div;
  uint32_t fegtm_div;
  uint32_t fmcanh_div;

  uint32_t fmcani_div;
  uint32_t fasclinf_div;
  uint32_t fasclinsi_div;
  uint32_t fqspi_div;
  uint32_t fi2c_div;

  uint32_t val;

  /* Rebuild PLL output freqs using compile-time config from header */

  sys_vco = (uint64_t)TC4X_FOSC_HZ *
            (uint64_t)TC4X_SYSPLL_NDIV /
            (uint64_t)TC4X_SYSPLL_PDIV;
  fsource0 = (uint32_t)(sys_vco /
              ((uint64_t)TC4X_SYSPLL_K2DIV * (uint64_t)TC4X_SYSPLL_K2PREDIV));

  per_vco = (uint64_t)TC4X_FOSC_HZ *
            (uint64_t)TC4X_PERPLL_NDIV /
            (uint64_t)TC4X_PERPLL_PDIV;
  fsource1 = (uint32_t)(per_vco /
              ((uint64_t)TC4X_PERPLL_K2DIV * (uint64_t)TC4X_PERPLL_K2PREDIV));

  /* ---- System domain dividers from fsource0 (500 MHz) ---- */

  fsri_div  = DIV_ROUND_UP(fsource0, TC4X_FSRI_TARGET_HZ) & 0xFu;
  fspb_div  = DIV_ROUND_UP(fsource0, TC4X_FSPB_TARGET_HZ) & 0xFu;
  ftpb_div  = DIV_ROUND_UP(fsource0, TC4X_FTPB_TARGET_HZ) & 0xFu;
  fstm_div  = DIV_ROUND_UP(fsource0, TC4X_FSTM_TARGET_HZ) & 0xFu;
  fleth_div = DIV_ROUND_UP(fsource0, TC4X_FLETH_TARGET_HZ) & 0xFu;

  ffsi_div  = fsri_div;                      /* simple choice */
  fgeth_div = fleth_div;                     /* same target as lethy */
  fegtm_div = 1u;                            /* full speed for eGTM */
  fmcanh_div = DIV_ROUND_UP(fsource0, 100000000u) & 0xFu; /* 100 MHz */

  /* ---- Peripheral domain dividers from fsource1 (160 MHz) ---- */

  fmcani_div    = DIV_ROUND_UP(fsource1, 80000000u) & 0xFu; /* 80 MHz */
  fasclinf_div  = DIV_ROUND_UP(fsource1, 80000000u) & 0xFu; /* 80 MHz */
  fasclinsi_div = DIV_ROUND_UP(fsource1, 80000000u) & 0xFu; /* 80 MHz */

  fqspi_div     = DIV_ROUND_UP(fsource1, 200000000u) & 0xFu; /* <= 160, so 1 */
  fi2c_div      = DIV_ROUND_UP(fsource1, 66666667u) & 0xFu;  /* ~66.7 MHz */

  /* ---- Program SYSCCUCON0 ---- */

  val  = 0;
  val |= FIELD_PREP(TC4X_SYSCCUCON0_SPBDIV_MASK,
                    TC4X_SYSCCUCON0_SPBDIV_SHIFT,
                    fspb_div);
  val |= FIELD_PREP(TC4X_SYSCCUCON0_TPBDIV_MASK,
                    TC4X_SYSCCUCON0_TPBDIV_SHIFT,
                    ftpb_div);
  val |= FIELD_PREP(TC4X_SYSCCUCON0_SRIDIV_MASK,
                    TC4X_SYSCCUCON0_SRIDIV_SHIFT,
                    fsri_div);
  val |= FIELD_PREP(TC4X_SYSCCUCON0_FSIDIV_MASK,
                    TC4X_SYSCCUCON0_FSIDIV_SHIFT,
                    ffsi_div);
  val |= FIELD_PREP(TC4X_SYSCCUCON0_STMDIV_MASK,
                    TC4X_SYSCCUCON0_STMDIV_SHIFT,
                    fstm_div);

  val |= TC4X_SYSCCUCON0_FSI2DIV;

  /* LPDIV left at reset */
  val |= TC4X_SYSCCUCON0_UP;

  tc4x_ccu_wait_unlocked();
  putreg32(val, REGADDR(TC4X_CLOCK_SYSCCUCON0_OFFSET));

  /* ---- Program SYSCCUCON1 ---- */

  val  = 0;
  val |= FIELD_PREP(TC4X_SYSCCUCON1_GETHDIV_MASK,
                    TC4X_SYSCCUCON1_GETHDIV_SHIFT,
                    fgeth_div);
  val |= FIELD_PREP(TC4X_SYSCCUCON1_EGTMDIV_MASK,
                    TC4X_SYSCCUCON1_EGTMDIV_SHIFT,
                    fegtm_div);
  val |= FIELD_PREP(TC4X_SYSCCUCON1_MCANHDIV_MASK,
                    TC4X_SYSCCUCON1_MCANHDIV_SHIFT,
                    fmcanh_div);
  val |= FIELD_PREP(TC4X_SYSCCUCON1_LETHDIV_MASK,
                    TC4X_SYSCCUCON1_LETHDIV_SHIFT,
                    fleth_div);
  /* CANXLHDIV left at reset */
  val |= TC4X_SYSCCUCON1_UP;

  tc4x_ccu_wait_unlocked();
  putreg32(val, REGADDR(TC4X_CLOCK_SYSCCUCON1_OFFSET));

  /* ---- Program PERCCUCON0 ---- */

  val  = 0;
  val |= FIELD_PREP(TC4X_PERCCUCON0_MCANDIV_MASK,
                    TC4X_PERCCUCON0_MCANDIV_SHIFT,
                    fmcani_div);
  /* CLKSELMCAN = 0: use PERPLL (fsource1) */
  val |= FIELD_PREP(TC4X_PERCCUCON0_CLKSELMCAN_MASK,
                    TC4X_PERCCUCON0_CLKSELMCAN_SHIFT,
                    0);
  /* MSCDIV, CLKSELMSC left at reset */
  val |= FIELD_PREP(TC4X_PERCCUCON0_QSPIDIV_MASK,
                    TC4X_PERCCUCON0_QSPIDIV_SHIFT,
                    fqspi_div);
  /* CLKSELQSPI = 0: PERPLL */
  val |= FIELD_PREP(TC4X_PERCCUCON0_CLKSELQSPI_MASK,
                    TC4X_PERCCUCON0_CLKSELQSPI_SHIFT,
                    0);
  val |= FIELD_PREP(TC4X_PERCCUCON0_I2CDIV_MASK,
                    TC4X_PERCCUCON0_I2CDIV_SHIFT,
                    fi2c_div);
  /* PPUDIV left at reset */

  // tc4x_ccu_wait_unlocked(); // FIXME something wrong here?
  putreg32(val, REGADDR(TC4X_CLOCK_PERCCUCON0_OFFSET));

  /* ---- Program PERCCUCON1 ---- */

  val  = 0;
  val |= FIELD_PREP(TC4X_PERCCUCON1_ASCLINFDIV_MASK,
                    TC4X_PERCCUCON1_ASCLINFDIV_SHIFT,
                    fasclinf_div);
  val |= FIELD_PREP(TC4X_PERCCUCON1_ASCLINSDIV_MASK,
                    TC4X_PERCCUCON1_ASCLINSDIV_SHIFT,
                    fasclinsi_div);
  /* CLKSELASCLINS = 0: PERPLL */
  val |= FIELD_PREP(TC4X_PERCCUCON1_CLKSELASCLINS_MASK,
                    TC4X_PERCCUCON1_CLKSELASCLINS_SHIFT,
                    0);
  /* CANXL, ADC, ERAY, xSPI, SDMMC, HSCT, LETH100 power bits left as needed.
   * For now, we only ensure LETH100 is on if you want Ethernet 100M.
   */
  val |= TC4X_PERCCUCON1_LETH100PERON;

  tc4x_ccu_wait_unlocked();
  putreg32(val, REGADDR(TC4X_CLOCK_PERCCUCON1_OFFSET));
}

/* --------------------------------------------------------------------------
 * Select root sources: system from SYSPLL, peripheral from PERPLL
 * -------------------------------------------------------------------------- */

void tc4x_select_sources(void)
{
  uint32_t addr = REGADDR(TC4X_CLOCK_CCUCON_OFFSET);
  uint32_t val  = getreg32(addr);

  /* SYS clock source */
  val &= ~TC4X_CCUCON_CLKSELS_MASK;
  val |= FIELD_PREP(TC4X_CCUCON_CLKSELS_MASK,
                    TC4X_CCUCON_CLKSELS_SHIFT,
                    TC4X_SYSCLK_SOURCE_PLL);

  /* PER clock source */
  val &= ~TC4X_CCUCON_CLKSELP_MASK;
  val |= FIELD_PREP(TC4X_CCUCON_CLKSELP_MASK,
                    TC4X_CCUCON_CLKSELP_SHIFT,
                    TC4X_PERCLK_SOURCE_PLL);

  tc4x_ccu_wait_unlocked();
  putreg32(val, addr);
}

/* --------------------------------------------------------------------------
 * Public entry point
 * -------------------------------------------------------------------------- */

#if 0
#define PORT_BASE(port)		0xF003A000 + (port * 0x400)
#define PORT_OMR(port)		PORT_BASE(port) + 0x3C
#define PORT_PADCFG(port, pin)	PORT_BASE(port) + 0x304 + (pin * 0x10)
static void tc4x_led_blink(uint32_t port, uint32_t pin)
{

	uint32_t extcon = REGADDR(TC4X_CLOCK_EXTCON_OFFSET);
	tc4x_ccu_wait_unlocked();
	putreg32(BIT(0), extcon);

//	uint32_t mask = GENMASK(7, 4) | GENMASK(1, 0);
	uint32_t mode = 6;
//	void *addr = PORT_PADCFG(port, pin);
	void *addr = 0xF003FF14;

//	modreg32(mode, mask, addr);
	//putreg32(mode << 4 | BIT(0), addr);
	tc4x_ccu_wait_unlocked();
	putreg32(0x61, addr);
	tc4x_busywait(100000);
	while (1) {
		putreg32(BIT(pin) | BIT(pin + 16), PORT_OMR(port));
		tc4x_busywait(100000 * 100);
	}
}
tc4x_led_blink(23, 1);
#endif

void up_clockconfig(void)
{
  /* External oscillator */
  tc4x_osc_init();

  /* PLLs */
  tc4x_syspll_init();
  tc4x_perpll_init();

  /* CCU dividers for CPU + bus + periph clocks */
  tc4x_ccu_set_dividers();

  /* Select PLLs as clock sources */
  tc4x_select_sources();
}
