#if 0
#define IFX_CPU_WDT_BASE 0xf0000000
#define IFX_CPU_WDT_OFF 0x18
#define IFX_CPU_WDT_SAFE_OFF 0x184

#define IFX_CPUy_WDT(y) (IFX_CPU_WDT_BASE + (y * 0x30))
#define IFX_CPUy_WDT_CTRLA(y) (IFX_CPUy_WDT(y) + 0x3c)
#define IFX_CPUy_WDT_CTRLB(y) (IFX_CPUy_WDT(y) + 0x40)
#define IFX_CPUy_WDT_STAT(y) (IFX_CPUy_WDT(y) + 0x44)

#define __IO volatile
/** \brief WDTCPU object */
typedef volatile struct _Ifx_WTU_WDTCPU
{
       __IO Ifx_WTU_PROT                        PROTSE;                 /**< \brief 0, CPU0 WDT PROT register safe endinit - SE0*/
       __IO Ifx_WTU_WDTCPU_ACCEN                ACCEN;                  /**< \brief 4, CPU0 WDT access enable registers*/
       __I  Ifx_UReg_8Bit                       reserved_1C[4];         /**< \brief 1C, \internal Reserved */
       __IO Ifx_WTU_SMUFSP                      SMUFSP;                 /**< \brief 20, CPU0 WDT SMU partitions register*/
       __IO Ifx_WTU_CTRLA                       CTRLA;                  /**< \brief 24, CPU0 WDT control register A*/
       __IO Ifx_WTU_CTRLB                       CTRLB;                  /**< \brief 28, CPU0 WDT control register B*/
       __I  Ifx_WTU_STAT                        STAT;                   /**< \brief 2C, CPU0 WDT status register*/
} Ifx_WTU_WDTCPU;


/** \brief WDT Control Register A   */
typedef union
{
    __IO Ifx_UReg_32Bit U;                 /**< \brief Unsigned access */
    __IO Ifx_SReg_32Bit I;                 /**< \brief Signed access */
    Ifx_WTU_CTRLA_Bits B;                  /**< \brief Bitfield access */
} Ifx_WTU_CTRLA;

/** \brief WDT Control Register B   */
typedef union
{
    __IO Ifx_UReg_32Bit U;                 /**< \brief Unsigned access */
    __IO Ifx_SReg_32Bit I;                 /**< \brief Signed access */
    Ifx_WTU_CTRLB_Bits B;                  /**< \brief Bitfield access */
} Ifx_WTU_CTRLB;


static inline void wdt_disable(uintptr_t base)
{
	volatile Ifx_WTU_WDTCPU *wdt = (Ifx_WTU_WDTCPU *)base;
	Ifx_WTU_CTRLA wtu_ctrla;
	Ifx_WTU_CTRLB wtu_ctrlb = {.B.DR = 1};

	wtu_ctrla = wdt->CTRLA;
	if (wtu_ctrla.B.LCK) {
		wtu_ctrla.B.LCK = 0;
		wtu_ctrla.B.PW ^= 0x007F;

		wdt->CTRLA = wtu_ctrla;
	}
	wdt->CTRLB = wtu_ctrlb;

	wtu_ctrla.B.LCK = 1;
	wdt->CTRLA = wtu_ctrla;
}

void tricore_wdt_disable(void)
{
  wdt_disable(IFX_CPUy_WDT(0) + IFX_CPU_WDT_SAFE_OFF);
}
#endif

#include <nuttx/bits.h>
#include <tricore_internal.h>

#define IFX_CPU_WDTSYS_CTRLA 0xf00001A8
#define IFX_CPU_WDTSYS_CTRLB 0xf00001AC
void tricore_wdt_disable(void)
{
	uint32_t ctrla = getreg32(IFX_CPU_WDTSYS_CTRLA);
	uint32_t ctrlb = getreg32(IFX_CPU_WDTSYS_CTRLB) | BIT(0); // DR = 1;

	if (ctrla & BIT(0)) {
		ctrla &= ~BIT(0);
		ctrla ^= (0x7f << 1);
		putreg32(ctrla, IFX_CPU_WDTSYS_CTRLA);
	}

	putreg32(ctrlb, IFX_CPU_WDTSYS_CTRLB);

	ctrla |= BIT(0);
	putreg32(ctrla, IFX_CPU_WDTSYS_CTRLA);
}
