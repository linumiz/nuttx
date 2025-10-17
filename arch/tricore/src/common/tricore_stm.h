#ifndef TRICORE_STM_H
#define TRICORE_STM_H

#include <nuttx/bits.h>

#if 0
enum ifx_cpux {
	IFX_CPU0,
	IFX_CPU1,
	IFX_CPU2,
	IFX_CPU3,
	IFX_CPU4,
	IFX_CPU5,
	IFX_CPUcs,
};
#endif

#define IFX_STM_CPU_BASE 0xf8800000
#define IFX_STM_CPUx_BASE(x) (IFX_STM_CPU_BASE + (x * 0x40000))

#define IFX_STM_CLC(x) (IFX_STM_CPUx_BASE(x) + 0x0)
#define IFX_STM_OCS(x) (IFX_STM_CPUx_BASE(x) + 0x4)
#define IFX_STM_ABS(x) (IFX_STM_CPUx_BASE(x) + 0x20)

#define IFX_STM_CMP0(x, n) (IFX_STM_CPUx_BASE(x) + 0x100 + ((n >> 1) * 0x20))
#define IFX_STM_CMP1(x, n) (IFX_STM_CPUx_BASE(x) + 0x104 + ((n >> 1) * 0x20))
#define IFX_STM_CMCON(x, n) (IFX_STM_CPUx_BASE(x) + 0x108 + ((n >> 1) * 0x20))
#define IFX_STM_ICR(x, n) (IFX_STM_CPUx_BASE(x) + 0x10c + ((n >> 1) * 0x20))
#define IFX_STM_ISCR(x, n) (IFX_STM_CPUx_BASE(x) + 0x110 + ((n >> 1) * 0x20))
#define IFX_STM_ISR(x, n) (IFX_STM_CPUx_BASE(x) + 0x114 + ((n >> 1) * 0x20))
#define IFX_STM_RELITM(x, n) (IFX_STM_CPUx_BASE(x) + 0x118 + ((n >> 1) * 0x20))

#define IFX_STM_DEFAULT 2

/* IFX_STM_CLC */
#define IFX_STM_CLC_DISR BIT(0)
#define IFX_STM_CLC_DISS BIT(1)

/* IFX_STM_CMCON */
#define IFX_STM_CMCON_MSIZE0 GENMASK(4, 0)
#define IFX_STM_CMCON_MSTART0 GENMASK(12, 8)
#define IFX_STM_CMCON_RELCOMP0 BIT(15)
#define IFX_STM_CMCON_MSIZE1 GENMASK(16, 20)
#define IFX_STM_CMCON_MSTART1 GENMASK(24, 28)
#define IFX_STM_CMCON_RELCOMP1 BIT(31)

/* IFX_STM_ICR */
#define IFX_STM_ICR_CMP0EN BIT(0)
#define IFX_STM_ICR_CMP0OS BIT(1)
#define IFX_STM_ICR_CMP1EN BIT(4)
#define IFX_STM_ICR_CMP1OS BIT(5)

/* IFX_STM_ISCR */
#define IFX_STM_ISCR_CMP0IRR BIT(0)
#define IFX_STM_ISCR_CMP0IRS BIT(1)
#define IFX_STM_ISCR_CMP1IRR BIT(2)
#define IFX_STM_ISCR_CMP1IRS BIT(3)

/* IFX_STM_ISR */
#define IFX_STM_ISR_CMP0IR BIT(0)
#define IFX_STM_ISR_CMP1IR BIT(1)

#endif
