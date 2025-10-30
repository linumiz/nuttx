#ifndef TRICORE_ARCH_H
#define TRICORE_ARCH_H

#define IFX_CPU_VCON0					0xB000
#define IFX_CPU_PCXI					0xFE00
#define IFX_CPU_PSW					0xFE04
#define IFX_CPU_CORE_ID					0xFE1C
#define IFX_CPU_BIV					0xFE20
#define IFX_CPU_BTV					0xFE24
#define IFX_CPU_ISP					0xFE28
#define IFX_CPU_FCX					0xFE38
#define IFX_CPU_LCX					0xFE3C

#define IFX_CFG_SSW_PSW_DEFAULT			0x980
#define IFX_CPU_CORE_ID_MASK GENMASK(2, 0)

/* Upper CSA */

#define REG_UPCXI        0
#define REG_PSW          1
#define REG_A10          2
#define REG_UA11         3
#define REG_D8           4
#define REG_D9           5
#define REG_D10          6
#define REG_D11          7
#define REG_A12          8
#define REG_A13          9
#define REG_A14          10
#define REG_A15          11
#define REG_D12          12
#define REG_D13          13
#define REG_D14          14
#define REG_D15          15

/* Lower CSA */

#define REG_LPCXI        0
#define REG_LA11         1
#define REG_A2           2
#define REG_A3           3
#define REG_D0           4
#define REG_D1           5
#define REG_D2           6
#define REG_D3           7
#define REG_A4           8
#define REG_A5           9
#define REG_A6           10
#define REG_A7           11
#define REG_D4           12
#define REG_D5           13
#define REG_D6           14
#define REG_D7           15

#define REG_RA           REG_UA11
#define REG_SP           REG_A10
#define REG_UPC          REG_UA11

#define REG_LPC          REG_LA11

#define TC_CONTEXT_REGS  (16)

#define XCPTCONTEXT_REGS (TC_CONTEXT_REGS)
#define XCPTCONTEXT_SIZE (sizeof(void *) * TC_CONTEXT_REGS)


/* PSW: Program Status Word Register */

#define PSW_CDE         (1 << 7) /* Bits 7: Call Depth Count Enable */
#define PSW_IS          (1 << 9) /* Bits 9: Interrupt Stack Control */
#define PSW_IO          (10)     /* Bits 10-11: Access Privilege Level Control (I/O Privilege) */
#  define PSW_IO_USER0      (0 << PSW_IO)
#  define PSW_IO_USER1      (1 << PSW_IO)
#  define PSW_IO_SUPERVISOR (2 << PSW_IO)

/* PCXI: Previous Context Information and Pointer Register */

#define PCXI_UL         (1 << 20) /* Bits 20: Upper or Lower Context Tag */
#define PCXI_PIE        (1 << 21) /* Bits 21: Previous Interrupt Enable */

/* FCX: Free CSA List Head Pointer Register */

#define FCX_FCXO        (0)       /* Bits 0-15: FCX Offset Address */
#define FCX_FCXS        (16)      /* Bits 16-19: FCX Segment Address */
#define FCX_FCXO_MASK   (0xffff << FCX_FCXO)
#define FCX_FCXS_MASK   (0xf    << FCX_FCXS)
#define FCX_FREE        (FCX_FCXS_MASK | FCX_FCXO_MASK) /* Free CSA manipulation */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
struct xcptcontext
{
  /* These are saved copies of the context used during
   * signal processing.
   */

  uintptr_t *saved_regs;

  /* Register save area with XCPTCONTEXT_SIZE, only valid when:
   * 1.The task isn't running or
   * 2.The task is interrupted
   * otherwise task is running, and regs contain the stale value.
   */

  uintptr_t *regs;
};
#endif /* __ASSEMBLY__ */

#define IFX_CFG_SSW_CSA_BOOT_PTR_START		(0x7010EC00)
#define IFX_CFG_SSW_CSA_USTACK_PTR		(0x7010EB00)
#define	IFX_CFG_DSPR0_START			(0x70000000)
#define IFX_LWSRZ_VMY				(0xF4430c00)

#ifndef __ASSEMBLY__
#define IFX_MTCR(reg, val)  { __asm__ __volatile__ ("dsync" : : : "memory"); \
			      __asm__ __volatile__ ("mtcr %0,%1"::"i"(reg),"d"(val): "memory"); \
			      __asm__ __volatile__ ("isync" : : : "memory")}

#define IFX_MFCR(reg, val)  {__asm__ __volatile__ ("mfcr %0,%1": "=d"(val) :"i"(reg): "memory")}
#endif /* __ASSEMBLY__ */

#endif /* TRICORE_ARCH_H */
