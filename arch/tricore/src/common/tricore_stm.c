#include <stdint.h>
#include <time.h>
#include <arch/arch.h>
#include <nuttx/irq.h>

#include <tricore_irq.h>
#include <tricore_stm.h>
#include <tricore_internal.h>

static uint8_t core_id;

static inline uint32_t get_time32(void)
{
	return getreg32(IFX_STM_ABS(core_id));
}

static inline uint64_t get_time64(void)
{
	return getreg64(IFX_STM_ABS(core_id));
}

static inline void set_compare(uint32_t cmp)
{
	putreg32(cmp, IFX_STM_CMP0(core_id, IFX_STM_DEFAULT));
}

static int tricore_timerisr(int irq, uint32_t *regs, void *arg)
{
	uint32_t val;

	val = getreg32(IFX_STM_CMP0(core_id, IFX_STM_DEFAULT));
	set_compare((uint32_t)val + CLOCKS_PER_SEC);

	nxsched_process_timer();

	/* clear interrupt */
	val = getreg32(IFX_STM_ISCR(core_id, IFX_STM_DEFAULT));
	val |= IFX_STM_ISCR_CMP0IRR;
	putreg32(val, IFX_STM_ISCR(core_id, IFX_STM_DEFAULT));

	return 0;
}

//FIXME generalize for mfcr asm
static inline unsigned int tricore_get_core_id(void)
{
	unsigned int core_id;

	__asm__ __volatile__ ("mfcr %0, %1" : "=d"(core_id) : "i"(IFX_CPU_CORE_ID): "memory");

	return core_id & IFX_CPU_CORE_ID_MASK;
}

void up_timer_initialize(void)
{
	uint64_t last_count;
	uint64_t last_ticks;
	uint32_t val;

	core_id = tricore_get_core_id();

	val = getreg32(IFX_STM_CMCON(core_id, IFX_STM_DEFAULT));
	val |= IFX_STM_CMCON_MSIZE0;
	val &= ~IFX_STM_CMCON_MSTART0;
	putreg32(val, IFX_STM_CMCON(core_id, IFX_STM_DEFAULT));

	val = getreg32(IFX_STM_ISCR(core_id, IFX_STM_DEFAULT));
	val |= IFX_STM_ISCR_CMP0IRR;
	putreg32(val, IFX_STM_ISCR(core_id, IFX_STM_DEFAULT));

	last_ticks = get_time64() / CLOCKS_PER_SEC;
	last_count = (last_ticks + 1) * CLOCKS_PER_SEC;

	val = getreg32(IFX_STM_ICR(core_id, IFX_STM_DEFAULT));
	val &= ~IFX_STM_ICR_CMP0OS;
	val |= IFX_STM_ICR_CMP0EN;
	putreg32(val, IFX_STM_ICR(core_id, IFX_STM_DEFAULT));

	irq_attach(IFX_STM_IR_SRN(IFX_STM_DEFAULT), (xcpt_t)tricore_timerisr, NULL);
	set_compare((uint32_t)last_count + CLOCKS_PER_SEC);
	up_enable_irq(IFX_STM_IR_SRN(IFX_STM_DEFAULT));
}
