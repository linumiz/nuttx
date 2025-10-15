
union z_tricore_context __kstackmem __aligned(4 * 16) z_tricore_csa[CONFIG_TRICORE_CSA_COUNT];

/*
- How to configure interrupt
	- timer init ->isr
	- i2c ??
- What needs to be done when intrrupt occurs
- ?
*/
int up_prioritize_irq(int irq, int priority)
{
}
