#include <stddef.h>
#include <stdint.h>

enum tricore_trap_class {
	TRICORE_CLASS_MMU,
	TRICORE_CLASS_IP,
	TRICORE_CLASS_IE,
	TRICORE_CLASS_CTX,
	TRICORE_CLASS_BUS,
	TRICORE_CLASS_ASSERT,
	TRICORE_CLASS_SYSCALL,
	TRICORE_CLASS_NMI
};

void tricore_trap_handler(uint32_t class, uint32_t tin)
{
	switch (class) {
	case TRICORE_CLASS_MMU:
		break;
	case TRICORE_CLASS_IP:
		break;
	case TRICORE_CLASS_IE:
		break;
	case TRICORE_CLASS_CTX:
		break;
	case TRICORE_CLASS_BUS:
		break;
	case TRICORE_CLASS_ASSERT:
		break;
	case TRICORE_CLASS_SYSCALL:
		tricore_svcall(NULL);
		break;
	case TRICORE_CLASS_NMI:
		break;
	}
}
