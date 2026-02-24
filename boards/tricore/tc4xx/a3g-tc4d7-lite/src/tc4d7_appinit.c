#include <sys/types.h>
#include <debug.h>
#include <sys/sysinfo.h>

#include <nuttx/config.h>
#include <nuttx/kthread.h>
#include <sched.h>
#include <unistd.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>

int board_app_initialize(uintptr_t arg)
{
  return OK;
}
