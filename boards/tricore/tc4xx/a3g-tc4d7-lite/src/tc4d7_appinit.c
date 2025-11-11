#include <sys/types.h>

#include <nuttx/config.h>
#include <nuttx/kthread.h>
#include <sched.h>
#include <unistd.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>

static int worker(int argc, char **argv)
{
  for (;;) {
    for (volatile int i = 0; i < 2000; i++);
    usleep(1000);
  }
  return 0;
}

int board_app_initialize(uintptr_t arg)
{
#if 0
  int pid;
  struct sched_param p = { .sched_priority = 100 };

  pid = kthread_create("w1", 101, 2048, worker, NULL);
  sched_setscheduler(pid, SCHED_RR, &p);
  pid = kthread_create("w2", 102, 2048, worker, NULL);
  sched_setscheduler(pid, SCHED_RR, &p);
#endif

  return OK;
}
