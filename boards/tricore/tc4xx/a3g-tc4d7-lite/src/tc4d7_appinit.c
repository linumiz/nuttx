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
    for (volatile int i = 0; i < 2000; i++) {
	//printf("%d\n", i);
    }
    //usleep(1000);
  }

  return 0;
}

int board_app_initialize(uintptr_t arg)
{
  int pid;
  struct sched_param p = { .sched_priority = 100 };

  pid = kthread_create("w1", 100, 2048, worker, NULL);
  pid = kthread_create("w2", 100, 2048, worker, NULL);

  return OK;
}
