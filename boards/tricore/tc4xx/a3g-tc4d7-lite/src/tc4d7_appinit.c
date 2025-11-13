#include <sys/types.h>

#include <nuttx/config.h>
#include <nuttx/kthread.h>
#include <sched.h>
#include <unistd.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>

int g_app_pid1, g_app_pid2;
void siguser_action(int signo, siginfo_t *siginfo, void *arg)
{
	printf("signal %d\n", signo);
}

static int worker(int argc, char **argv)
{
  int status;
  struct sigaction act;
  struct sigaction oact1;

  memset(&act, 0, sizeof(struct sigaction));
  act.sa_sigaction = siguser_action;
  act.sa_flags     = SA_SIGINFO;

  sigemptyset(&act.sa_mask);

  status = sigaction(SIGUSR1, &act, &oact1);

  for (;;) {
    for (volatile int i = 0; i < 2000; i++) {
	printf("%d\n", i);
    }
    usleep(1000);
  }

  return 0;
}

int board_app_initialize(uintptr_t arg)
{
	g_app_pid1 = kthread_create("w1", 100, 2048, worker, NULL);
	g_app_pid2 = kthread_create("w2", 100, 2048, worker, NULL);

	while (1) {
		for (volatile int i = 0; i < 1000; i++) {
			printf("%d\n", i);
		}
		usleep(10000);
		kill(g_app_pid1, SIGUSR1);
		usleep(10000);
		kill(g_app_pid2, SIGUSR1);
	}

	return OK;
}
