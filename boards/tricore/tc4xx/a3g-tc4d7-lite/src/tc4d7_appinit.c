#include <sys/types.h>
#include <debug.h>
#include <sys/sysinfo.h>

#include <nuttx/config.h>
#include <nuttx/kthread.h>
#include <sched.h>
#include <unistd.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>

#include "tc4xx_gpio.h"

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
	uint32_t cnt = 0;
	struct sysinfo info;
	bool ledon = true;
	gpio_pinset_t led1 = GPIO_PAD_CFG(GPIO_PORT3, GPIO_PIN9, GPIO_OUTPUT, GPIO_PAD_CONFIG_OUT_GPIO);

#if 0
	g_app_pid1 = kthread_create("w1", 100, 2048, worker, NULL);
	g_app_pid2 = kthread_create("w2", 100, 2048, worker, NULL);
#endif

	aurix_config_gpio(led1);
	while (1) {
		aurix_gpio_write(led1, (ledon = !ledon));
		//tc4x_asclin0_puts("Hello from NuttX on ASCLIN0!\n");
		sysinfo(&info);
		lowsyslog("Hello from NuttX on ASCLIN0 %d -- %lld!\n", cnt++, info.uptime);
		usleep(500 * 1000);
#if 0
		for (volatile int i = 0; i < 1000; i++) {
			printf("%d\n", i);
		}
		usleep(10000);
		kill(g_app_pid1, SIGUSR1);
		usleep(10000);
		kill(g_app_pid2, SIGUSR1);
#endif
	}

	return OK;
}
