#include <sys/types.h>
#include <debug.h>
#include <sys/sysinfo.h>

#include <nuttx/config.h>
#include <nuttx/kthread.h>
#include <sched.h>
#include <unistd.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <nuttx/net/mii.h>

#include "tc4xx_gpio.h"

/* PHY reset/configuration delays in milliseconds */
#define TRICORE_PHY_AUTONEG 1
#define PHY_RESET_DELAY   (65)
#define PHY_CONFIG_DELAY  (1000)
#define PHY_RETRY_TIMEOUT (0x0004ffff)

#define BOARD_PHY_NAME        "DP83825I"
#define BOARD_PHYID1          MII_PHYID1_DP83825I
#define BOARD_PHYID2          MII_PHYID2_DP83825I
#define BOARD_PHY_STATUS      MII_DP83825I_PHYSTS
#define BOARD_PHY_ADDR        (0)
#define BOARD_PHY_10BASET(s)  (((s) & MII_DP83825I_PHYSTS_SPEED) != 0)
#define BOARD_PHY_100BASET(s) (((s) & MII_DP83825I_PHYSTS_SPEED) == 0)
#define BOARD_PHY_ISDUPLEX(s) (((s) & MII_DP83825I_PHYSTS_DUPLEX) != 0)

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

static inline int tricore_initphy(struct mdio_bus_s *mdio)
{
#ifdef TRICORE_PHY_AUTONEG
  volatile uint32_t timeout;
#endif
  uint8_t phyaddr = BOARD_PHY_ADDR;
  uint32_t regval;
  uint16_t phyval;
  int ret;
  int to;

  /* Reset PHY */
  ret = mdio_write(mdio, phyaddr, MII_MCR, MII_MCR_RESET);
  if (ret < 0)
    {
      nerr("ERROR: Failed to reset the PHY: %d\n", ret);
      return ret;
    }

  /* Set RMII mode and Indicate 50MHz clock */

  mdio_write(mdio, phyaddr, MII_DP83825I_RCSR,
             MII_DP83825I_RCSC_ELAST_2 | MII_DP83825I_RCSC_RMIICS);

  mdio_write(mdio, phyaddr, MII_ADVERTISE,
                     MII_ADVERTISE_100BASETXFULL |
                     MII_ADVERTISE_100BASETXHALF |
                     MII_ADVERTISE_10BASETXFULL |
                     MII_ADVERTISE_10BASETXHALF |
                     MII_ADVERTISE_CSMA);

  /* Start auto negotiation */
  ninfo("%s: Start Autonegotiation...\n",  BOARD_PHY_NAME);
  to = PHY_RESET_DELAY;
  do
    {
      up_mdelay(10);
      to -= 10;
      phyval = 0xffff;
      ret = mdio_read(mdio, phyaddr, MII_MCR, &phyval);

      ninfo("MII_MCR: phyval: %u ret: %d\n", phyval, ret);
    }
  while (phyval & MII_MCR_RESET && to > 0);

  if (to <= 0)
    {
      nerr("ERROR: Phy reset timeout\n");
      return ret;
    }
  else
    {
      ninfo("Phy reset in %d ms\n", PHY_RESET_DELAY - to);
    }

  ret = mdio_read(mdio, phyaddr, MII_PHYID1, &phyval);

  if (ret < 0)
    {
      nerr("ERROR: Failed to read PHYID1: %d\n", ret);
      return ret;
    }

  if (phyval != BOARD_PHYID1)
    {
      nerr("ERROR: Incorrect PHYID1: %u expected: %u\n",
            phyval, BOARD_PHYID1);
      return -ENXIO;
    }

  ninfo("MII_PHYID1: phyval: %u ret: %d\n", phyval, ret);

  /* Wait for link status */

#ifdef TRICORE_PHY_AUTONEG
  for (timeout = 0; timeout < PHY_RETRY_TIMEOUT; timeout++)
    {
      ret = mdio_read(mdio, phyaddr, MII_MSR, &phyval);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read the PHY MSR: %d\n", ret);
          return ret;
        }
      else if ((phyval & MII_MSR_LINKSTATUS) != 0)
        {
          ninfo("MII_MSR: phyval: %u ret: %d \n", phyval, ret);
          break;
        }

      nxsig_usleep(100);
    }

  if (timeout >= PHY_RETRY_TIMEOUT)
    {
      nerr("ERROR: Timed out waiting for link status: %04x\n", phyval);
      return -ETIMEDOUT;
    }

  /* Enable auto-negotiation */

  ret = mdio_write(mdio, phyaddr, MII_MCR, MII_MCR_ANENABLE);
  if (ret < 0)
    {
      nerr("ERROR: Failed to enable auto-negotiation: %d\n", ret);
      return ret;
    }

  /* Wait until auto-negotiation completes */

  for (timeout = 0; timeout < PHY_RETRY_TIMEOUT; timeout++)
    {
      ret = mdio_read(mdio, phyaddr, MII_MSR, &phyval);
      if (ret < 0)
        {
          nerr("ERROR: Failed to read the PHY MSR: %d\n", ret);
          return ret;
        }
      else if ((phyval & MII_MSR_ANEGCOMPLETE) != 0)
        {
          break;
        }

      nxsig_usleep(100);
    }

  if (timeout >= PHY_RETRY_TIMEOUT)
    {
      nerr("ERROR: Timed out waiting for auto-negotiation\n");
      return -ETIMEDOUT;
    }

#endif

  return OK;
}

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))
void board_config_gethpin(void)
{
	gpio_pinset_t geth[] = {
		GPIO_PAD_CFG(GPIO_PORT16, GPIO_PIN6, GPIO_PERIPH_OWN_PAD, GPIO_PAD_CONFIG_OUT_ALT10),
		GPIO_PAD_CFG(GPIO_PORT16, GPIO_PIN8, GPIO_PERIPH_OWN_PAD, GPIO_PAD_CONFIG_OUT_ALT10),
		GPIO_PAD_CFG(GPIO_PORT16, GPIO_PIN13, GPIO_PERIPH_OWN_PAD, GPIO_PAD_CONFIG_OUT_ALT10),
		GPIO_PAD_CFG(GPIO_PORT16, GPIO_PIN4, GPIO_INPUT, GPIO_PAD_CONFIG_IN_TRISTATE),
		GPIO_PAD_CFG(GPIO_PORT16, GPIO_PIN0, GPIO_INPUT, GPIO_PAD_CONFIG_IN_TRISTATE),
		GPIO_PAD_CFG(GPIO_PORT16, GPIO_PIN1, GPIO_INPUT, GPIO_PAD_CONFIG_IN_TRISTATE),
		GPIO_PAD_CFG(GPIO_PORT16, GPIO_PIN2, GPIO_INPUT, GPIO_PAD_CONFIG_IN_TRISTATE),
		/* MDIO pins */
		GPIO_PAD_CFG(GPIO_PORT21, GPIO_PIN2, GPIO_PERIPH, GPIO_PAD_CONFIG_OUT_ALT08),
		GPIO_PAD_CFG(GPIO_PORT21, GPIO_PIN3, GPIO_INPUT, GPIO_PAD_CONFIG_IN_TRISTATE),
	};

	for (int i = 0; i < ARRAY_SIZE(geth); i++) {
		aurix_config_gpio(geth[i]);
	}
}

int __attribute__((optimize("O0"))) board_app_initialize(uintptr_t arg)
{
	uint32_t cnt = 0;
	struct sysinfo info;
	bool ledon = true;
	struct mdio_bus_s *mdio;
	gpio_pinset_t led1 = GPIO_PAD_CFG(GPIO_PORT3, GPIO_PIN9, GPIO_OUTPUT, GPIO_PAD_CONFIG_OUT_GPIO);

#if 0
	g_app_pid1 = kthread_create("w1", 100, 2048, worker, NULL);
	g_app_pid2 = kthread_create("w2", 100, 2048, worker, NULL);
#endif

	board_config_gethpin();
	aurix_config_gpio(led1);
        xgmac_initialize(0);
	mdio = tricore_mdio_bus_initialize(0, CONFIG_DWXGMAC_BASE);
	if (mdio != NULL) {
		tricore_initphy(mdio);
	}

	//while (1)
	{
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
