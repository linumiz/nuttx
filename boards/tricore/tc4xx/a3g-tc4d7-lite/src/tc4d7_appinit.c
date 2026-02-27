#include <sys/types.h>
#include <debug.h>
#include <sys/sysinfo.h>

#include <nuttx/config.h>
#include <nuttx/kthread.h>
#include <sched.h>
#include <unistd.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>
#include "tricore_gpio.h"

#ifdef CONFIG_TC4X_WDT
#include "hardware/tc4x_wdt.h"
#endif

#ifdef CONFIG_TC4XX_I2C
#include "tc4xx_i2c.h"
#endif

void board_aurix_setup_i2c_pin(int bus)
{
  gpio_pinset_t sclpin = AURIX_GPIO(13, 1, GPIO_PERIPH, GPIO_ALT6);
  gpio_pinset_t sdapin = AURIX_GPIO(13, 2, GPIO_PERIPH, GPIO_ALT6);

  aurix_config_gpio(sclpin);
  aurix_config_gpio(sdapin);
}

void board_aurix_setup_serial_pin(void)
{
  gpio_pinset_t txpin = AURIX_GPIO(14, 0, GPIO_PERIPH, GPIO_ALT2);
  gpio_pinset_t rxpin = AURIX_GPIO(14, 1, GPIO_INPUT, GPIO_PULL_UP);

  aurix_config_gpio(txpin);
  aurix_config_gpio(rxpin);
}

int board_app_initialize(uintptr_t arg)
{
  bool ledon = true;
  uint8_t i = 10;
  int ret;
  gpio_pinset_t led1 = AURIX_GPIO(GPIO_PORT3, GPIO_PIN9, GPIO_OUTPUT, GPIO_ALT0);

  aurix_config_gpio(led1);
  while (i--) {
    aurix_gpio_write(led1, (ledon = !ledon));
    usleep(500 * 1000);
  }

#ifdef CONFIG_TC4X_WDT
  ret = tc4x_wdt_initialize("/dev/watchdog0", TC4X_WDT_CPU0);
  if (ret < 0)
    {
      _err("WDT init failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_TC4XX_I2C
  ret = tc4xx_i2c_bringup();
  if (ret < 0)
    {
      _err("I2C init failed: %d\n", ret);
    }
#endif

  return OK;
}
