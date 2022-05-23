/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file init.c
 *
 * PX4FMU-specific early startup code.  This file implements the
 * board_app_initialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialization.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>
#include <nuttx/mm/gran.h>

#include "board_config.h"
#include "esp32_rtc.h"


#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>

#include <systemlib/px4_macros.h>

#include <px4_platform_common/init.h>
#include <px4_platform/board_dma_alloc.h>

#include <drivers/drv_pwm_output.h>

#ifdef CONFIG_ESP32_SPIFLASH
#include "esp32_spiflash.h"
#endif
/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/**
 * Ideally we'd be able to get these from arm_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */
__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
__END_DECLS

/****************************************************************************
 * Protected Functions
 ****************************************************************************/
/****************************************************************************
 * Public Functions
 ****************************************************************************/
/************************************************************************************
 * Name: board_peripheral_reset
 *
 * Description:
 *
 ************************************************************************************/
__EXPORT void board_peripheral_reset(int ms)
{

	// Wait for the peripheral rail to reach GND.
	usleep(ms * 1000);
	syslog(LOG_DEBUG, "reset done, %d ms\n", ms);


}

/************************************************************************************
 * Name: board_on_reset
 *
 * Description:
 * Optionally provided function called on entry to board_system_reset
 * It should perform any house keeping prior to the rest.
 *
 * status - 1 if resetting to boot loader
 *          0 if just resetting
 *
 ************************************************************************************/
__EXPORT void board_on_reset(int status)
{

}

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void
esp32_board_initialize(void)
{
	// Reset all PWM to Low outputs.
	board_on_reset(-1);

	// Configure LEDs.
	board_autoled_initialize();

	px4_esp32_configgpio(GPIO_VDD_BRICK_VALID);
	px4_esp32_configgpio(GPIO_VDD_USB_VALID);

	px4_esp32_configgpio(GPIO_OUTPUT | 12);
	esp32_gpiowrite(12, false);
	up_udelay(20);
	esp32_gpiowrite(12, true);

	esp32_spiinitialize();

}

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/
#ifdef CONFIG_ESP32_SPI2
static struct spi_dev_s *spi2;
#endif

#ifdef CONFIG_ESP32_SPI3
static struct spi_dev_s *spi3;
#endif

void test_poll(void)
{
	static uint8_t cnt = 0;

	if (cnt % 2 == 0) {
		(*(volatile uint32_t *)(0x3FF44008) = (1 << 0)); //HIGH

	} else {
		(*(volatile uint32_t *)(0x3FF4400C) = (1 << 0)); //LOW
	}

	cnt++;
	//hrt_abstime time = hrt_absolute_time();
	//syslog(LOG_INFO,"%lld %lld\n",time,time/1000/1000);
}

__EXPORT int board_app_initialize(uintptr_t arg)
{

	syslog(LOG_INFO, "\n[boot] CPU SPEED %d\n", esp_rtc_clk_get_cpu_freq());

	px4_platform_init();

	// Initial LED state.
	drv_led_start();
	led_off(LED_RED);
	led_off(LED_GREEN);
	led_off(LED_BLUE);

	// px4_esp32_configgpio(GPIO_OUTPUT|2);
	// esp32_gpiowrite(2,true);

	// px4_esp32_configgpio(GPIO_OUTPUT|4);
	// esp32_gpiowrite(4,true);

	px4_esp32_configgpio(GPIO_OUTPUT | 0);
	esp32_gpiowrite(0, true);
	px4_esp32_configgpio(GPIO_OUTPUT | 2);
	esp32_gpiowrite(2, true);
	px4_esp32_configgpio(GPIO_OUTPUT | 15);
	esp32_gpiowrite(15, true);

	// px4_esp32_configgpio(GPIO_OUTPUT | 14);  //TEST PIN
	// esp32_gpiowrite(14, true);

	// Configure SPI-based devices.
#ifdef CONFIG_ESP32_SPI2
	spi2 = esp32_spibus_initialize(2);

	if (!spi2) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port 2\n");
		led_on(LED_RED);
	}

	// Default SPI1 to 10MHz
	SPI_SETFREQUENCY(spi2, 10000000);
	SPI_SETBITS(spi2, 8);
	SPI_SETMODE(spi2, SPIDEV_MODE3);
	up_udelay(20);

	px4_platform_configure();
#endif

#ifdef CONFIG_ESP32_SPI3
	spi3 = esp32_spibus_initialize(3);

	if (!spi3) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port 3\n");
		led_on(LED_RED);
	}

	SPI_SETFREQUENCY(spi3, 10000000);
	SPI_SETBITS(spi3, 8);
	SPI_SETMODE(spi3, SPIDEV_MODE3);

	up_udelay(20);

	px4_platform_configure();
#endif


#ifdef CONFIG_ESP32_SPIFLASH
	// esp32 flash mtd init.
	// int ret = 0;
	// FAR struct mtd_dev_s *mtd;
	// mtd = esp32_spiflash_alloc_mtdpart(CONFIG_ESP32_STORAGE_MTD_OFFSET,
	// 				   CONFIG_ESP32_STORAGE_MTD_SIZE,
	// 				   false);
	// if (!mtd) {
	// 	ferr("ERROR: Failed to alloc MTD partition of SPI Flash\n");
	// 	return -ENOMEM;
	// }
	// ret = register_mtddriver("/dev/esp32flash", mtd, 0755, NULL);
	// if (ret < 0) {
	// 	ferr("ERROR: Failed to register MTD: %d\n", ret);
	// 	return ret;
	// }
#endif


	static struct hrt_call test_call;
	hrt_call_every(&test_call, 1000000, 1000000, (hrt_callout)test_poll, NULL);

	return OK;
}
