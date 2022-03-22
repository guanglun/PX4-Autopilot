#include <px4_platform_common/px4_config.h>
#include <systemlib/px4_macros.h>

#include <arch/board/board.h>

#include <px4_arch/micro_hal.h>
#include <errno.h>
#include <stdio.h>

int px4_esp32_configgpio(uint32_t pinset)
{
	if ((pinset & GPIO_NUM_MASK) >= ESP32_NGPIOS) {
		return -EINVAL;
	}
	return esp32_configgpio((int)(pinset & GPIO_NUM_MASK),(uint16_t)(pinset >> GPIO_SET_SHIFT));
}

int px4_esp32_unconfiggpio(uint32_t pinset)
{
	return px4_esp32_configgpio((pinset & GPIO_NUM_MASK) | GPIO_INPUT | GPIO_OPEN_DRAIN);
}
