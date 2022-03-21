/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <board_config.h>
#include <systemlib/px4_macros.h>
#include <px4_platform_common/spi.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <unistd.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

__EXPORT void esp32_spiinitialize()
{
	px4_set_spi_buses_from_hw_version();
	board_control_spi_sensors_power_configgpio();
	board_control_spi_sensors_power(true, 0xffff);
}

void board_control_spi_sensors_power(bool enable_power, int bus_mask)
{
// 	const px4_spi_bus_t *buses = px4_spi_buses;
// 	// this might be called very early on boot where we have not yet determined the hw version
// 	// (we expect all versions to have the same power GPIO)
// #if BOARD_NUM_SPI_CFG_HW_VERSIONS > 1

// 	if (!buses) {
// 		buses = &px4_spi_buses_all_hw[0].buses[0];
// 	}

// #endif

// 	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
// 		if (buses[bus].bus == -1) {
// 			break;
// 		}

// 		const bool bus_matches = bus_mask & (1 << (buses[bus].bus - 1));

// 		if (buses[bus].power_enable_gpio == 0 ||
// 		    !board_has_bus(BOARD_SPI_BUS, buses[bus].bus) ||
// 		    !bus_matches) {
// 			continue;
// 		}

// 		px4_arch_gpiowrite(buses[bus].power_enable_gpio, enable_power ? 1 : 0);
// 	}
}

void board_control_spi_sensors_power_configgpio()
{
// 	const px4_spi_bus_t *buses = px4_spi_buses;
// 	// this might be called very early on boot where we have yet not determined the hw version
// 	// (we expect all versions to have the same power GPIO)
// #if BOARD_NUM_SPI_CFG_HW_VERSIONS > 1

// 	if (!buses) {
// 		buses = &px4_spi_buses_all_hw[0].buses[0];
// 	}

// #endif

// 	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus) {
// 		if (buses[bus].bus == -1) {
// 			break;
// 		}

// 		if (buses[bus].power_enable_gpio == 0 ||
// 		    !board_has_bus(BOARD_SPI_BUS, buses[bus].bus)) {
// 			continue;
// 		}

// 		px4_arch_configgpio(buses[bus].power_enable_gpio);
// 	}
}

