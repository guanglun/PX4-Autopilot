/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file board_config.h
 *
 * PX4FMUv4 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/* PX4FMU GPIOs ***********************************************************************************/
/* LEDs */

#define GPIO_LED_RED                 (GPIO_OUTPUT|15)
#define GPIO_LED_GREEN               (GPIO_OUTPUT|0)
#define GPIO_LED_BLUE                (GPIO_OUTPUT|2)
// #define GPIO_LED_SAFETY              GPIO_LED_BLUE

#define BOARD_HAS_CONTROL_STATUS_LEDS 1
#define BOARD_OVERLOAD_LED     LED_RED
#define BOARD_ARMED_LED        LED_BLUE
// #define BOARD_ARMED_STATE_LED  LED_GREEN


// #define GPIO_SENSORS_3V3_EN           (GPIO_OUTPUT | 12)

#define HRT_TIMER                    3  /* use timer 3 for the HRT */

#define BOARD_SPI_BUS_MAX_BUS_ITEMS 2


/**
 * ADC channels:
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver.
 */

#define ADC_BATTERY_VOLTAGE_CHANNEL_IO	36
#define ADC_BATTERY_CURRENT_CHANNEL_IO	37
#define ADC_5V_RAIL_SENSE_IO		38


#define ADC_BATTERY_VOLTAGE_CHANNEL    	0
#define ADC_BATTERY_CURRENT_CHANNEL    	1
#define ADC_5V_RAIL_SENSE            	2


#define ADC_CHANNELS \
(	(1 << ADC_5V_RAIL_SENSE) 		|\
 	(1 << ADC_BATTERY_VOLTAGE_CHANNEL) 	|\
  	(1 << ADC_BATTERY_CURRENT_CHANNEL))

#define ADC_V5_V_FULL_SCALE (7.17f)

#define RC_SERIAL_PORT		"/dev/ttyS2"

/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS	4

/* PWM directly wired to transistor. Duty cycle directly corresponds to power
 * So we need to override the defaults
 */

#define PX4_PWM_ALTERNATE_RANGES
#define PWM_LOWEST_MIN 0
#define PWM_MOTOR_OFF	0
#define PWM_DEFAULT_MIN 20
#define PWM_HIGHEST_MIN 0
#define PWM_HIGHEST_MAX 255
#define PWM_DEFAULT_MAX 255
#define PWM_LOWEST_MAX 255
#define PWM_DEFAULT_TRIM 1500


// #define BOARD_ENABLE_CONSOLE_BUFFER

/* Power supply control and monitoring GPIOs. */
// #define GPIO_VDD_BRICK_VALID         (GPIO_INPUT|GPIO_PULLUP|32)
// #define GPIO_VDD_USB_VALID           (GPIO_INPUT|GPIO_PULLUP|35)


// #define BOARD_ADC_USB_CONNECTED      (px4_arch_gpioread(GPIO_VDD_USB_VALID))
// #define BOARD_ADC_BRICK_VALID        (px4_arch_gpioread(GPIO_VDD_BRICK_VALID))
// #define BOARD_ADC_USB_VALID          (px4_arch_gpioread(GPIO_VDD_USB_VALID))

__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void esp32_spiinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>


#endif /* __ASSEMBLY__ */

__END_DECLS
