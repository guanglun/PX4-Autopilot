/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
#include <stdint.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_hrt.h>
#include <px4_arch/adc.h>

// #include <esp32_adc.h>
#include <esp32_gpio.h>
#include "sens_reg.h"
#include "rtc_io_channel.h"
#include "rtc_cntl_reg.h"
#include "hardware/esp32_iomux.h"

int px4_arch_adc_init(uint32_t base_address)
{

#ifdef ADC_BATTERY_VOLTAGE_CHANNEL_IO
	adcAttachPin(ADC_BATTERY_VOLTAGE_CHANNEL_IO);
#endif

#ifdef ADC_BATTERY_CURRENT_CHANNEL_IO
	adcAttachPin(ADC_BATTERY_CURRENT_CHANNEL_IO);
#endif

#ifdef ADC_5V_RAIL_SENSE_IO
	adcAttachPin(ADC_5V_RAIL_SENSE_IO);
#endif


	return 0;
}

void px4_arch_adc_uninit(uint32_t base_address)
{
	// nothing to do
}

uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{
	if(channel == ADC_BATTERY_VOLTAGE_CHANNEL)
		return analogRead(channel);
	else if(channel == ADC_BATTERY_CURRENT_CHANNEL)
		return 0;
	else if(channel == ADC_5V_RAIL_SENSE)
		return 3100;//always 5V
	else
		return 0;
}

float px4_arch_adc_reference_v()
{
	return BOARD_ADC_POS_REF_V;	// TODO: provide true vref
}

uint32_t px4_arch_adc_temp_sensor_mask()
{
	return 1 << PX4_ADC_INTERNAL_TEMP_SENSOR_CHANNEL;

}

uint32_t px4_arch_adc_dn_fullcount()
{
	return 1 << 12; // 12 bit ADC
}


