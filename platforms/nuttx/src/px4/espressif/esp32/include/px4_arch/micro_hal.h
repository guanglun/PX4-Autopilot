/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
#pragma once


#include <px4_platform/micro_hal.h>

__BEGIN_DECLS

#include <esp32_tim.h>
#include <esp32_spi.h>
#include <esp32_i2c.h>

#define PX4_CPU_UUID_BYTE_LENGTH                12
#define PX4_CPU_UUID_WORD32_LENGTH              (PX4_CPU_UUID_BYTE_LENGTH/sizeof(uint32_t))
#define PX4_CPU_MFGUID_BYTE_LENGTH              PX4_CPU_UUID_BYTE_LENGTH

#define px4_enter_critical_section()       enter_critical_section()
#define px4_leave_critical_section(flags)  leave_critical_section(flags)

#define px4_udelay(usec) up_udelay(usec)
#define px4_mdelay(msec) up_mdelay(msec)

#define px4_cache_aligned_data()
#define px4_cache_aligned_alloc malloc

#define px4_arch_configgpio(pinset)
#define px4_arch_unconfiggpio(pinset)
#define px4_arch_gpioread(pinset)
#define px4_arch_gpiowrite(pinset, value)
#define px4_arch_gpiosetevent(pinset,r,f,e,fp,a)	esp32_gpiosetevent(pinset,r,f,e,fp,a)


#define px4_spibus_initialize(bus_num_1based)   esp32_spibus_initialize(bus_num_1based)

#define px4_i2cbus_initialize(bus_num_1based)   esp32_i2cbus_initialize(bus_num_1based)
#define px4_i2cbus_uninitialize(pdev)           esp32_i2cbus_uninitialize(pdev)

#define PX4_NUMBER_I2C_BUSES    1

__END_DECLS
