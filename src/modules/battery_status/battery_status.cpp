/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file sensors.cpp
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomas@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_adc.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <lib/battery/battery.h>
#include <lib/conversion/rotation.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/adc_report.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include "analog_battery.h"

using namespace time_literals;

#define GPADC_SAMPLE_FREQUENCY_HZ            10
#define GPADC_SAMPLE_INTERVAL_US             (1_s / GPADC_SAMPLE_FREQUENCY_HZ)

/**
 * The channel definitions (e.g., ADC_BATTERY_VOLTAGE_CHANNEL, ADC_BATTERY_CURRENT_CHANNEL, and ADC_AIRSPEED_VOLTAGE_CHANNEL) are defined in board_config.h
 */

#ifndef BOARD_NUMBER_BRICKS
#error "battery_status module requires power bricks"
#endif

#if BOARD_NUMBER_BRICKS == 0
#error "battery_status module requires power bricks"
#endif

class BatteryStatus : public ModuleBase<BatteryStatus>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	BatteryStatus();
	~BatteryStatus() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	Battery 		  _battery;
	uORB::SubscriptionInterval	_parameter_update_sub{ORB_ID(parameter_update), 1_s};				/**< notification of parameter updates */
	uORB::SubscriptionCallbackWorkItem _adc_report_sub{this, ORB_ID(adc_report)};

	static constexpr uint32_t SAMPLE_FREQUENCY_HZ = 100;
	static constexpr uint32_t SAMPLE_INTERVAL_US  = 1_s / SAMPLE_FREQUENCY_HZ;

	FILE *in0_raw_file;
	AnalogBattery _battery1;

#if BOARD_NUMBER_BRICKS > 1
	AnalogBattery _battery2;
#endif

	AnalogBattery *_analogBatteries[BOARD_NUMBER_BRICKS] {
		&_battery1,
#if BOARD_NUMBER_BRICKS > 1
		&_battery2,
#endif
	}; // End _analogBatteries

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	/**
	 * Check for changes in parameters.
	 */
	void 		parameter_update_poll(bool forced = false);

	/**
	 * Poll the ADC and update readings to suit.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		adc_poll();
};

BatteryStatus::BatteryStatus() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_battery(1, this, GPADC_SAMPLE_INTERVAL_US, battery_status_s::BATTERY_SOURCE_POWER_MODULE),
	_battery1(2, this, SAMPLE_INTERVAL_US, battery_status_s::BATTERY_SOURCE_POWER_MODULE, 0),
#if BOARD_NUMBER_BRICKS > 1
	_battery2(3, this, SAMPLE_INTERVAL_US, battery_status_s::BATTERY_SOURCE_POWER_MODULE, 1),
#endif
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME))
{
	updateParams();
}

BatteryStatus::~BatteryStatus()
{
	ScheduleClear();
}

void
BatteryStatus::parameter_update_poll(bool forced)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || forced) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}
}

volatile unsigned int *GPADC_BASE_MAP;

#define GPADC_CH0_DATA  (GPADC_BASE_MAP + 0x80/4)
#define GPADC_CTRL      (GPADC_BASE_MAP + 0x04/4)
#define GPADC_CS_EN     (GPADC_BASE_MAP + 0x08/4)
#define GPADC_DATA_INTC (GPADC_BASE_MAP + 0x28/4)
#define GPADC_DATA_INTS (GPADC_BASE_MAP + 0x38/4)

#define GPADC_BASE 0x02009000

void
BatteryStatus::adc_poll()
{

	float in0_voltage = 0;
	int in0_raw_value;

	in0_raw_value = *GPADC_CH0_DATA;
	in0_voltage = in0_raw_value*1.8f/4096/22*222;
	in0_voltage *= 1.03f;

	// PX4_INFO("%d %0.2f",in0_raw_value,(double)in0_voltage);

	if(in0_voltage > 7.0f)
	{
		_battery.setConnected(true);

	}else{
		_battery.setConnected(false);
	}

	_battery.updateVoltage(in0_voltage);
	_battery.updateCurrent(0);
	_battery.updateAndPublishBatteryStatus(hrt_absolute_time());
}

void
BatteryStatus::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* check parameters for updates */
	parameter_update_poll();

	/* check battery voltage */
	adc_poll();

	perf_end(_loop_perf);
}

int
BatteryStatus::task_spawn(int argc, char *argv[])
{
	BatteryStatus *instance = new BatteryStatus();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

bool
BatteryStatus::init()
{

	int mm_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (mm_fd < 0)
	{
		PX4_ERR("GPADC open(/dev/mem) failed.");
		return -1;
	}

	GPADC_BASE_MAP = (volatile unsigned int *)mmap(0, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, mm_fd, GPADC_BASE);

	if (GPADC_BASE_MAP == MAP_FAILED)
	{
		PX4_ERR("GPADC_BASE_MAP mmap fail.");
		return -1;
	}

	*GPADC_CS_EN = 1;
        *GPADC_CTRL = __uint32_t((1<<16)|(1<<19));

	ScheduleOnInterval(300_ms);
	return 1;

}

int BatteryStatus::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int BatteryStatus::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

The provided functionality includes:
- Read the output from the ADC driver (via ioctl interface) and publish `battery_status`.


### Implementation
It runs in its own thread and polls on the currently selected gyro topic.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("battery_status", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int battery_status_main(int argc, char *argv[])
{
	return BatteryStatus::main(argc, argv);
}
