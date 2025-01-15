/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
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

#include "stm32g0_aux_control.hpp"

#include <drivers/drv_hrt.h>

#include <px4_platform_common/sem.hpp>

#include <termios.h>

volatile bool _task_should_exit = false;

px4_task_t _task_handle = -1;

int _uart_fd = 0;

int task_main(int argc, char *argv[]);

G0AUX::G0AUX() :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": interval"))
{
}

G0AUX::~G0AUX()
{
	perf_free(_cycle_perf);
	perf_free(_interval_perf);

}

#define RDATA_SIZE 30

void parse_data(uint8_t *rdata)
{
	static uint16_t adc[4];
	static uint16_t pwm[4];
	static uint16_t pwm_count_fps,loop_count,uart_callback_count,is_connect;
	static uint16_t is_connect_last = 0;

	for(int i=0;i<4;i++)
	{
		adc[i] = (uint16_t)((rdata[i*2+1]<<8)|rdata[i*2]);
		pwm[i] = (uint16_t)((rdata[i*2+9]<<8)|rdata[i*2+8]);
	}


	pwm_count_fps = (uint16_t)((rdata[17]<<8)|rdata[16]);
	loop_count = (uint16_t)((rdata[19]<<8)|rdata[18]);
	uart_callback_count = (uint16_t)((rdata[21]<<8)|rdata[20]);
	is_connect = (uint16_t)((rdata[23]<<8)|rdata[22]);

	if(is_connect_last != is_connect)
	{
		is_connect_last = is_connect;
		if(is_connect)
		{
			PX4_INFO("G0 AUX Connect");
		}else{
			PX4_ERR("G0 AUX Connect Error");
		}
	}
	// PX4_INFO("adc:%d %d %d %d pwm:%d %d %d %d fps:%d loop:%d urx:%d isconnect:%d",
	// adc[0],adc[1],adc[2],adc[3],
	// pwm[0],pwm[1],pwm[2],pwm[3],
	// pwm_count_fps,loop_count,uart_callback_count,is_connect);
}

void uart_recv(uint8_t *data,int len)
{
	static int state = 0;
	static int datalen = 0;
	static uint8_t rdata[RDATA_SIZE-2];

	for(int i=0;i<len;i++)
	{
		if(state == 0 && data[i] == 0x12)
		{
			state = 1;
		}else if(state == 1 && data[i] == 0x34)
		{
			state = 2;
			datalen = 0;
		}else if(state == 2 && datalen < (RDATA_SIZE-2))
		{
			rdata[datalen] = data[i];
			datalen++;
			if(datalen == (RDATA_SIZE-2))
			{
				parse_data(rdata);
				state = 0;
			}
		}else if(data[i] == 0x12)
		{
			state = 1;
		}
	}
}


int task_main(int argc, char *argv[])
{
	static uint8_t rdata[RDATA_SIZE];
	int rlen = 0;
	while (true) {
		if(_uart_fd > 0)
		{
			rlen = read(_uart_fd, rdata, RDATA_SIZE);
			if(rlen > 0)
			{
				// PX4_INFO("recv: %d",rlen);
				uart_recv(rdata,rlen);
			}

		}
		px4_usleep(10000);
	}
	return 0;
}

int G0AUX::init()
{

	int speed = B115200;
	_uart_fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY);
	if (_uart_fd < 0) {
		PX4_ERR(">>>>> ERROR opening UART, aborting..\n");
		return -1;
	}

	// system("stty -F /dev/ttyS0 speed 460800 min 1 time 1 ignbrk -brkint -icrnl -imaxbel -opost -onlcr
	// 	-isig -icanon -iexten -echo -echoe -echok -echoctl -echoke");

	// system("stty -F /dev/ttyS0 speed 460800 min 1 time 1 ignbrk -brkint -icrnl -imaxbel -opost -onlcr
	// 	-isig -icanon -iexten -echo -echoe -echok -echoctl -echoke");

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;

	/* Initialize the uart config */
	if ((termios_state = tcgetattr(_uart_fd, &uart_config)) < 0)
	{
		PX4_ERR("tcgetattr fail.");
		return -1;
	}
	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* Set baud rate */
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0)
	{
		PX4_ERR("cfsetispeed fail.");
		return -1;
	}

#if defined(__PX4_LINUX) || defined(__PX4_DARWIN) || defined(__PX4_CYGWIN)
	/* Put in raw mode */
	cfmakeraw(&uart_config);
#endif

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0)
	{
		PX4_ERR("tcsetattr fail.");
		return -1;
	}

	// _task_should_exit = false;

	_task_handle = px4_task_spawn_cmd("stm32g0_rx_in_main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  2000,
					  (px4_main_t)&task_main,
					  nullptr);

	ScheduleNow();

	return 0;
}

int G0AUX::task_spawn(int argc, char *argv[])
{
	G0AUX *instance = new G0AUX();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
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

#define SDATA_SIZE 10

bool G0AUX::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
				unsigned num_outputs, unsigned num_control_groups_updated)
{
	static char sdata[SDATA_SIZE]={0xAB,0xCD};
	int cnt = 2;

	for (int i = 0; i < 4; i++) {
		uint16_t pout = outputs[i]-1000;
		sdata[cnt++] = (char)(pout);
		sdata[cnt++] = (char)(pout>>8);
	}

	if(_uart_fd > 0)
	{
		write(_uart_fd, sdata, SDATA_SIZE);
	}

	//PX4_INFO("PWM: %d %d %d %d", outputs[0], outputs[1], outputs[2], outputs[3]);

	return true;
}

void G0AUX::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	_mixing_output.update();

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	_mixing_output.updateSubscriptions(false);

	perf_end(_cycle_perf);
}

int G0AUX::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int G0AUX::print_status()
{
	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);
	_mixing_output.printStatus();
	return 0;
}

int G0AUX::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
stm32g0 aux control.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("stm32g0_aux_control", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int stm32g0_aux_control_main(int argc, char *argv[])
{
	return G0AUX::main(argc, argv);
}
