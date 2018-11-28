/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "obstacle_avoidance.h"

#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/obstacle_avoidance_distance.h>
#include <uORB/topics/sensor_combined.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/vehicle_local_position.h>



int red_line_distance_local;
int green_line_distance_local;
float time_threshold_local;
bool control_en_flag_local;

int Obstacle_Avoidance::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
			R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("obstacle_avoidance", "modules");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('e', "Enable attitude control", false);
	PRINT_MODULE_USAGE_PARAM_INT('r', 100, 0, 500, "Red line threshold in cm", false);
	PRINT_MODULE_USAGE_PARAM_INT('g', 1000, 0, 1200, "Green line threshold in cm", false);
	PRINT_MODULE_USAGE_PARAM_FLOAT('t', 2.0, 0.0, 1000.0, "time threshold in second", false);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int Obstacle_Avoidance::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int Obstacle_Avoidance::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int Obstacle_Avoidance::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("Obstacle Avoidance",
			SCHED_DEFAULT,
			SCHED_PRIORITY_DEFAULT,
			2000,
			(px4_main_t)&run_trampoline,
			(char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

Obstacle_Avoidance *Obstacle_Avoidance::instantiate(int argc, char *argv[])
{
	//int example_param = 0;
	red_line_distance_local = 0;
	green_line_distance_local = 0;
	time_threshold_local = 0;
	control_en_flag_local = false;
	//bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "r:g:t:e", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'r':
			red_line_distance_local = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'g':
			green_line_distance_local = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 't':
			time_threshold_local = (float)strtol(myoptarg, nullptr, 10);
			break;

		case 'e':
			control_en_flag_local = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	Obstacle_Avoidance *instance = new Obstacle_Avoidance(red_line_distance_local, green_line_distance_local, time_threshold_local, control_en_flag_local);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

Obstacle_Avoidance::Obstacle_Avoidance(int red_line_distance, int green_line_distance, float time_threshold,  bool control_en_flag)
: ModuleParams(nullptr)
{
}

void Obstacle_Avoidance::run()
{
	// Example: run the loop synchronized to the sensor_combined topic publication
	int distance_sensor_sub = orb_subscribe(ORB_ID(distance_sensor));
	int acc_sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	int vel_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	//orb_advert_t	_mavlink_log_pub;		/**< mavlink log advert */
	px4_pollfd_struct_t fds[1];
	fds[0].fd = distance_sensor_sub;
	fds[0].events = POLLIN;

	// initialize parameters
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	parameters_update(parameter_update_sub, true);

	while (!should_exit()) {

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {

			struct distance_sensor_s distance_sensor;
			orb_copy(ORB_ID(distance_sensor), distance_sensor_sub, &distance_sensor);
			struct sensor_combined_s acc_sensor;
			orb_copy(ORB_ID(sensor_combined), acc_sensor_sub, &acc_sensor);
			struct vehicle_local_position_s vel_reading;
			orb_copy(ORB_ID(vehicle_local_position), vel_sub, &vel_reading);
			// TODO: do something with the data...
			if(distance_sensor.orientation == distance_sensor_s::ROTATION_FORWARD_FACING) //make tf mini data processed only facing forward
			{
				//sensor.accelerometer_m_s2[0];
				//printf("acc sensor readings: %.3f, %.3f, %.3f\n", (double)acc_sensor.accelerometer_m_s2[0], (double)acc_sensor.accelerometer_m_s2[1], (double)acc_sensor.accelerometer_m_s2[2]);				//mavlink_log_info(&_mavlink_log_pub, "[oa] running");
				//printf("param get: %i, %i, %.1f, %i\n", red_line_distance_local, green_line_distance_local, (double)time_threshold_local, control_en_flag_local);
				//mavlink_log_info(&_mavlink_log_pub, "[oa] running");

				if(vel_reading.vx > 0){ //only deal with uav fly towards obstacle
					if((distance_sensor.current_distance > ( ((float)red_line_distance_local) / 100) )){
						if((vel_reading.vx * time_threshold_local) > (distance_sensor.current_distance -  ( ((float)red_line_distance_local) / 100))) //cm
						{
							printf("I get it, current distance %.3f, vel %.3f, time_thre %.1f, red line %i \n", (double)distance_sensor.current_distance, (double)vel_reading.vx, (double)time_threshold_local, red_line_distance_local);

						}
					}
					else
					{
						printf("distance is smaller than red line threshold\n");

					}
				}

			}

		}

		parameters_update(parameter_update_sub);
	}

	orb_unsubscribe(distance_sensor_sub);
	orb_unsubscribe(vel_sub);
	orb_unsubscribe(acc_sensor_sub);
	orb_unsubscribe(parameter_update_sub);
}

void Obstacle_Avoidance::parameters_update(int parameter_update_sub, bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(parameter_update_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_upd);
	}

	if (force || updated) {
		updateParams();
	}
}


int obstacle_avoidance_main(int argc, char *argv[])
{
	return Obstacle_Avoidance::main(argc, argv);
}
