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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <math.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>

#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_attitude.h>

extern "C" __EXPORT int px4_simple_app_main(int argc, char *argv[]);
uint64_t timestamp_before_publishing;
uint64_t timestamp_after_publishing;
uint64_t timestamp_feedback;
uint64_t timestamp1;
uint64_t timestamp2;


uint64_t _orb_stat;

int px4_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");
	/* trigger subscription updated */

	struct camera_trigger_s trig;

	int _trigger_sub =  orb_subscribe(ORB_ID(camera_trigger));
	/* If there were any publications of the topic prior to the subscription,
	 * an orb_check right after orb_subscribe will return true.*/

	// this block of code clear any messages published before the subscription
	{
		bool updated;
		orb_check(_trigger_sub, &updated);
		while(updated)
		{
			orb_check(_trigger_sub, &updated);
			orb_copy(ORB_ID(camera_trigger), _trigger_sub, &trig);
		}

	}





	/*try to trigger camera*/
	px4_pollfd_struct_t fds[] = {
   	 { .fd = _trigger_sub,   .events = POLLIN },
	};
	vehicle_command_s cmd{};

	// memset(&cam, 0, sizeof(cam));
	orb_advert_t veh_trig = orb_advertise(ORB_ID(vehicle_command), &cmd);
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL;
	cmd.param5 = 1;


	timestamp_before_publishing= hrt_absolute_time();
	orb_publish(ORB_ID(vehicle_command), veh_trig, &cmd);
	timestamp_after_publishing= hrt_absolute_time();


	while(true) {

		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */

		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");
		}
		else {
			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */

				/* copy sensors raw data into local buffer */
				bool updated;
				orb_check(_trigger_sub, &updated);

				// false alarm
				if (!updated) continue;

				while(updated){
					orb_copy(ORB_ID(camera_trigger), _trigger_sub, &trig);
					orb_check(_trigger_sub, &updated);
					PX4_INFO("got updated = %llu", trig.timestamp);
				}

				if (trig.timestamp == 0) continue; // advertising

				timestamp_feedback = trig.timestamp;
				break;
			}
		}
	}
	orb_unadvertise(veh_trig);

	PX4_INFO("timestamp_before_publishing: %llu ",timestamp_before_publishing);
	PX4_INFO("timestamp_after_publishing: %llu ",timestamp_after_publishing);
	PX4_INFO("feedback =: %llu ",timestamp_feedback);

	PX4_INFO("delay in us (ts_feedback-ts_before_publishing) =: %llu ",timestamp_feedback-timestamp_before_publishing);

	// double check the feedback should be empty
	bool updated;
	orb_check(_trigger_sub, &updated);
	PX4_INFO("_trigger_sub updated (should be zero) = %d", updated);

	orb_unsubscribe(_trigger_sub);
	return 0;
}
