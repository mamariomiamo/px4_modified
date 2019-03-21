/****************************************************************************
 *
 *   Copyright (c) 2015, 2016 NUS UAV research team. All rights reserved.
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
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 * The controller has two loops: P loop for angular error and PD loop for angular rate error.
 * Desired rotation calculated keeping in mind that yaw response is normally slower than roll/pitch.
 * For small deviations controller rotates copter to have shortest path of thrust vector and independently rotates around yaw,
 * so actual rotation axis is not constant. For large deviations controller rotates copter around fixed axis.
 * These two approaches fused seamlessly with weight depending on angular error.
 * When thrust vector directed near-horizontally (e.g. roll ~= PI/2) yaw setpoint ignored because of singularity.
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 * If rotation matrix setpoint is invalid it will be generated from Euler angles for compatibility with old position controllers.
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_getopt.h>
#include <px4_defines.h>
#include <px4_log.h>
#include <px4_module.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>

#ifdef __PX4_NUTTX
#include <nuttx/fs/ioctl.h>
#endif

#include <arch/board/board.h>


#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/battery_status.h>
//#include <uORB/topics/motor_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_land_detected.h>


//#include <systemlib/param/param.h>
#include <px4_module_params.h>

#include <systemlib/err.h>
#include <perf/perf_counter.h>
//#include <systemlib/systemlib.h>
//#include <systemlib/circuit_breaker.h>
#include <circuit_breaker/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
//#include <lib/geo/geo.h>
#include <lib/ecl/geo/geo.h>


//#include "systemlib/systemlib.h"
#include "systemlib/err.h"
//#include "systemlib/param/param.h"
#include "drivers/drv_pwm_output.h"
#include <matrix/matrix/math.hpp>
#include <lib/mixer/mixer.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <conversion/rotation.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <systemlib/hysteresis/hysteresis.h>
#include <float.h>
#include <controllib/blocks.hpp>
#include <mathlib/mathlib.h>
#include <lib/FlightTasks/FlightTasks.hpp>

/**
 * Multicopter attitude control app start / stop handling function
 *
 * @ingroup apps
 */

extern "C" __EXPORT int mc_att_control_lqr_main(int argc, char *argv[]);

#define YAW_DEADZONE	0.05f
#define MIN_TAKEOFF_THRUST    0.2f
#define ANGLE_I_LIMIT	1.0f
#define QUAD  //if define, the code is for quadrotor
//#define HEX   //if define, the code is for HEX
//#define OCT		//if define, the code is for OCT
//#define FIX_POS_SHUTTER

#define PWM_OUTPUT_BASE_DEVICE_PATH "/dev/pwm_output"
#define PWM_OUTPUT0_DEVICE_PATH	"/dev/pwm_output0"

class MulticopterAttitudeControlLQR
{
public:
	/**
	 * Constructor
	 */
	MulticopterAttitudeControlLQR();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~MulticopterAttitudeControlLQR();

	/**
	 * Start the sensors task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool	_task_should_exit;		/**< if true, sensor task should exit */
	int		_control_task;			/**< task handle for sensor task */

	int		_v_att_sub;				/**< vehicle attitude subscription */
	int		_v_att_sp_sub;			/**< vehicle attitude setpoint subscription */
	int		_v_rates_sp_sub;		/**< vehicle rates setpoint subscription */
	int		_v_control_mode_sub;	/**< vehicle control mode subscription */
	int		_params_sub;			/**< parameter updates subscription */
	int		_manual_control_sp_sub;	/**< manual control setpoint subscription */
	int		_armed_sub;				/**< arming status subscription */
	int     _home_sub;              /**< home position subscription */
	int		_local_pos_sub;	    /**< local position setpoint */
	int     _gps_sub;				/**< GPS sub*/
	int 	_battery_sub;
	int		_sensor_combined_sub;
	int		_v_status_sub;
	//	int		_pos_sp_triplet_sub;
	//	int     _odroid_status_sub;
	int     _odroid_preflight_status_sub;
	int     _vehicle_land_detected_sub;

	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
	orb_advert_t	_v_rates_sp_pub;		/**< rate setpoint publication */
	orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */
	orb_advert_t    _actuators_1_pub;
	//	orb_advert_t    _motor_status_pub;

	bool		_actuators_0_circuit_breaker_enabled;	/**< circuit breaker to suppress output */

	struct vehicle_attitude_s			_v_att;				/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s	_v_att_sp;			/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s		_v_rates_sp;		/**< vehicle rates setpoint */
	struct manual_control_setpoint_s	_manual_control_sp;	/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode;	/**< vehicle control mode */
	struct actuator_controls_s			_actuators;			/**< actuator controls */
	struct actuator_controls_s			_actuators1;
	struct actuator_armed_s				_armed;				/**< actuator arming status */
	struct home_position_s              _home;
	struct vehicle_local_position_s    _local_pos;
	struct vehicle_gps_position_s       _gps;
	struct battery_status_s				_battery;
	//	struct motor_status_s				_motor_status;
	struct sensor_combined_s			_sensor_combined;
	struct vehicle_status_s				_v_status;
	struct vehicle_land_detected_s		_v_land_detected;
	//	struct position_setpoint_triplet_s		_pos_sp_triplet;
	//struct odroid_status_s                   _odroid_status;
	//	struct odroid_preflight_status_s _odroid_preflight_status;
	//	struct vehicle_serial_command_s        _vehicle_serial_cmd;

	bool gps_valid;
	bool home_valid;
	int64_t armed_timing;
	bool  was_armed;
	float pitch_sp_pre;
	float roll_sp_pre;
	float yaw_sp_pre;
	float yaw_sp_rate_pre;
	float temp_filtered;
	float pressure_filtered;


	perf_counter_t	_loop_perf;			/**< loop performance counter */

	matrix::Vector3f		_rates_prev;	/**< angular rates on previous step */
	matrix::Vector3f		_rates_sp;		/**< angular rates setpoint */
	matrix::Vector3f		_rates_d_prev;
	matrix::Vector3f		_rates_d;
	matrix::Vector3f		_angle_int;		/**< angular rates integral error */
	float				_thrust_sp;		/**< thrust setpoint */
	matrix::Vector3f		_att_control;	/**< attitude control vector */
#ifdef QUAD
	matrix::Matrix<float, 4,4>     _u_omega2;           /**< Matrix to convert the u to omega2*/
#endif
#ifdef HEX
	math::Matrix<6,4>	  _u_omega2;
#endif
#ifdef OCT
	math::Matrix<8,4>     _u_omega2;
#endif

	matrix::Vector<float, 4>    _u;              //* the desire control input */
	matrix::Vector3f		_angle_error;   /** to indicate the angle error */

	matrix::Matrix<float, 3, 3>  _I;				/**< identity matrix */

	bool	_reset_yaw_sp;			/**< reset yaw setpoint flag */

	struct {
		param_t roll_p;
		param_t roll_rate_p;
		param_t roll_i;
		param_t roll_rate_d;
		param_t pitch_p;
		param_t pitch_rate_p;
		param_t pitch_i;
		param_t pitch_rate_d;
		param_t yaw_p;
		param_t yaw_rate_p;
		param_t yaw_i;
		param_t yaw_rate_d;
		param_t yaw_ff;
		param_t yaw_rate_max;

		param_t man_roll_max;
		param_t man_pitch_max;
		param_t man_yaw_max;
		param_t acro_roll_max;
		param_t acro_pitch_max;
		param_t acro_yaw_max;
		param_t fixed_time_shutter;
		param_t check_current;
		param_t check_on_off;
		param_t Ix;
		param_t Iy;
		param_t	Iz;
		param_t Kt1;
		param_t Kt2;
		param_t	Kq1;
		param_t Kq2;
		param_t Lm;
		param_t Cm;
		param_t Ct;
		param_t M;
		param_t temp;
	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		matrix::Vector3f att_p;					/**< P gain for angular error */
		matrix::Vector3f rate_p;				/**< P gain for angular rate error */
		matrix::Vector3f angle_i;				/**< I gain for angular rate error */
		matrix::Vector3f rate_d;				/**< D gain for angular rate error */
		float yaw_ff;						/**< yaw control feed-forward */
		float yaw_rate_max;					/**< max yaw rate */

		float man_roll_max;
		float man_pitch_max;
		float man_yaw_max;
		matrix::Vector3f acro_rate_max;		/**< max attitude rates in acro mode */
		float fixed_time_shutter;
		float check_current;
		float check_on_off;
		float Kt1,Kt2;
		float temp;
	}		_params;



	//	 float max_1_current;
	//	 float max_2_current;
	//	 float max_3_current;
	//	 float max_4_current;
	//	 float ave_current;
	bool  motor_checked;
	int   loop_count;
	float Ix,Iy,Iz;
	float Kt1,Kt2,Kq1,Kq2;
	float Lm,Mg;
	float Cm,Ct;
	float min_omega_square;
	float max_omega_square;
	float air_density_ratio;

	/**
	 * Update our local parameter cache.
	 */
	int			parameters_update();

	/**
	 * Check for parameter update and handle it.
	 */
	void		parameter_update_poll();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();

	void		vehicle_status_poll();

	/**
	 * Check for attitude setpoint updates.
	 */
	void		vehicle_attitude_setpoint_poll();

	/**
	 * Check for rates setpoint updates.
	 */
	void		vehicle_rates_setpoint_poll();

	/**
	 * Check for arming status updates.
	 */
	void		arming_status_poll();

	/**
	 * Check for home position updates.
	 */
	void		home_position_poll();


	/**
	 * Check for local position updates.
	 */
	void		local_position_poll();


	void        gps_poll();

	void 		battery_poll();

	void 		sensor_combined_poll();

	float	 	scale_control(float ctl, float end, float dz);

	bool		check_motor();

	void        calculate_delta();

	void        pre_arm_check();

	void		vehicle_land_deteced_poll();
	/**
	 * Attitude controller.
	 */
	void		control_attitude(float dt);

	/**
	 * Attitude rates controller.
	 */
	void		control_attitude_rates(float dt);

	void        update_u_omega2();



	float constrain_ref(float x, float cons);

	float limit_range(float x, float min, float max);

	float limit_acc(float cur,float pre,float max_acc,float dt);


	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task.
	 */
	void		task_main();

	void task_main_pseudo();
};

namespace mc_att_control_lqr
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

MulticopterAttitudeControlLQR	*g_control;
}

float MulticopterAttitudeControlLQR::constrain_ref(float x, float cons)
{
	if(x>cons)
		x = cons;
	else if (x< - cons)
		x = - cons;
	return x;
}

float MulticopterAttitudeControlLQR::limit_range(float x, float min, float max)
{
	if(x>max)
		x = max;
	if(x<min)
		x = min;
	return x;

}

float MulticopterAttitudeControlLQR::limit_acc(float cur, float pre, float max_acc, float dt)
{
	float error = cur-pre;
	float error_dot = error/dt;
	error_dot = constrain_ref(error_dot,max_acc);
	return pre + error_dot * dt;
}




MulticopterAttitudeControlLQR::MulticopterAttitudeControlLQR() :

							_task_should_exit(false),
							_control_task(-1),

							/* subscriptions */
							_v_att_sub(-1),
							_v_att_sp_sub(-1),
							_v_control_mode_sub(-1),
							_params_sub(-1),
							_manual_control_sp_sub(-1),
							_armed_sub(-1),
							_vehicle_land_detected_sub(-1),

							/* publications */
							_att_sp_pub(nullptr),
							_v_rates_sp_pub(nullptr),
							_actuators_0_pub(nullptr),
							_actuators_1_pub(nullptr),
							//    _motor_status_pub(-1),
							_actuators_0_circuit_breaker_enabled(false),

							/* performance counters */
							_loop_perf(perf_alloc(PC_ELAPSED, "mc_att_control_lqr"))

{
	memset(&_v_att, 0, sizeof(_v_att));
	memset(&_v_att_sp, 0, sizeof(_v_att_sp));
	memset(&_v_rates_sp, 0, sizeof(_v_rates_sp));
	memset(&_manual_control_sp, 0, sizeof(_manual_control_sp));
	memset(&_v_control_mode, 0, sizeof(_v_control_mode));
	memset(&_actuators, 0, sizeof(_actuators));
	memset(&_armed, 0, sizeof(_armed));
	memset(&_v_status,0,sizeof(_v_status));
	memset(&_v_land_detected, 0, sizeof(_v_land_detected));

	_params.att_p.zero();
	_params.rate_p.zero();
	_params.angle_i.zero();
	_params.rate_d.zero();
	_params.yaw_ff = 0.0f;
	_params.yaw_rate_max = 0.0f;
	_params.man_roll_max = 0.0f;
	_params.man_pitch_max = 0.0f;
	_params.man_yaw_max = 0.0f;
	_params.acro_rate_max.zero();
	_params.fixed_time_shutter = 0.0f;
	_params.check_current = 0.0f;
	_params.check_on_off = 0.0f;

	_rates_prev.zero();
	_rates_sp.zero();
	_angle_int.zero();
	_rates_d_prev.zero();
	_rates_d.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();
	_u_omega2.identity();
	_u.zero();
	_angle_error.zero();

	_I.identity();

	_params_handles.roll_p			= 	param_find("MC_ROLL_P_L");
	_params_handles.roll_rate_p		= 	param_find("MC_ROLLRATE_P_L");
	_params_handles.roll_i			= 	param_find("MC_ROLL_I_L");
	_params_handles.roll_rate_d		= 	param_find("MC_ROLLRATE_D_L");
	_params_handles.pitch_p			= 	param_find("MC_PITCH_P_L");
	_params_handles.pitch_rate_p	= 	param_find("MC_PITCHRATE_P_L");
	_params_handles.pitch_i			= 	param_find("MC_PITCH_I_L");
	_params_handles.pitch_rate_d	= 	param_find("MC_PITCHRATE_D_L");
	_params_handles.yaw_p			=	param_find("MC_YAW_P_L");
	_params_handles.yaw_rate_p		= 	param_find("MC_YAWRATE_P_L");
	_params_handles.yaw_i			= 	param_find("MC_YAW_I_L");
	_params_handles.yaw_rate_d		= 	param_find("MC_YAWRATE_D_L");
	_params_handles.yaw_ff			= 	param_find("MC_YAW_FF_L");
	_params_handles.yaw_rate_max	= 	param_find("MC_YAWRATE_MAX_L");
	_params_handles.man_roll_max	= 	param_find("MC_MAN_R_MAX_L");
	_params_handles.man_pitch_max	= 	param_find("MC_MAN_P_MAX_L");
	_params_handles.man_yaw_max		= 	param_find("MC_MAN_Y_MAX_L");
	_params_handles.acro_roll_max	= 	param_find("MC_ACRO_R_MAX_L");
	_params_handles.acro_pitch_max	= 	param_find("MC_ACRO_P_MAX_L");
	_params_handles.acro_yaw_max	= 	param_find("MC_ACRO_Y_MAX_L");
	_params_handles.fixed_time_shutter = param_find("FIX_TIME_SHUT");
	_params_handles.check_current = param_find("CHECK_CURRENT");
	_params_handles.check_on_off = param_find("CHECK_ON_OFF");
	_params_handles.Ix 	= param_find("UAV_IX");
	_params_handles.Iy 	= param_find("UAV_IY");
	_params_handles.Iz 	= param_find("UAV_IZ");
	_params_handles.Kt1 = param_find("UAV_KT1");
	_params_handles.Kt2 = param_find("UAV_KT2");
	_params_handles.Kq1 = param_find("UAV_KQ1");
	_params_handles.Kq2 = param_find("UAV_KQ2");
	_params_handles.Lm 	= param_find("UAV_LM");
	_params_handles.Cm 	= param_find("UAV_CM");
	_params_handles.Ct 	= param_find("UAV_CT");
	_params_handles.M 	= param_find("UAV_MASS");
	_params_handles.temp = param_find("UAV_TEMP");

	/* fetch initial parameter values */
	parameters_update();
}

MulticopterAttitudeControlLQR::~MulticopterAttitudeControlLQR()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	mc_att_control_lqr::g_control = nullptr;
}

int
MulticopterAttitudeControlLQR::parameters_update()
{
	float v;

	/* roll gains */
	param_get(_params_handles.roll_p, &v);
	_params.att_p(0) = v;
	param_get(_params_handles.roll_rate_p, &v);
	_params.rate_p(0) = v;
	param_get(_params_handles.roll_i, &v);
	_params.angle_i(0) = v;
	param_get(_params_handles.roll_rate_d, &v);
	_params.rate_d(0) = v;

	/* pitch gains */
	param_get(_params_handles.pitch_p, &v);
	_params.att_p(1) = v;
	param_get(_params_handles.pitch_rate_p, &v);
	_params.rate_p(1) = v;
	param_get(_params_handles.pitch_i, &v);
	_params.angle_i(1) = v;
	param_get(_params_handles.pitch_rate_d, &v);
	_params.rate_d(1) = v;

	/* yaw gains */
	param_get(_params_handles.yaw_p, &v);
	_params.att_p(2) = v;
	param_get(_params_handles.yaw_rate_p, &v);
	_params.rate_p(2) = v;
	param_get(_params_handles.yaw_i, &v);
	_params.angle_i(2) = v;
	param_get(_params_handles.yaw_rate_d, &v);
	_params.rate_d(2) = v;

	param_get(_params_handles.yaw_ff, &_params.yaw_ff);
	param_get(_params_handles.yaw_rate_max, &_params.yaw_rate_max);
	_params.yaw_rate_max = math::radians(_params.yaw_rate_max);

	/* manual control scale */
	param_get(_params_handles.man_roll_max, &_params.man_roll_max);
	param_get(_params_handles.man_pitch_max, &_params.man_pitch_max);
	param_get(_params_handles.man_yaw_max, &_params.man_yaw_max);
	_params.man_roll_max = math::radians(_params.man_roll_max);
	_params.man_pitch_max = math::radians(_params.man_pitch_max);
	_params.man_yaw_max = math::radians(_params.man_yaw_max);

	/* acro control scale */
	param_get(_params_handles.acro_roll_max, &v);
	_params.acro_rate_max(0) = math::radians(v);
	param_get(_params_handles.acro_pitch_max, &v);
	_params.acro_rate_max(1) = math::radians(v);
	param_get(_params_handles.acro_yaw_max, &v);
	_params.acro_rate_max(2) = math::radians(v);

	param_get(_params_handles.fixed_time_shutter, &_params.fixed_time_shutter);
	param_get(_params_handles.check_current, &_params.check_current);
	param_get(_params_handles.check_on_off, &_params.check_on_off);
	param_get(_params_handles.Ix,&Ix);
	param_get(_params_handles.Iy,&Iy);
	param_get(_params_handles.Iz,&Iz);
	param_get(_params_handles.Kt1,&_params.Kt1);
	param_get(_params_handles.Kt2,&_params.Kt2);
	param_get(_params_handles.Kq1,&Kq1);
	param_get(_params_handles.Kq2,&Kq2);
	param_get(_params_handles.Lm,&Lm);
	float M;
	param_get(_params_handles.M,&M);
	Mg = M * 9.8f;
	param_get(_params_handles.Cm,&Cm);
	param_get(_params_handles.Ct,&Ct);
	param_get(_params_handles.temp,&_params.temp);
	update_u_omega2();

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

	return OK;
}

void MulticopterAttitudeControlLQR::update_u_omega2()
{

	Kt1 = _params.Kt1 * air_density_ratio;
	Kt2 = _params.Kt2 * air_density_ratio;

#ifdef QUAD
	_u_omega2(0,0) = -1/(2.828f*Lm*Kt1);
	_u_omega2(0,1) =  1/(2.828f*Lm*Kt1);
	_u_omega2(0,2) =  0.5f*Kt2/(Kt1*Kq2+Kt2*Kq1);
	_u_omega2(0,3) =  0.5f*Kq2/(Kt1*Kq2+Kt2*Kq1);

	_u_omega2(1,0) =  1/(2.828f*Lm*Kt1);
	_u_omega2(1,1) = - 1/(2.828f*Lm*Kt1);
	_u_omega2(1,2) =  0.5f*Kt2/(Kt1*Kq2+Kt2*Kq1);
	_u_omega2(1,3) =  0.5f*Kq2/(Kt1*Kq2+Kt2*Kq1);

	_u_omega2(2,0) =  1/(2.828f*Lm*Kt2);
	_u_omega2(2,1) =  1/(2.828f*Lm*Kt2);
	_u_omega2(2,2) =  -0.5f*Kt1/(Kt1*Kq2+Kt2*Kq1);
	_u_omega2(2,3) =   0.5f*Kq1/(Kt1*Kq2+Kt2*Kq1);


	_u_omega2(3,0) = -1/(2.828f*Lm*Kt2);
	_u_omega2(3,1) = -1/(2.828f*Lm*Kt2);
	_u_omega2(3,2) = -0.5f*Kt1/(Kt1*Kq2+Kt2*Kq1);
	_u_omega2(3,3) =  0.5f*Kq1/(Kt1*Kq2+Kt2*Kq1);
#endif

#ifdef HEX
	_u_omega2(0,0) = -1/(4*Kt1*Lm);
	_u_omega2(0,1) = 0.2877f/(Kt1*Lm);
	_u_omega2(0,2) = 1/(6*Kq1);
	_u_omega2(0,3) = 1/(6*Kt1);

	_u_omega2(1,0) = -1/(4*Kt2*Lm);
	_u_omega2(1,1) = 0.0f;
	_u_omega2(1,2) = -1/(6*Kq1);
	_u_omega2(1,3) = 1/(6*Kt2);

	_u_omega2(2,0) = -1/(4*Kt1*Lm);
	_u_omega2(2,1) = -0.2877f/(Kt1*Lm);
	_u_omega2(2,2) = 1/(6*Kq1);
	_u_omega2(2,3) = 1/(6*Kt1);

	_u_omega2(3,0) = 1/(4*Kt2*Lm);
	_u_omega2(3,1) = -0.2877f/(Kt2*Lm);
	_u_omega2(3,2) = -1/(6*Kq1);
	_u_omega2(3,3) = 1/(6*Kt2);

	_u_omega2(4,0) = 1/(4*Kt1*Lm);
	_u_omega2(4,1) = 0;
	_u_omega2(4,2) = 1/(6*Kq1);
	_u_omega2(4,3) = 1/(6*Kt1);

	_u_omega2(5,0) = 1/(4*Kt2*Lm);
	_u_omega2(5,1) = 0.2877f/(Kt2*Lm);
	_u_omega2(5,2) = -1/(6*Kq1);
	_u_omega2(5,3) = 1/(6*Kt2);
#endif


#ifdef OCT

	float Den = 4*(Kq1*Kt2 + Kt1*Kq2);
	_u_omega2(0,0) = -1.4142f*Kq2/(Lm*Den);
	_u_omega2(0,1) =  1.4142f*Kq2/(Lm*Den);
	_u_omega2(0,2) =  Kt2/Den;
	_u_omega2(0,3) =  Kq2/Den;

	_u_omega2(1,0) =  1.4142f*Kq2/(Lm*Den);
	_u_omega2(1,1) = -1.4142f*Kq2/(Lm*Den);
	_u_omega2(1,2) =  Kt2/Den;
	_u_omega2(1,3) =  Kq2/Den;


	_u_omega2(2,0) =  1.4142f*Kq1/(Lm*Den);
	_u_omega2(2,1) =  1.4142f*Kq1/(Lm*Den);
	_u_omega2(2,2) = -Kt1/Den;
	_u_omega2(2,3) =  Kq1/Den;

	_u_omega2(3,0) = -1.4142f*Kq1/(Lm*Den);
	_u_omega2(3,1) = -1.4142f*Kq1/(Lm*Den);
	_u_omega2(3,2) = -Kt1/Den;
	_u_omega2(3,3) =  Kq1/Den;

	_u_omega2(4,0) = -1.4142f*Kq1/(Lm*Den);
	_u_omega2(4,1) =  1.4142f*Kq1/(Lm*Den);
	_u_omega2(4,2) = -Kt1/Den;
	_u_omega2(4,3) =  Kq1/Den;

	_u_omega2(5,0) =  1.4142f*Kq1/(Lm*Den);
	_u_omega2(5,1) = -1.4142f*Kq1/(Lm*Den);
	_u_omega2(5,2) = -Kt1/Den;
	_u_omega2(5,3) =  Kq1/Den;

	_u_omega2(6,0) =  1.4142f*Kq2/(Lm*Den);
	_u_omega2(6,1) =  1.4142f*Kq2/(Lm*Den);
	_u_omega2(6,2) =  Kt2/Den;
	_u_omega2(6,3) =  Kq2/Den;

	_u_omega2(7,0) = -1.4142f*Kq2/(Lm*Den);
	_u_omega2(7,1) = -1.4142f*Kq2/(Lm*Den);
	_u_omega2(7,2) =  Kt2/Den;
	_u_omega2(7,3) =  Kq2/Den;
#endif
	min_omega_square = (0.15f*Cm + Ct)*(0.15f*Cm + Ct); // limit the minimal throttle
	max_omega_square = (0.7f *Cm + Ct)*(0.7f*Cm + Ct);
}

void
MulticopterAttitudeControlLQR::parameter_update_poll()
{
	bool updated;

	/* Check HIL state if vehicle status has changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}

void
MulticopterAttitudeControlLQR::vehicle_control_mode_poll()
{
	bool updated;

	/* Check HIL state if vehicle status has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

void
MulticopterAttitudeControlLQR::vehicle_status_poll()
{
	bool updated;
	orb_check(_v_status_sub,&updated);
	if(updated){
		orb_copy(ORB_ID(vehicle_status),_v_status_sub,&_v_status);
	}

}

void
MulticopterAttitudeControlLQR::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

void
MulticopterAttitudeControlLQR::vehicle_attitude_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
	}
}

void
MulticopterAttitudeControlLQR::vehicle_rates_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	}
}

void
MulticopterAttitudeControlLQR::arming_status_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	}
}

void
MulticopterAttitudeControlLQR::home_position_poll()
{
	bool updated;
	orb_check(_home_sub,&updated);

	if(updated){
		orb_copy(ORB_ID(home_position),_home_sub,&_home);
		home_valid = true;
	}
}

void
MulticopterAttitudeControlLQR::local_position_poll()
{
	bool updated;
	orb_check(_local_pos_sub,&updated);

	if(updated){
		orb_copy(ORB_ID(vehicle_local_position),_local_pos_sub,&_local_pos);
	}
}

void
MulticopterAttitudeControlLQR::gps_poll()
{
	bool updated;
	orb_check(_gps_sub,&updated);

	if(updated){
		orb_copy(ORB_ID(vehicle_gps_position),_gps_sub,&_gps);
		if (_gps.eph < 20 * 0.7f && _gps.epv < 20 * 0.7f && _gps.fix_type >= 3)
			gps_valid = true;
	}
}

void MulticopterAttitudeControlLQR::battery_poll()
{
	bool updated;
	orb_check(_battery_sub,&updated);
	if(updated)
		orb_copy(ORB_ID(battery_status),_battery_sub,&_battery);
}

void MulticopterAttitudeControlLQR::sensor_combined_poll()
{
	bool updated;
	orb_check(_sensor_combined_sub,&updated);
	if(updated)
		orb_copy(ORB_ID(sensor_combined),_sensor_combined_sub,&_sensor_combined);

}

void MulticopterAttitudeControlLQR::vehicle_land_deteced_poll()
{
	bool updated;
	orb_check(_vehicle_land_detected_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_v_land_detected);
	}
}

float
MulticopterAttitudeControlLQR::scale_control(float ctl, float end, float dz)
{
	if (ctl > dz) {
		return (ctl - dz) / (end - dz);

	} else if (ctl < -dz) {
		return (ctl + dz) / (end - dz);

	} else {
		return 0.0f;
	}
}


/*
 * Attitude controller.
 * Input: 'manual_control_setpoint' and 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp', 'vehicle_attitude_setpoint' topic (for manual modes)
 */
void
MulticopterAttitudeControlLQR::control_attitude(float dt)
{
	float yaw_sp_move_rate = 0.0f;
	bool publish_att_sp = false;

	if (_v_control_mode.flag_control_manual_enabled) {
		/* manual input, set or modify attitude setpoint */

		if (_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_climb_rate_enabled) {
			/* in assisted modes poll 'vehicle_attitude_setpoint' topic and modify it */
			vehicle_attitude_setpoint_poll();
		}

		if (!_v_control_mode.flag_control_climb_rate_enabled) {
			/* pass throttle directly if not in altitude stabilized mode */
			_v_att_sp.thrust = _manual_control_sp.z;
			publish_att_sp = true;
		}

		if (!_armed.armed) {
			/* reset yaw setpoint when disarmed */
			_reset_yaw_sp = true;
		}

		/* move yaw setpoint in all modes */
		if (_v_att_sp.thrust < 0.3f) {
			//		if (_v_land_detected.landed) {
			// TODO
			//if (_status.condition_landed) {
			/* reset yaw setpoint if on ground */
			_reset_yaw_sp = true;
			//}
		} else {
			/* move yaw setpoint */

			/*move the yaw setpoint directly*/
			yaw_sp_move_rate = _manual_control_sp.r * _params.man_yaw_max;
			yaw_sp_move_rate = limit_acc(yaw_sp_move_rate,yaw_sp_rate_pre,1.0f,dt);
			yaw_sp_rate_pre = yaw_sp_move_rate;
			_v_att_sp.yaw_body = wrap_pi(_v_att_sp.yaw_body + yaw_sp_move_rate * dt);

			/*limit the yaw offset error*/

			float yaw_offs_max = _params.man_yaw_max;
			matrix::Quatf q_mea(_v_att.q[0], _v_att.q[1], _v_att.q[2], _v_att.q[3]);
			matrix::Vector3f euler_mea =  matrix::Eulerf(q_mea);

			float yaw_offs = wrap_pi(_v_att_sp.yaw_body - euler_mea(2));
			if (yaw_offs < - yaw_offs_max) {
				_v_att_sp.yaw_body = wrap_pi(/*_v_att.yaw*/euler_mea(2)  - yaw_offs_max);

			} else if (yaw_offs > yaw_offs_max) {
				_v_att_sp.yaw_body = wrap_pi(/*_v_att.yaw*/ euler_mea(2) + yaw_offs_max);
			}
			//			_v_att_sp.R_valid = false;
			_v_att_sp.q_d_valid = false;
			publish_att_sp = true;
		}

		/* reset yaw setpint to current position if needed */
		if (_reset_yaw_sp) {
			matrix::Quatf q_mea(_v_att.q[0], _v_att.q[1], _v_att.q[2], _v_att.q[3]);
			matrix::Vector3f euler_mea = matrix::Eulerf(q_mea);

			_reset_yaw_sp = false;
			yaw_sp_pre = euler_mea(2); // _v_att.yaw;
			yaw_sp_rate_pre = 0.0f;
			_v_att_sp.yaw_body = euler_mea(2); // _v_att.yaw;
			_v_att_sp.q_d_valid = false;
			publish_att_sp = true;
		}

		if (!_v_control_mode.flag_control_velocity_enabled) {
			/* update attitude setpoint if not in position control mode */
			_v_att_sp.roll_body = _manual_control_sp.y * _params.man_roll_max;
			_v_att_sp.pitch_body = -_manual_control_sp.x * _params.man_pitch_max;
			_v_att_sp.q_d_valid = false;
			publish_att_sp = true;

		}

	} else {
		/* in non-manual mode use 'vehicle_attitude_setpoint' topic */
		vehicle_attitude_setpoint_poll();
		yaw_sp_move_rate = _v_att_sp.yaw_sp_move_rate; //_v_att_sp.yaw_speed;
		yaw_sp_move_rate = limit_acc(yaw_sp_move_rate,yaw_sp_rate_pre,1.0f,dt);
		yaw_sp_rate_pre = yaw_sp_move_rate;
		/* reset yaw setpoint after non-manual control mode */
		//_reset_yaw_sp = true;
	}

	_thrust_sp = _v_att_sp.thrust;



	float pitch_sp_cur = limit_acc(_v_att_sp.pitch_body,pitch_sp_pre,2.0f,dt);
	 //_rates_sp(0) = (pitch_sp_cur - pitch_sp_pre)/dt;
	_rates_sp(0) = 0.0f;
	pitch_sp_pre = pitch_sp_cur;
	_v_att_sp.pitch_body = pitch_sp_cur;

	float roll_sp_cur = limit_acc(_v_att_sp.roll_body,roll_sp_pre,2.0f,dt);
	// _rates_sp(1) = (roll_sp_cur - roll_sp_pre)/dt;
	_rates_sp(1) = 0.0f;
	roll_sp_pre = roll_sp_cur;
	_v_att_sp.roll_body = roll_sp_cur;
//	static unsigned int loop1 = 0;
//	if (loop1++ % 400 == 0) {
//		printf("test: %.3f, %.3f\n", (double)_v_att_sp.roll_body, (double)_v_att_sp.pitch_body);
//	}
	_rates_sp(2) = _params.yaw_ff * yaw_sp_rate_pre;


	/* construct attitude setpoint rotation matrix */
	matrix::Dcmf R_sp;
	/* rotation matrix in _att_sp is not valid, use euler angles instead */
	R_sp = matrix::Eulerf(_v_att_sp.roll_body, _v_att_sp.pitch_body, _v_att_sp.yaw_body);
	/* copy rotation matrix back to setpoint struct */
	//memcpy(&_v_att_sp.R_body[0][0], &R_sp.data[0][0], sizeof(_v_att_sp.R_body));
//	for(int i=0; i<3; i++)
//	{
//		R_sp(0,i) = _v_att_sp.R_body0[i];
//		R_sp(1,i) = _v_att_sp.R_body1[i];
//		R_sp(2,i) = _v_att_sp.R_body2[i];
//	}
	matrix::Quatf q_sp;
	q_sp.from_dcm(R_sp);
	//	math::Matrix<3, 3> R_sp = q_sp.to_dcm();

	_v_att_sp.q_d[0] = q_sp(0);
	_v_att_sp.q_d[1] = q_sp(1);
	_v_att_sp.q_d[2] = q_sp(2);
	_v_att_sp.q_d[3] = q_sp(3);
	_v_att_sp.q_d_valid = true;

	//	math::Quaternion q_sp(_v_att_sp.q_d[0], _v_att_sp.q_d[1], _v_att_sp.q_d[2], _v_att_sp.q_d[3]);
	//	math::Matrix<3, 3> R_sp = q_sp.to_dcm();

	/* publish the attitude setpoint if needed */
	if (publish_att_sp) {
		_v_att_sp.timestamp = hrt_absolute_time();

		if (_att_sp_pub != nullptr) {
			orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_v_att_sp);

		} else {
			_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_v_att_sp);
		}
	}

	static unsigned int loop = 0;
	//	math::Quaternion q_mea1(_v_att.q[0], _v_att.q[1], _v_att.q[2], _v_att.q[3]);
	//	math::Vector<3> euler_mea1 = q_mea1.to_euler();
	if (loop++ % 400 == 0) {
		//		printf("_v_att: %.3f, %.3f, %.3f\n", (double)euler_mea1(0), (double)euler_mea1(1), (double)euler_mea1(2));
//				printf("_v_att_sp: %.3f, %.3f, %.3f, %.3f\n",
//						(double)_v_att_sp.roll_body, (double)_v_att_sp.pitch_body, (double)_v_att_sp.yaw_body, (double)_v_att_sp.thrust);
	}

	/* rotation matrix for current state */
	matrix::Dcmf R;

	//	R.set(_v_att.R);
	matrix::Quatf q_tmp(_v_att.q[0], _v_att.q[1], _v_att.q[2], _v_att.q[3]);
	R = q_tmp.to_dcm();
	//	math::Matrix<3, 3> Rb = q_att.to_dcm();
	//	math::Matrix<3, 3> R = Rb.transposed();

	/* all input data is ready, run controller itself */

	/* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
	matrix::Vector3f R_z(R(0, 2), R(1, 2), R(2, 2));
	matrix::Vector3f R_sp_z(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

	/* axis and sin(angle) of desired rotation (indexes: 0=pitch, 1=roll, 2=yaw).
	 * This is for roll/pitch only (tilt), e_R(2) is 0 */
	matrix::Vector3f e_R = R.transpose() * (R_z % R_sp_z);

	/* calculate angle error */
	float e_R_z_sin = e_R.length();
	float e_R_z_cos = R_z * R_sp_z;

	/* calculate weight for yaw control */
	float yaw_w = R_sp(2, 2) * R_sp(2, 2);

	/* calculate rotation matrix after roll/pitch only rotation */
	matrix::Dcmf R_rp;
	if (loop % 400 == 0) {
//		printf("att_sp: %.3f, %.3f, %.3f\n", (double)_v_att_sp.q_d[0], (double)_v_att_sp.q_d[1], (double)_v_att_sp.q_d[2],(double)_v_att_sp.q_d[3]);
//					printf("Rsp0: %.3f, %.3f, %.3f\n", (double)R_sp(0,0), (double)R_sp(0,1), (double)R_sp(0,2));
//					printf("Rsp1: %.3f, %.3f, %.3f\n", (double)R_sp(1,0), (double)R_sp(1,1), (double)R_sp(1,2));
//					printf("Rsp2: %.3f, %.3f, %.3f\n", (double)R_sp(2,0), (double)R_sp(2,1), (double)R_sp(2,2));
//					printf("att_sp: %.3f, %.3f, %.3f\n", (double)_v_att_sp.roll_body, (double)_v_att_sp.pitch_body, (double)_v_att_sp.yaw_body);
		/*			printf("e_R: %.3f, %.3f, %.3f\n", (double)e_R(0), (double)e_R(1), (double)e_R(2));
		printf("e_R_z_sin: %.3f, e_R_z_cos: %.3f\n", (double)e_R_z_sin, (double)e_R_z_cos);
		printf("e_R_z_axis: %.3f, %.3f, %.3f\n", (double)e_R_z_axis(0), (double)e_R_z_axis(1), (double)e_R_z_axis(2));
		printf("e_R_z_angle: %.3f\n", (double)e_R_z_angle);
		printf("R_z: %.3f, %.3f, %.3f\n", (double)R_z(0), (double)R_z(1), (double)R_z(2));
		printf("R_sp_z: %.3f, %.3f, %.3f\n", (double)R_sp_z(0), (double)R_sp_z(1), (double)R_sp_z(2));*/

	}
	if (e_R_z_sin > 0.0f) {
		/* get axis-angle representation */
		float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
		matrix::Vector3f e_R_z_axis = e_R / e_R_z_sin;

		e_R = e_R_z_axis * e_R_z_angle;

		if (loop % 400 == 0) {
//						printf("R0: %.3f, %.3f, %.3f\n", (double)R(0,0), (double)R(0,1), (double)R(0,2));
//						printf("R1: %.3f, %.3f, %.3f\n", (double)R(1,0), (double)R(1,1), (double)R(1,2));
//						printf("R2: %.3f, %.3f, %.3f\n", (double)R(2,0), (double)R(2,1), (double)R(2,2));
			/*			printf("e_R: %.3f, %.3f, %.3f\n", (double)e_R(0), (double)e_R(1), (double)e_R(2));
			printf("e_R_z_sin: %.3f, e_R_z_cos: %.3f\n", (double)e_R_z_sin, (double)e_R_z_cos);
			printf("e_R_z_axis: %.3f, %.3f, %.3f\n", (double)e_R_z_axis(0), (double)e_R_z_axis(1), (double)e_R_z_axis(2));
			printf("e_R_z_angle: %.3f\n", (double)e_R_z_angle);
			printf("R_z: %.3f, %.3f, %.3f\n", (double)R_z(0), (double)R_z(1), (double)R_z(2));
			printf("R_sp_z: %.3f, %.3f, %.3f\n", (double)R_sp_z(0), (double)R_sp_z(1), (double)R_sp_z(2));*/

		}
		/* cross product matrix for e_R_axis */
		matrix::Dcmf e_R_cp;
		e_R_cp.zero();
		e_R_cp(0, 1) = -e_R_z_axis(2);
		e_R_cp(0, 2) = e_R_z_axis(1);
		e_R_cp(1, 0) = e_R_z_axis(2);
		e_R_cp(1, 2) = -e_R_z_axis(0);
		e_R_cp(2, 0) = -e_R_z_axis(1);
		e_R_cp(2, 1) = e_R_z_axis(0);

		/* rotation matrix for roll/pitch only rotation */
		R_rp = R * (_I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos));

	} else {
		/* zero roll/pitch rotation */
		R_rp = R;
	}

	/* R_rp and R_sp has the same Z axis, calculate yaw error */
	matrix::Vector3f R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));
	matrix::Vector3f R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));
	e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;

	if (e_R_z_cos < 0.0f) {
		/* for large thrust vector rotations use another rotation method:
		 * calculate angle and axis for R -> R_sp rotation directly */
		matrix::Quatf q;
		q.from_dcm(R.transpose() * R_sp);
		matrix::Vector3f e_R_d = q.imag();
		e_R_d.normalize();
		e_R_d *= 2.0f * atan2f(e_R_d.length(), q(0));

		/* use fusion of Z axis based rotation and direct rotation */
		float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
		e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
	}

	_angle_error(0) = e_R(0);
	_angle_error(1) = e_R(1);
	_angle_error(2) = wrap_pi(e_R(2));
//	if (loop % 400 == 0) {
//		printf("angle_error: %.3f, %.3f, %.3f\n", (double)_angle_error(0),(double)_angle_error(1),(double)_angle_error(2));
//	}
}

void MulticopterAttitudeControlLQR::pre_arm_check()
{
	_actuators.control[0] = -1.0f;
	_actuators.control[1] = -1.0f;
	_actuators.control[2] = -1.0f;
	_actuators.control[3] = -1.0f;
#ifdef HEX
	_actuators.control[4] = -1.0f;
	_actuators.control[5] = -1.0f;
#endif

#ifdef OCT
	_actuators.control[4] = -1.0f;
	_actuators.control[5] = -1.0f;
	_actuators.control[6] = -1.0f;
	_actuators.control[7] = -1.0f;
#endif
	if(hrt_absolute_time() - armed_timing>3000*1000 && hrt_absolute_time() - armed_timing <4000*1000 )
	{
#ifdef QUAD
		_actuators.control[0] = -0.7f;
		_actuators.control[1] = -1.0f;
		_actuators.control[2] = -1.0f;
		_actuators.control[3] = -1.0f;
#endif
#ifdef OCT
		_actuators.control[0] = -0.8f;
		_actuators.control[1] = -1.0f;
		_actuators.control[2] = -1.0f;
		_actuators.control[3] = -1.0f;
		_actuators.control[4] = -0.8f;
		_actuators.control[5] = -1.0f;
		_actuators.control[6] = -1.0f;
		_actuators.control[7] = -1.0f;
#endif
		/*			if(_battery.current_a > max_1_current)
			{
				max_1_current = _battery.current_a;
			}*/
	}

	if(hrt_absolute_time() - armed_timing>5000*1000 && hrt_absolute_time() - armed_timing <6000*1000 )
	{
#ifdef QUAD
		_actuators.control[0] = -1.0f;
		_actuators.control[1] = -0.7f;
		_actuators.control[2] = -1.0f;
		_actuators.control[3] = -1.0f;
#endif
#ifdef OCT
		_actuators.control[0] = -1.0f;
		_actuators.control[1] = -0.8f;
		_actuators.control[2] = -1.0f;
		_actuators.control[3] = -1.0f;
		_actuators.control[4] = -1.0f;
		_actuators.control[5] = -0.8f;
		_actuators.control[6] = -1.0f;
		_actuators.control[7] = -1.0f;
#endif
		/*			if(_battery.current_a > max_2_current)
			{
				max_2_current = _battery.current_a;
			}*/
	}
	else if(hrt_absolute_time() - armed_timing>7000*1000 && hrt_absolute_time() - armed_timing<8000*1000 )
	{
#ifdef QUAD
		_actuators.control[0] = -1.0f;
		_actuators.control[1] = -1.0f;
		_actuators.control[2] = -0.7f;
		_actuators.control[3] = -1.0f;
#endif
#ifdef OCT
		_actuators.control[0] = -1.0f;
		_actuators.control[1] = -1.0f;
		_actuators.control[2] = -0.8f;
		_actuators.control[3] = -1.0f;
		_actuators.control[4] = -1.0f;
		_actuators.control[5] = -1.0f;
		_actuators.control[6] = -0.8f;
		_actuators.control[7] = -1.0f;
#endif
		/*			if(_battery.current_a > max_3_current)
			{
				max_3_current = _battery.current_a;
			}*/
	}
	else if(hrt_absolute_time() - armed_timing>9000*1000 && hrt_absolute_time() - armed_timing<10000*1000 )
	{
#ifdef QUAD
		_actuators.control[0] = -1.0f;
		_actuators.control[1] = -1.0f;
		_actuators.control[2] = -1.0f;
		_actuators.control[3] = -0.7f;
#endif
#ifdef OCT
		_actuators.control[0] = -1.0f;
		_actuators.control[1] = -1.0f;
		_actuators.control[2] = -1.0f;
		_actuators.control[3] = -0.8f;
		_actuators.control[4] = -1.0f;
		_actuators.control[5] = -1.0f;
		_actuators.control[6] = -1.0f;
		_actuators.control[7] = -0.8f;
#endif
		/*			if(_battery.current_a > max_4_current)
			{
				max_4_current = _battery.current_a;
			}*/

	}
	else if(hrt_absolute_time() - armed_timing>11000*1000 && hrt_absolute_time() - armed_timing<11100*1000){
		if(! motor_checked)
		{
			check_motor();
			motor_checked = true;
		}

	}


}


#ifdef QUAD
void MulticopterAttitudeControlLQR::calculate_delta()
{

	/* u = [torque_x, torque_y, torque_z, thrust] */
	matrix::Vector<float, 4> _omega_square;
	if(_thrust_sp>0.1f){
		_u(0) = PX4_ISFINITE(_att_control(0))?_att_control(0)*Ix:0.0f;
		_u(1) = PX4_ISFINITE(_att_control(1))?_att_control(1)*Iy:0.0f;
		_u(2) = PX4_ISFINITE(_att_control(2))?_att_control(2)*Iz:0.0f;
	}
	else
	{
		_u(0) = 0.0f;
		_u(1) = 0.0f;
		_u(2) = 0.0f;
	}
	_u(3) = PX4_ISFINITE(_thrust_sp)?_thrust_sp*2.0f*Mg:0.0f;

	static unsigned int loop = 0;

	_omega_square = _u_omega2*_u;

	/*limit the ratating speed of the motor, the rotating speed could not be less than 0.1*/
	if(_omega_square(0)<0)
		_omega_square(0) = 0;
	if(_omega_square(1)<0)
		_omega_square(1) = 0;
	if(_omega_square(2)<0)
		_omega_square(2) = 0;
	if(_omega_square(3)<0)
		_omega_square(3) = 0;


	_actuators1.control[0] = _u(0);
	_actuators1.control[1] = _u(1);
	_actuators1.control[2] = _u(2);
	_actuators1.control[3] = _thrust_sp;

	// Compute the individual PWM signal ratio and convert from [0, 1] to [-1, 1]
	_actuators.control[0] = limit_range(((sqrtf(_omega_square(0))-Ct)/Cm)*2-1,-1.0f,1.0f);
	_actuators.control[1] = limit_range(((sqrtf(_omega_square(1))-Ct)/Cm)*2-1,-1.0f,1.0f);
	_actuators.control[2] = limit_range(((sqrtf(_omega_square(2))-Ct)/Cm)*2-1,-1.0f,1.0f);
	_actuators.control[3] = limit_range(((sqrtf(_omega_square(3))-Ct)/Cm)*2-1,-1.0f,1.0f);

	if (loop++ % 400 == 0) {
		//	  printf("thrust_sp: %.3f\n", (double)_thrust_sp);
		//	  printf("att_control: %.3f, %.3f, %.3f, %.3f\n", (double)_att_control(0), (double)_att_control(1), (double)_att_control(2), (double)_att_control(3));
		//	  printf("omega2: %.3f, %.3f, %.3f, %.3f\n", (double)_omega_square(0), (double)_omega_square(1), (double)_omega_square(2), (double)_omega_square(3));
		//printf("u: %.3f, %.3f, %.3f, %.3f\n", (double)_u(0), (double)_u(1), (double)_u(2), (double)_u(3));
			  printf("actuators: %.3f, %.3f, %.3f, %.3f, _att_control, %.3f, %.3f, %.3f\n",
					  (double)_actuators.control[0], (double)_actuators.control[1], (double)_actuators.control[2], (double)_actuators.control[3],
					  (double)_att_control(0),(double)_att_control(1),(double)_att_control(2));
	}

}
#endif

#ifdef HEX
void MulticopterAttitudeControl::calculate_delta()
{

	math::Vector <6> _omega_square;
	if(_thrust_sp>0.1f){
		_u(0) = isfinite(_att_control(0))?_att_control(0)*Ix:0.0f;
		_u(1) = isfinite(_att_control(1))?_att_control(1)*Iy:0.0f;
		_u(2) = isfinite(_att_control(2))?_att_control(2)*Iz:0.0f;
	}
	else
	{
		_u(0) = 0.0f;
		_u(1) = 0.0f;
		_u(2) = 0.0f;
	}
	_u(3) = isfinite(_thrust_sp)?_thrust_sp*2.0f*Mg:0.0f;

	_actuators1.control[0] = _att_control(0);
	_actuators1.control[1] = _att_control(1);
	_actuators1.control[2] = _att_control(2);
	_actuators1.control[3] = _thrust_sp;

	_omega_square = _u_omega2*_u;

	if(_omega_square(0)<0.0f)
		_omega_square(0) = 0.0f;
	if(_omega_square(1)<0.0f)
		_omega_square(1) = 0.0f;
	if(_omega_square(2)<0.0f)
		_omega_square(2) = 0.0f;
	if(_omega_square(3)<0.0f)
		_omega_square(3) = 0.0f;
	if(_omega_square(4)<0.0f)
		_omega_square(4) = 0.0f;
	if(_omega_square(5)<0.0f)
		_omega_square(5) = 0.0f;

	_actuators.control[0] = constrain_ref(((sqrtf(_omega_square(0))-Ct)/Cm)*2-1,1.0f);
	_actuators.control[1] = constrain_ref(((sqrtf(_omega_square(1))-Ct)/Cm)*2-1,1.0f);
	_actuators.control[2] = constrain_ref(((sqrtf(_omega_square(2))-Ct)/Cm)*2-1,1.0f);
	_actuators.control[3] = constrain_ref(((sqrtf(_omega_square(3))-Ct)/Cm)*2-1,1.0f);
	_actuators.control[4] = constrain_ref(((sqrtf(_omega_square(4))-Ct)/Cm)*2-1,1.0f);
	_actuators.control[5] = constrain_ref(((sqrtf(_omega_square(5))-Ct)/Cm)*2-1,1.0f);


}
#endif

#ifdef OCT
void MulticopterAttitudeControl::calculate_delta()
{

	math::Vector <8> _omega_square;
	if(_thrust_sp>0.1f){
		_u(0) = isfinite(_att_control(0))?_att_control(0)*Ix:0.0f;
		_u(1) = isfinite(_att_control(1))?_att_control(1)*Iy:0.0f;
		_u(2) = isfinite(_att_control(2))?_att_control(2)*Iz:0.0f;
	}
	else
	{
		_u(0) = 0.0f;
		_u(1) = 0.0f;
		_u(2) = 0.0f;
	}
	_u(3) = isfinite(_thrust_sp)?_thrust_sp*2.0f*Mg:0.0f;





	_omega_square = _u_omega2*_u;


	if(_omega_square(0)<0)
		_omega_square(0) = 0;
	if(_omega_square(1)<0)
		_omega_square(1) = 0;
	if(_omega_square(2)<0)
		_omega_square(2) = 0;
	if(_omega_square(3)<0)
		_omega_square(3) = 0;
	if(_omega_square(4)<0)
		_omega_square(4) = 0;
	if(_omega_square(5)<0)
		_omega_square(5) = 0;
	if(_omega_square(6)<0)
		_omega_square(6) = 0;
	if(_omega_square(7)<0)
		_omega_square(7) = 0;



	_actuators1.control[0] = _u(0);
	_actuators1.control[1] = _u(1);
	_actuators1.control[2] = _u(2);
	_actuators1.control[3] = _thrust_sp;

	_actuators.control[0] = limit_range(((sqrtf(_omega_square(0))-Ct)/Cm)*2-1,-1.0f,1.0f);
	_actuators.control[1] = limit_range(((sqrtf(_omega_square(1))-Ct)/Cm)*2-1,-1.0f,1.0f);
	_actuators.control[2] = limit_range(((sqrtf(_omega_square(2))-Ct)/Cm)*2-1,-1.0f,1.0f);
	_actuators.control[3] = limit_range(((sqrtf(_omega_square(3))-Ct)/Cm)*2-1,-1.0f,1.0f);
	_actuators.control[4] = limit_range(((sqrtf(_omega_square(4))-Ct)/Cm)*2-1,-1.0f,1.0f);
	_actuators.control[5] = limit_range(((sqrtf(_omega_square(5))-Ct)/Cm)*2-1,-1.0f,1.0f);
	_actuators.control[6] = limit_range(((sqrtf(_omega_square(6))-Ct)/Cm)*2-1,-1.0f,1.0f);
	_actuators.control[7] = limit_range(((sqrtf(_omega_square(7))-Ct)/Cm)*2-1,-1.0f,1.0f);


}


#endif







bool MulticopterAttitudeControlLQR::check_motor()
{
	/*		ave_current = _params.check_current;
		_motor_status.motor_fault = false;
		_motor_status.motor1 = true;
		_motor_status.motor2 = true;
		_motor_status.motor3 = true;
		_motor_status.motor4 = true;


	if(max_1_current < ave_current)
	{
		_motor_status.motor1 = false;
		_motor_status.motor_fault = true;
	}

	if(max_2_current < ave_current)
	{
		_motor_status.motor2 = false;
		_motor_status.motor_fault = true;
	}

	if(max_3_current < ave_current)
	{
		_motor_status.motor3 = false;
		_motor_status.motor_fault = true;
	}

	if(max_4_current < ave_current)
	{
		_motor_status.motor4 = false;
		_motor_status.motor_fault = true;
	}


	if(_motor_status_pub > 0)
	{
		orb_publish(ORB_ID(motor_status),_motor_status_pub,&_motor_status);
	}
	else
		_motor_status_pub = orb_advertise(ORB_ID(motor_status),&_motor_status);*/

	return false;
}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
void
MulticopterAttitudeControlLQR::control_attitude_rates(float dt)
{
	/* reset integral if disarmed */
	if (!_armed.armed) {
		_angle_int.zero();
	}

	/* current body angular rates */
	matrix::Vector3f rates;
	rates(0) = _v_att.rollspeed;
	rates(1) = _v_att.pitchspeed;
	rates(2) = _v_att.yawspeed;

	/* angular rates error */
	matrix::Vector3f rates_err = _rates_sp - rates;

	_rates_d = (rates - _rates_prev)/dt;


	_att_control = _params.rate_p.emult(rates_err) - _params.rate_d.emult(_rates_d) + _angle_int + _params.att_p.emult(_angle_error);
	_rates_prev = rates;
	_rates_d_prev = _rates_d;


	/* update integral only if not saturated on low limit */
	if (_thrust_sp > MIN_TAKEOFF_THRUST) {
		for (int i = 0; i < 3; i++) {
			if (fabsf(_att_control(i)/19.6f) < _thrust_sp) {
				float angle_i = _angle_int(i) + _params.angle_i(i) * _angle_error(i) * dt;

				if (PX4_ISFINITE(angle_i) && angle_i > -ANGLE_I_LIMIT && angle_i < ANGLE_I_LIMIT &&_att_control(i) > -ANGLE_I_LIMIT && _att_control(i) < ANGLE_I_LIMIT) {
					_angle_int(i) = angle_i;
				}
			}
		}
	}

	static unsigned int loop1 = 0;
	if (loop1++ % 400 == 0) {
//				printf("[control_attitude_rates]: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
//						(double)rates_err(0), (double)rates_err(1), (double)rates_err(2),
//						(double)_rates_d(0), (double)_rates_d(1), (double)_rates_d(2));
//				printf("[control_attitude_rates]: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
//										(double)_rates_sp(0), (double)_rates_sp(1), (double)_rates_sp(2),
//										(double)rates(0), (double)rates(1), (double)rates(2));
		//
		//		printf("[control_attitude_rates]: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
		//				(double)_att_control(0), (double)_att_control(1), (double)_att_control(2),
		//				(double)_angle_int(0), (double)_angle_int(1), (double)_angle_int(2));
	}
}


void
MulticopterAttitudeControlLQR::task_main_trampoline(int argc, char *argv[])
{
	mc_att_control_lqr::g_control->task_main();
}

void MulticopterAttitudeControlLQR::task_main_pseudo()
{
	PX4_WARN("task_main_pseudo started.\n");
}

void
MulticopterAttitudeControlLQR::task_main()
{
	PX4_WARN("started");
	fflush(stdout);

	/*
	 * do subscriptions
	 */
	printf("do_subscriptions.\n");

	_v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	_home_sub = orb_subscribe(ORB_ID(home_position));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	_battery_sub = orb_subscribe(ORB_ID(battery_status));
	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	_v_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));

	//_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	//_odroid_status_sub=orb_subscribe(ORB_ID(odroid_status));

	memset(&_battery,0,sizeof(_battery));


	gps_valid = false;
	home_valid = false;
	pitch_sp_pre = 0.0f;
	roll_sp_pre = 0.0f;
	yaw_sp_pre = 0.0f;
	was_armed = false;
	armed_timing = 0;
#ifdef QUAD
	// int fixed_time_shutter_count = 0;
	// int time_in_sec = 0;
#endif

	//	  max_1_current = 0.0f;
	//	  max_2_current = 0.0f;
	//	  max_3_current = 0.0f;
	//	  max_4_current = 0.0f;
	//	  ave_current = 1.0f;
	motor_checked = false;
	loop_count = 0;


	Ix  = 0.24f;
	Iy  = 0.26f;
	Iz  = 0.5992f;
	Kt1 = 0.00039915f;
	Kt2 = 0.00044879f;
	Kq1 = 0.000012087f;
	Kq2 = 0.000013324f;
	Lm  = 0.500f;
	Cm  = 440.9687f;  // omega = Cm * delta + Ct
	Ct  = 37.0633f;
	Mg  = 61.5538f; //9.268 Kg  kangli
	air_density_ratio = 1.0f;
	pressure_filtered = 1006.0f;
	temp_filtered = 25.0f;

	//	bool airdensity_filter_init = false;
	//	int  baro_count = 0;
	//	float sum_pressure = 0.0f;
	//	float sum_temp = 0.0f;

	memset(&_actuators,0,sizeof(_actuators));
	memset(&_actuators1,0,sizeof(_actuators1));

	matrix::Matrix<float, 4,4> omega_square_to_u;

	printf("parameters_update.\n");
	/* initialize parameters cache */
	parameters_update();


	/* direct feed motor pwm */
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	//	int alt_rate = -1; // Default to indicate not set.
	//	uint32_t alt_channel_groups = 0;
	//	bool alt_channels_set = false;
	//	bool print_verbose = false;
	//	bool error_on_warn = false;
	//	bool oneshot = false;
	//	int ch;
	int ret;
	char *ep;
	uint32_t set_mask = 0;
	//	unsigned group;
	unsigned long channels;
	unsigned single_ch = 0;
	//int pwm_value;
	//
	//	int myoptind = 1;
	const char *myoptarg = nullptr;
	//set mask
	channels = strtoul(myoptarg, &ep, 0);

	while ((single_ch = channels % 10)) {

		set_mask |= 1 << (single_ch - 1);
		channels /= 10;
	}
	set_mask = 0xf;
	/* open for ioctl only */
	int fd = px4_open(dev, 0);
	if (fd < 0) {
		PX4_ERR("can't open %s", dev);
	}
	/* get the number of servo channels */
	unsigned servo_count;
	ret = px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);
	if (ret != OK) {
		PX4_ERR("PWM_SERVO_GET_COUNT");
	}


	/* wakeup source: vehicle attitude */
	struct pollfd fds[1];

	fds[0].fd = _v_att_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {

		/* wait for up to 100ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		perf_begin(_loop_perf);
		loop_count++;

		/* run controller on attitude changes */
		if (fds[0].revents & POLLIN) {
			static uint64_t last_run = 0;
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too small (< 2ms) and too large (> 20ms) dt's */
			if (dt < 0.002f) {
				dt = 0.002f;

			} else if (dt > 0.02f) {
				dt = 0.02f;
			}


			/* copy attitude topic */
			orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);

			/* check for updates in other topics */
			parameter_update_poll();


			vehicle_control_mode_poll();
			arming_status_poll();
			vehicle_manual_poll();
			home_position_poll();
			local_position_poll();
			gps_poll();
			battery_poll();
			vehicle_status_poll();
			sensor_combined_poll();

			/*			if(loop_count % 10 == 0)
			{
				if(!airdensity_filter_init)
				{
					sum_pressure += _sensor_combined.baro_pres_mbar;
					sum_temp += _sensor_combined.baro_temp_celcius;
					baro_count++;
					if(baro_count>=10)
					{
						pressure_filtered = sum_pressure/baro_count;
						//temp_filtered = sum_temp/baro_count;
						airdensity_filter_init = true;
					}
				}
				else
				{
					pressure_filtered = 0.99f*pressure_filtered + 0.01f*_sensor_combined.baro_pres_mbar;
					//temp_filtered = temp_filtered * 0.99f + 0.0095f * _sensor_combined.baro_temp_celcius;
					temp_filtered = _params.temp;
					air_density_ratio = (pressure_filtered/1006.0f)*((25.0f + 273.15f)/(temp_filtered + 273.15f));
					if(air_density_ratio > 1.3f)
						air_density_ratio = 1.3f;
					else if(air_density_ratio < 0.5f)
						air_density_ratio = 0.5f;
					if(loop_count % 200 == 0)
						update_u_omega2();
				}
			}*/


			if (_v_control_mode.flag_control_attitude_enabled) {
				control_attitude(dt);

				/* publish attitude rates setpoint */
				_v_rates_sp.roll = _rates_sp(0);
				_v_rates_sp.pitch = _rates_sp(1);
				_v_rates_sp.yaw = _rates_sp(2);
				_v_rates_sp.thrust = _thrust_sp;
				_v_rates_sp.timestamp = hrt_absolute_time();

				if (_v_rates_sp_pub != nullptr) {
					orb_publish(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_pub, &_v_rates_sp);

				} else {
					_v_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_v_rates_sp);
				}

			} else {
				/* attitude controller disabled, poll rates setpoint topic */
				if (_v_control_mode.flag_control_manual_enabled) {
					/* manual rates control - ACRO mode */
					_rates_sp = matrix::Vector3f(_manual_control_sp.y, -_manual_control_sp.x, _manual_control_sp.r).emult(_params.acro_rate_max);
					_thrust_sp = _manual_control_sp.z;

					/* reset yaw setpoint after ACRO */
					_reset_yaw_sp = true;

					/* publish attitude rates setpoint */
					_v_rates_sp.roll = _rates_sp(0);
					_v_rates_sp.pitch = _rates_sp(1);
					_v_rates_sp.yaw = _rates_sp(2);
					_v_rates_sp.thrust = _thrust_sp;
					_v_rates_sp.timestamp = hrt_absolute_time();

					if (_v_rates_sp_pub != nullptr) {
						orb_publish(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_pub, &_v_rates_sp);

					} else {
						_v_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_v_rates_sp);
					}

				} else {
					/* attitude controller disabled, poll rates setpoint topic */
					vehicle_rates_setpoint_poll();
					_rates_sp(0) = _v_rates_sp.roll;
					_rates_sp(1) = _v_rates_sp.pitch;
					_rates_sp(2) = _v_rates_sp.yaw;
					_thrust_sp = _v_rates_sp.thrust;
				}
			}

			if (_v_control_mode.flag_control_rates_enabled) {
				control_attitude_rates(dt); //kangli

				if(_armed.armed)
				{
					if(was_armed == false)
					{
						was_armed = true;
						armed_timing = hrt_absolute_time();
					}


					if(hrt_absolute_time() - armed_timing<11100*1000 && _params.check_on_off > 0.5f)  //kangli modified on 8th Dec
					{
						//							pre_arm_check();
					}
					else
					{

						calculate_delta();
						//gen pwm to motors
/*						for (unsigned i = 0; i < 4; i++) {
							if (set_mask & 1 << i) {
								pwm_value = (int)((_actuators.control[i] + 1.0f)  * 500.0f + 1000.0f);
								//convert from -1 - 1 to 0-2, and add 1000, and times 500 (because range from pwm output is 1000-2000, and control output is 0-2)
								ret = px4_ioctl(fd, PWM_SERVO_SET(i), pwm_value);

								if (ret != OK) {
									PX4_ERR("PWM_SERVO_SET(%d)", i);
									//return 1;
								}
							}
						}*/
						//printf("actuators: %.3f, %.3f, %.3f, %.3f\n",
						//					  (double)_actuators.control[0], (double)_actuators.control[1], (double)_actuators.control[2], (double)_actuators.control[3]);

					}
				}
				else{
					_actuators.control[0] = -1.0f;
					_actuators.control[1] = -1.0f;
					_actuators.control[2] = -1.0f;
					_actuators.control[3] = -1.0f;
#ifdef QUAD
					_actuators.control[4] = -0.4f;
					_actuators.control[5] =  0.0f;
#endif

#ifdef HEX
					_actuators.control[4] = -1.0f;
					_actuators.control[5] = -1.0f;
#endif

#ifdef OCT
					_actuators.control[4] = -1.0f;
					_actuators.control[5] = -1.0f;
					_actuators.control[6] = -1.0f;
					_actuators.control[7] = -1.0f;

#endif

					_actuators1.control[0] = 0.0f;
					_actuators1.control[1] = 0.0f;
					_actuators1.control[2] = 0.0f;
					_actuators1.control[3] = 0.0f;

					//gen pwm to motors
//					for (unsigned i = 0; i < 4; i++) {
//						if (set_mask & 1 << i) {
//							ret = px4_ioctl(fd, PWM_SERVO_SET(i), 900);
//
//							if (ret != OK) {
//								PX4_ERR("PWM_SERVO_SET(%d)", i);
//								//return 1;
//							}
//						}
//					}

				}

				/*
#ifdef QUAD
				if(_vehicle_serial_cmd.cmd1==1)
					{
				//	printf("start to act...................\n");
					_actuators.control[5] = 1.0f;

					}

					else
						_actuators.control[5] = -1.0f;
#endif

				 */
				//				if(!_v_control_mode.flag_control_manual_enabled)
				//				 {
				//					_actuators1.control[4] = _vehicle_serial_cmd.servo1;
				//					_actuators1.control[5] = _vehicle_serial_cmd.servo2;
				//					_actuators1.control[6] = _vehicle_serial_cmd.servo3;
				//					_actuators1.control[7] = _vehicle_serial_cmd.servo4;
				//				 }
				//				else
				//				{
				//					_actuators1.control[4] = _manual_control_sp.aux1;
				//					_actuators1.control[5] = _manual_control_sp.aux2;
				//					_actuators1.control[6] = _manual_control_sp.aux3;
				//					_actuators1.control[7] = _manual_control_sp.aux4;
				//
				//				}
				_actuators.timestamp = hrt_absolute_time();

				if (!_actuators_0_circuit_breaker_enabled) {
					if (_actuators_0_pub != nullptr) {
						orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);
					} else {
						_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
					}
					/*mingjie, gimbal_aux will publish this topic*/
					/*					if (_actuators_1_pub != nullptr) {
						orb_publish(ORB_ID(actuator_controls_1), _actuators_1_pub, &_actuators1);

					} else {
						_actuators_1_pub = orb_advertise(ORB_ID(actuator_controls_1), &_actuators1);
					}*/

				}
			}
		}

		perf_end(_loop_perf);
	}

	warnx("exit");

	_control_task = -1;
	_exit(0);
}

int
MulticopterAttitudeControlLQR::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("mc_att_control_lqr",
			SCHED_DEFAULT,
			SCHED_PRIORITY_MAX - 5,
			4000, //2000,
			(px4_main_t)&MulticopterAttitudeControlLQR::task_main_trampoline,
			nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int mc_att_control_lqr_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: mc_att_control_lqr {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (mc_att_control_lqr::g_control != nullptr)
			errx(1, "already running");

		mc_att_control_lqr::g_control = new MulticopterAttitudeControlLQR;

		if (mc_att_control_lqr::g_control == nullptr)
			errx(1, "alloc failed");

		if (OK != mc_att_control_lqr::g_control->start()) {
			delete mc_att_control_lqr::g_control;
			mc_att_control_lqr::g_control = nullptr;
			err(1, "start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (mc_att_control_lqr::g_control == nullptr)
			errx(1, "not running");

		delete mc_att_control_lqr::g_control;
		mc_att_control_lqr::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (mc_att_control_lqr::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
