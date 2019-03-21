/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file mc_pos_control_rpt_main.cpp
 * Multicopter position controller.
 *
 * The controller has two loops: P loop for position error and PID loop for velocity error.
 * Output of velocity controller is thrust vector that splitted to thrust direction
 * (i.e. rotation matrix for multicopter orientation) and thrust module (i.e. multicopter thrust itself).
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module_params.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <drivers/drv_hrt.h>
#include <systemlib/hysteresis/hysteresis.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
//#include <arch/board/board.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
//#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
//#include <systemlib/param/param.h>

#include <float.h>
#include <systemlib/err.h>
#include <controllib/blocks.hpp>
//#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <lib/FlightTasks/FlightTasks.hpp>

#include <lib/ecl/geo/geo.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <systemlib/mavlink_log.h>
#include <matrix/matrix/math.hpp>


#define TILT_COS_MAX	0.7f
#define SIGMA			0.000001f


/**
 * Multicopter position control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_pos_control_rpt_main(int argc, char *argv[]);

class MulticopterPositionControlRPT
{
public:
	/**
	 * Constructor
	 */
	MulticopterPositionControlRPT();

	/**
	 * Destructor, also kills task.
	 */
	~MulticopterPositionControlRPT();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:
	const float alt_ctl_dz = 0.05f;

	bool		_task_should_exit;		/**< if true, task should exit */
	int		_control_task;			/**< task handle for task */
	int		_mavlink_fd;			/**< mavlink fd */

	int		_att_sub;				/**< vehicle attitude subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_control_mode_sub;		/**< vehicle control mode subscription */
	int		_params_sub;			/**< notification of parameter updates */
	int		_manual_sub;			/**< notification of manual control updates */
	int		_arming_sub;			/**< arming status of outputs */
	int		_local_pos_sub;			/**< vehicle local position */
	int		_pos_sp_triplet_sub;	/**< position setpoint triplet */
	int		_local_pos_sp_sub;		/**< offboard local position setpoint */
//	int		_global_vel_sp_sub;		/**< offboard global velocity setpoint */
	int     _v_status_sub; 		/** < vehicel status */
	int     _vehicle_land_detected_sub;	/**< vehicle land detected status */

	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
	orb_advert_t	_local_pos_sp_pub;		/**< vehicle local position setpoint publication */
//	orb_advert_t	_global_vel_sp_pub;		/**< vehicle global velocity setpoint publication */
	orb_advert_t	_mavlink_log_pub;		/**< mavlink log advert */

	struct vehicle_attitude_s			_att;			/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct vehicle_control_mode_s			_control_mode;	/**< vehicle control mode */
	struct actuator_armed_s				_arming;		/**< actuator arming status */
	struct vehicle_local_position_s			_local_pos;		/**< vehicle local position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;	/**< vehicle global position setpoint triplet */
	struct vehicle_local_position_setpoint_s	_local_pos_sp;		/**< vehicle local position setpoint */
//	struct vehicle_global_velocity_setpoint_s	_global_vel_sp;	/**< vehicle global velocity setpoint */
    struct vehicle_status_s                  _v_status;
    struct vehicle_land_detected_s		_v_land_detected;

	struct {
		param_t thr_min;
		param_t thr_max;
		param_t z_p;
		param_t z_vel_p;
		param_t z_vel_i;
		param_t z_i;
		param_t z_vel_d;
		param_t z_vel_max;
		param_t z_ff;
		param_t xy_p;
		param_t xy_vel_p;
		param_t xy_vel_i;
		param_t xy_i;
		param_t xy_vel_d;
		param_t xy_vel_max;
		param_t xy_ff;
		param_t tilt_max_air;
		param_t land_speed;
		param_t tilt_max_land;
		param_t nav_z_max;
		param_t nav_r_max;
		param_t nav_acc_max;
		param_t nav_z_acc_max;
		param_t nav_max_jerk_xy;
		param_t max_xy_off;
		param_t max_z_off;
		param_t max_z_control;
		param_t max_man_int;
		param_t max_z_height;
	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		float thr_min;
		float thr_max;
		float tilt_max_air;
		float land_speed;
		float tilt_max_land;
		float nav_zmax;
		float nav_rmax;
        float max_xy_acc;
        float max_z_acc;
        matrix::Vector3f pos_p;
        matrix::Vector3f vel_p;
        matrix::Vector3f vel_i;
        matrix::Vector3f pos_i;
        matrix::Vector3f vel_d;
        matrix::Vector3f vel_ff;
        matrix::Vector3f vel_max;
        matrix::Vector3f sp_offs_max;
		float nav_max_jerk_xy;
		float max_z_control;
		float max_man_int;
		float max_z_height;

	}		_params;


	struct map_projection_reference_s _ref_pos;
	float _ref_alt;
	hrt_abstime _ref_timestamp;

	bool _reset_pos_sp;
	bool _reset_alt_sp;
	bool  reset_int_z;

	bool _reset_pos_loiter_sp;
	bool _reset_alt_loiter_sp;


	bool pos_control_init;
	bool semi_auto_pos_control;
	bool semi_auto_takeoff_init;   // init the takeoff process if triggered
	bool semi_auto_takeoff_finish; // indicate whether the semi-auto takeoff has been enabled or not
	bool semi_auto_takeoff_enable; // when landed, then this is enabled





	matrix::Vector3f _pos;
	matrix::Vector3f _pos_sp;
	matrix::Vector3f _vel;
	matrix::Vector3f _vel_sp;
	matrix::Vector3f _acc_sp;             /** acceleration reference **/
	matrix::Vector3f _acc_sp_govened;     /** acceleration govened, ensure that the acc sp is smoothed **/
	matrix::Vector3f _vel_prev;			/**< velocity on previous step */
	matrix::Vector3f _vel_ff;
	matrix::Vector3f _sp_move_rate;

    float dzdt;
    float dyawdt;
    float dt;
    int loop_count;


	/**
	 * Update our local parameter cache.
	 */
	int			parameters_update(bool force);

	/**
	 * Update control outputs
	 */
	void		control_update();

	/**
	 * Check for changes in subscribed topics.
	 */
	void		poll_subscriptions();

	static float	scale_control(float ctl, float end, float dz);

	/**
	 * Update reference for local position projection
	 */
	void		update_ref();
	/**
	 * Reset position setpoint to current position
	 */
	void		reset_pos_sp();

	/**
	 * Reset altitude setpoint to current altitude
	 */
	void		reset_alt_sp();

	bool 		switch_pos_control();

	void        reset_pos_sp_loiter();

	void        reset_alt_sp_loiter();

	/**
	 *
	 *
	 * Set position setpoint using manual control
	 */
	void		control_manual();

	/**
	 * Set position setpoint using offboard control
	 */
	void		control_offboard();

	/**
	 * Select between barometric and global (AMSL) altitudes
	 */
	void		select_alt(bool global);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);


	float constrain_ref(float x, float cons);

	float limit_acc(float cur,float pre,float max_acc);

	int sign(float in);
	/**
	 * Main sensor collection task.
	 */
	void		task_main();
};

namespace pos_control_rpt
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

MulticopterPositionControlRPT	*g_control;
}

MulticopterPositionControlRPT::MulticopterPositionControlRPT() :

	_task_should_exit(false),
	_control_task(-1),
	_mavlink_fd(-1),

/* subscriptions */
	_att_sub(-1),
	_att_sp_sub(-1),
	_control_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_arming_sub(-1),
	_local_pos_sub(-1),
	_pos_sp_triplet_sub(-1),
//	_global_vel_sp_sub(-1),
	_v_status_sub(-1),
	_vehicle_land_detected_sub(-1),

/* publications */
	_att_sp_pub(nullptr),
	_local_pos_sp_pub(nullptr),
//	_global_vel_sp_pub(nullptr),
	_mavlink_log_pub(nullptr),

	_ref_alt(0.0f),
	_ref_timestamp(0),

	_reset_pos_sp(true),
	_reset_alt_sp(true),
	reset_int_z(false),
	_reset_pos_loiter_sp(false),
	_reset_alt_loiter_sp(false),
	pos_control_init(false),
	semi_auto_pos_control(false),
	semi_auto_takeoff_init(false),
	semi_auto_takeoff_finish(false),
	semi_auto_takeoff_enable(false),
	dt(0.0f),
	loop_count(0)
{
	memset(&_att, 0, sizeof(_att));
	memset(&_att_sp, 0, sizeof(_att_sp));
	memset(&_manual, 0, sizeof(_manual));
	memset(&_control_mode, 0, sizeof(_control_mode));
	memset(&_arming, 0, sizeof(_arming));
	memset(&_local_pos, 0, sizeof(_local_pos));
	memset(&_pos_sp_triplet, 0, sizeof(_pos_sp_triplet));
	memset(&_local_pos_sp, 0, sizeof(_local_pos_sp));
//	memset(&_global_vel_sp, 0, sizeof(_global_vel_sp));
	memset(&_v_status,0,sizeof(_v_status));
	memset(&_ref_pos, 0, sizeof(_ref_pos));
	memset(&_v_land_detected, 0, sizeof(_v_land_detected));

	_params.pos_p.zero();
	_params.vel_p.zero();
	_params.vel_i.zero();
	_params.vel_d.zero();
	_params.vel_max.zero();
	_params.vel_ff.zero();
	_params.sp_offs_max.zero();

	_pos.zero();
	_pos_sp.zero();
	_vel.zero();
	_vel_sp.zero();
	_acc_sp.zero();
	_acc_sp_govened.zero();
	_vel_prev.zero();
	_vel_ff.zero();
	_sp_move_rate.zero();

	_params_handles.thr_min		= param_find("_MPC_THR_MIN");
	_params_handles.thr_max		= param_find("_MPC_THR_MAX");
	_params_handles.z_p			= param_find("_MPC_Z_P");
	_params_handles.z_vel_p		= param_find("_MPC_Z_VEL_P");
	_params_handles.z_vel_i		= param_find("_MPC_Z_VEL_I");
	_params_handles.z_i 		= param_find("_MPC_Z_I");
	_params_handles.z_vel_d		= param_find("_MPC_Z_VEL_D");
	_params_handles.z_vel_max	= param_find("_MPC_Z_VEL_MAX");
	_params_handles.z_ff		= param_find("_MPC_Z_FF");
	_params_handles.xy_p		= param_find("_MPC_XY_P");
	_params_handles.xy_vel_p	= param_find("_MPC_XY_VEL_P");
	_params_handles.xy_vel_i	= param_find("_MPC_XY_VEL_I");
	_params_handles.xy_i		= param_find("_MPC_XY_I");
	_params_handles.xy_vel_d	= param_find("_MPC_XY_VEL_D");
	_params_handles.xy_vel_max	= param_find("_MPC_XY_VEL_MAX");
	_params_handles.xy_ff		= param_find("_MPC_XY_FF");
	_params_handles.tilt_max_air	= param_find("_MPC_TILTMAX_AIR");
	_params_handles.land_speed	= param_find("_MPC_LAND_SPEED");
	_params_handles.tilt_max_land	= param_find("_MPC_TILTMAX_LND");
	_params_handles.nav_z_max = param_find("NAV_ZMAX");
	_params_handles.nav_r_max = param_find("NAV_RMAX");
	_params_handles.nav_acc_max = param_find("NAV_MAX_ACC_XY");
	_params_handles.nav_z_acc_max = param_find("NAV_MAX_ACC_Z");
	_params_handles.nav_max_jerk_xy = param_find("NAV_MAX_JERK");
	_params_handles.max_xy_off = param_find("_MPC_XY_OFF_MAX");
	_params_handles.max_z_off = param_find("_MPC_Z_OFF_MAX");
	_params_handles.max_z_control = param_find("_MPC_Z_CTL_MAX");
	_params_handles.max_man_int = param_find("_MPC_MAN_MAX_INT");
	_params_handles.max_z_height = param_find("_MPC_Z_MAX");
 	/* fetch initial parameter values */
	parameters_update(true);
}

MulticopterPositionControlRPT::~MulticopterPositionControlRPT()
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

	pos_control_rpt::g_control = nullptr;
}

int
MulticopterPositionControlRPT::parameters_update(bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(_params_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_upd);
	}

	if (updated || force) {
		param_get(_params_handles.thr_min, &_params.thr_min);
		param_get(_params_handles.thr_max, &_params.thr_max);
		param_get(_params_handles.tilt_max_air, &_params.tilt_max_air);
		_params.tilt_max_air = math::radians(_params.tilt_max_air);
		param_get(_params_handles.land_speed, &_params.land_speed);
		param_get(_params_handles.tilt_max_land, &_params.tilt_max_land);
		_params.tilt_max_land = math::radians(_params.tilt_max_land);
        param_get(_params_handles.nav_z_max, & _params.nav_zmax);
        param_get(_params_handles.nav_r_max, & _params.nav_rmax);
        param_get(_params_handles.nav_acc_max,&_params.max_xy_acc);
        param_get(_params_handles.nav_z_acc_max,&_params.max_z_acc);
        param_get(_params_handles.nav_max_jerk_xy,&_params.nav_max_jerk_xy);

		float v;
		param_get(_params_handles.xy_p, &v);
		_params.pos_p(0) = v;
		_params.pos_p(1) = v;
		param_get(_params_handles.z_p, &v);
		_params.pos_p(2) = v;
		param_get(_params_handles.xy_i,&v);
		_params.pos_i(0) = v;
		_params.pos_i(1) = v;
		param_get(_params_handles.z_i,&v);
		_params.pos_i(2) = v;
		param_get(_params_handles.xy_vel_p, &v);
		_params.vel_p(0) = v;
		_params.vel_p(1) = v;
		param_get(_params_handles.z_vel_p, &v);
		_params.vel_p(2) = v;
		param_get(_params_handles.xy_vel_i, &v);
		_params.vel_i(0) = v;
		_params.vel_i(1) = v;
		param_get(_params_handles.z_vel_i, &v);
		_params.vel_i(2) = v;
		param_get(_params_handles.xy_vel_d, &v);
		_params.vel_d(0) = v;
		_params.vel_d(1) = v;
		param_get(_params_handles.z_vel_d, &v);
		_params.vel_d(2) = v;
		param_get(_params_handles.xy_vel_max, &v);
		_params.vel_max(0) = v;
		_params.vel_max(1) = v;
		param_get(_params_handles.z_vel_max, &v);
		_params.vel_max(2) = v;
		param_get(_params_handles.xy_ff, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.vel_ff(0) = v;
		_params.vel_ff(1) = v;
		param_get(_params_handles.z_ff, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.vel_ff(2) = v;

		param_get(_params_handles.max_xy_off,&_params.sp_offs_max(0));
		param_get(_params_handles.max_xy_off,&_params.sp_offs_max(1));
		param_get(_params_handles.max_z_off,&_params.sp_offs_max(2));
		param_get(_params_handles.max_z_control, & _params.max_z_control);
		param_get(_params_handles.max_man_int,&_params.max_man_int);
		param_get(_params_handles.max_z_height,&_params.max_z_height);
		_params.max_z_height = -_params.max_z_height;
	}

	return OK;
}

void
MulticopterPositionControlRPT::poll_subscriptions()
{
	bool updated;

	orb_check(_att_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);
	}

	orb_check(_att_sp_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
	}

	orb_check(_control_mode_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}

	orb_check(_manual_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}

	orb_check(_arming_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _arming_sub, &_arming);
	}

	orb_check(_local_pos_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	}

	orb_check(_v_status_sub,&updated);
	if(updated) {
		orb_copy(ORB_ID(vehicle_status), _v_status_sub, &_v_status);
	}

	orb_check(_vehicle_land_detected_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_v_land_detected);
	}
}

float
MulticopterPositionControlRPT::scale_control(float ctl, float end, float dz)
{
	if (ctl > dz) {
		return (ctl - dz) / (end - dz);

	} else if (ctl < -dz) {
		return (ctl + dz) / (end - dz);

	} else {
		return 0.0f;
	}
}

void
MulticopterPositionControlRPT::task_main_trampoline(int argc, char *argv[])
{
	pos_control_rpt::g_control->task_main();
}

void
MulticopterPositionControlRPT::update_ref()
{
	if (_local_pos.ref_timestamp != _ref_timestamp) {
		double lat_sp, lon_sp;
		float alt_sp = 0.0f;

		if (_ref_timestamp != 0) {
			/* calculate current position setpoint in global frame */
			map_projection_reproject(&_ref_pos, _pos_sp(0), _pos_sp(1), &lat_sp, &lon_sp);
			alt_sp = _ref_alt - _pos_sp(2);
		}

		/* update local projection reference */
		map_projection_init(&_ref_pos, _local_pos.ref_lat, _local_pos.ref_lon);
		_ref_alt = _local_pos.ref_alt;

		if (_ref_timestamp != 0) {
			/* reproject position setpoint to new reference */
			map_projection_project(&_ref_pos, lat_sp, lon_sp, &_pos_sp(0), &_pos_sp(1));
			_pos_sp(2) = -(alt_sp - _ref_alt);
		}

		_ref_timestamp = _local_pos.ref_timestamp;
	}
}

void
MulticopterPositionControlRPT::reset_pos_sp()
{
	if (_reset_pos_sp) {
		_reset_pos_sp = false;
		/* shift position setpoint to make attitude setpoint continuous */
		_pos_sp(0) = _pos(0);// + (_vel(0)*_params.vel_p(0) -  _att_sp.R_body[0][2] * 0.5f ) / _params.pos_p(0); // 0.5f is the pre set throttle hover trim
		_pos_sp(1) = _pos(1);// + (_vel(1)*_params.vel_p(1)  - _att_sp.R_body[1][2] * 0.5f ) / _params.pos_p(1);

		matrix::Quatf quat(_att_sp.q_d[0], _att_sp.q_d[1], _att_sp.q_d[2], _att_sp.q_d[3]);
		matrix::Matrix<float, 3,3> R_body = quat.to_dcm();

//		_vel_sp(0) = _vel(0)-  _att_sp.R_body[0][2] * 0.5f /_params.vel_p(0) /_att_sp.R_body[2][2];
//		_vel_sp(1) = _vel(1)-  _att_sp.R_body[1][2] * 0.5f /_params.vel_p(1) /_att_sp.R_body[2][2];
		_vel_sp(0) = _vel(0)-  R_body(0, 2) * 0.5f /_params.vel_p(0) / R_body(2, 2);
		_vel_sp(1) = _vel(1)-  R_body(1, 2) * 0.5f /_params.vel_p(1) / R_body(2, 2);
		_vel_prev = _vel; // to solve the crash bug
//		mavlink_log_info(_mavlink_fd, "[mpc] reset pos sp: %.2f, %.2f", (double)_pos_sp(0), (double)_pos_sp(1));

//		_vel_sp(0) = _vel(0)-  _att_sp.R_body[0][2] * 0.5f ;
//		_vel_sp(1) = _vel(1)-  _att_sp.R_body[1][2] * 0.5f ;

	}
}

void
MulticopterPositionControlRPT::reset_alt_sp()
{
	if (_reset_alt_sp) {
		_reset_alt_sp = false;

		_pos_sp(2) = _pos(2);
		_vel_sp(2) = _vel(2);

	}
}
bool
MulticopterPositionControlRPT::switch_pos_control()
{
	float vel_sq = _vel(0) * _vel(0) + _vel(1) * _vel(1);

	if(fabsf(_vel_sp(0))<0.0001f && fabsf(_vel_sp(1))<0.0001f && vel_sq < 0.09f)
		return true;

	return false;
}

void
MulticopterPositionControlRPT::reset_pos_sp_loiter()
{
	if (_reset_pos_loiter_sp) {
		_acc_sp(0) = 0;
		_acc_sp(1) = 0;

		_pos_sp(0) += _vel_sp(0) * dt + 0.5f*_params.max_xy_acc*sign(0 - _vel_sp(0)) * dt * dt;
		_pos_sp(1) += _vel_sp(1) * dt + 0.5f*_params.max_xy_acc*sign(0 - _vel_sp(1)) * dt * dt;

		_vel_sp(0) = limit_acc(0,_vel_sp(0),_params.max_xy_acc);
		_vel_sp(1) = limit_acc(0,_vel_sp(1),_params.max_xy_acc);

		if(switch_pos_control())
		{
			_reset_pos_loiter_sp = false;
		}
	}
}

void
MulticopterPositionControlRPT::reset_alt_sp_loiter()
{
	if (_reset_alt_loiter_sp) {

		_acc_sp(2) = 0;
		_pos_sp(2) += _vel_sp(2) * dt + 0.5f*_params.max_z_acc*sign(0 - _vel_sp(2)) * dt * dt;
		_vel_sp(2) = limit_acc(0,_vel_sp(2),_params.max_z_acc);

		if(_vel_sp(2)<=0.001f && _vel(2)*_vel(2) < 0.09f)
			_reset_alt_loiter_sp = false;
	}
}



void
MulticopterPositionControlRPT::control_manual()
{
	_sp_move_rate.zero();

	if(_v_land_detected.landed && _control_mode.flag_control_altitude_enabled)
	{
		semi_auto_takeoff_enable = true;
		_pos_sp(0) = _pos(0);
		_pos_sp(1) = _pos(1);
		_pos_sp(2) = _pos(2);
		_vel_sp(0) = _vel(0);
		_vel_sp(1) = _vel(1);
		_vel_sp(2) = _vel(2);
		_acc_sp(0) = 0;
		_acc_sp(1) = 0;
		_acc_sp(2) = 0;
		semi_auto_takeoff_init = false;
	}

	if (_control_mode.flag_control_altitude_enabled) {
		/* move altitude setpoint with throttle stick */
		_sp_move_rate(2) = -scale_control(_manual.z - 0.5f, 0.5f, alt_ctl_dz);
		if(_sp_move_rate(2)>0.5f)
			_sp_move_rate(2) = 0.5f;

	}

	if(semi_auto_takeoff_enable == true && fabsf(_manual.z-0.5f)<0.05f)
	{
		semi_auto_takeoff_enable = false;
	}


	if (_control_mode.flag_control_position_enabled) {
		/* move position setpoint with roll/pitch stick */
		_sp_move_rate(0) = scale_control(_manual.x, 1.0f, 0.05f);
		_sp_move_rate(1) = scale_control(_manual.y, 1.0f, 0.05f);
	}

	matrix::Dcmf R_yaw_sp;
	//matrix::Matrix<float, 3,3> R_yaw_sp;
	matrix::Quatf quat1(_att.q[0], _att.q[1], _att.q[2], _att.q[3]);
	matrix::Vector3f euler1 = matrix::Eulerf(quat1);
	R_yaw_sp = matrix::Eulerf(0.0f, 0.0f, euler1(0));

	/* _sp_move_rate scaled to 0..1, scale it to max speed and rotate around yaw */
	_vel_ff = R_yaw_sp * _sp_move_rate.emult(_params.vel_max);



	if (_control_mode.flag_control_altitude_enabled) {
		/* reset alt setpoint to current altitude if needed */
		reset_alt_sp();
	}

	if (_control_mode.flag_control_position_enabled && _reset_pos_sp) {
		/* reset position setpoint to current position if needed */
		reset_pos_sp();
	}

	if(semi_auto_pos_control)
	{
         if(fabsf(_vel_ff(0))>0.01f||fabsf(_vel_ff(1))>0.01f)
        	 semi_auto_pos_control = false;
	}
	else
	{
		pos_control_init = false;

		_acc_sp(0) = 0;
		_acc_sp(1) = 0;

		_pos_sp(0) += _vel_sp(0) * dt + 0.5f*_params.max_xy_acc*sign(_vel_ff(0) - _vel_sp(0)) * dt * dt;
		_pos_sp(1) += _vel_sp(1) * dt + 0.5f*_params.max_xy_acc*sign(_vel_ff(1) - _vel_sp(1)) * dt * dt;

		_vel_sp(0) = limit_acc(_vel_ff(0),_vel_sp(0),_params.max_xy_acc);
		_vel_sp(1) = limit_acc(_vel_ff(1),_vel_sp(1),_params.max_xy_acc);


        if(fabsf(_vel_ff(0))<0.01f&&fabsf(_vel_ff(1))<0.01f&&switch_pos_control())
        	semi_auto_pos_control = true;

	}


	matrix::Vector3f pos_sp_offs;
	pos_sp_offs.zero();

	pos_sp_offs(0) = _pos_sp(0) - _pos(0);
	pos_sp_offs(1) = _pos_sp(1) - _pos(1);


	float pos_sp_offs_xy = sqrtf(pos_sp_offs(0)*pos_sp_offs(0) + pos_sp_offs(1) * pos_sp_offs(1));
	float pos_sp_offs_xy_norm = pos_sp_offs_xy/_params.sp_offs_max(0);

	if(pos_sp_offs_xy_norm > 1.0f)
	{
		pos_sp_offs(0) = pos_sp_offs(0)/pos_sp_offs_xy_norm;
		pos_sp_offs(1) = pos_sp_offs(1)/pos_sp_offs_xy_norm;
		_pos_sp(0) = _pos(0) + pos_sp_offs(0);
		_pos_sp(1) = _pos(1) + pos_sp_offs(1);
	}


	if(!semi_auto_takeoff_enable)
	{


		/* feed forward setpoint move rate with weight vel_ff */

		/* move position setpoint */

		_acc_sp(2) = 0;
		_pos_sp(2) += _vel_sp(2) * dt + 0.5f*_params.max_z_acc*sign(_vel_ff(2) - _vel_sp(2)) * dt * dt;
		_vel_sp(2) = limit_acc(_vel_ff(2),_vel_sp(2),_params.max_z_acc);

		/* check if position setpoint is too far from actual position */
		pos_sp_offs(2) = _pos_sp(2) - _pos(2);


		float pos_sp_offs_norm = fabsf(pos_sp_offs(2))/_params.sp_offs_max(2);
		if (pos_sp_offs_norm > 1.0f) {
			pos_sp_offs(2) /= pos_sp_offs_norm;
			_pos_sp(2) = _pos(2) + pos_sp_offs(2);
		}

	}

}


float MulticopterPositionControlRPT::constrain_ref(float x, float cons)
{
	if(x>cons)
		x = cons;
	else if (x< - cons)
		x = - cons;
	return x;
}

float MulticopterPositionControlRPT::limit_acc(float cur, float pre, float max_acc)
{
	float error = cur-pre;
	float error_dot = error/dt;
	error_dot = constrain_ref(error_dot,max_acc);
	return pre + error_dot * dt;
}


int MulticopterPositionControlRPT::sign(float in)
{
	if(in > 0.001f)
		return 1;
	else if(in< -0.001f)
		return -1;

	return 0;
}

void
MulticopterPositionControlRPT::task_main()
{
	PX4_WARN("started");

//	_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
//	mavlink_log_info(_mavlink_fd, "[mpc] started");
	mavlink_log_info(&_mavlink_log_pub, "[mpc] stopped");

	/*
	 * do subscriptions
	 */
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_arming_sub = orb_subscribe(ORB_ID(actuator_armed));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
//	_global_vel_sp_sub = orb_subscribe(ORB_ID(vehicle_global_velocity_setpoint));
	_v_status_sub  = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));

	parameters_update(true);

	/* initialize values of critical structs until first regular update */
	_arming.armed = false;

	/* get an initial update for all sensor and status data */
	poll_subscriptions();
	_acc_sp_govened.zero();

	bool reset_int_z_manual = false;
	bool reset_int_xy = true;
	bool was_armed = false;


	hrt_abstime t_prev = 0;

	matrix::Vector3f thrust_int;
	thrust_int.zero();
	matrix::Dcmf R;
	R.identity();


	_pos(0) = _local_pos.x;
	_pos(1) = _local_pos.y;
	_pos(2) = _local_pos.z;

	_vel(0) = _local_pos.vx;
	_vel(1) = _local_pos.vy;
	_vel(2) = _local_pos.vz;

	/* wakeup source */
	struct pollfd fds[1];

	fds[0].fd = _local_pos_sub;
	fds[0].events = POLLIN;


	while (!_task_should_exit) {
		/* wait for up to 500ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		loop_count++;
		poll_subscriptions();
		parameters_update(false);

		hrt_abstime t = hrt_absolute_time();
	    dt = t_prev != 0 ? (t - t_prev) * 0.000001f : 0.005f;
		t_prev = t;

		if (_control_mode.flag_armed && !was_armed) {
			/* reset setpoints and integrals on arming */
			_reset_pos_sp = true;
			_reset_alt_sp = true;
			reset_int_z = true;
			reset_int_xy = true;
		}

		was_armed = _control_mode.flag_armed;

		update_ref();
		if (_control_mode.flag_control_altitude_enabled ||
		    _control_mode.flag_control_position_enabled ||
		    _control_mode.flag_control_climb_rate_enabled ||
		    _control_mode.flag_control_velocity_enabled ||
			_control_mode.flag_control_manual_enabled) {

			_pos(0) = _local_pos.x;
			_pos(1) = _local_pos.y;
			_pos(2) = _local_pos.z;

			_vel(0) = _local_pos.vx;
			_vel(1) = _local_pos.vy;
			_vel(2) = _local_pos.vz;

			_vel_ff.zero();
			_sp_move_rate.zero();



			/* select control source */
			if (_control_mode.flag_control_manual_enabled) {
				/* manual control */
				control_manual();
				_reset_pos_loiter_sp = true;
			}  else {
				/* AUTO */
//				printf("pos_control: auto!\n");
				semi_auto_pos_control = false;
				semi_auto_takeoff_enable = false;
//				_reset_alt_sp = false;
//				_reset_pos_sp = false;
				bool updated;
				orb_check(_pos_sp_triplet_sub, &updated);

				if (updated) {
					orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
				}

				if (_pos_sp_triplet.current.valid) {
					/* in case of interrupted mission don't go to waypoint but stay at current position */
					_reset_pos_loiter_sp = true;
					_reset_alt_loiter_sp = true;
					/* project setpoint to local frame */
					_pos_sp(0) = _pos_sp_triplet.current.x;
					_pos_sp(1) = _pos_sp_triplet.current.y;
					_pos_sp(2) = -_pos_sp_triplet.current.alt;
					_vel_sp(0) = _pos_sp_triplet.current.vx;
					_vel_sp(1) = _pos_sp_triplet.current.vy;
					_vel_sp(2) = -_pos_sp_triplet.current.vz;

					_acc_sp(0) = _pos_sp_triplet.current.a_x;
					_acc_sp(1) = _pos_sp_triplet.current.a_y;
					_acc_sp(2) = -_pos_sp_triplet.current.a_z;

					if (PX4_ISFINITE(_pos_sp_triplet.current.yaw)) {
						dyawdt = wrap_pi(_pos_sp_triplet.current.yaw - _att_sp.yaw_body)/dt;
						dyawdt = constrain_ref(dyawdt,1.2f*_params.nav_rmax);
						_att_sp.yaw_body = wrap_pi(_att_sp.yaw_body + dyawdt*dt);
						_att_sp.yaw_sp_move_rate = _pos_sp_triplet.current.yawspeed;
					}


				} else {
					/* no waypoint, loiter, reset position setpoint if needed */
					if(_v_land_detected.landed == false){
						reset_pos_sp();
						reset_alt_sp();
						reset_pos_sp_loiter();
						reset_alt_sp_loiter();
					}
					else
					{
						R.identity();
						/*
						 * Derive corresponding quarternion from rotation matrix R and
						 * set to attitude setpoint
						 */
						matrix::Quatf quat2;
						quat2.from_dcm(R);
						_att_sp.q_d[0] = quat2(0);
						_att_sp.q_d[1] = quat2(1);
						_att_sp.q_d[2] = quat2(2);
						_att_sp.q_d[3] = quat2(3);
						_att_sp.q_d_valid = true;

//						memcpy(&_att_sp.R_body[0][0], R.data, sizeof(_att_sp.R_body));
						for(int i=0; i<3; i++)
						{
							_att_sp.R_body0[i] = R(0,i);
							_att_sp.R_body1[i] = R(1,i);
							_att_sp.R_body2[i] = R(2,i);
						}
						_att_sp.R_valid = true;

						matrix::Quatf quat_att(_att.q[0], _att.q[1], _att.q[2], _att.q[3]);
						matrix::Vector3f euler_att = matrix::Eulerf(quat_att);

						_att_sp.roll_body = 0.0f;
						_att_sp.pitch_body = 0.0f;
						_att_sp.yaw_body = euler_att(2); //_att.yaw;
						_att_sp.thrust = 0.0f;
						_acc_sp(0) = 0.0f;
						_acc_sp(1) = 0.0f;
						_acc_sp(2) = 0.0f;
						_pos_sp(0) = _pos(0);
						_pos_sp(1) = _pos(1);
						_pos_sp(2) = _pos(2);

						_vel_sp(0) = _vel(0);
						_vel_sp(1) = _vel(1);
						_vel_sp(2) = _vel(2);

						_att_sp.timestamp = hrt_absolute_time();
						thrust_int.zero();
					}
				}
			}

			// limit the offset between the height setpoint and the height measurement
			if(_pos_sp(2) - _pos(2) > _params.sp_offs_max(2))
			{
				_pos_sp(2) = _pos(2) + _params.sp_offs_max(2);
			}
			else if (_pos_sp(2) - _pos(2) < -_params.sp_offs_max(2))
			{
				_pos_sp(2) = _pos(2) - _params.sp_offs_max(2);
			}

			if(_pos_sp(2) < _params.max_z_height)
				_pos_sp(2) = _params.max_z_height;

			/* fill local position setpoint */
			_local_pos_sp.timestamp = hrt_absolute_time();
			_local_pos_sp.x = _pos_sp(0);
			_local_pos_sp.y = _pos_sp(1);
			_local_pos_sp.z = _pos_sp(2);
			_local_pos_sp.yaw = _att_sp.yaw_body;



			/* publish local position setpoint */
			if (_local_pos_sp_pub != nullptr) {
				orb_publish(ORB_ID(vehicle_local_position_setpoint), _local_pos_sp_pub, &_local_pos_sp);

			} else {
				_local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &_local_pos_sp);
			}

			if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid &&
					_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
				/* idle state, don't run controller and set zero thrust */
				R.identity();
				matrix::Quatf quat_att_sp;
				quat_att_sp.from_dcm(R);
				_att_sp.q_d[0] = quat_att_sp(0);
				_att_sp.q_d[1] = quat_att_sp(1);
				_att_sp.q_d[2] = quat_att_sp(2);
				_att_sp.q_d[3] = quat_att_sp(3);
				_att_sp.q_d_valid = true;

//				memcpy(&_att_sp.R_body[0][0], R.data, sizeof(_att_sp.R_body));
				for(int i=0; i<3; i++)
				{
					_att_sp.R_body0[i] = R(0,i);
					_att_sp.R_body1[i] = R(1,i);
					_att_sp.R_body2[i] = R(2,i);
				}
				_att_sp.R_valid = true;

				matrix::Quatf quat_att(_att.q[0], _att.q[1], _att.q[2], _att.q[3]);
				matrix::Vector3f euler_att = matrix::Eulerf(quat_att);

				_att_sp.roll_body = 0.0f;
				_att_sp.pitch_body = 0.0f;
				_att_sp.yaw_body = euler_att(2); //_att.yaw;
				_att_sp.thrust = 0.0f;
				_acc_sp(0) = 0.0f;
				_acc_sp(1) = 0.0f;
				_acc_sp(2) = 0.0f;
				_att_sp.timestamp = hrt_absolute_time();
				thrust_int.zero();

				/* publish attitude setpoint */
				if (_att_sp_pub != nullptr) {
					orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_att_sp);

				} else {
					_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
				}

			} else {

				/* run position & altitude controllers, calculate velocity setpoint */
				matrix::Vector3f pos_err = _pos_sp - _pos;
//				static unsigned int loop1 = 0;
//				if (loop1++ % 20 == 0) {
//				printf("pos_err: %.3f, %.3f, %.3f\n", (double)pos_err(0),(double)pos_err(1),(double)pos_err(2));
//				}

				if (!_control_mode.flag_control_altitude_enabled) {
					_reset_alt_sp = true;
					_vel_sp(2) = 0.0f;
				}

				if (!_control_mode.flag_control_position_enabled) {
					_reset_pos_sp = true;
					_vel_sp(0) = 0.0f;
					_vel_sp(1) = 0.0f;
				}

				if (_control_mode.flag_control_climb_rate_enabled || _control_mode.flag_control_velocity_enabled) {
					/* reset integrals if needed */
					if (_control_mode.flag_control_climb_rate_enabled) {
						if (reset_int_z) {
							reset_int_z = false;
							float i = 0.0f;//_params.thr_min;

							if (reset_int_z_manual) {
									i = _manual.z;

								if (i < 0.0f) {
									i = 0.0f;

								} else if (i > _params.thr_max) {
									i = _params.thr_max;
								}
							}

							thrust_int(2) = -i;
						}

					} else {
						reset_int_z = true;
					}


					if(_control_mode.flag_control_manual_enabled == true) // this part is to pass the manual input to the throttle signal in the semi-auto takeoff process
					{
						if(semi_auto_takeoff_enable)
						{
							thrust_int(2) = -_manual.z * 0.8f;
							if(thrust_int(2) < - _params.max_man_int) //slightly larger than the in air throttle, so that the auto_take_off could trigger the in air mode
								thrust_int(2) = - _params.max_man_int;
						}
					}


					if (_control_mode.flag_control_velocity_enabled) {
						if (reset_int_xy) {
							reset_int_xy = false;
							thrust_int(0) = 0.0f;
							thrust_int(1) = 0.0f;
						}

					} else {
						reset_int_xy = true;
					}

					/* velocity error */
					matrix::Vector3f vel_err = _vel_sp - _vel;
//					static unsigned int loop1 = 0;
//					if (loop1++ % 20 == 0) {
//					printf("vel_err is %.3f, %.3f, %.3f\n", (double)vel_err(0),(double)vel_err(1),(double)vel_err(2));
//					}

					float acc_dot_temp;
					acc_dot_temp = (_acc_sp(0) - _acc_sp_govened(0))/dt;
					acc_dot_temp = constrain_ref(acc_dot_temp,_params.nav_max_jerk_xy*1.2f);
					_acc_sp_govened(0) = _acc_sp_govened(0) + acc_dot_temp * dt;

					acc_dot_temp = (_acc_sp(1) - _acc_sp_govened(1))/dt;
					acc_dot_temp = constrain_ref(acc_dot_temp,_params.nav_max_jerk_xy*1.2f);
					_acc_sp_govened(1) = _acc_sp_govened(1) + acc_dot_temp * dt;

					acc_dot_temp = (_acc_sp(2) - _acc_sp_govened(2))/dt;
					acc_dot_temp = constrain_ref(acc_dot_temp,_params.nav_max_jerk_xy*1.2f);
					_acc_sp_govened(2) = _acc_sp_govened(2) + acc_dot_temp * dt;


//					printf("acc x is %8.4lf,acc y is %8.4lf, acc z is %8.4lf\n",(double)_acc_sp_govened(0),(double)_acc_sp_govened(1),(double)_acc_sp_govened(2));

					//Mingjie modified to log thrust_int
//					_global_vel_sp.vx = thrust_int(2);//)_vel_sp(0);
//					//printf("thrust int: %8.4f\n", (double)thrust_int(2));
//					_global_vel_sp.vy = vel_err(2);//_vel_sp(1);
//					_global_vel_sp.vz = pos_err(2);//_vel_sp(2);
//					_global_vel_sp.accx = _acc_sp_govened(2);//_acc_sp_govened(0);
//					_global_vel_sp.accy = 0;//_acc_sp_govened(1);
//					_global_vel_sp.accz = 0;//_acc_sp_govened(2);
//
//					/* publish velocity setpoint */
//					if (_global_vel_sp_pub > 0) {
//						orb_publish(ORB_ID(vehicle_global_velocity_setpoint), _global_vel_sp_pub, &_global_vel_sp);
//                        //printf("publishing global\n");
//					} else {
//						_global_vel_sp_pub = orb_advertise(ORB_ID(vehicle_global_velocity_setpoint), &_global_vel_sp);
//					}

					/* limit thrust vector and check for saturation */
					bool saturation_xy = false;
					bool saturation_z = false;

					/* thrust vector in NED frame */
					matrix::Vector3f thrust_sp;

					thrust_sp(0) = vel_err(0)*_params.vel_p(0)  + thrust_int(0) + pos_err(0)*_params.pos_p(0) + _acc_sp_govened(0)/20.0f; // acceleration reference add to achieve better control performance
					thrust_sp(1) = vel_err(1)*_params.vel_p(1)  + thrust_int(1) + pos_err(1)*_params.pos_p(1) + _acc_sp_govened(1)/20.0f; // RPT control structure
					thrust_sp(2) = vel_err(2)*_params.vel_p(2)  + thrust_int(2) + pos_err(2)*_params.pos_p(2) + _acc_sp_govened(2)/20.0f; // height control is different from the remaining


					if (!_control_mode.flag_control_velocity_enabled) {
						thrust_sp(0) = 0.0f;
						thrust_sp(1) = 0.0f;
					}

					if (!_control_mode.flag_control_climb_rate_enabled) {
						thrust_sp(2) = 0.0f;
					}


					/* limit min lift */
					float thr_min = _params.thr_min;
					float tilt_max = _params.tilt_max_air;

					/* adjust limits for landing mode */
					if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid
							&& _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
						/* limit max tilt and min lift when landing */
						tilt_max = _params.tilt_max_land;

						if (thr_min < 0.0f) {
							thr_min = 0.0f;
						}
					}

					/* limit min lift */
					if(_v_land_detected.landed == false) {
						if(!((_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND
								&& !_control_mode.flag_control_manual_enabled)||(_control_mode.flag_control_manual_enabled&&_manual.z<0.3f)))
							if (-thrust_sp(2) < thr_min) {
								thrust_sp(2) = -thr_min;
								saturation_z = true;
							}
					}

					if(-thrust_sp(2) < 0.0f)
						thrust_sp(2) = 0.0f;

					if (_control_mode.flag_control_velocity_enabled) {
						/* limit max tilt */
						if (thr_min >= 0.0f && tilt_max < M_PI_F / 2 - 0.05f) {
							/* absolute horizontal thrust */
							float thrust_sp_xy_len = matrix::Vector2f(thrust_sp(0), thrust_sp(1)).length();

							if (thrust_sp_xy_len > 0.01f) {
								/* max horizontal thrust for given vertical thrust*/
								float thrust_xy_max = 0.5f* tanf(tilt_max);

								if (thrust_sp_xy_len > thrust_xy_max) {
									float k = thrust_xy_max / thrust_sp_xy_len;
									thrust_sp(0) *= k;
									thrust_sp(1) *= k;
									saturation_xy = true;
								}
							}
						}

					}


					/* limit max thrust */
					float thrust_abs = thrust_sp.length();

					if (thrust_abs > _params.thr_max) {
						thrust_abs = _params.thr_max;
					}


					/* update integrals */
					if (_control_mode.flag_control_velocity_enabled && !saturation_xy) {
						thrust_int(0) += pos_err(0) * _params.pos_i(0) * dt;
						thrust_int(1) += pos_err(1) * _params.pos_i(1) * dt;
					}

					if (_control_mode.flag_control_climb_rate_enabled && !saturation_z) {
						thrust_int(2) += pos_err(2) * _params.pos_i(2) * dt;
						//printf("pos_error: %8.4f thrust_int: %8.4f\n", (double)pos_err(2), (double)thrust_int(2));
						/* protection against flipping on ground when landing */
						if (thrust_int(2) > 0.0f) {
							thrust_int(2) = 0.0f;
						}
					}

					/* calculate attitude setpoint from thrust vector */
					if (_control_mode.flag_control_velocity_enabled) {
						/* desired body_z axis = -normalize(thrust_vector) */
						matrix::Vector3f body_x;
						matrix::Vector3f body_y;
						matrix::Vector3f body_z;
						matrix::Vector3f thrust_sp_fixed;
						thrust_sp_fixed(0) = thrust_sp(0);
						thrust_sp_fixed(1) = thrust_sp(1);
						thrust_sp_fixed(2) = -0.5f;
						float thrust_abs_fixed = thrust_sp_fixed.length();


						if (-thrust_sp(2) > SIGMA) {
							body_z = -thrust_sp_fixed / thrust_abs_fixed;

						} else {
							/* no thrust, set Z axis to safe value */
							body_z.zero();
							body_z(2) = 1.0f;
						}

						/* vector of desired yaw direction in XY plane, rotated by PI/2 */
						matrix::Vector3f y_C(-sinf(_att_sp.yaw_body), cosf(_att_sp.yaw_body), 0.0f);

						if (fabsf(body_z(2)) > SIGMA) {
							/* desired body_x axis, orthogonal to body_z */
							body_x = y_C % body_z;

							/* keep nose to front while inverted upside down */
							if (body_z(2) < 0.0f) {
								body_x = -body_x;
							}

							body_x.normalize();

						} else {
							/* desired thrust is in XY plane, set X downside to construct correct matrix,
							 * but yaw component will not be used actually */
							body_x.zero();
							body_x(2) = 1.0f;
						}

						/* desired body_y axis */
						body_y = body_z % body_x;

						/* fill rotation matrix */
						for (int i = 0; i < 3; i++) {
							R(i, 0) = body_x(i);
							R(i, 1) = body_y(i);
							R(i, 2) = body_z(i);
						}

						/* copy rotation matrix to attitude setpoint topic */
						matrix::Quatf quat_sp;
						quat_sp.from_dcm(R);
						_att_sp.q_d[0] = quat_sp(0);
						_att_sp.q_d[1] = quat_sp(1);
						_att_sp.q_d[2] = quat_sp(2);
						_att_sp.q_d[3] = quat_sp(3);
						_att_sp.q_d_valid = true;

						//memcpy(&_att_sp.R_body[0][0], R.data, sizeof(_att_sp.R_body));
						for(int i=0; i<3; i++)
						{
							_att_sp.R_body0[i] = R(0,i);
							_att_sp.R_body1[i] = R(1,i);
							_att_sp.R_body2[i] = R(2,i);
						}

						_att_sp.R_valid = true;

						/* calculate euler angles, for logging only, must not be used for control */
						matrix::Vector3f euler = matrix::Eulerf(R);
						_att_sp.roll_body = euler(0);
						_att_sp.pitch_body = euler(1);
						/* yaw already used to construct rot matrix, but actual rotation matrix can have different yaw near singularity */

					} else if (!_control_mode.flag_control_manual_enabled) {
						/* autonomous altitude control without position control (failsafe landing),
						 * force level attitude, don't change yaw */
						matrix::Quatf q_att(_att.q[0], _att.q[1], _att.q[2], _att.q[3]);
						matrix::Vector3f euler_att = matrix::Eulerf(q_att);
						R = matrix::Eulerf(0.0f, 0.0f, euler_att(2));

						/* copy rotation matrix to attitude setpoint topic */
						matrix::Quatf q_att_sp;
						q_att_sp.from_dcm(R);
						_att_sp.q_d[0] = q_att_sp(0);
						_att_sp.q_d[1] = q_att_sp(1);
						_att_sp.q_d[2] = q_att_sp(2);
						_att_sp.q_d[3] = q_att_sp(3);
						_att_sp.q_d_valid = true;

						//memcpy(&_att_sp.R_body[0][0], R.data, sizeof(_att_sp.R_body));
						for(int i=0; i<3; i++)
						{
							_att_sp.R_body0[i] = R(0,i);
							_att_sp.R_body1[i] = R(1,i);
							_att_sp.R_body2[i] = R(2,i);
						}
						_att_sp.R_valid = true;

						_att_sp.roll_body = 0.0f;
						_att_sp.pitch_body = 0.0f;
					}

                    if(_manual.z<0.05f && _control_mode.flag_control_manual_enabled
                    		&&_v_status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL)
                    	{
                    		_att_sp.thrust = 0.0f;
                    		reset_int_z = true;
                    	}
                    else
                    	_att_sp.thrust = thrust_abs;

					_att_sp.timestamp = hrt_absolute_time();

					/* publish attitude setpoint */
					if (_att_sp_pub != nullptr) {
						orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_att_sp);

					} else {
						_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
					}

				} else {
					reset_int_z = true;
				}
			}

		}

		if( !_control_mode.flag_control_position_enabled && !_control_mode.flag_control_velocity_enabled)
		{
			/* position controller disabled, reset setpoints */
			_reset_pos_sp = true;
            _reset_pos_loiter_sp = true;
			reset_int_xy = true;
			semi_auto_pos_control = false;
		}

		if (_local_pos_sp_pub != nullptr) {
			orb_publish(ORB_ID(vehicle_local_position_setpoint), _local_pos_sp_pub, &_local_pos_sp);

		} else {
			_local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &_local_pos_sp);
		}

		if(!_control_mode.flag_control_climb_rate_enabled)
		{
			semi_auto_takeoff_enable = false;
		}

		_reset_alt_sp = _control_mode.flag_armed && _control_mode.flag_control_manual_enabled && !_control_mode.flag_control_climb_rate_enabled;
		reset_int_z = _control_mode.flag_armed && _control_mode.flag_control_manual_enabled && !_control_mode.flag_control_climb_rate_enabled;
		/* reset altitude controller integral (hovering throttle) to manual throttle after manual throttle control */
		reset_int_z_manual = _control_mode.flag_armed && _control_mode.flag_control_manual_enabled && !_control_mode.flag_control_climb_rate_enabled;

	}

	PX4_WARN("stopped");
	mavlink_log_info(&_mavlink_log_pub, "[mpc] stopped");

	_control_task = -1;
	_exit(0);
}

int
MulticopterPositionControlRPT::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("mc_pos_control_rpt",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2000,
				       (px4_main_t)&MulticopterPositionControlRPT::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		PX4_WARN("task start failed");
		return -errno;
	}

	return OK;
}

int mc_pos_control_rpt_main(int argc, char *argv[])
{
	if (argc < 1) {
		errx(1, "usage: mc_pos_control_rpt_rpt {start|stop|status}");
	}

	if (!strcmp(argv[1], "start")) {

		if (pos_control_rpt::g_control != nullptr) {
			warnx("already running");
			return 1;		}

		pos_control_rpt::g_control = new MulticopterPositionControlRPT;

		if (pos_control_rpt::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != pos_control_rpt::g_control->start()) {
			delete pos_control_rpt::g_control;
			pos_control_rpt::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (pos_control_rpt::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete pos_control_rpt::g_control;
		pos_control_rpt::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (pos_control_rpt::g_control) {
			warnx("running");
			return 0;
		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
