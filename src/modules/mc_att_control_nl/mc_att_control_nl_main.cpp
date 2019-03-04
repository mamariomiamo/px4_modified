/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 *
 * Publication for the desired attitude tracking:
 * Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
 * Int. Conf. on Robotics and Automation, Shanghai, China, May 2011.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 *
 * The controller has two loops: P loop for angular error and PD loop for angular rate error.
 * Desired rotation calculated keeping in mind that yaw response is normally slower than roll/pitch.
 * For small deviations controller rotates copter to have shortest path of thrust vector and independently rotates around yaw,
 * so actual rotation axis is not constant. For large deviations controller rotates copter around fixed axis.
 * These two approaches fused seamlessly with weight depending on angular error.
 * When thrust vector directed near-horizontally (e.g. roll ~= PI/2) yaw setpoint ignored because of singularity.
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 * If rotation matrix setpoint is invalid it will be generated from Euler angles for compatibility with old position controllers.
 */

#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/tailsitter_recovery/tailsitter_recovery.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <systemlib/circuit_breaker.h>
#include <systemlib/err.h>
#include <lib/mixer/mixer.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/mc_att_ctrl_status.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/path_scheduler_output.h>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_innerloop_cmd.h>
#include <uORB/topics/vehicle_force_moment_cmd.h>
#include <uORB/uORB.h>

#include "lut.h"
/**
 * Multicopter attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_att_control_nl_main(int argc, char *argv[]);

//extern float lut_f_m_rpm_va_thetab;

#define YAW_DEADZONE	0.05f
#define MIN_TAKEOFF_THRUST    0.2f
#define TPA_RATE_LOWER_LIMIT 0.05f
#define ATTITUDE_TC_DEFAULT 0.2f

#define AXIS_INDEX_ROLL 0
#define AXIS_INDEX_PITCH 1
#define AXIS_INDEX_YAW 2
#define AXIS_COUNT 3

#define MAX_GYRO_COUNT 3

class MulticopterAttitudeControlNL
{
public:
	/**
	 * Constructor
	 */
	MulticopterAttitudeControlNL();

	/**
	 * Destructor, also kills the main task
	 */
	~MulticopterAttitudeControlNL();

	/**
	 * Start the multicopter attitude control task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool	_task_should_exit;		/**< if true, task_main() should exit */
	int		_control_task;			/**< task handle */

	int		_v_att_sub;		/**< vehicle attitude subscription */
	int		_v_att_sp_sub;			/**< vehicle attitude setpoint subscription */
	int		_v_rates_sp_sub;		/**< vehicle rates setpoint subscription */
	int		_v_control_mode_sub;	/**< vehicle control mode subscription */
	int		_v_innerloop_cmd_sub;	/**< vehicle innerloop cmd subscription */
	int		_params_sub;			/**< parameter updates subscription */
	int		_path_sched_output_sub;	/**< path scheduler output subscription */
	int		_manual_control_sp_sub;	/**< manual control setpoint subscription */
	int		_vehicle_status_sub;	/**< vehicle status subscription */
	int		_motor_limits_sub;		/**< motor limits subscription */
	int		_battery_status_sub;	/**< battery status subscription */
	int		_sensor_gyro_sub[MAX_GYRO_COUNT];	/**< gyro data subscription */
	int		_sensor_correction_sub;	/**< sensor thermal correction subscription */
	int		_sensor_bias_sub;	/**< sensor in-run bias correction subscription */

	unsigned _gyro_count;
	int _selected_gyro;

	orb_advert_t	_v_fm_cmd_pub;			/**< force moment command publication */
	orb_advert_t	_v_rates_sp_pub;		/**< rate setpoint publication */
	orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */
	orb_advert_t	_controller_status_pub;	/**< controller status publication */

	orb_id_t _rates_sp_id;	/**< pointer to correct rates setpoint uORB metadata structure */
	orb_id_t _actuators_id;	/**< pointer to correct actuator controls0 uORB metadata structure */

	bool		_actuators_0_circuit_breaker_enabled;	/**< circuit breaker to suppress output */

	struct vehicle_attitude_s		_v_att;			/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s	_v_att_sp;		/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s		_v_rates_sp;		/**< vehicle rates setpoint */
	struct vehicle_innerloop_cmd_s		_v_innerloop_cmd;	/**< vehicle innterloop cmd */
	struct vehicle_force_moment_cmd_s	_v_fm_cmd;			/**< vehicle force moment cmd */
	struct manual_control_setpoint_s	_manual_control_sp;	/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode;	/**< vehicle control mode */
	struct actuator_controls_s		_actuators;		/**< actuator controls */
	struct vehicle_status_s			_vehicle_status;	/**< vehicle status */
	struct mc_att_ctrl_status_s 		_controller_status;	/**< controller status */
	struct battery_status_s			_battery_status;	/**< battery status */
	struct sensor_gyro_s			_sensor_gyro;		/**< gyro data before thermal correctons and ekf bias estimates are applied */
	struct sensor_correction_s		_sensor_correction;	/**< sensor thermal corrections */
	struct sensor_bias_s			_sensor_bias;		/**< sensor in-run bias corrections */
	struct path_scheduler_output_s	_path_sched_output;	/**< path scheduler ouptput */

	MultirotorMixer::saturation_status _saturation_status{};

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_controller_latency_perf;

	math::Vector<3>		_rates_prev;	/**< angular rates on previous step */
	math::Vector<3>		_rates_sp_prev; /**< previous rates setpoint */
	math::Vector<3>		_rates_sp;		/**< angular rates setpoint */
	math::Vector<3>		_rates_int;		/**< angular rates integral error */
	float				_thrust_sp;		/**< thrust setpoint */
	math::Vector<3>		_att_control;	/**< attitude control vector */

	math::Vector<3>	 	_eulers;
	math::Vector<3> 	_omegab;		/**< angular rate in body frame */
	math::Vector<3>		_dfcmd;			/**< 1st derivative of [phic, thetac, psic] */
	math::Vector<3>		_ddfcmd;		/**< 2nd derivative of [phic, thetac, psic] */
	float			 	_Omega[3][3];
	float 				_FC_Jb[3][3];
	math::Vector<4> 	_Fm;			/**< current force and moment w.r.t the look-up table */
	math::Matrix<4,4> 	_pFmpn;
	math::Matrix<3,3>	_OmegaE;
	math::Vector<3> 	_Fa, _Ma, _Fb, _Mb;	/**< individual force and moment in properller frame and body frame */
	math::Vector<3> 	_Fbt, _Mbt;		/**< total force and moment in body frame */
	math::Vector<4>		_delta;			/**< control input for each channel */
	math::Vector<3> 	_Mbc;			/**< moment command generated in attitude control */
	math::Matrix<3, 3>  _I;				/**< identity matrix */
	math::Vector<3> 	_Vba;			/**< wind velocity: Va, beta, alpha */
	math::Vector<4> 	_omegar0;		/**< Spinning rate of each motor */
	math::Vector<4> 	_rpm0;			/**< RPM value of each motor */
	math::Vector<4> 	_delta0;		/**< control input of each motor */
	math::Vector<6> 	_fm;
	math::Vector<6>		_pfmpn;
	float				_anzc;			/**< z-axis load acceleration in bodya frame */

	math::Matrix<3, 3>	_board_rotation = {};	/**< rotation matrix for the orientation that the board is mounted */

	struct {
		param_t roll_p;
		param_t roll_rate_p;
		param_t roll_rate_i;
		param_t roll_rate_integ_lim;
		param_t roll_rate_d;
		param_t roll_rate_ff;
		param_t pitch_p;
		param_t pitch_rate_p;
		param_t pitch_rate_i;
		param_t pitch_rate_integ_lim;
		param_t pitch_rate_d;
		param_t pitch_rate_ff;
		param_t tpa_breakpoint_p;
		param_t tpa_breakpoint_i;
		param_t tpa_breakpoint_d;
		param_t tpa_rate_p;
		param_t tpa_rate_i;
		param_t tpa_rate_d;
		param_t yaw_p;
		param_t yaw_rate_p;
		param_t yaw_rate_i;
		param_t yaw_rate_integ_lim;
		param_t yaw_rate_d;
		param_t yaw_rate_ff;
		param_t yaw_ff;
		param_t roll_rate_max;
		param_t pitch_rate_max;
		param_t yaw_rate_max;
		param_t yaw_auto_max;

		param_t acro_roll_max;
		param_t acro_pitch_max;
		param_t acro_yaw_max;
		param_t acro_expo;
		param_t acro_superexpo;
		param_t rattitude_thres;

		param_t roll_tc;
		param_t pitch_tc;

		param_t vtol_type;
		param_t vtol_opt_recovery_enabled;
		param_t vtol_wv_yaw_rate_scale;

		param_t bat_scale_en;

		param_t board_rotation;

		param_t board_offset[3];

	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		math::Vector<3> att_p;					/**< P gain for angular error */
		math::Vector<3> rate_p;				/**< P gain for angular rate error */
		math::Vector<3> rate_i;				/**< I gain for angular rate error */
		math::Vector<3> rate_int_lim;			/**< integrator state limit for rate loop */
		math::Vector<3> rate_d;				/**< D gain for angular rate error */
		math::Vector<3>	rate_ff;			/**< Feedforward gain for desired rates */
		float yaw_ff;						/**< yaw control feed-forward */

		float tpa_breakpoint_p;				/**< Throttle PID Attenuation breakpoint */
		float tpa_breakpoint_i;				/**< Throttle PID Attenuation breakpoint */
		float tpa_breakpoint_d;				/**< Throttle PID Attenuation breakpoint */
		float tpa_rate_p;					/**< Throttle PID Attenuation slope */
		float tpa_rate_i;					/**< Throttle PID Attenuation slope */
		float tpa_rate_d;					/**< Throttle PID Attenuation slope */

		float roll_rate_max;
		float pitch_rate_max;
		float yaw_rate_max;
		float yaw_auto_max;
		math::Vector<3> mc_rate_max;		/**< attitude rate limits in stabilized modes */
		math::Vector<3> auto_rate_max;		/**< attitude rate limits in auto modes */
		matrix::Vector3f acro_rate_max;		/**< max attitude rates in acro mode */
		float acro_expo;					/**< function parameter for expo stick curve shape */
		float acro_superexpo;				/**< function parameter for superexpo stick curve shape */
		float rattitude_thres;

		int32_t vtol_type;					/**< 0 = Tailsitter, 1 = Tiltrotor, 2 = Standard airframe */
		bool vtol_opt_recovery_enabled;
		float vtol_wv_yaw_rate_scale;			/**< Scale value [0, 1] for yaw rate setpoint  */

		int32_t bat_scale_en;

		int32_t board_rotation;

		float board_offset[3];

	}		_params;

	TailsitterRecovery *_ts_opt_recovery{nullptr};	/**< Computes optimal rates for tailsitter recovery */

	FMLut *_fmLut;
	/**
	 * Update our local parameter cache.
	 */
	void			parameters_update();

	/**
	 * Check for parameter update and handle it.
	 */
	void		battery_status_poll();
	void		parameter_update_poll();
	void 		path_scheduler_output_poll();
	void		sensor_bias_poll();
	void		sensor_correction_poll();
	void		vehicle_attitude_poll();
	void		vehicle_attitude_setpoint_poll();
	void		vehicle_control_mode_poll();
	void		vehicle_manual_poll();
	void		vehicle_motor_limits_poll();
	void		vehicle_rates_setpoint_poll();
	void		vehicle_status_poll();
	void		vehicle_innerloop_cmd_poll();

	/**
	 * Attitude controller.
	 */
	void		control_attitude(float dt);

	/**
	 * Attitude rates controller.
	 */
	void		control_attitude_rates(float dt);

	/**
	 * Attitude law
	 */
	void		attitude_law();

	/**
	 * Force and Moment allocator
	 */
	void		force_moment_allocator();

	/**
	 * Nonlinear control law simple test
	 */
	void simple_test();

	/**
	 * Throttle PID attenuation.
	 */
	math::Vector<3> pid_attenuations(float tpa_breakpoint, float tpa_rate);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task.
	 */
	void		task_main();
};

namespace mc_att_control_nl
{

MulticopterAttitudeControlNL	*g_control;
}

MulticopterAttitudeControlNL::MulticopterAttitudeControlNL() :

	_task_should_exit(false),
	_control_task(-1),

	/* subscriptions */
	_v_att_sub(-1),
	_v_att_sp_sub(-1),
	_v_control_mode_sub(-1),
	_v_innerloop_cmd_sub(-1),
	_params_sub(-1),
	_path_sched_output_sub(-1),
	_manual_control_sp_sub(-1),
	_vehicle_status_sub(-1),
	_motor_limits_sub(-1),
	_battery_status_sub(-1),
	_sensor_correction_sub(-1),
	_sensor_bias_sub(-1),

	/* gyro selection */
	_gyro_count(1),
	_selected_gyro(0),

	/* publications */
	_v_fm_cmd_pub(nullptr),
	_v_rates_sp_pub(nullptr),
	_actuators_0_pub(nullptr),
	_controller_status_pub(nullptr),
	_rates_sp_id(nullptr),
	_actuators_id(nullptr),

	_actuators_0_circuit_breaker_enabled(false),

	_v_att{},
	_v_att_sp{},
	_v_rates_sp{},
	_v_innerloop_cmd{},
	_v_fm_cmd{},
	_manual_control_sp{},
	_v_control_mode{},
	_actuators{},
	_vehicle_status{},
	_controller_status{},
	_battery_status{},
	_sensor_gyro{},
	_sensor_correction{},
	_sensor_bias{},
	_path_sched_output{},
	_saturation_status{},
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "mc_att_control_nl")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency"))
{
	for (uint8_t i = 0; i < MAX_GYRO_COUNT; i++) {
		_sensor_gyro_sub[i] = -1;
	}

	_vehicle_status.is_rotary_wing = true;

	_params.att_p.zero();
	_params.rate_p.zero();
	_params.rate_i.zero();
	_params.rate_int_lim.zero();
	_params.rate_d.zero();
	_params.rate_ff.zero();
	_params.yaw_ff = 0.0f;
	_params.roll_rate_max = 0.0f;
	_params.pitch_rate_max = 0.0f;
	_params.yaw_rate_max = 0.0f;
	_params.mc_rate_max.zero();
	_params.auto_rate_max.zero();
	_params.acro_rate_max.zero();
	_params.rattitude_thres = 1.0f;

	_params.bat_scale_en = 0;

	_params.board_rotation = 0;

	_params.board_offset[0] = 0.0f;
	_params.board_offset[1] = 0.0f;
	_params.board_offset[2] = 0.0f;

	_rates_prev.zero();
	_rates_sp.zero();
	_rates_sp_prev.zero();
	_rates_int.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();
	_delta.zero();

	_I.identity();
	_board_rotation.identity();

	_Vba.zero();
	_omegar0.zero();
	_rpm0.zero();
	_delta0.zero();
	_Fbt.zero();
	_Mbt.zero();
	_FC_Jb[0][0] = 0.288;
	_FC_Jb[1][1] = 0.288;
	_FC_Jb[2][2] = 0.5184;

	_params_handles.roll_p			= 	param_find("MC_ROLL_P");
	_params_handles.roll_rate_p		= 	param_find("MC_ROLLRATE_P");
	_params_handles.roll_rate_i		= 	param_find("MC_ROLLRATE_I");
	_params_handles.roll_rate_integ_lim	= 	param_find("MC_RR_INT_LIM");
	_params_handles.roll_rate_d		= 	param_find("MC_ROLLRATE_D");
	_params_handles.roll_rate_ff		= 	param_find("MC_ROLLRATE_FF");

	_params_handles.pitch_p			= 	param_find("MC_PITCH_P");
	_params_handles.pitch_rate_p		= 	param_find("MC_PITCHRATE_P");
	_params_handles.pitch_rate_i		= 	param_find("MC_PITCHRATE_I");
	_params_handles.pitch_rate_integ_lim	= 	param_find("MC_PR_INT_LIM");
	_params_handles.pitch_rate_d		= 	param_find("MC_PITCHRATE_D");
	_params_handles.pitch_rate_ff 		= 	param_find("MC_PITCHRATE_FF");

	_params_handles.tpa_breakpoint_p 	= 	param_find("MC_TPA_BREAK_P");
	_params_handles.tpa_breakpoint_i 	= 	param_find("MC_TPA_BREAK_I");
	_params_handles.tpa_breakpoint_d 	= 	param_find("MC_TPA_BREAK_D");
	_params_handles.tpa_rate_p	 	= 	param_find("MC_TPA_RATE_P");
	_params_handles.tpa_rate_i	 	= 	param_find("MC_TPA_RATE_I");
	_params_handles.tpa_rate_d	 	= 	param_find("MC_TPA_RATE_D");

	_params_handles.yaw_p			=	param_find("MC_YAW_P");
	_params_handles.yaw_rate_p		= 	param_find("MC_YAWRATE_P");
	_params_handles.yaw_rate_i		= 	param_find("MC_YAWRATE_I");
	_params_handles.yaw_rate_integ_lim	= 	param_find("MC_YR_INT_LIM");
	_params_handles.yaw_rate_d		= 	param_find("MC_YAWRATE_D");
	_params_handles.yaw_rate_ff	 	= 	param_find("MC_YAWRATE_FF");
	_params_handles.yaw_ff			= 	param_find("MC_YAW_FF");

	_params_handles.roll_rate_max		= 	param_find("MC_ROLLRATE_MAX");
	_params_handles.pitch_rate_max		= 	param_find("MC_PITCHRATE_MAX");
	_params_handles.yaw_rate_max		= 	param_find("MC_YAWRATE_MAX");
	_params_handles.yaw_auto_max		= 	param_find("MC_YAWRAUTO_MAX");

	_params_handles.acro_roll_max		= 	param_find("MC_ACRO_R_MAX");
	_params_handles.acro_pitch_max		= 	param_find("MC_ACRO_P_MAX");
	_params_handles.acro_yaw_max		= 	param_find("MC_ACRO_Y_MAX");
	_params_handles.acro_expo		= 	param_find("MC_ACRO_EXPO");
	_params_handles.acro_superexpo		= 	param_find("MC_ACRO_SUPEXPO");

	_params_handles.rattitude_thres 	= 	param_find("MC_RATT_TH");

	_params_handles.roll_tc			= 	param_find("MC_ROLL_TC");
	_params_handles.pitch_tc		= 	param_find("MC_PITCH_TC");

	_params_handles.bat_scale_en		=	param_find("MC_BAT_SCALE_EN");

	/* rotations */
	_params_handles.board_rotation		=	param_find("SENS_BOARD_ROT");

	/* rotation offsets */
	_params_handles.board_offset[0]		=	param_find("SENS_BOARD_X_OFF");
	_params_handles.board_offset[1]		=	param_find("SENS_BOARD_Y_OFF");
	_params_handles.board_offset[2]		=	param_find("SENS_BOARD_Z_OFF");

	/* fetch initial parameter values */
	parameters_update();

	/* initialize thermal corrections as we might not immediately get a topic update (only non-zero values) */
	for (unsigned i = 0; i < 3; i++) {
		// used scale factors to unity
		_sensor_correction.gyro_scale_0[i] = 1.0f;
		_sensor_correction.gyro_scale_1[i] = 1.0f;
		_sensor_correction.gyro_scale_2[i] = 1.0f;
	}
}

MulticopterAttitudeControlNL::~MulticopterAttitudeControlNL()
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

	if (_ts_opt_recovery != nullptr) {
		delete _ts_opt_recovery;
	}

	mc_att_control_nl::g_control = nullptr;
}

void
MulticopterAttitudeControlNL::parameters_update()
{
	float v;

	float roll_tc, pitch_tc;

	param_get(_params_handles.roll_tc, &roll_tc);
	param_get(_params_handles.pitch_tc, &pitch_tc);

	/* roll gains */
	param_get(_params_handles.roll_p, &v);
	_params.att_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_rate_p, &v);
	_params.rate_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_rate_i, &v);
	_params.rate_i(0) = v;
	param_get(_params_handles.roll_rate_integ_lim, &v);
	_params.rate_int_lim(0) = v;
	param_get(_params_handles.roll_rate_d, &v);
	_params.rate_d(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_rate_ff, &v);
	_params.rate_ff(0) = v;

	/* pitch gains */
	param_get(_params_handles.pitch_p, &v);
	_params.att_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_p, &v);
	_params.rate_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_i, &v);
	_params.rate_i(1) = v;
	param_get(_params_handles.pitch_rate_integ_lim, &v);
	_params.rate_int_lim(1) = v;
	param_get(_params_handles.pitch_rate_d, &v);
	_params.rate_d(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_ff, &v);
	_params.rate_ff(1) = v;

	param_get(_params_handles.tpa_breakpoint_p, &_params.tpa_breakpoint_p);
	param_get(_params_handles.tpa_breakpoint_i, &_params.tpa_breakpoint_i);
	param_get(_params_handles.tpa_breakpoint_d, &_params.tpa_breakpoint_d);
	param_get(_params_handles.tpa_rate_p, &_params.tpa_rate_p);
	param_get(_params_handles.tpa_rate_i, &_params.tpa_rate_i);
	param_get(_params_handles.tpa_rate_d, &_params.tpa_rate_d);

	/* yaw gains */
	param_get(_params_handles.yaw_p, &v);
	_params.att_p(2) = v;
	param_get(_params_handles.yaw_rate_p, &v);
	_params.rate_p(2) = v;
	param_get(_params_handles.yaw_rate_i, &v);
	_params.rate_i(2) = v;
	param_get(_params_handles.yaw_rate_integ_lim, &v);
	_params.rate_int_lim(2) = v;
	param_get(_params_handles.yaw_rate_d, &v);
	_params.rate_d(2) = v;
	param_get(_params_handles.yaw_rate_ff, &v);
	_params.rate_ff(2) = v;

	param_get(_params_handles.yaw_ff, &_params.yaw_ff);

	/* angular rate limits */
	param_get(_params_handles.roll_rate_max, &_params.roll_rate_max);
	_params.mc_rate_max(0) = math::radians(_params.roll_rate_max);
	param_get(_params_handles.pitch_rate_max, &_params.pitch_rate_max);
	_params.mc_rate_max(1) = math::radians(_params.pitch_rate_max);
	param_get(_params_handles.yaw_rate_max, &_params.yaw_rate_max);
	_params.mc_rate_max(2) = math::radians(_params.yaw_rate_max);

	/* auto angular rate limits */
	param_get(_params_handles.roll_rate_max, &_params.roll_rate_max);
	_params.auto_rate_max(0) = math::radians(_params.roll_rate_max);
	param_get(_params_handles.pitch_rate_max, &_params.pitch_rate_max);
	_params.auto_rate_max(1) = math::radians(_params.pitch_rate_max);
	param_get(_params_handles.yaw_auto_max, &_params.yaw_auto_max);
	_params.auto_rate_max(2) = math::radians(_params.yaw_auto_max);

	/* manual rate control acro mode rate limits and expo */
	param_get(_params_handles.acro_roll_max, &v);
	_params.acro_rate_max(0) = math::radians(v);
	param_get(_params_handles.acro_pitch_max, &v);
	_params.acro_rate_max(1) = math::radians(v);
	param_get(_params_handles.acro_yaw_max, &v);
	_params.acro_rate_max(2) = math::radians(v);
	param_get(_params_handles.acro_expo, &_params.acro_expo);
	param_get(_params_handles.acro_superexpo, &_params.acro_superexpo);

	/* stick deflection needed in rattitude mode to control rates not angles */
	param_get(_params_handles.rattitude_thres, &_params.rattitude_thres);

	if (_vehicle_status.is_vtol) {
		param_get(_params_handles.vtol_type, &_params.vtol_type);

		int32_t tmp;
		param_get(_params_handles.vtol_opt_recovery_enabled, &tmp);
		_params.vtol_opt_recovery_enabled = (tmp == 1);

		param_get(_params_handles.vtol_wv_yaw_rate_scale, &_params.vtol_wv_yaw_rate_scale);

	} else {
		_params.vtol_opt_recovery_enabled = false;
		_params.vtol_wv_yaw_rate_scale = 0.f;
	}

	param_get(_params_handles.bat_scale_en, &_params.bat_scale_en);

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

	/* rotation of the autopilot relative to the body */
	param_get(_params_handles.board_rotation, &(_params.board_rotation));

	/* fine adjustment of the rotation */
	param_get(_params_handles.board_offset[0], &(_params.board_offset[0]));
	param_get(_params_handles.board_offset[1], &(_params.board_offset[1]));
	param_get(_params_handles.board_offset[2], &(_params.board_offset[2]));

	/* get transformation matrix from sensor/board to body frame */
	get_rot_matrix((enum Rotation)_params.board_rotation, &_board_rotation);

	/* fine tune the rotation */
	math::Matrix<3, 3> board_rotation_offset;
	board_rotation_offset.from_euler(M_DEG_TO_RAD_F * _params.board_offset[0],
					 M_DEG_TO_RAD_F * _params.board_offset[1],
					 M_DEG_TO_RAD_F * _params.board_offset[2]);
	_board_rotation = board_rotation_offset * _board_rotation;
}

void
MulticopterAttitudeControlNL::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}

void MulticopterAttitudeControlNL::path_scheduler_output_poll()
{
	bool updated;

	orb_check(_path_sched_output_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(path_scheduler_output), _path_sched_output_sub, &_path_sched_output);
	}
}

void
MulticopterAttitudeControlNL::vehicle_control_mode_poll()
{
	bool updated;

	/* Check if vehicle control mode has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

void
MulticopterAttitudeControlNL::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

void
MulticopterAttitudeControlNL::vehicle_attitude_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
	}
}

void
MulticopterAttitudeControlNL::vehicle_rates_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	}
}

void MulticopterAttitudeControlNL::vehicle_innerloop_cmd_poll()
{
	/* check if there is a new innerloop cmd */
	bool updated;
	orb_check(_v_innerloop_cmd_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_innerloop_cmd), _v_innerloop_cmd_sub, &_v_innerloop_cmd);
	}
}

void
MulticopterAttitudeControlNL::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_rates_sp_id) {
			if (_vehicle_status.is_vtol) {
				_rates_sp_id = ORB_ID(mc_virtual_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_virtual_mc);

			} else {
				_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_0);
			}
		}
	}
}

void
MulticopterAttitudeControlNL::vehicle_motor_limits_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_motor_limits_sub, &updated);

	if (updated) {
		multirotor_motor_limits_s motor_limits = {};
		orb_copy(ORB_ID(multirotor_motor_limits), _motor_limits_sub, &motor_limits);

		_saturation_status.value = motor_limits.saturation_status;
	}
}

void
MulticopterAttitudeControlNL::battery_status_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
	}
}

void
MulticopterAttitudeControlNL::vehicle_attitude_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_v_att_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
	}
}

void
MulticopterAttitudeControlNL::sensor_correction_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_sensor_correction_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(sensor_correction), _sensor_correction_sub, &_sensor_correction);
	}

	/* update the latest gyro selection */
	if (_sensor_correction.selected_gyro_instance < _gyro_count) {
		_selected_gyro = _sensor_correction.selected_gyro_instance;
	}
}

void
MulticopterAttitudeControlNL::sensor_bias_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_sensor_bias_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(sensor_bias), _sensor_bias_sub, &_sensor_bias);
	}

}

/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp'
 */
void
MulticopterAttitudeControlNL::control_attitude(float dt)
{
	vehicle_attitude_setpoint_poll();

	_thrust_sp = _v_att_sp.thrust;

	/* construct attitude setpoint rotation matrix */
	math::Quaternion q_sp(_v_att_sp.q_d[0], _v_att_sp.q_d[1], _v_att_sp.q_d[2], _v_att_sp.q_d[3]);
	math::Matrix<3, 3> R_sp = q_sp.to_dcm();

	/* get current rotation matrix from control state quaternions */
	math::Quaternion q_att(_v_att.q[0], _v_att.q[1], _v_att.q[2], _v_att.q[3]);
	math::Matrix<3, 3> R = q_att.to_dcm();

	/* all input data is ready, run controller itself */

	/* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
	math::Vector<3> R_z(R(0, 2), R(1, 2), R(2, 2));
	math::Vector<3> R_sp_z(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

	/* axis and sin(angle) of desired rotation (indexes: 0=pitch, 1=roll, 2=yaw).
	 * This is for roll/pitch only (tilt), e_R(2) is 0 */
	math::Vector<3> e_R = R.transposed() * (R_z % R_sp_z);

	/* calculate angle error */
	float e_R_z_sin = e_R.length(); // == sin(tilt angle error)
	float e_R_z_cos = R_z * R_sp_z; // == cos(tilt angle error) == (R.transposed() * R_sp)(2, 2)

	/* calculate rotation matrix after roll/pitch only rotation */
	math::Matrix<3, 3> R_rp;

	if (e_R_z_sin > 0.0f) {
		/* get axis-angle representation */
		float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
		math::Vector<3> e_R_z_axis = e_R / e_R_z_sin;

		e_R = e_R_z_axis * e_R_z_angle;

		/* cross product matrix for e_R_axis */
		math::Matrix<3, 3> e_R_cp;
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

	/* R_rp and R_sp have the same Z axis, calculate yaw error */
	math::Vector<3> R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));
	math::Vector<3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));

	/* calculate the weight for yaw control
	 * Make the weight depend on the tilt angle error: the higher the error of roll and/or pitch, the lower
	 * the weight that we use to control the yaw. This gives precedence to roll & pitch correction.
	 * The weight is 1 if there is no tilt error.
	 */
	float yaw_w = e_R_z_cos * e_R_z_cos;

	/* calculate the angle between R_rp_x and R_sp_x (yaw angle error), and apply the yaw weight */
	e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;

	if (e_R_z_cos < 0.0f) {
		/* for large thrust vector rotations use another rotation method:
		 * calculate angle and axis for R -> R_sp rotation directly */
		math::Quaternion q_error;
		q_error.from_dcm(R.transposed() * R_sp);
		math::Vector<3> e_R_d = q_error(0) >= 0.0f ? q_error.imag()  * 2.0f : -q_error.imag() * 2.0f;

		/* use fusion of Z axis based rotation and direct rotation */
		float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
		e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
	}

	/* calculate angular rates setpoint */
	_rates_sp = _params.att_p.emult(e_R);


	/* Feed forward the yaw setpoint rate. We need to transform the yaw from world into body frame.
	 * The following is a simplification of:
	 * R.transposed() * math::Vector<3>(0.f, 0.f, _v_att_sp.yaw_sp_move_rate * _params.yaw_ff)
	 */
	math::Vector<3> yaw_feedforward_rate(R(2, 0), R(2, 1), R(2, 2));
	yaw_feedforward_rate *= _v_att_sp.yaw_sp_move_rate * _params.yaw_ff;

	yaw_feedforward_rate(2) *= yaw_w;
	_rates_sp += yaw_feedforward_rate;


	/* limit rates */
	for (int i = 0; i < 3; i++) {
		if ((_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_auto_enabled) &&
		    !_v_control_mode.flag_control_manual_enabled) {
			_rates_sp(i) = math::constrain(_rates_sp(i), -_params.auto_rate_max(i), _params.auto_rate_max(i));

		} else {
			_rates_sp(i) = math::constrain(_rates_sp(i), -_params.mc_rate_max(i), _params.mc_rate_max(i));
		}
	}

	/* VTOL weather-vane mode, dampen yaw rate */
	if (_vehicle_status.is_vtol && _v_att_sp.disable_mc_yaw_control) {
		if (_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_auto_enabled) {

			const float wv_yaw_rate_max = _params.auto_rate_max(2) * _params.vtol_wv_yaw_rate_scale;
			_rates_sp(2) = math::constrain(_rates_sp(2), -wv_yaw_rate_max, wv_yaw_rate_max);

			// prevent integrator winding up in weathervane mode
			_rates_int(2) = 0.0f;
		}
	}
}

void MulticopterAttitudeControlNL::attitude_law()
{
//	vehicle_attitude_setpoint_poll();
	path_scheduler_output_poll();
	vehicle_innerloop_cmd_poll();

	static unsigned int loopcnt = 0;
	loopcnt++;

	// Get all the control data
	math::Vector<3> deptp;
	math::Vector<3> cmd_eulers = {0};
	cmd_eulers(0) = _v_innerloop_cmd.fcmd[0]; 	// phic, _v_att_sp.roll_body;
	cmd_eulers(1) = _v_innerloop_cmd.fcmd[1]; 	// thetac, _v_att_sp.pitch_body;
    cmd_eulers(2) = _path_sched_output.psi;		// _v_att_sp.yaw_body;    // psic
//    _anzc = _v_innerloop_cmd.fcmd[2];		// anzc
//	_anzc = _v_att_sp.thrust * (-2) * CONSTANTS_ONE_G; // anzc

	_omegab(0) = _sensor_bias.gyro_x_bias; //_v_att.rollspeed; // -0.00;
	_omegab(1) = _sensor_bias.gyro_y_bias; //_v_att.pitchspeed; // -0.0;
	_omegab(2) = _sensor_bias.gyro_z_bias; //_v_att.yawspeed; // -0.0;

	float epsi = 0.000; // epsi = ispi, from ??
	// Derive attitude angle error

	// 1st derivative of fcmd from cmdgenerator
	_dfcmd(0) = _v_innerloop_cmd.dfcmd[0];	// dfcmd=[dphic; dthetac; 0];
	_dfcmd(1) = _v_innerloop_cmd.dfcmd[1];
	_dfcmd(2) = _v_innerloop_cmd.dfcmd[2];

	// 2nd derivative of fcmd
//	math::Vector<3> ddfcmd = {0};

	/* get current rotation matrix from control state quaternions */
	math::Quaternion q_att(_v_att.q[0], _v_att.q[1], _v_att.q[2], _v_att.q[3]);
	_eulers = q_att.to_euler();	// phi, theta, psi

	// Use tmp eulers readings
//	eulers(0) = 0.0019; eulers(1) = -0.0206; eulers(2) = 0.0000;

	deptp(0) = _eulers(0) - cmd_eulers(0);	// roll angle error
	deptp(1) = _eulers(1) - cmd_eulers(1);	// pitch angle error
	deptp(2) = _wrap_pi(_eulers(2) - _path_sched_output.psi);	// depsi - heading tracking error

	// Construct the omegaE matrix
	_OmegaE(0,0) = 1;
	_OmegaE(0,1) = tanf(_eulers(1)) * sinf(_eulers(0));
	_OmegaE(0,2) = tanf(_eulers(1)) * cosf(_eulers(0));
	_OmegaE(1,0) = 0;
	_OmegaE(1,1) = cosf(_eulers(0));
	_OmegaE(1,2) = -sinf(_eulers(0));
	_OmegaE(2,0) = 0;
	_OmegaE(2,1) = sinf(_eulers(0)) / cosf(_eulers(1));
	_OmegaE(2,2) = cosf(_eulers(0)) / cosf(_eulers(1));

	math::Vector<3> deulers = _OmegaE * _omegab;
	math::Vector<3> ddeptp;
	ddeptp(0) = deulers(0) - _dfcmd(0);
	ddeptp(1) = deulers(1) - _dfcmd(1);
	ddeptp(2) = deulers(2) - _path_sched_output.dpsi; // dpsir

	math::Vector<3> uptp;
	float FCFpt = -39.4784; float FCFdpt = -12.5664;
	uptp(0) = FCFpt * deptp(0) + FCFdpt * ddeptp(0) + _ddfcmd(0);
	uptp(1) = FCFpt * deptp(1) + FCFdpt * ddeptp(1) + _ddfcmd(1);

	// Calculate the heading control
	float FCFpsi0 = -3.9688; float FCFpsi1 = -7.8957; float FCFpsi2 = -5.0265;
	// If without integrator
//	uptp(2) = 0 + FCFpsi1 * deptp(1) + FCFpsi2 * deptp(2);
	// If with integrator
	uptp(2) = FCFpsi0 * epsi + FCFpsi1 * deptp(2) + FCFpsi2 * deptp(2);

	// Construct angular velocity matrix: Omega
	float Omega[3][3] = {0};
	Omega[0][0] = 0;
	Omega[0][1] = -_omegab(2);
	Omega[0][2] = _omegab(1);
	Omega[1][0] = _omegab(2);
	Omega[1][1] = 0;
	Omega[1][2] = -_omegab(0);
	Omega[2][0] = -_omegab(1);
	Omega[2][1] = _omegab(0);
	Omega[2][2] = 0;

	float dphi[3][3] = {0};
	dphi[0][0] = 0;
	dphi[0][1] = tanf(_eulers(1)) * cosf(_eulers(0));
	dphi[0][2] = -tanf(_eulers(1)) * sinf(_eulers(0));
	dphi[1][0] = 0;
	dphi[1][1] = -sinf(_eulers(0));
	dphi[1][2] = -cosf(_eulers(0));
	dphi[2][0] = 0;
	dphi[2][1] = cosf(_eulers(0)) / cosf(_eulers(1));
	dphi[2][2] = -sinf(_eulers(0)) / cosf(_eulers(1));

	float dtheta[3][3];
	dtheta[0][0] = 0.0;
//	dtheta[0][1] = pow(cosf(_eulers(1)), -2) * sinf(_eulers(0));
	dtheta[0][1] = ( (float)1 / (cosf(_eulers(1)) * cosf(_eulers(1))) ) * sinf(_eulers(0));
	dtheta[0][2] = powf(cosf(_eulers(1)), -2) * cosf(_eulers(0));
	dtheta[1][0] = 0.0;
	dtheta[1][1] = 0.0;
	dtheta[1][2] = 0.0;
	dtheta[2][0] = 0.0;
	dtheta[2][1] = -tanf(_eulers(1)) * sinf(_eulers(0)) / cosf(_eulers(1));
	dtheta[2][2] = -tanf(_eulers(1)) * cosf(_eulers(0)) / cosf(_eulers(1));

	math::Vector<3> dphi_omegab;
	dphi_omegab(0) = dphi[0][1] * _omegab(1) + dphi[0][2] * _omegab(2);
	dphi_omegab(1) = dphi[1][1] * _omegab(1) + dphi[1][2] * _omegab(2);
	dphi_omegab(2) = dphi[2][1] * _omegab(1) + dphi[2][2] * _omegab(2);

	math::Vector<3> dtheta_omegab;
	dtheta_omegab(0) = dtheta[0][1] * _omegab(1) + dtheta[0][2] * _omegab(2);
	dtheta_omegab(1) = 0.0;
	dtheta_omegab(2) = dtheta[2][1] * _omegab(1) + dtheta[2][2] * _omegab(2);

	float OmegadE[3][3];
	OmegadE[0][0] = dphi_omegab(0);
	OmegadE[0][1] = dtheta_omegab(0);
	OmegadE[0][2] = 0.0;
	OmegadE[1][0] = dphi_omegab(1);
	OmegadE[1][1] = dtheta_omegab(1);
	OmegadE[1][2] = 0.0;
	OmegadE[2][0] = dphi_omegab(2);
    OmegadE[2][1] = dtheta_omegab(2);
	OmegadE[2][2] = 0.0;

	// Calculate Mbc
	math::Matrix<3,3> OmegadE_OmegaE;
	OmegadE_OmegaE(0,0) = OmegadE[0][0] * _OmegaE(0,0);
	OmegadE_OmegaE(0,1) = OmegadE[0][0] * _OmegaE(0,1) + OmegadE[0][1] * _OmegaE(1,1);
	OmegadE_OmegaE(0,2) = OmegadE[0][0] * _OmegaE(0,2) + OmegadE[0][1] * _OmegaE(1,2);
	OmegadE_OmegaE(1,0) = OmegadE[1][0] * _OmegaE(0,0);
	OmegadE_OmegaE(1,1) = OmegadE[1][0] * _OmegaE(0,1) + OmegadE[1][1] * _OmegaE(1,1);
	OmegadE_OmegaE(1,2) = OmegadE[1][0] * _OmegaE(0,2) + OmegadE[1][1] * _OmegaE(1,2);
	OmegadE_OmegaE(2,0) = OmegadE[2][0] * _OmegaE(0,0);
	OmegadE_OmegaE(2,1) = OmegadE[2][0] * _OmegaE(0,1) + OmegadE[2][1] * _OmegaE(1,1);
	OmegadE_OmegaE(2,2) = OmegadE[2][0] * _OmegaE(0,2) + OmegadE[2][1] * _OmegaE(1,2);

	math::Vector<3> OmegadE_OmegaE_omegab = OmegadE_OmegaE * _omegab;

	math::Matrix<3,3> Omega_FCJb;
	Omega_FCJb(0,0) = 0.0;
	Omega_FCJb(0,1) = Omega[0][1] * _FC_Jb[1][1];
	Omega_FCJb(0,2) = Omega[0][2] * _FC_Jb[2][2];
	Omega_FCJb(1,0) = Omega[1][0] * _FC_Jb[0][0];
	Omega_FCJb(1,1) = 0.0;
	Omega_FCJb(1,2) = Omega[1][2] * _FC_Jb[2][2];
	Omega_FCJb(2,0) = Omega[2][0] * _FC_Jb[0][0];
	Omega_FCJb(2,1) = Omega[2][1] * _FC_Jb[1][1];
	Omega_FCJb(2,2) = 0.0;

	// calculate moment command
	math::Matrix<3,3> invOmegaE = _OmegaE.inversed();
	math::Matrix<3,3> FCJB_invOmegaE;
	FCJB_invOmegaE(0,0) = _FC_Jb[0][0] * invOmegaE(0,0);
	FCJB_invOmegaE(0,1) = _FC_Jb[0][0] * invOmegaE(0,1);
	FCJB_invOmegaE(0,2) = _FC_Jb[0][0] * invOmegaE(0,2);
	FCJB_invOmegaE(1,0) = _FC_Jb[1][1] * invOmegaE(1,0);
	FCJB_invOmegaE(1,1) = _FC_Jb[1][1] * invOmegaE(1,1);
	FCJB_invOmegaE(1,2) = _FC_Jb[1][1] * invOmegaE(1,2);
	FCJB_invOmegaE(2,0) = _FC_Jb[2][2] * invOmegaE(2,0);
	FCJB_invOmegaE(2,1) = _FC_Jb[2][2] * invOmegaE(2,1);
	FCJB_invOmegaE(2,2) = _FC_Jb[2][2] * invOmegaE(2,2);


	_Mbc = FCJB_invOmegaE * (uptp - OmegadE_OmegaE_omegab)
			+ Omega_FCJb * _omegab;

	if (loopcnt % 200 == 0) {
		printf("attitude: fcmd: phic %.3f, thetac %.3f, anzc %.3f\n",
				(double)_v_innerloop_cmd.fcmd[0], (double)_v_innerloop_cmd.fcmd[1], (double)_v_innerloop_cmd.fcmd[2]);
		printf("attitude: Mbc: %.4f, %.4f, %.4f\n", (double)_Mbc(0), (double)_Mbc(1), (double)_Mbc(2));
	}
	return;
}

void MulticopterAttitudeControlNL::force_moment_allocator()
{
	static unsigned int loopcnt = 0;
	loopcnt++;

	float Va = _Vba(0);	// 1.6228;
	float beta = _Vba(1); // -0.0085;
	float alpha = _Vba(2); 	//-0.0349;

	math::Vector<4> Fmc;	///< Fm command
	Fmc(0) = (float)4.3 * _v_innerloop_cmd.fcmd[2]; 	// -44.0003; mb * anzc;
	Fmc(1) = _Mbc(0);
	Fmc(2) = _Mbc(1);
	Fmc(3) = _Mbc(2);

	math::Vector<4> cw;		///< rotor direction matrix
	cw(0) = -1; cw(1) = -1; cw(2) = 1; cw(3) = 1;

	math::Matrix<3,3> Sbw;
	Sbw(0,0) = cosf(beta) * cosf(alpha);
	Sbw(0,1) = sinf(beta);
	Sbw(0,2) = cosf(beta) * sinf(alpha);
	Sbw(1,0) = -sinf(beta) * cosf(alpha);
	Sbw(1,1) = cosf(beta);
	Sbw(1,2) = -sinf(beta) * sinf(alpha);
	Sbw(2,0) = -sinf(alpha);
	Sbw(2,1) = 0;
	Sbw(2,2) = cosf(alpha);

	math::Vector<3> Vab(0.0, 0.0, 0.0);
	math::Vector<3> Va_tmp(Va, 0, 0);
	Vab = Sbw.transposed() * Va_tmp;

	Va = Vab.length();	// Norm of Vab

	float thetaa, thetab, psib;
	if ( (double)Va > 0.1 ) {
		thetaa = asinf(Vab(2) / Va);
		psib = atan2f(Vab(1), Vab(0));
	}
	else {
		psib = thetaa = 0;
	}
	thetab = (float)3.1415926/2.0f - thetaa;
//	thetab = thetab-1;
	// Construct matrix Bbp
	math::Matrix<3,3> Bbp;
	math::Matrix<3,3> tmpA;
	tmpA(0,0) = 1; tmpA(0,1) = 0; tmpA(0,2) = 0;
	tmpA(1,0) = 0; tmpA(1,1) = -1; tmpA(1,2) = 0;
	tmpA(2,0) = 0; tmpA(2,1) = 0; tmpA(2,2) = -1;

	math::Matrix<3,3> tmpB;
	tmpB(0,0) = cosf(psib); tmpB(0,1) = sinf(psib); tmpB(0,2) = 0;
	tmpB(1,0) = (float)(-1.0)*sinf(psib); tmpB(1,1) = cosf(psib); tmpB(1,2) = 0;
	tmpB(2,0) = 0; tmpB(2,1) = 0; tmpB(2,2) = 1;

//	Bbp = tmpA * tmpB;
	Bbp(0,0) = tmpA(0,0) * tmpB(0,0);
	Bbp(0,1) = tmpA(0,0) * tmpB(0,1);
	Bbp(0,2) = 0.0;
	Bbp(1,0) = -tmpB(1,0);
	Bbp(1,1) = -tmpB(1,1);
	Bbp(1,2) = 0.0;
	Bbp(2,0) = 0.0;
	Bbp(2,1) = 0.0;
	Bbp(2,2) = -1.0;

	math::Matrix<4,4> diagCW;
	diagCW(0,0) = -1; diagCW(0,1) = 0; diagCW(0,2) = 0; diagCW(0,3) = 0;
	diagCW(1,0) = 0; diagCW(1,1) = -1; diagCW(1,2) = 0; diagCW(1,3) = 0;
	diagCW(2,0) = 0; diagCW(2,1) = 0; diagCW(2,2) = 1; diagCW(2,3) = 0;
	diagCW(3,0) = 0; diagCW(3,1) = 0; diagCW(3,2) = 0; diagCW(3,3) = 1;

//	_rpm0 = diagCW * _omegar0 * 30 / 3.1415926;
	_rpm0(0) = -_omegar0(0) * (float)9.5493;
	_rpm0(1) = -_omegar0(1) * (float)9.5493;
	_rpm0(2) = _omegar0(2) * (float)9.5493;
	_rpm0(3) = _omegar0(3) * (float)9.5493;

	/* Calculate the F and M of each rotor */
	math::Vector<3> pr;
	math::Matrix<3,3> Opr;
	_Fbt.zero();
	_Mbt.zero();

	math::Vector<3> pFapn, pMapn, pFbpn, pMbpn;
	math::Matrix<3, 4> pFbtpn, pMbtpn;
	pFbtpn.zero();
	pMbtpn.zero();

	for (int i = 0; i < 4; i++) {
		unsigned int n = _rpm0(i);
		// Arm length of each motor
		if (i == 0) {
			pr(0) = 0.37; pr(1) = 0.37; pr(2) = 0;
		} else if (i == 1) {
			pr(0) = -0.37; pr(1) = -0.37; pr(2) = 0;
		} else if (i == 2) {
			pr(0) = 0.37; pr(1) = -0.37; pr(2) = 0;
		} else if (i == 3) {
			pr(0) = -0.37; pr(1) = 0.37; pr(2) = 0;
		}

		// Construct Opr
		Opr(0,0) = 0; Opr(0,1) = -pr(2); Opr(0,2) = pr(1);
		Opr(1,0) = pr(2); Opr(1,1) = 0; Opr(1,2) = -pr(0);
		Opr(2,0) = -pr(1); Opr(2,1) = pr(0); Opr(2,2) = 0;

		FM_PARAM fmParam;
		fmParam.n = n; fmParam.Va = Va; fmParam.thetab = thetab;
//		math::Vector<6> fm, pfmpn;
		if (cw(i) > 0) {
			_fm = _fmLut->interpolation3(fmParam, FMLut::lut_f_m_rpm_va_thetab, FMLut::lut_rpm, FMLut::lut_Va, FMLut::lut_thetab);
			_pfmpn = _fmLut->interpolation3(fmParam, FMLut::lut_pfmpn_rpm_va_thetab, FMLut::lut_rpm, FMLut::lut_Va, FMLut::lut_thetab);
		} else {
			_fm = _fmLut->interpolation3(fmParam, FMLut::lut_f_m_c_rpm_va_thetab, FMLut::lut_rpm, FMLut::lut_Va, FMLut::lut_thetab);
			_pfmpn = _fmLut->interpolation3(fmParam, FMLut::lut_pfmpn_c_rpm_va_thetab, FMLut::lut_rpm, FMLut::lut_Va, FMLut::lut_thetab);
		}

		// Force and Moment of this propeller
		_Fa(0) = 0; _Fa(1) = 0; _Fa(2) = _fm(2);
		_Ma(0) = 0; _Ma(1) = 0; _Ma(2) = _fm(5);
//		Fb = Bbp.transposed() * Fa;
		math::Matrix<3,3> Bbp_t = Bbp.transposed();
		_Fb(0) = Bbp_t(0,2) * _Fa(2);
		_Fb(1) = Bbp_t(1,2) * _Fa(2);
		_Fb(2) = Bbp_t(2,2) * _Fa(2);

		float Opr_Fb[3];
		Opr_Fb[0] = Opr(0,1) * _Fb(1) + Opr(0,2) * _Fb(2);
		Opr_Fb[1] = Opr(1,0) * _Fb(0) + Opr(1,2) * _Fb(2);
		Opr_Fb[2] = Opr(2,0) * _Fb(0) + Opr(2,1) * _Fb(1);

		float Bbpt_Ma[3];
		Bbpt_Ma[0] = Bbp_t(0,2) * _Ma(2);
		Bbpt_Ma[1] = Bbp_t(1,2) * _Ma(2);
		Bbpt_Ma[2] = Bbp_t(2,2) * _Ma(2);

		for (int k = 0; k < 3; k++) {
			_Mb(k) = Bbpt_Ma[k] + Opr_Fb[k];
		}
		_Fbt = _Fbt + _Fb;
		_Mbt = _Mbt + _Mb;
//		Mb = Bbp.transposed() * Ma + Opr * Fb;


		pFapn(0) = 0; pFapn(1) = 0; pFapn(2) = _pfmpn(2);
		pMapn(0) = 0; pMapn(1) = 0; pMapn(2) = _pfmpn(5);
		pFbpn(0) = 0;
		pFbpn(1) = 0;
		pFbpn(2) = Bbp_t(2,2) * pFapn(2);

//		pFbpn = Bbp_t * pFapn;
		float Bbpt_pManpn[3];
		Bbpt_pManpn[0] = 0;
		Bbpt_pManpn[1] = 0;
		Bbpt_pManpn[2] = Bbp_t(2,2) * pMapn(2);

		float Opr_pFbpn[3];
		Opr_pFbpn[0] = Opr(0,2) * pFbpn(2);
		Opr_pFbpn[1] = Opr(1,2) * pFbpn(2);
		Opr_pFbpn[2] = 0;

		for (int j = 0; j < 3; j++) {
			pMbpn(j) = Bbpt_pManpn[j] + Opr_pFbpn[j];
		}
//		pMbpn = Bbpt_pManpn + Opr_pFbpn;

		pFbtpn.set_col(i, pFbpn);
		pMbtpn.set_col(i, pMbpn);
	}

	_Fm(0) = _Fbt(2); _Fm(1) = _Mbt(0); _Fm(2) = _Mbt(1); _Fm(3) = _Mbt(2);
	_pFmpn(0,0) = pFbtpn(2,0); _pFmpn(0,1) = pFbtpn(2,1); _pFmpn(0,2) = pFbtpn(2,2); _pFmpn(0,3) = pFbtpn(2,3);	// Construct row 0 of _pFmpn
	_pFmpn(1,0) = pMbtpn(0,0); _pFmpn(1,1) = pMbtpn(0,1); _pFmpn(1,2) = pMbtpn(0,2); _pFmpn(1,3) = pMbtpn(0,3); // Construct row 1
	_pFmpn(2,0) = pMbtpn(1,0); _pFmpn(2,1) = pMbtpn(1,1); _pFmpn(2,2) = pMbtpn(1,2); _pFmpn(2,3) = pMbtpn(1,3); // Construct row 2
	_pFmpn(3,0) = pMbtpn(2,0); _pFmpn(3,1) = pMbtpn(2,1); _pFmpn(3,2) = pMbtpn(2,2); _pFmpn(3,3) = pMbtpn(2,3); // Construct row 3

	math::Vector<4> nr;	///< Rotors RPM command
	nr = _rpm0 + _pFmpn.inversed() * (Fmc - _Fm);

	// Limit the RPM value
	for (unsigned int i = 0; i < nr.get_size(); i++) {
		// TO check
		if (nr(i) < 2000) {
			nr(i) = 2000;
		}
		else if (nr(i) > 5000) {
			nr(i) = 5000;
		}
	}

	// Update new spinning rate omegar
//	_omegar0 = diagCW * nr * 3.1415926 / 30.0;
	_omegar0(0) = -nr(0) * (float)0.1047;
	_omegar0(1) = -nr(1) * (float)0.1047;
	_omegar0(2) = nr(2) * (float)0.1047;
	_omegar0(3) = nr(3) * (float)0.1047;

	// Update control input
	math::Vector<3> x;
	float n, pnpi;
	for (int i = 0; i < 4; i++) {
		x(0) = _delta0(i); x(1) = Va; x(2) = thetab;
		if (cw(i) > 0) {
			n = _fmLut->interpolation3(x, FMLut::lut_rpmi, FMLut::lut_di, FMLut::lut_Va, FMLut::lut_thetab);
			pnpi = _fmLut->interpolation3(x, FMLut::lut_prpmipi, FMLut::lut_di, FMLut::lut_Va, FMLut::lut_thetab);
		}
		else {
			n = _fmLut->interpolation3(x, FMLut::lut_rpmic, FMLut::lut_di, FMLut::lut_Va, FMLut::lut_thetab);
			pnpi = _fmLut->interpolation3(x, FMLut::lut_prpmicpi, FMLut::lut_di, FMLut::lut_Va, FMLut::lut_thetab);
		}

		_delta0(i) = _delta0(i) + ((float)1.0 / pnpi) * (nr(i) - n);
//		_delta(i) = math::constrain(&_delta(i), (double)0.2, (double)0.8);
//		printf("Loop %d, _delta %.3f\n", i, (double)_delta(i));
	}

//	/* publish actuator controls */
//	// Shift control input from [0,1] to [-1,1] to comply with the mixer definitions
//	float delta_shift[4];
//	delta_shift[0] = _delta0(0)*2.0f - 1.0f;
//	delta_shift[1] = _delta0(1)*2.0f - 1.0f;
//	delta_shift[2] = _delta0(2)*2.0f - 1.0f;
//	delta_shift[3] = _delta0(3)*2.0f - 1.0f;
//
//	_actuators.control[0] = delta_shift[0]; //(PX4_ISFINITE(delta_shift[0])) ? delta_shift[0] : -1.0f;
//	_actuators.control[1] = delta_shift[1]; //(PX4_ISFINITE(delta_shift[1])) ? delta_shift[1] : -1.0f;
//	_actuators.control[2] = delta_shift[2]; //(PX4_ISFINITE(delta_shift[2])) ? delta_shift[2] : -1.0f;
//	_actuators.control[3] = delta_shift[3]; //(PX4_ISFINITE(delta_shift[3])) ? delta_shift[3] : -1.0f;
//	_actuators.control[7] = _v_att_sp.landing_gear;
//	_actuators.timestamp = hrt_absolute_time();
//	_actuators.timestamp_sample = _v_att.timestamp;
//
//	/* scale effort by battery status */
///*	if (_params.bat_scale_en && _battery_status.scale > 0.0f) {
//		for (int i = 0; i < 4; i++) {
//			_actuators.control[i] *= _battery_status.scale;
//		}
//	}*/
//
//	_controller_status.roll_rate_integ = _rates_int(0);
//	_controller_status.pitch_rate_integ = _rates_int(1);
//	_controller_status.yaw_rate_integ = _rates_int(2);
//	_controller_status.timestamp = hrt_absolute_time();
//
//	if (!_actuators_0_circuit_breaker_enabled) {
//		if (_actuators_0_pub != nullptr) {
//
//			orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
//			perf_end(_controller_latency_perf);
//
//		} else if (_actuators_id) {
//			_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
//		}
//	}

	_v_fm_cmd.force = Fmc(0);
	_v_fm_cmd.Mbc[0] = Fmc(1);
	_v_fm_cmd.Mbc[1] = Fmc(2);
	_v_fm_cmd.Mbc[2] = Fmc(3);

	_v_fm_cmd.omega[0] = nr(0);
	_v_fm_cmd.omega[1] = nr(1);
	_v_fm_cmd.omega[2] = nr(2);
	_v_fm_cmd.omega[3] = nr(3);

	_v_fm_cmd.timestamp = hrt_absolute_time();

	if (loopcnt % 200 == 0) {
		printf("**nr %.3f, %.3f, %.3f, %.3f\n", (double)nr(0), (double)nr(1), (double)nr(2), (double)nr(3));
//		printf("**_delta %.3f, %.3f, %.3f, %.3f\n", (double)_delta0(0), (double)_delta0(1), (double)_delta0(2), (double)_delta0(3));
//		printf("**delta_shift %.3f, %.3f, %.3f, %.3f\n", (double)delta_shift[0], (double)delta_shift[1], (double)delta_shift[2], (double)delta_shift[3]);
	}
}

/*
 * Throttle PID attenuation
 * Function visualization available here https://www.desmos.com/calculator/gn4mfoddje
 * Input: 'tpa_breakpoint', 'tpa_rate', '_thrust_sp'
 * Output: 'pidAttenuationPerAxis' vector
 */
math::Vector<3>
MulticopterAttitudeControlNL::pid_attenuations(float tpa_breakpoint, float tpa_rate)
{
	/* throttle pid attenuation factor */
	float tpa = 1.0f - tpa_rate * (fabsf(_v_rates_sp.thrust) - tpa_breakpoint) / (1.0f - tpa_breakpoint);
	tpa = fmaxf(TPA_RATE_LOWER_LIMIT, fminf(1.0f, tpa));

	math::Vector<3> pidAttenuationPerAxis;
	pidAttenuationPerAxis(AXIS_INDEX_ROLL) = tpa;
	pidAttenuationPerAxis(AXIS_INDEX_PITCH) = tpa;
	pidAttenuationPerAxis(AXIS_INDEX_YAW) = 1.0;

	return pidAttenuationPerAxis;
}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
void
MulticopterAttitudeControlNL::control_attitude_rates(float dt)
{
	/* reset integral if disarmed */
	if (!_v_control_mode.flag_armed || !_vehicle_status.is_rotary_wing) {
		_rates_int.zero();
	}

	// get the raw gyro data and correct for thermal errors
	math::Vector<3> rates;

	if (_selected_gyro == 0) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_0[0]) * _sensor_correction.gyro_scale_0[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_0[1]) * _sensor_correction.gyro_scale_0[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_0[2]) * _sensor_correction.gyro_scale_0[2];

	} else if (_selected_gyro == 1) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_1[0]) * _sensor_correction.gyro_scale_1[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_1[1]) * _sensor_correction.gyro_scale_1[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_1[2]) * _sensor_correction.gyro_scale_1[2];

	} else if (_selected_gyro == 2) {
		rates(0) = (_sensor_gyro.x - _sensor_correction.gyro_offset_2[0]) * _sensor_correction.gyro_scale_2[0];
		rates(1) = (_sensor_gyro.y - _sensor_correction.gyro_offset_2[1]) * _sensor_correction.gyro_scale_2[1];
		rates(2) = (_sensor_gyro.z - _sensor_correction.gyro_offset_2[2]) * _sensor_correction.gyro_scale_2[2];

	} else {
		rates(0) = _sensor_gyro.x;
		rates(1) = _sensor_gyro.y;
		rates(2) = _sensor_gyro.z;
	}

	// rotate corrected measurements from sensor to body frame
	rates = _board_rotation * rates;

	// correct for in-run bias errors
	rates(0) -= _sensor_bias.gyro_x_bias;
	rates(1) -= _sensor_bias.gyro_y_bias;
	rates(2) -= _sensor_bias.gyro_z_bias;

	math::Vector<3> rates_p_scaled = _params.rate_p.emult(pid_attenuations(_params.tpa_breakpoint_p, _params.tpa_rate_p));
	//math::Vector<3> rates_i_scaled = _params.rate_i.emult(pid_attenuations(_params.tpa_breakpoint_i, _params.tpa_rate_i));
	math::Vector<3> rates_d_scaled = _params.rate_d.emult(pid_attenuations(_params.tpa_breakpoint_d, _params.tpa_rate_d));

	/* angular rates error */
	math::Vector<3> rates_err = _rates_sp - rates;

	_att_control = rates_p_scaled.emult(rates_err) +
		       _rates_int +
		       rates_d_scaled.emult(_rates_prev - rates) / dt +
		       _params.rate_ff.emult(_rates_sp);

	_rates_sp_prev = _rates_sp;
	_rates_prev = rates;

	/* update integral only if motors are providing enough thrust to be effective */
	if (_thrust_sp > MIN_TAKEOFF_THRUST) {
		for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
			// Check for positive control saturation
			bool positive_saturation =
				((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_pos) ||
				((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_pos) ||
				((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_pos);

			// Check for negative control saturation
			bool negative_saturation =
				((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_neg) ||
				((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_neg) ||
				((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_neg);

			// prevent further positive control saturation
			if (positive_saturation) {
				rates_err(i) = math::min(rates_err(i), 0.0f);

			}

			// prevent further negative control saturation
			if (negative_saturation) {
				rates_err(i) = math::max(rates_err(i), 0.0f);

			}

			// Perform the integration using a first order method and do not propaate the result if out of range or invalid
			float rate_i = _rates_int(i) + _params.rate_i(i) * rates_err(i) * dt;

			if (PX4_ISFINITE(rate_i) && rate_i > -_params.rate_int_lim(i) && rate_i < _params.rate_int_lim(i)) {
				_rates_int(i) = rate_i;

			}
		}
	}

	/* explicitly limit the integrator state */
	for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
		_rates_int(i) = math::constrain(_rates_int(i), -_params.rate_int_lim(i), _params.rate_int_lim(i));

	}
}

void
MulticopterAttitudeControlNL::task_main_trampoline(int argc, char *argv[])
{
	mc_att_control_nl::g_control->task_main();
}

void
MulticopterAttitudeControlNL::task_main()
{

	/*
	 * do subscriptions
	 */
	_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_v_innerloop_cmd_sub = orb_subscribe(ORB_ID(vehicle_innerloop_cmd));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_path_sched_output_sub = orb_subscribe(ORB_ID(path_scheduler_output));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));

	_gyro_count = math::min(orb_group_count(ORB_ID(sensor_gyro)), MAX_GYRO_COUNT);

	if (_gyro_count == 0) {
		_gyro_count = 1;
	}

	for (unsigned s = 0; s < _gyro_count; s++) {
		_sensor_gyro_sub[s] = orb_subscribe_multi(ORB_ID(sensor_gyro), s);
	}

	_sensor_correction_sub = orb_subscribe(ORB_ID(sensor_correction));
	_sensor_bias_sub = orb_subscribe(ORB_ID(sensor_bias));

	/* initialize parameters cache */
	parameters_update();

	/* wakeup source: gyro data from sensor selected by the sensor app */
	px4_pollfd_struct_t poll_fds = {};
	poll_fds.events = POLLIN;

	while (!_task_should_exit) {

		poll_fds.fd = _sensor_gyro_sub[_selected_gyro];

		/* wait for up to 100ms for data */
		int pret = px4_poll(&poll_fds, 1, 100);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("mc att ctrl: poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		perf_begin(_loop_perf);

		/* run controller on gyro changes */
		if (poll_fds.revents & POLLIN) {
			static uint64_t last_run = 0;
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too small (< 2ms) and too large (> 20ms) dt's */
			if (dt < 0.002f) {
				dt = 0.002f;

			} else if (dt > 0.02f) {
				dt = 0.02f;
			}

			/* copy gyro data */
			orb_copy(ORB_ID(sensor_gyro), _sensor_gyro_sub[_selected_gyro], &_sensor_gyro);

			/* check for updates in other topics */
			parameter_update_poll();
			vehicle_control_mode_poll();
			vehicle_manual_poll();
			vehicle_status_poll();
			vehicle_motor_limits_poll();
			vehicle_innerloop_cmd_poll();
			path_scheduler_output_poll();
			battery_status_poll();
			vehicle_attitude_poll();
			sensor_correction_poll();
			sensor_bias_poll();

			/* Check if we are in rattitude mode and the pilot is above the threshold on pitch
			 * or roll (yaw can rotate 360 in normal att control).  If both are true don't
			 * even bother running the attitude controllers */
			if (_v_control_mode.flag_control_rattitude_enabled) {
				if (fabsf(_manual_control_sp.y) > _params.rattitude_thres ||
				    fabsf(_manual_control_sp.x) > _params.rattitude_thres) {
					_v_control_mode.flag_control_attitude_enabled = false;
				}
			}

			if (_v_control_mode.flag_control_attitude_enabled) {

				if (_ts_opt_recovery == nullptr) {
					if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL) {
						// the  tailsitter recovery instance has not been created, thus, the vehicle
						// is not a tailsitter, do normal attitude control
						control_attitude(dt);
					} else if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL) {
						attitude_law();
						force_moment_allocator();

						/* publish force_moment_cmd */
						if (_v_fm_cmd_pub != nullptr) {
							orb_publish(ORB_ID(vehicle_force_moment_cmd), _v_fm_cmd_pub, &_v_fm_cmd);
						} else {
							_v_fm_cmd_pub = orb_advertise(ORB_ID(vehicle_force_moment_cmd), &_v_fm_cmd);
						}

						/* publish actuator controls */
						// Shift control input from [0,1] to [-1,1] to comply with the mixer definitions
						float delta_shift[4];
						delta_shift[0] = 0.6f; //_delta0(0)*2.0f - 1.0f;
						delta_shift[1] = 0.6f; //_delta0(1)*2.0f - 1.0f;
						delta_shift[2] = 0.6f; //_delta0(2)*2.0f - 1.0f;
						delta_shift[3] = 0.6f; //_delta0(3)*2.0f - 1.0f;

						_actuators.control[0] = delta_shift[0]; //(PX4_ISFINITE(delta_shift[0])) ? delta_shift[0] : -1.0f;
						_actuators.control[1] = delta_shift[1]; //(PX4_ISFINITE(delta_shift[1])) ? delta_shift[1] : -1.0f;
						_actuators.control[2] = delta_shift[2]; //(PX4_ISFINITE(delta_shift[2])) ? delta_shift[2] : -1.0f;
						_actuators.control[3] = delta_shift[3]; //(PX4_ISFINITE(delta_shift[3])) ? delta_shift[3] : -1.0f;
						_actuators.control[7] = _v_att_sp.landing_gear;
						_actuators.timestamp = hrt_absolute_time();
						_actuators.timestamp_sample = _v_att.timestamp;

						/* scale effort by battery status */
					/*	if (_params.bat_scale_en && _battery_status.scale > 0.0f) {
							for (int i = 0; i < 4; i++) {
								_actuators.control[i] *= _battery_status.scale;
							}
						}*/

						if (!_actuators_0_circuit_breaker_enabled) {
							if (_actuators_0_pub != nullptr) {

								orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
								perf_end(_controller_latency_perf);

							} else if (_actuators_id) {
								_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
							}
						}
					}

				} else {
					vehicle_attitude_setpoint_poll();
					_thrust_sp = _v_att_sp.thrust;
					math::Quaternion q(_v_att.q[0], _v_att.q[1], _v_att.q[2], _v_att.q[3]);
					math::Quaternion q_sp(&_v_att_sp.q_d[0]);
					_ts_opt_recovery->setAttGains(_params.att_p, _params.yaw_ff);
					_ts_opt_recovery->calcOptimalRates(q, q_sp, _v_att_sp.yaw_sp_move_rate, _rates_sp);

					/* limit rates */
					for (int i = 0; i < 3; i++) {
						_rates_sp(i) = math::constrain(_rates_sp(i), -_params.mc_rate_max(i), _params.mc_rate_max(i));
					}
				}

				/* publish attitude rates setpoint */
				_v_rates_sp.roll = _rates_sp(0);
				_v_rates_sp.pitch = _rates_sp(1);
				_v_rates_sp.yaw = _rates_sp(2);
				_v_rates_sp.thrust = _thrust_sp;
				_v_rates_sp.timestamp = hrt_absolute_time();

				if (_v_rates_sp_pub != nullptr) {
					orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

				} else if (_rates_sp_id) {
					_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
				}

			} else {
				/* attitude controller disabled, poll rates setpoint topic */
				if (_v_control_mode.flag_control_manual_enabled) {
					/* manual rates control - ACRO mode */
					matrix::Vector3f man_rate_sp;
					man_rate_sp(0) = math::superexpo(_manual_control_sp.y, _params.acro_expo, _params.acro_superexpo);
					man_rate_sp(1) = math::superexpo(-_manual_control_sp.x, _params.acro_expo, _params.acro_superexpo);
					man_rate_sp(2) = math::superexpo(_manual_control_sp.r, _params.acro_expo, _params.acro_superexpo);
					man_rate_sp = man_rate_sp.emult(_params.acro_rate_max);
					_rates_sp = math::Vector<3>(man_rate_sp.data());
					_thrust_sp = _manual_control_sp.z;

					/* publish attitude rates setpoint */
					_v_rates_sp.roll = _rates_sp(0);
					_v_rates_sp.pitch = _rates_sp(1);
					_v_rates_sp.yaw = _rates_sp(2);
					_v_rates_sp.thrust = _thrust_sp;
					_v_rates_sp.timestamp = hrt_absolute_time();

					if (_v_rates_sp_pub != nullptr) {
						orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

					} else if (_rates_sp_id) {
						_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
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

			if (_v_control_mode.flag_control_rates_enabled &&
				_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL) {
				control_attitude_rates(dt);

				/* publish actuator controls */
				_actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
				_actuators.control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
				_actuators.control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
				_actuators.control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
				_actuators.control[7] = _v_att_sp.landing_gear;
				_actuators.timestamp = hrt_absolute_time();
				_actuators.timestamp_sample = _sensor_gyro.timestamp;

				/* scale effort by battery status */
				if (_params.bat_scale_en && _battery_status.scale > 0.0f) {
					for (int i = 0; i < 4; i++) {
						_actuators.control[i] *= _battery_status.scale;
					}
				}

				_controller_status.roll_rate_integ = _rates_int(0);
				_controller_status.pitch_rate_integ = _rates_int(1);
				_controller_status.yaw_rate_integ = _rates_int(2);
				_controller_status.timestamp = hrt_absolute_time();

				if (!_actuators_0_circuit_breaker_enabled) {
					if (_actuators_0_pub != nullptr) {

						orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
						perf_end(_controller_latency_perf);

					} else if (_actuators_id) {
						_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
					}

				}

				/* publish controller status */
				if (_controller_status_pub != nullptr) {
					orb_publish(ORB_ID(mc_att_ctrl_status), _controller_status_pub, &_controller_status);

				} else {
					_controller_status_pub = orb_advertise(ORB_ID(mc_att_ctrl_status), &_controller_status);
				}
			}

			if (_v_control_mode.flag_control_termination_enabled) {
				if (!_vehicle_status.is_vtol) {

					_rates_sp.zero();
					_rates_int.zero();
					_thrust_sp = 0.0f;
					_att_control.zero();

					/* publish actuator controls */
					_actuators.control[0] = 0.0f;
					_actuators.control[1] = 0.0f;
					_actuators.control[2] = 0.0f;
					_actuators.control[3] = 0.0f;
					_actuators.timestamp = hrt_absolute_time();
					_actuators.timestamp_sample = _sensor_gyro.timestamp;

					if (!_actuators_0_circuit_breaker_enabled) {
						if (_actuators_0_pub != nullptr) {

							orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
							perf_end(_controller_latency_perf);

						} else if (_actuators_id) {
							_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
						}
					}

					_controller_status.roll_rate_integ = _rates_int(0);
					_controller_status.pitch_rate_integ = _rates_int(1);
					_controller_status.yaw_rate_integ = _rates_int(2);
					_controller_status.timestamp = hrt_absolute_time();

					/* publish controller status */
					if (_controller_status_pub != nullptr) {
						orb_publish(ORB_ID(mc_att_ctrl_status), _controller_status_pub, &_controller_status);

					} else {
						_controller_status_pub = orb_advertise(ORB_ID(mc_att_ctrl_status), &_controller_status);
					}

					/* publish attitude rates setpoint */
					_v_rates_sp.roll = _rates_sp(0);
					_v_rates_sp.pitch = _rates_sp(1);
					_v_rates_sp.yaw = _rates_sp(2);
					_v_rates_sp.thrust = _thrust_sp;
					_v_rates_sp.timestamp = hrt_absolute_time();

					if (_v_rates_sp_pub != nullptr) {
						orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

					} else if (_rates_sp_id) {
						_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
					}
				}
			}

		}
		perf_end(_loop_perf);
	}

	_control_task = -1;
}

int
MulticopterAttitudeControlNL::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("mc_att_control_nl",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   5100, //1700,
					   (px4_main_t)&MulticopterAttitudeControlNL::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int mc_att_control_nl_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: mc_att_control_nl {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (mc_att_control_nl::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		mc_att_control_nl::g_control = new MulticopterAttitudeControlNL;

		if (mc_att_control_nl::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != mc_att_control_nl::g_control->start()) {
			delete mc_att_control_nl::g_control;
			mc_att_control_nl::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (mc_att_control_nl::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete mc_att_control_nl::g_control;
		mc_att_control_nl::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (mc_att_control_nl::g_control) {
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
