/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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
 * @file hover_thrust_estimator.cpp
 *
 * @author Mathieu Bresciani 	<brescianimathieu@gmail.com>
 */

#include "FwAutoTrim.hpp"

#include <mathlib/mathlib.h>

using namespace time_literals;
using matrix::Vector3f;

FwAutoTrim::FwAutoTrim(bool is_vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vehicle_torque_setpoint_sub(this, ORB_ID(vehicle_torque_setpoint), is_vtol ? 1 : 0)
{
	updateParams();
	reset();
}

FwAutoTrim::~FwAutoTrim()
{
	perf_free(_cycle_perf);
}

bool FwAutoTrim::init()
{
	if (!_vehicle_torque_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_auto_trim_status_pub.advertise();

	return true;
}

void FwAutoTrim::reset()
{
}

void FwAutoTrim::updateParams()
{
	ModuleParams::updateParams();
}

void FwAutoTrim::Run()
{
	if (should_exit()) {
		_vehicle_torque_setpoint_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected;

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;
		}
	}


	vehicle_torque_setpoint_s vehicle_torque_setpoint;

	if (!_vehicle_torque_setpoint_sub.update(&vehicle_torque_setpoint)
	    || (vehicle_torque_setpoint.timestamp == _timestamp_last)) {
		return;
	}

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	perf_begin(_cycle_perf);

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
			_fixed_wing = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING);
			_tailsitter = vehicle_status.is_vtol_tailsitter;
		}
	}

	if (_airspeed_validated_sub.updated()) {
		airspeed_validated_s airspeed_validated;

		if (_airspeed_validated_sub.copy(&airspeed_validated)) {
			_airspeed = airspeed_validated.true_airspeed_m_s;
		}
	}

	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude;

		if (_vehicle_attitude_sub.copy(&vehicle_attitude)) {
			const Vector3f earth_z_in_body_frame = matrix::Quatf(vehicle_attitude.q).rotateVectorInverse(Vector3f(0.f, 0.f, 1.f));
			_cos_tilt = _tailsitter ? earth_z_in_body_frame(0) : earth_z_in_body_frame(2);
		}
	}

	const float dt = (vehicle_torque_setpoint.timestamp - _timestamp_last) * 1e-6f;
	_timestamp_last = vehicle_torque_setpoint.timestamp;

	const hrt_abstime now = hrt_absolute_time();
	const Vector3f torque{vehicle_torque_setpoint.xyz};
	state state_prev = _state;

	const bool run_auto_trim = _fixed_wing
				   && _armed
				   && !_landed
				   && (dt > 0.001f) && (dt < 1.f)
				   && torque.isAllFinite()
				   && _airspeed >= _param_fw_airspd_min.get()
				   && _airspeed <= _param_fw_airspd_max.get()
				   && _cos_tilt > cosf(math::radians(_kTiltMaxDeg));

	if (run_auto_trim) {
		switch (_state) {
		default:

		// fallthrough
		case state::idle:
			_trim_estimate.reset();
			_trim_test.reset();
			_state = state::sampling;
			_state_start_time = now;
			break;

		case state::sampling:
			_trim_estimate.update(torque);

			if ((now - _state_start_time) > 5_s) {
				_state = state::sampling_test;
				_state_start_time = now;
			}

			break;

		case state::sampling_test:
			_trim_test.update(torque);

			if ((now - _state_start_time) > 2_s) {
				_state = state::verification;
				_state_start_time = now;
			}

			break;

		case state::verification: {
				// Perform a statistical test between the estimated and test data
				const float gate = 5.f;
				const Vector3f innovation = _trim_test.mean() - _trim_estimate.mean();
				const Vector3f innovation_variance = _trim_test.variance() + _trim_test.variance();
				const float test_ratio = Vector3f(innovation.edivide(innovation_variance * gate * gate)) * innovation;

				if ((test_ratio <= 1.f) && !innovation.longerThan(0.05f)) {
					_trim_validated = _trim_estimate.mean();
					_state = state::complete;
					_state_start_time = now;

				} else {
					_state = state::fail;
				}

			} break;

		case state::complete:
			//TODO: depending on param, stay or restart
			_state = state::idle;
			break;
		}

	} else {
		_state = state::fail;
	}

	if (_state != state_prev) {
		publishStatus(vehicle_torque_setpoint.timestamp_sample);
	}

	perf_end(_cycle_perf);
}

void FwAutoTrim::publishStatus(const hrt_abstime &timestamp_sample)
{
	auto_trim_status_s status_msg{};

	status_msg.timestamp_sample = timestamp_sample;

	_trim_validated.copyTo(status_msg.trim_validated);
	_trim_estimate.mean().copyTo(status_msg.trim_estimate);
	_trim_estimate.variance().copyTo(status_msg.trim_estimate_var);
	_trim_test.mean().copyTo(status_msg.trim_test);
	_trim_test.variance().copyTo(status_msg.trim_test_var);
	status_msg.state = static_cast<int>(_state);

	status_msg.timestamp = hrt_absolute_time();

	_auto_trim_status_pub.publish(status_msg);
}

int FwAutoTrim::task_spawn(int argc, char *argv[])
{
	bool is_vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			is_vtol = true;
		}
	}

	FwAutoTrim *instance = new FwAutoTrim(is_vtol);

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

int FwAutoTrim::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FwAutoTrim::print_status()
{
	perf_print_counter(_cycle_perf);
	printf("Status: %d\n", static_cast<int>(_state));
	printf("Trim validated\n");
	_trim_validated.print();

	return 0;
}

int FwAutoTrim::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_auto_trim", "auto-trim");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_auto_trim_main(int argc, char *argv[])
{
	return FwAutoTrim::main(argc, argv);
}
