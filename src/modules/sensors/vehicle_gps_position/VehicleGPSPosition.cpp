/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "VehicleGPSPosition.hpp"

#include <px4_platform_common/log.h>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/mathlib.h>

#include <systemlib/mavlink_log.h>
#include <inttypes.h>
#include <cstdlib>

namespace sensors
{
VehicleGPSPosition::VehicleGPSPosition() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

VehicleGPSPosition::~VehicleGPSPosition()
{
	Stop();
	perf_free(_cycle_perf);
}

bool VehicleGPSPosition::Start()
{
	// force initial updates
	ParametersUpdate(true);

	ScheduleNow();

	return true;
}

void VehicleGPSPosition::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_gps_sub) {
		sub.unregisterCallback();
	}
}

void VehicleGPSPosition::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		if (_param_sens_gps_mask.get() == 0) {
			_sensor_gps_sub[0].registerCallback();

		} else {
			for (auto &sub : _sensor_gps_sub) {
				sub.registerCallback();
			}
		}

		_gps_blending.setBlendingUseSpeedAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_SPD_ACC);
		_gps_blending.setBlendingUseHPosAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_HPOS_ACC);
		_gps_blending.setBlendingUseVPosAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_VPOS_ACC);
		_gps_blending.setBlendingTimeConstant(_param_sens_gps_tau.get());
		_gps_blending.setPrimaryInstance(_param_sens_gps_prime.get());
	}
}

void VehicleGPSPosition::Run()
{
	perf_begin(_cycle_perf);
	ParametersUpdate();

	// GPS blending
	ScheduleDelayed(500_ms); // backup schedule

	// Check all GPS instance
	bool any_gps_updated = false;
	bool gps_updated = false;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		gps_updated = _sensor_gps_sub[i].updated();

		sensor_gps_s gps_data;

		if (gps_updated) {
			any_gps_updated = true;

			_sensor_gps_sub[i].copy(&gps_data);
			_gps_blending.setGpsData(gps_data, i);

			if (!_sensor_gps_sub[i].registered()) {
				_sensor_gps_sub[i].registerCallback();
			}
		}
	}

	if (any_gps_updated) {
		_gps_blending.update(hrt_absolute_time());

		if (_gps_blending.isNewOutputDataAvailable()) {
			Publish(_gps_blending.getOutputGpsData(), _gps_blending.getSelectedGps());
		}
	}

	perf_end(_cycle_perf);
}

unsigned int gps_print_cnt;
unsigned int gps_measure_noise;
unsigned int gps_measure_noise_cnt;
unsigned int gps_measure_agc;
unsigned int gps_measure_agc_cnt;
uint64_t gps_last_measured_time;
bool gps_noise_set = false;
bool gps_noise_param_init = false;
bool gps_noise_measure = false;
bool gps_agc_measure = false;
int gps_time_cnt;

void VehicleGPSPosition::CheckGPSSpoofing(const sensor_gps_s &gps)
{
	static orb_advert_t mavlink_log_pub = nullptr;

	// Step 1. Initialize the parameter value related to the GPS noise level
	if ( (gps_noise_param_init == false) && (gps_noise_set == false) ) {
		_param_gps_noise_base.set(0);
		_param_gps_noise_base.commit();
		_param_gps_spoofing.set(0);
		_param_gps_spoofing.commit();

		gps_noise_param_init = true;
	}

	// Step 2. Measure baseline noise level under the normal condition
	// Let's start to measure the baseline after time specified in 'GPS_NOISE_TIME' parameter
	// because the GPS noise level is not stable during the GPS receiver's booting process
	if ( (gps_noise_measure == true) && (gps_noise_set == false) && (_param_gps_noise_base.get() == 0) ) {
		gps_measure_noise += gps.noise_per_ms;
		gps_measure_noise_cnt++;
	}

	gps_print_cnt++;

	// Step 3. Let's store the measured baseline noise level
	if (gps_print_cnt > _param_gps_noise_time.get()) {
		gps_print_cnt = 0;

		if ( (gps_noise_set == false) && (gps_noise_measure == true) ){
			gps_noise_set = true;
			int baseline_noise_level = gps_measure_noise/gps_measure_noise_cnt;

			mavlink_log_info(&mavlink_log_pub, "Measured baseline noise level: %d (%d/%d)", baseline_noise_level, gps_measure_noise, gps_measure_noise_cnt);

			_param_gps_noise_base.set(baseline_noise_level);
			_param_gps_noise_base.commit();
		}

		gps_noise_measure = true;
	}

	gps_measure_agc += gps.automatic_gain_control;
	gps_measure_agc_cnt++;

	// Step 4. Let's check whether there is a GPS spoofing attack every 'GPS_AGC_TIME'.
	if (gps_measure_agc_cnt > _param_gps_agc_time.get()) {

		int agc_avg = gps_measure_agc/gps_measure_agc_cnt;
		_param_gps_agc_avg.set(agc_avg);
		_param_gps_agc_avg.commit();

		mavlink_log_info(&mavlink_log_pub, "[DEBUG] Noise: %d, AGC: %d, AGC_avg: %d", gps.noise_per_ms, gps.automatic_gain_control, agc_avg);
		//mavlink_log_info(&mavlink_log_pub, "[DEBUG] last_t: %llu, cur_t: %llu (ms)", gps_last_measured_time/1000, gps.time_utc_usec/1000);

		gps_measure_agc = 0;
		gps_measure_agc_cnt = 0;

		// Step 5. If the current noise level is above this threshold value from the baseline
		if ( (gps_noise_set == true) && gps.noise_per_ms > (_param_gps_noise_threshold.get() + _param_gps_noise_base.get()) ) {

			// If the current AGC is smaller than the moving average of AGC, we conclude that GPS spoofing attack is ongoing.
			// Why? GPS spoofing attacks make (1) the noise level increase and (2) the AGC decrease
			if ( gps.automatic_gain_control <= _param_gps_agc_avg.get() ) {
				mavlink_log_info(&mavlink_log_pub, "[WARNING] Noise_t:%d, Noise_baseline: %d, AGC_t: %d, AGC_avg: %d", gps.noise_per_ms, _param_gps_noise_base.get(), gps.automatic_gain_control, _param_gps_agc_avg.get());
				mavlink_log_info(&mavlink_log_pub, "[WARNING] Detect a GPS spoofing attack");

				// Step 6: Change the 'GPS_SPOOFING' parameter value to trigger a GPS failsafe
				_param_gps_spoofing.set(1);
				_param_gps_spoofing.commit();
			}
		}
	}

	// Step 7. Let's check a GPS spoofing attack by detecting time jump.
	// (Reference) https://gpspatron.com/spoofing-attacks-chapter-2/#:~:text=PPS%20monitoring%20with%20time%20server
	//
	// (How?) We can compare the internal time with the time determined by the navigation module (GPS receiver).
	// A severe time jump can point to the presence of the GPS spoofing.

	// Step 7-1: When PX4 first gets the time from the GPS receiver.

	//mavlink_log_info(&mavlink_log_pub, "[DEBUG] last_t: %llu, cur_t: %llu", gps_last_measured_time/1000, gps.time_utc_usec/1000);
	if (gps_last_measured_time == 0) {
		gps_last_measured_time = gps.time_utc_usec;
	}
	// Step 7-2: Check the time jump
	else {
		if ( (abs(gps.time_utc_usec - gps_last_measured_time)/1000) > _param_gps_time_threshold.get() ) {
			mavlink_log_info(&mavlink_log_pub, "[WARNING] Detect a GPS spoofing attack");
			mavlink_log_info(&mavlink_log_pub, "[WARNING] last_t: %llu, cur_t: %llu (ms)", gps_last_measured_time/1000, gps.time_utc_usec/1000);

			// Step 7-3: Change the 'GPS_SPOOFING' parameter value to trigger a GPS failsafe
			_param_gps_spoofing.set(1);
			_param_gps_spoofing.commit();
		}
		else {
			gps_last_measured_time = gps.time_utc_usec;
		}
	}

}

void VehicleGPSPosition::Publish(const sensor_gps_s &gps, uint8_t selected)
{
	CheckGPSSpoofing(gps);

	vehicle_gps_position_s gps_output{};

	gps_output.timestamp = gps.timestamp;
	gps_output.time_utc_usec = gps.time_utc_usec;
	gps_output.lat = gps.lat;
	gps_output.lon = gps.lon;
	gps_output.alt = gps.alt;
	gps_output.alt_ellipsoid = gps.alt_ellipsoid;
	gps_output.s_variance_m_s = gps.s_variance_m_s;
	gps_output.c_variance_rad = gps.c_variance_rad;
	gps_output.eph = gps.eph;
	gps_output.epv = gps.epv;
	gps_output.hdop = gps.hdop;
	gps_output.vdop = gps.vdop;
	gps_output.noise_per_ms = gps.noise_per_ms;
	gps_output.jamming_indicator = gps.jamming_indicator;
	gps_output.jamming_state = gps.jamming_state;
	gps_output.vel_m_s = gps.vel_m_s;
	gps_output.vel_n_m_s = gps.vel_n_m_s;
	gps_output.vel_e_m_s = gps.vel_e_m_s;
	gps_output.vel_d_m_s = gps.vel_d_m_s;
	gps_output.cog_rad = gps.cog_rad;
	gps_output.timestamp_time_relative = gps.timestamp_time_relative;
	gps_output.heading = gps.heading;
	gps_output.heading_offset = gps.heading_offset;
	gps_output.fix_type = gps.fix_type;
	gps_output.vel_ned_valid = gps.vel_ned_valid;
	gps_output.satellites_used = gps.satellites_used;

	gps_output.selected = selected;

	_vehicle_gps_position_pub.publish(gps_output);
}

void VehicleGPSPosition::PrintStatus()
{
	//PX4_INFO("selected GPS: %d", _gps_select_index);
}

}; // namespace sensors
