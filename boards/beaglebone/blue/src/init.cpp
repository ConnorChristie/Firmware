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

/**
 * @file init.c
 *
 * BBBLUE specific initialization
 */
#include <stddef.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/shutdown.h>

#include <robotcontrol.h>

#include "board_config.h"

bool rc_cleanup_hook()
{
	PX4_INFO("Cleaning up librobotcontrol ...");

	rc_set_state(EXITING);

	rc_servo_cleanup();
	rc_adc_cleanup();
	rc_led_cleanup();

	rc_remove_pid_file();

	return true;
}

// initialize roboticscape library similar to the deprecated rc_initialize()
int rc_init(void)
{
#ifdef __RC_V0_3
	return rc_initialize();
#else

	if (rc_get_state() == RUNNING) {  return 0; }

	PX4_INFO("Initializing librobotcontrol ...");

	if (rc_kill_existing_process(2.0f) < -2) {
		PX4_ERR("rc_init failed to kill existing processes");
		return -1;
	}

	rc_make_pid_file();

	// Due to device tree issue, rc_pinmux_set_default() currently does not work correctly
	// with kernel 4.14, use a simplified version for now
	//
	// shared pins
	int ret = 0;
	ret |= rc_pinmux_set(DSM_HEADER_PIN, PINMUX_UART);
	ret |= rc_pinmux_set(GPS_HEADER_PIN_3, PINMUX_UART);
	ret |= rc_pinmux_set(GPS_HEADER_PIN_4, PINMUX_UART);
	ret |= rc_pinmux_set(UART1_HEADER_PIN_3, PINMUX_UART);
	ret |= rc_pinmux_set(UART1_HEADER_PIN_4, PINMUX_UART);

	if (ret != 0) {
		PX4_ERR("rc_init failed to set default pinmux");
		return -1;
	}

	if (rc_servo_init()) {
		PX4_ERR("rc_init failed to run rc_servo_init()");
		return -1;
	}

	if (rc_servo_set_esc_range(1000, 2000)) {
		PX4_ERR("rc_init failed to run rc_servo_set_esc_range()");
		return -1;
	}

	if (rc_servo_power_rail_en(0)) {
		PX4_ERR("rc_init failed to run rc_servo_power_rail_en(1)");
		return -1;
	}

	if (rc_adc_init()) {
		PX4_ERR("rc_init failed to run rc_adc_init()");
		return -1;
	}

	// i2c, barometer and mpu will be initialized later

	rc_set_state(RUNNING);

	px4_register_shutdown_hook(&rc_cleanup_hook);

	return 0;
#endif
}

void rc_cleaning(void)
{
	rc_cleanup_hook();
}
