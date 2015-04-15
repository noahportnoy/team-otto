
void writeToMotors(long &rcthr, long &pitch_output, long &roll_output, long &yaw_output, float &yaw_target,
				   float accelYaw) {
	if(ESC_CALIBRATION) {
		// Calibrate the ESCs using constant throttle across all four motors
		hal.rcout->write(MOTOR_FL, rcthr);
		hal.rcout->write(MOTOR_BL, rcthr);
		hal.rcout->write(MOTOR_FR, rcthr);
		hal.rcout->write(MOTOR_BR, rcthr);

	} else if (rcthr < RC_THR_MIN + 50) {
		rcthr = 1000;
		droneOff();
		yaw_target = accelYaw;

	} else if (PRINT_DEBUG != 0) {
		rcthr = 1000;
		droneOff();
		yaw_target = accelYaw;

	} else if (autopilotState == OFF) {
		rcthr = 1000;
		droneOff();
		yaw_target = accelYaw;											// reset yaw target so we maintain this on takeoff

	} else {
		// Throttle raised, turn on motors.

		// adjust HOVER_THR based on battery voltage
		if (rcthr > ADJ_THR_THRESHOLD) {adjustThrottleForBatteryLevel();}

		// mix pid outputs
		long MOTOR_FL_output = rcthr + roll_output + pitch_output - yaw_output;
		long MOTOR_BL_output = rcthr + roll_output - pitch_output + yaw_output;
		long MOTOR_FR_output = rcthr - roll_output + pitch_output + yaw_output;
		long MOTOR_BR_output = rcthr - roll_output - pitch_output - yaw_output;

		// gain to simulate an ESC calibration
		if (CAL_COMP_TYPE == SW_CAL || CAL_COMP_TYPE == MIXED) {
			MOTOR_FL_output = map(MOTOR_FL_output, SW_CAL_MIN, SW_CAL_MAX, ESC_CAL_MIN, ESC_CAL_MAX);
			MOTOR_BL_output = map(MOTOR_BL_output, SW_CAL_MIN, SW_CAL_MAX, ESC_CAL_MIN, ESC_CAL_MAX);
			MOTOR_FR_output = map(MOTOR_FR_output, SW_CAL_MIN, SW_CAL_MAX, ESC_CAL_MIN, ESC_CAL_MAX);
			MOTOR_BR_output = map(MOTOR_BR_output, SW_CAL_MIN, SW_CAL_MAX, ESC_CAL_MIN, ESC_CAL_MAX);
		}

		// send outputs to the motors
		hal.rcout->write(MOTOR_FL, MOTOR_FL_output);
		hal.rcout->write(MOTOR_BL, MOTOR_BL_output);
		hal.rcout->write(MOTOR_FR, MOTOR_FR_output);
		hal.rcout->write(MOTOR_BR, MOTOR_BR_output);
	}
}

void droneOff() {
	hal.rcout->write(MOTOR_FL, 1000);
	hal.rcout->write(MOTOR_BL, 1000);
	hal.rcout->write(MOTOR_FR, 1000);
	hal.rcout->write(MOTOR_BR, 1000);

	// reset PID integrals
	for(int i=0; i<10; i++) {
		pids[i].reset_I();
	}
}
