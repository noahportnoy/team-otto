
void setPidConstants(int config) {
	if (config == DEFAULT) {
		//Attidude PID's - uses MPU
		pids[PID_PITCH_STAB].kP(6);
		pids[PID_PITCH_STAB].kI(3);
		pids[PID_PITCH_STAB].imax(50);

		pids[PID_ROLL_STAB].kP(6);
		pids[PID_ROLL_STAB].kI(3);
		pids[PID_ROLL_STAB].imax(50);

		pids[PID_YAW_STAB].kP(10);
		pids[PID_YAW_STAB].kI(0);
		pids[PID_YAW_STAB].imax(10);

		//Rate PID's - uses Gyro
		pids[PID_PITCH_RATE].kP(0.2);
		pids[PID_PITCH_RATE].kI(0.0);
		pids[PID_PITCH_RATE].imax(50);

		pids[PID_ROLL_RATE].kP(0.2);
		pids[PID_ROLL_RATE].kI(0.0);
		pids[PID_ROLL_RATE].imax(50);

		pids[PID_YAW_RATE].kP(0.7);
		pids[PID_YAW_RATE].kI(0.1);
		pids[PID_YAW_RATE].imax(50);

		//Below are the PIDs for altitude hold
		pids[ALT_STAB].kP(8);
		pids[ALT_STAB].kI(1.5);
		pids[ALT_STAB].kD(9);
		pids[ALT_STAB].imax(20);

		//Below are the PIDs for autonomous control
		pids[PITCH_CMD].kP(0.6);
		pids[PITCH_CMD].kI(0.0);
		pids[PITCH_CMD].imax(50);

		pids[ROLL_CMD].kP(0.6);
		pids[ROLL_CMD].kI(0.0);
		pids[ROLL_CMD].imax(50);

		pids[YAW_CMD].kP(0.7);
		pids[YAW_CMD].kI(0.1);
		pids[YAW_CMD].imax(50);
		
		pids[LAND].kP(0.6);
		pids[LAND].kI(0.1);
		pids[LAND].imax(50);
		
	} else {
		hal.console->print("Error: PID constants not set for provided configuration");
		hal.console->println(config);
		while(1);
	}
}

void runPidFeedback(long &pitch_output, long &roll_output, long &yaw_output, long &alt_output, float &yaw_target,
				 long rcpit, long rcroll, long rcyaw,
				 float accelPitch, float accelRoll, float accelYaw,
				 float gyroPitch, float gyroRoll, float gyroYaw,
				 float alt, float desired_alt) {
	// Stablize PIDS
	float pitch_stab_output = constrain(pids[PID_PITCH_STAB].get_pid((float)rcpit - accelPitch, 1), -250, 250);
	float roll_stab_output = constrain(pids[PID_ROLL_STAB].get_pid((float)rcroll - accelRoll, 1), -250, 250);
	float yaw_stab_output = constrain(pids[PID_YAW_STAB].get_pid(yaw_target - accelYaw, 1), -360, 360);
	float appliedPidGain;

	// is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
	if(abs(rcyaw ) > 5) {
		yaw_stab_output = rcyaw;
		yaw_target = accelYaw;   // remember this yaw for when pilot stops
	}

	if (CAL_COMP_TYPE == PID_GAIN || CAL_COMP_TYPE == MIXED)	{appliedPidGain = PID_GAIN_VAL;}
	else														{appliedPidGain = 1;}

	// rate PIDS
	pitch_output =  (long) appliedPidGain * constrain(pids[PID_PITCH_RATE].get_pid(pitch_stab_output - gyroPitch, 1), -500, 500);
	roll_output =  (long) appliedPidGain * constrain(pids[PID_ROLL_RATE].get_pid(roll_stab_output - gyroRoll, 1), -500, 500);
	yaw_output =  (long) appliedPidGain * constrain(pids[PID_YAW_RATE].get_pid(yaw_stab_output - gyroYaw, 1), -500, 500);
	alt_output = (long) appliedPidGain * constrain(pids[ALT_STAB].get_pid(desired_alt - alt, 1), -250, 250);
}
