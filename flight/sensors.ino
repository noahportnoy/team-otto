
void updateReadings(uint16_t channels[], long &safety,
					float &accelPitch, float &accelRoll, float &accelYaw,
					float &gyroPitch, float &gyroRoll, float &gyroYaw,
					float &alt, float &climb_rate, float &accelZ, float &AVG_OFF_BUTTON_VALUE) {

	// Wait until new orientation data (normally 5ms max)
	while(ins.num_samples_available() == 0);

	uartMessaging.receive();

	battery_mon.read();													// Get battery stats: update voltage and current readings
	float instantaneousVoltage = battery_mon.voltage();
	batteryVoltage = movingAvg(batteryVoltage, instantaneousVoltage, 0.7);

	hal.rcin->read(channels, 8);
	safety = channels[4];
	AVG_OFF_BUTTON_VALUE = OFF_BUTTON_VALUE->voltage_average();
	getAltitudeData(alt, climb_rate);
	getAccel(accelPitch, accelRoll, accelYaw, accelZ);
	getGyro(gyroPitch, gyroRoll, gyroYaw);

	updateDroneCoordinates();
	updateTargetCoordinates(GPS_TRACKING_TARGET);

	if(PRINT_DEBUG) {
		// hal.console->printf("\nVoltage: %.2f \tCurrent: %.2f \tTotCurr:%.2f  ",
		// battery_mon.voltage(), //voltage
		// battery_mon.current_amps(), //Inst current
		// battery_mon.current_total_mah()); //Accumulated current
	}
}

void getAccel(float &accelPitch, float &accelRoll, float &accelYaw, float &accelZ) {
	float trim_roll = -0.081;
	float trim_pitch = 0.145;

	ins.update();

	ins.quaternion.to_euler(&accelRoll, &accelPitch, &accelYaw);		// Ask MPU6050 for orientation

	Matrix3f temp;
	temp.from_euler(accelRoll, accelPitch, accelYaw);
	temp.rotate(Vector3f(trim_roll, trim_pitch, 0));
	temp.to_euler(&accelRoll, &accelPitch, &accelYaw);

	accelPitch = ToDeg(accelPitch);
	accelRoll = ToDeg(accelRoll);
	accelYaw = ToDeg(accelYaw);

	Vector3f accel = ins.get_accel();
	accelZ = (accel.z) / ((cos(ToRad(accelPitch))*cos(ToRad(accelRoll))));

	// ahrs.update();

	// accelPitch = ahrs.pitch;
	// accelRoll = ahrs.roll;
	// accelYaw = ahrs.yaw;

	// Matrix3f temp;
	// temp.from_euler(accelRoll, accelPitch, accelYaw);
	// temp.rotate(Vector3f(trim_roll, trim_pitch, 0));
	// temp.to_euler(&accelRoll, &accelPitch, &accelYaw);

	// accelPitch = ToDeg(accelPitch);
	// accelRoll = ToDeg(accelRoll);
	// accelYaw = ToDeg(accelYaw);

	// Vector3f accel = ins.get_accel();
	// accelZ = accel.z;
}

void getGyro(float &gyroPitch, float &gyroRoll, float &gyroYaw) {
	Vector3f gyro = ins.get_gyro();										// Ask MPU6050 for gyro data

	gyroPitch = ToDeg(gyro.y);
	gyroRoll = ToDeg(gyro.x);
	gyroYaw = ToDeg(gyro.z);
}

float getHeading() {
	/*COMPASS OPERATION:
		 True North is 0 (degrees)
		 Eastern headings are negative numbers
		 Western headings are positive numbers
		 South is 180 or -180
	*/

	//Use AHRS for Heading
	ahrs.update();
	compass.read();
	float heading = compass.calculate_heading(ahrs.get_dcm_matrix());
	//Vector3f drift  = ahrs.get_gyro_drift();
	compass.null_offsets();

	//NOTE: AHRS can provide pitch, roll, and yaw angles

	current_heading = ToDeg(heading);
	current_heading = -current_heading; 	// correct for proper handling in the rotation matrix

	return current_heading;
}

float getAltitude() {
	baro.read();
	if (!baro.healthy) {
		hal.console->println("not healthy");
		return 0.0;
	}

	float a = baro.get_altitude();
	return a;
}

float getClimbRate() {
	baro.read();
	if (!baro.healthy) {
		hal.console->println("not healthy");
		return 0.0;
	}
	return (baro.get_climb_rate());
}

void getAltitudeData(float &alt, float &climb_rate) {
	if( (hal.scheduler->micros() - altitude_timer) > 100000UL ) {				// Update altitude data on altitude_timer
		float last_alt = alt;
		alt = getAltitude();

		//Smooth raw data (last parameter is the smoothing constant)
		//alt = movingAvg(last_alt, alt, .5);

		//Verify that the altitude values are within scope
		if(abs(alt-last_alt) > 10){
			alt=last_alt;
		}

		//Get Climb Rate and smooth (last parameter is the smoothing constant)
		float last_climb_rate = climb_rate;
		climb_rate = constrain(getClimbRate(), -15, 15);
		climb_rate = movingAvg(last_climb_rate, climb_rate, .9);

		//Verify that the climb rate values are within scope
		if(abs(climb_rate-last_climb_rate) > 100){
			climb_rate=last_climb_rate;
		}

		//Scheduling
		altitude_timer = hal.scheduler->micros();

		if(PRINT_DEBUG) {
			// hal.console->printf("\nVoltage: %.2f \tCurrent: %.2f \tTotCurr:%.2f",
			// battery_mon.voltage(),
			// battery_mon.current_amps(),
			// battery_mon.current_total_mah());
		}
	}
}
