
void runFlightControl(long &rcthr, long &rcpit, long &rcroll, long &rcyaw, float &desired_alt,
					long alt_output, float alt, float climb_rate, float accelZ, uint16_t channels[]) {

	// TODO when ready, use autonomousFollowMode(rcthr, rcpit, rcroll, rcyaw, alt_output)
	// TODO add switch functionality for autonomous land

	if 	(switchState == AUTO_PERFORMANCE) {
		if (autopilotState == TAKEOFF) {		autonomousTakeoffMode(rcthr, rcpit, rcroll, rcyaw, desired_alt, alt, alt_output, channels);}
		else if (autopilotState == ALT_HOLD) {	semiautonomousAltitudeHoldMode(rcthr, rcpit, rcroll, rcyaw, alt_output, channels);}
		else if (autopilotState == LAND) {		autonomousLandMode(rcthr, rcpit, rcroll, rcyaw, climb_rate, accelZ, channels);}
	}

	else if (switchState == MANUAL) {			manualFlightMode(rcthr, rcpit, rcroll, rcyaw, channels);}
	else if (switchState == AUTO_ALT_HOLD) {	semiautonomousAltitudeHoldMode(rcthr, rcpit, rcroll, rcyaw, alt_output, channels);}
}

void manualFlightMode(long &rcthr, long &rcpit, long &rcroll, long &rcyaw,
				   uint16_t channels[]) {

	if(ESC_CALIBRATION) 		{rcthr = map(channels[2], RC_THR_MIN, RC_THR_MAX, ESC_CAL_MIN, ESC_CAL_MAX);}
	else 						{rcthr = getRcThrottle(channels);}

	if(autopilotState == THROTTLE_ASSIST) {
		rcthr = map(rcthr, RC_THR_MIN_MAPPED, rcthrAtSwitch, RC_THR_MIN_MAPPED, Static_HOVER_THR);
	}

	rcthr = constrain(rcthr, RC_THR_MIN_MAPPED, RC_THR_MAX_MAPPED);

	// GET RC pitch, roll, and yaw
	rcpit = map(channels[0], RC_ROL_MIN, RC_ROL_MAX, 45, -45);
	rcroll = map(channels[1], RC_PIT_MIN, RC_PIT_MAX, 45, -45);
	rcyaw = map(channels[3], RC_YAW_MIN, RC_YAW_MAX, -180, 180);
}

void autonomousTakeoffMode(long &rcthr, long &rcpit, long &rcroll, long &rcyaw,
					   float desired_alt, float alt, long alt_output, uint16_t channels[]) {

	// TODO improve autonomousTakeoffMode and give it rcpit, rcroll = 0

	long MAX_TAKEOFF_THR = HOVER_THR + 15;
	long MIN_TAKEOFF_THR = HOVER_THR - 15;

	if (alt < (desired_alt/2)) {
		rcthr = MAX_TAKEOFF_THR;
	} else if (alt < desired_alt)	{
		rcthr = map(alt, desired_alt/2, desired_alt, MAX_TAKEOFF_THR, MIN_TAKEOFF_THR);
	} else {
		// Otto is above desired_alt. Reset PID integral for altitude hold.
		// pids[ALT_STAB].reset_I();

		autopilotState = ALT_HOLD;
	}

	// rcpit = 0;
	// rcroll = 0;
	rcpit = map(channels[0], RC_ROL_MIN, RC_ROL_MAX, 45, -45);
	rcroll = map(channels[1], RC_PIT_MIN, RC_PIT_MAX, 45, -45);
	controlHeadingHold(rcyaw);
}

void autonomousLandMode( long &rcthr, long &rcpit, long &rcroll, long &rcyaw, float climb_rate, float accelZ, uint16_t channels[] ){
	
	rcthr = HOVER_THR - 5 * throttle_modifier;
	
	if( hal.scheduler->micros() - land_timer > land_interval ){
		land_total += accelZ;
		land_counter++;
		land_average = (land_total / land_counter);
		
		// hal.console->print( "\t\t\t\t\t\t\t\t\t Total : ");
		// hal.console->print( land_total );
		// hal.console->print( ", counter : " );
		// hal.console->print( land_counter );
		// hal.console->print( ", average : " );
		// hal.console->println( land_average );
		
		if( land_counter > 25 && ( land_average >= (-9.81) && land_average <= (-9.80)) ){
			//hal.console->println( "---------------REDUCE MOTOR SPEED---------------" );
			//rcthr = HOVER_THR - 5 * throttle_modifier;
			throttle_modifier++;
			
			land_total = 0;
			land_counter = 0;
			land_average = 0;
			land_timer = hal.scheduler->micros();
			land_interval = 2500000;
			
		}
		if( land_counter > 50 ){
			land_total = 0;
			land_counter = 0;
			land_average = 0;
		}
	}
	
	if( throttle_modifier > 10 ){
		//hal.console->println( "---------------MOTORS KILLED---------------" );
		droneOff();
	}
		
	// hal.console->print( "LAND ---> Climb rate : " );
	// hal.console->print( climb_rate );
	// hal.console->print( ", Z Accel : " );
	// hal.console->print( accelZ );
	// hal.console->print( ", RCTHR : " );
	// hal.console->println( rcthr );

	//hal.scheduler->delay(100);
	
	// rcpit = 0;
	// rcroll = 0;
	rcpit = map(channels[0], RC_ROL_MIN, RC_ROL_MAX, 45, -45);
	rcroll = map(channels[1], RC_PIT_MIN, RC_PIT_MAX, 45, -45);
	controlHeadingHold(rcyaw);
	
}
	
void semiautonomousAltitudeHoldMode(long &rcthr, long &rcpit, long &rcroll, long &rcyaw,
								long alt_output, uint16_t channels[]) {

	controlAltitudeHold(rcthr, alt_output);
	rcpit = map(channels[0], RC_ROL_MIN, RC_ROL_MAX, 45, -45);
	rcroll = map(channels[1], RC_PIT_MIN, RC_PIT_MAX, 45, -45);
	controlHeadingHold(rcyaw);
}

void autonomousFollowMode(long &rcthr, long &rcpit, long &rcroll, long &rcyaw,
					  long alt_output) {

	controlAltitudeHold(rcthr, alt_output);
	controlGpsTracking(rcpit, rcroll);
	controlHeadingHold(rcyaw);
}


void controlGpsTracking(long &rcpit, long &rcroll) {
	float current_heading_rad;
	//Vector format is x,y,z
	Vector3f lat_long_error, autonomous_pitch_roll;
	Matrix3f yaw_rotation_m;
	int32_t drone_coordinates[] = {0, 0};
	int32_t target_coordinates[] = {0, 0};

	if (gps->status() < 2) {
		///PID Feedback system for pitch and roll input 0 is bad GPS state
		rcpit = 0;
		rcroll = 0;
		return;
	}

	getTargetCoordinates(target_coordinates, GPS_TARGET);
	getDroneCoordinates(drone_coordinates);

	//Get Lat and Long error
	lat_long_error.x = (float)((target_coordinates[0] - drone_coordinates[0])*INT_LONG_TO_METER);
	lat_long_error.y = (float)((target_coordinates[1] - drone_coordinates[1])*INT_LAT_TO_METER);
	lat_long_error.z = 0;

	/*
	Generic Matrix Setup
	----           ----
	|  a.x  a.y  a.z  |
	|  b.x  b.y  b.z  |
	|  c.x  c.y  c.z  |
	----           ----
	*/

	//Get the radian representation of current_heading
	current_heading_rad = current_heading*PI/180;

	yaw_rotation_m.a = Vector3f(cos(current_heading_rad), sin(current_heading_rad), 0);
	yaw_rotation_m.b = Vector3f(-sin(current_heading_rad), cos(current_heading_rad), 0);
	yaw_rotation_m.c = Vector3f(0, 0, 1);

	//Multiply the lat_long_error matrix by the yaw rotation matrix to get pitch / roll proportions
	autonomous_pitch_roll = yaw_rotation_m*lat_long_error;

	//PID Feedback system for pitch and roll.
	rcpit = constrain(pids[PITCH_CMD].get_pid(autonomous_pitch_roll.y, 1), -5, 5);
	rcpit = -rcpit;		// flip rcpit for proper mapping (neg pitch is forward)
	rcroll = constrain(pids[ROLL_CMD].get_pid(autonomous_pitch_roll.x, 1), -5, 5);

	if (PRINT_DEBUG) {
		// hal.console->print("Yaw Rotation Matrix:  ");
		// hal.console->printf("a: %f %f %f b: %f %f %f    ", yaw_rotation_m.a.x, yaw_rotation_m.a.y, yaw_rotation_m.a.z, yaw_rotation_m.b.x, yaw_rotation_m.b.y, yaw_rotation_m.b.z);
		// hal.console->printf(", p/r: %f %f  ", autonomous_pitch_roll.x, autonomous_pitch_roll.y);

		// hal.console->printf(", gps status: %d", gps->status());
		// hal.console->printf(", currheading, %f, ", current_heading);
		// hal.console->print(", desired_heading, ");
		// hal.console->print(desired_heading);
		// hal.console->printf(",  drone_long, %ld, drone_lat, %ld, ", drone_coordinates[0], drone_coordinates[1]);
		// hal.console->printf(", target_long, %ld, target_lat, %ld, ", target_coordinates[0], target_coordinates[1]);
		// hal.console->printf(",  diff_long, %f, diff_lat, %f, ", lat_long_error.x, lat_long_error.y);
		// hal.console->print(",  desired heading, ");
		// hal.console->print(desired_heading);
		// hal.console->print(", seperation, ");
		// hal.console->print(seperation_dist);
		// hal.console->print(", accuracy, ");
		// hal.console->print(gps->horizontal_accuracy/1000.0);
	}

	/* ~~~~~ USE FOR SEPERATION DISTANCE ~~~~~~~~
	float gps_distance = get_distance(&loc, &loc2);
	hal.console->printf("\narducopter bearing: %f and distance: %f\n", bearing_heading, gps_distance);
	*/
}

void controlHeadingHold(long &rcyaw) {
//Compass accumulate should be called frequently to accumulate readings from the compass
	compass.accumulate();

	if((hal.scheduler->micros() - heading_timer) > 100000L){		// Run loop @ 10Hz ~ 100ms
		heading_timer = hal.scheduler->micros();
		current_heading = getHeading();
	}

	//desired_heading = getBearing();

	//Calculate the Heading error and use the PID feedback loop to translate that into a yaw input
	float heading_error = wrap_180(desired_heading - current_heading);
	rcyaw = constrain(pids[YAW_CMD].get_pid(heading_error, 1), -10, 10);
	rcyaw = rcyaw * -1;
}

void controlAltitudeHold(long &rcthr,
					 long alt_output) {

	rcthr = HOVER_THR + alt_output;
	rcthr = constrain(rcthr, RC_THR_MIN_MAPPED, RC_THR_MAX_MAPPED);
}

void adjustThrottleForBatteryLevel() {
	uint32_t delay;
	if (autopilotState == TAKEOFF) {
		delay = 200000UL;												// delay every 0.2 seconds when performing autonomous takeoff
	} else {
		delay = 2000000UL;												// delay every 2 seconds under normal operating conditions
	}

	if((hal.scheduler->micros() - hover_thr_timer) > delay) {
		hover_thr_timer = hal.scheduler->micros();

		float voltage = battery_mon.voltage();
		float new_hover_thr = map(voltage, 10, 11.8, ADJ_THR_MAX, ADJ_THR_MIN);		// map HOVER_THR based on voltage of drone in flight
		HOVER_THR = constrain(new_hover_thr, ADJ_THR_MIN_CONSTRAINT, ADJ_THR_MAX_CONSTRAINT);			// constrain hover throttle for saftey
	}
}

long getRcThrottle(uint16_t channels[]) {
	return map(channels[2], RC_THR_MIN, RC_THR_MAX, RC_THR_MIN_MAPPED, RC_THR_MAX_MAPPED);
}
