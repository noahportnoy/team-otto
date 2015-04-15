
//Coordinate Arrays: [latitude, longitude]
void getDroneCoordinates(int32_t drone_coordinates[]) {
	gps->update();
	if (gps->new_data) {
		drone_coordinates[1] = gps->latitude;
		drone_coordinates[0] = gps->longitude;
	}
}

//Coordinate Arrays: [latitude, longitude]
void getTargetCoordinates(int32_t target_coordinates[], int gpsTarget) {
	if(gpsTarget == PHONE) 			{getPhoneCoordinates(target_coordinates);}
	else if(gpsTarget == FIXED) 	{getFixedCoordinates(target_coordinates);}
}

void getPhoneCoordinates(int32_t target_coordinates[]) {
	if(uartMessaging.isUserLonLatest() && uartMessaging.isUserLatLatest()) {
		uartMessaging.getUserLat(&target_coordinates[1]);
		uartMessaging.getUserLon(&target_coordinates[0]);
	}
}

void getFixedCoordinates(int32_t target_coordinates[]) {
	// In the middle of the farther grassy area, in the engineering quad
	target_coordinates[1] = 423935750;
	target_coordinates[0] = -725293220;

	// Quad coordinates
	// target_coordinates[1] = 423935390;
	// target_coordinates[0] = -725294520;

	// Northern road from outside M5
	// target_coordinates[1] = 423945860;
	// target_coordinates[0] = -725291860;

	// Western road from outside M5
	// target_coordinates[1] = 423942870;
	// target_coordinates[0] = -725294590;
}

void getGPSLock() {
	int counter=0;
	hal.console->println("getting GPS lock");
	gps->update();

	while (gps->status() < 2) {
		gps->update();

		flash_leds(true);
		hal.scheduler->delay(200);
		flash_leds(false);
		hal.scheduler->delay(200);

		//Counter to exit while loop if cannot get GPS coordinate
		counter++;
		hal.console->print(counter);

		//If we get to 20 attempts, then it probably cannot attain coordinates
		if(counter >= 50){
			hal.console->println("\n Cannot attain GPS coordinates. Status code: ");
			hal.console->println(gps->status());
			flash_leds(true);
			hal.scheduler->delay(50);
			flash_leds(false);
			hal.scheduler->delay(50);
			flash_leds(true);
			hal.scheduler->delay(50);
			flash_leds(false);
			hal.scheduler->delay(50);
		}
	}

	hal.console->println(gps->status());
}

float getBearing() {
	float bearing;
	int32_t drone_coordinates[] = {0, 0};
	int32_t target_coordinates[] = {0, 0};

	if (gps->status() < 2) {
		// Force the drone to north if there is a GPS loss
		bearing = current_heading;
		return desired_heading;
	}

	getDroneCoordinates(drone_coordinates);
	getTargetCoordinates(target_coordinates, GPS_TARGET);

	/*COMPASS OPERATION:
		True North is 0 (degrees)
		Eastern headings are negative numbers
		Western headings are positive numbers
		South is 180 or -180
	*/

	struct Location loc = {0};
	loc.lat = drone_coordinates[1] * 1.0e7;
	loc.lng = drone_coordinates[0] * 1.0e7;

	struct Location loc2 = {0};
	loc2.lat = target_coordinates[1] * 1.0e7;
	loc2.lng = target_coordinates[0] * 1.0e7;

	bearing = 0.01 * get_bearing_cd(&loc, &loc2);

	if(PRINT_DEBUG) {
		// hal.console->printf("\n Drone bearing: %f\n", bearing);
	}

	return wrap_180(bearing);
}
