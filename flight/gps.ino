
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
		uartMessaging.getUserLon(&target_coordinates[0]);
		uartMessaging.getUserLat(&target_coordinates[1]);
  	}
}

void getFixedCoordinates(int32_t target_coordinates[]) {
	//Quad coordinates
	target_coordinates[1] = 423935390;
	target_coordinates[0] = -725294520;

	//target_coordinates[1] = 423945860;
	//target_coordinates[0] = -725291860;

	//target_coordinates[1] = 423942870;
	//target_coordinates[0] = -725294590;
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

	if (gps->status() < 2) {
		// Force the drone to north if there is a GPS loss
		bearing = current_heading;
		return bearing;
	}

	/*  COMPASS OPERATION:
	True North is 0 (degrees)
	Eastern headings are negative numbers
	Western headings are positive numbers
	South is 180 or -180  */

	getTargetCoordinates(user_coordinates, PHONE);
	
	struct Location drone = {0};
	drone.lat = drone_coordinates[1];
	drone.lng = drone_coordinates[0];

	struct Location user = {0};
	user.lat = user_coordinates[1];
	user.lng = user_coordinates[0];

	kalmanFilter(drone.lat, drone.lng);

	//Function returns bearing in centi-degrees
	bearing = -0.01 * get_bearing_cd(&drone_filtered, &user);

	if(PRINT_DEBUG) {
		hal.console->printf(", k_lat, %ld, k_lng, %ld, ",drone_filtered.lat, drone_filtered.lng);
		hal.console->printf(", target_lat, %ld, target_lng, %ld, ", user.lat, user.lng);
		// hal.console->printf(" Drone bearing: %f", bearing);
	}

	return wrap_180(bearing);
}

float getDistanceToUser(){

	getTargetCoordinates(user_coordinates, PHONE);

	struct Location user = {0};
	user.lat = user_coordinates[1];
	user.lng = user_coordinates[0];

	float gps_distance = get_distance(&drone_filtered, &user);
	hal.console->printf(", distance(m), %f, ", gps_distance);
	return gps_distance;
}

//Inputs are the 'noisy' sensor inputs
void kalmanFilter(float gps_lat, float gps_lng){
	/** A simple kalman filter example by Adrian Boeing  www.adrianboeing.com  */  
 
    //the noise in the system 
    float Q = 0.022;			//Process Noise
    float R = 1.0; //0.617;  	//Sensor Noise
     
    float K_1, K_2;
    float P_1, P_2;
    float P_temp_1, P_temp_2;
    float x_temp_est_1, x_temp_est_2;
    float kalman_lat, kalman_lng; 
    

    //FIRST INPUT
    //do a prediction 
    x_temp_est_1 = x_est_last_1; 
    P_temp_1 = P_last_1 + Q; 

    //calculate the Kalman gain 
    K_1 = P_temp_1 * (1.0/(P_temp_1 + R));
    
    //correct 
    kalman_lat = x_temp_est_1 + K_1 * (gps_lat - x_temp_est_1);  
    P_1 = (1- K_1) * P_temp_1; 
    //we have our new system 

    //hal.console->printf(", Mesaured position, %f, ",gps_lat); 
    //hal.console->printf(", Kalman position, %f, \n",kalman_lat); 
     
    //update our last's 
    P_last_1 = P_1; 
    x_est_last_1 = kalman_lat; 
    


    //SECOND INPUT

    //do a prediction 
    x_temp_est_2 = x_est_last_2; 
    P_temp_2 = P_last_2 + Q; 

    //calculate the Kalman gain 
    K_2 = P_temp_2 * (1.0/(P_temp_2 + R));
    
    //correct 
    kalman_lng = x_temp_est_2 + K_2 * (gps_lng - x_temp_est_2);  
    P_2 = (1- K_2) * P_temp_2; 
    //we have our new system 

    //hal.console->printf(", Mesaured position, %f, ",gps_lng); 
    //hal.console->printf(", Kalman position, %f, \n",kalman_lng); 
     
    //update our last's 
    P_last_2 = P_2; 
    x_est_last_2 = kalman_lng; 


    drone_filtered.lat =  kalman_lat;
    drone_filtered.lng =  kalman_lng;

    //TODO make this into an array multiply

}



