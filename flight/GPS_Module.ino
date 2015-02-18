//GPS / COMPASS Methods

float getHeading(float last_heading){
        //Use AHRS for Heading
        static uint32_t last_t, last_print;
        uint32_t now = hal.scheduler->micros();
        float heading = 0;
    
        ahrs.update();
        if((hal.scheduler->micros() - heading_timer) > 100000L){ //Run loop @ 10Hz
            heading_timer = hal.scheduler->micros();
            
            compass.read();
            heading = compass.calculate_heading(ahrs.get_dcm_matrix());
            gps->update();
            
            Vector3f drift  = ahrs.get_gyro_drift();
            /*
            hal.console->printf_P(
                    PSTR("r:%4.1f  p:%4.1f y:%4.1f "
                        "drift=(%5.1f %5.1f %5.1f) hdg=%.1f\n"),
                            ToDeg(ahrs.roll),
                            ToDeg(ahrs.pitch),
                            ToDeg(ahrs.yaw),
                            ToDeg(drift.x),
                            ToDeg(drift.y),
                            ToDeg(drift.z),
                            compass.use_for_yaw() ? ToDeg(heading) : 2.67767789
            );
            */
            current_heading =  ToDeg(heading);
        }
        //current_heading = movingAvg(last_heading, current_heading, .5);
        return current_heading;
}

float getDesiredHeading(float long_drone, float lat_drone, float long_target, float lat_target){
 
          //Calculate the heading angle of the user relative to the drone.
          float lat_seperation, long_seperation, angle, desired_heading;
          
          //Find the lat & long differences
          if(abs(lat_drone) > abs(lat_target)){
            lat_seperation = abs(lat_drone - lat_target);  
          }else{
            lat_seperation = abs(lat_target - lat_drone);
          }
          
          if(abs(long_drone) > abs(long_target)){
            long_seperation = abs(long_drone - long_target); 
          }else{
            long_seperation = abs(long_target - long_drone);
          }
          
          //Calculate the desired heading of the drone. By using the lattitude as of "opposite" (sohcahTOA) then this will 
          //always calculate an angle from either due N or S
          angle = lat_seperation / long_seperation;
          desired_heading = atan(angle);
          
          /*COMPASS OPERATION: 
             True North is 0 (degrees)
             Eastern headings are negative numbers
             Western headings are positive numbers
             South is 180 or -180
          */
        
          //If the Longitude of the drone is larger than the user than add 90 (degrees)
          if(long_drone > long_target){
             desired_heading = desired_heading + 90;
          }
          
          //If the Lattitude of the user is larger than the drones then the drone should be facing East
          //Convert this to a negative number (details above)
          if(lat_target > lat_drone){
            desired_heading = desired_heading*(-1);
          } 
          
          return desired_heading;
}

//Coordinate Arrays: [longitude, lattitude]
void getDroneCoordinates(float coords[]){
        gps->update();
	if (gps->new_data) { 
            //Check the accuracy of this data first
            //Longitude Check - points cannot be greater than 20 meters apart.
            if (abs(coords[0]) - abs(drone_coordinates[0]) > 0.000250){
                  coords[0] = drone_coordinates[0];
            }else{
                  coords[0] = gps->latitude/10000000.0;
            }
            
            //Lattitude Check - points cannot be greater than 20 meters apart.
            if (abs(coords[1]) - abs(drone_coordinates[1]) > 0.000180){
                  coords[1] = drone_coordinates[1];
            }else{
                  coords[1] = gps->longitude/10000000.0;
            }
            
            
	} else {
            hal.console->print("~~~~~~~~~~~~~~~~  Error. NO NEW GPS DATA!  ~~~~~~~~~~~~~~");
        }
}

 /*
 hal.console->print("Lat, ");
 hal.console->print(gps->latitude/10000000.0);
 hal.console->print(", Lon, ");
 hal.console->print(gps->longitude/10000000.0);
 hal.console->print(", g_speed, ");
 hal.console->println(gps->ground_speed/100.0);
 hal.console->printf(" Alt: %.2fm GSP: %.2fm/s CoG: %d SAT: %d TIM: %lu STATUS: %u\n",
           (float)gps->altitude / 100.0,
	   (float)gps->ground_speed / 100.0,
	   (int)gps->ground_course / 100,
	   gps->num_sats,
	   gps->time,
	   gps->status()
	   );
*/

//Coordinate Arrays: [longitude, lattitude]
bool getTargetCoordinates(float coords[]){
  
        if(uartMessaging.isUserLonLatest() && uartMessaging.isUserLatLatest()){
              uartMessaging.getUserLon(&target_coordinates[0]);
              uartMessaging.getUserLat(&target_coordinates[1]);
              return true;
        }
        
        hal.console->print("~~~~~~~~~~~~~~~~  WAITING ON NEW TARGET DATA!  ~~~~~~~~~~~~~~");
        return false;
}


//GPS Status Code
// 0 = no GPS, 1 = GPS but no fix, 2 = 2D fix, 3 = 3D fix
bool getGpsState(){
        gps->update();

	if (gps->new_data){
            int gps_status = gps->status();

            if(gps_status >=2) {
                  GPS_state = true;
                  return true;
            }else{
                  hal.console->printf("No GPS Lock, status code: %i \n", gps_status);
                  GPS_state = false;
                  return false;
            }
	}

        //No data on buffer - check connection
        hal.console->println("No GPS data on UART buffer");
        GPS_state = false;
        return false;
}

//Coordinate Arrays: [longitude, lattitude]
void getTakeoffCoordinates(float coords[]){
        int counter=0;
        hal.console->println("getting GPS lock");
        gps->update();
        
        while (gps->status() < 2){
              flash_leds(true);
              hal.scheduler->delay(50);
              flash_leds(false);
              hal.scheduler->delay(50);
              flash_leds(true);
              hal.scheduler->delay(50);
              
              //Counter to exit while loop if cannot get GPS coordinate
              counter++;
              //hal.console->print(counter);
              
              //If we get to 20 attempts, then it probalby cannot attain coordinates
              if(counter >= 50){
                      hal.console->print("\n\n Cannot attain GPS coordinates. Status code: ");        
                      hal.console->println(gps->status());
                      flash_leds(true);
                      hal.scheduler->delay(50);
                      flash_leds(false);
                      hal.scheduler->delay(50);
                      flash_leds(true);
                      hal.scheduler->delay(50);       
                      flash_leds(false);
                      hal.scheduler->delay(50);  
                      flash_leds(true);
                      hal.scheduler->delay(50);     
              }
              gps->update();
        }

        coords[1] = gps->latitude/10000000.0;
        coords[0] = gps->longitude/10000000.0;
        hal.console->println(gps->status());
        return;
}

