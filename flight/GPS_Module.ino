//GPS / COMPASS Methods

void getHeading(){
        //Use AHRS for Heading
        static uint32_t last_t, last_print;
        float heading = 0;
   
        if((hal.scheduler->micros() - heading_timer) > 100000L){ //Run loop @ 10Hz
            
            ahrs.update();
        
            heading_timer = hal.scheduler->micros();
            
            compass.read();
            heading = compass.calculate_heading(ahrs.get_dcm_matrix());
            gps->update();
            
            Vector3f drift  = ahrs.get_gyro_drift();
            last_heading = current_heading;
            current_heading =  ToDeg(heading);
        }
        //current_heading = movingAvg(last_heading, current_heading, .5);
}


double getDesiredHeading(){
          
          int32_t long_drone, lat_drone, long_target, lat_target;
          long_drone = drone_coordinates[0];
          lat_drone = drone_coordinates[1];
          long_target = target_coordinates[0];
          lat_target = target_coordinates[1];
  
          //Calculate the heading angle of the user relative to the drone.
          int32_t lat_seperation, long_seperation;
          double angle, desired_heading;
          
          //////////TEMPORARY //////////////////////////////////////////////
          //Fix when Albion always sends over 9 digits
          //This is to make the target coordinates proper size 
          if(abs(long_target) < 100000000UL){
            long_target = long_target*10;
          }
          if(abs(lat_target) < 100000000UL){
            lat_target = lat_target*10;
          }
          
          
          ///NOTE: DO MATH OUTSIDE OF ABS...
  
          
          //Find the lat & long differences
          if(abs(lat_drone) > abs(lat_target)){
            lat_seperation = lat_drone - lat_target;  
            lat_seperation = abs(lat_seperation);  
          }else{
            lat_seperation = lat_target - lat_drone;
            lat_seperation = abs(lat_seperation);
          }
          
          if(abs(long_drone) > abs(long_target)){
            long_seperation = long_drone - long_target;
            long_seperation = abs(long_seperation); 
          }else{
            long_seperation = long_target - long_drone;
            long_seperation = abs(long_seperation);
          }
          
          //Calculate the desired heading of the drone. By using the lattitude as of "opposite" (sohcahTOA) then this will 
          //always calculate an angle from either due N or S
          angle = (double)lat_seperation / (double)long_seperation;
                    
          //Calculate desired heading and convert radians to degrees
          desired_heading = atan(angle);
          desired_heading = desired_heading * 180/PI;
          
          //hal.console->printf("lat_seperation: %ld, long_seperation: %ld, angle: %f, desiredHeading: %f", lat_seperation, long_seperation, angle, desired_heading);
          
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
          
          //hal.console->printf("  desired_heading_fixed: %f, ", desired_heading);
          
          return desired_heading;
}


//Get Drone coordinates and filter them.
//Coordinate Arrays: [longitude, lattitude]
void getDroneCoordinates(int32_t coords[]){
        int32_t temp;
        gps->update();
       
	if (gps->new_data) { 
            //Check the accuracy of this data first
            
            temp = gps->longitude;
            //Longitude Checks
            coords[0] =  temp;
            
            
            temp = gps->latitude;
            //Lattitude Checks
            coords[1] =temp;
  
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
bool getTargetCoordinates(int32_t coords[]){
         //temp int32 array
         int32_t temp[2];
         
         if(uartMessaging.isUserLonLatest() && uartMessaging.isUserLatLatest())
          {
            uartMessaging.getUserLon(&temp[0]);
            coords[0] = temp[0];
            //hal.console->println(coords[0]);
            
            uartMessaging.getUserLat(&temp[1]);
            coords[1] = temp[1];
            //hal.console->println(coords[1]);
            return true;
          }
        
        //hal.console->print("~~~~~~~~~~~~~~~~  WAITING ON NEW TARGET DATA!  ~~~~~~~~~~~~~~");
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
void getTakeoffCoordinates(int32_t coords[]){
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
                      break;  
              }
              gps->update();
        }

        coords[1] = gps->latitude;
        coords[0] = gps->longitude;
        
        //Set the current drone Coordinates to this GPS location as well.
        drone_coordinates[1] = gps->latitude;
        drone_coordinates[0] = gps->longitude;
        
        hal.console->println(gps->status());
        return;
}


//Gets Seperation distnace between drone and target
int32_t getSeperationDistance(){
      
      int32_t lat_seperation, long_seperation;
      double hypotenuse, lat_seperation_m, long_seperation_m; //lat long seperation in meters
      
      //Find the lat & long differences
      if(abs(drone_coordinates[1]) > abs(target_coordinates[1])){
        lat_seperation = abs(drone_coordinates[1] - target_coordinates[1]);  
      }else{
        lat_seperation = abs(target_coordinates[1] - drone_coordinates[1]);
      }
      
      if(abs(drone_coordinates[0]) > abs(target_coordinates[0])){
        long_seperation = abs(drone_coordinates[0] - target_coordinates[0]); 
      }else{
        long_seperation = abs(target_coordinates[0] - drone_coordinates[0]);
      }
      
      //hal.console->printf(",  target_long, %ld, target_lat, %ld, drone_long, %ld, drone_lat, %ld, long_seperation: %ld, lat_seperation: %ld", target_coordinates[0], target_coordinates[1], drone_coordinates[0], drone_coordinates[1], long_seperation, lat_seperation);
      
      //Convert lat and long to meters
      lat_seperation_m = (lat_seperation)*LAT_TO_METER/10000000;
      long_seperation_m = (long_seperation)*LONG_TO_METER/10000000;
      
      //Pythagorem Theorem
      lat_seperation_m = (lat_seperation_m*lat_seperation_m);
      long_seperation_m = (long_seperation_m*long_seperation_m);
      hypotenuse = sqrt(lat_seperation_m + long_seperation_m);
      
      //Return the seperation distnace in meters
      return hypotenuse; 
}

