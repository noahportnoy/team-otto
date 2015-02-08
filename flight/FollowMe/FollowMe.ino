  /*------------------------ Autonomous Tracking. Overwrites RC input -----------------------------*/
  
  //Get Lattitude & Longitude
  float lat_drone, long_drone, lat_user, long_user, angle, desired_heading, current_heading;
  //getDroneCoor();
  //getUserCoor();
  //getSepDistance();
  
  //Calculate the heading angle of the user relative to the drone.
  float lat_seperation, long_seperation;
  
  //Find the lat & long differences
  if(abs(lat_drone) > abs(lat_user)){
    lat_seperation = abs(lat_drone - lat_user);
  }else{
    lat_seperation = abs(lat_user - lat_drone);
  }
  
  if(abs(long_drone) > abs(long_user)){
    long_seperation = abs(long_drone - long_user);
  }else{
    long_seperation = abs(long_user - long_drone);
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
  if(long_drone > long_user){
     desired_heading = desired_heading + 90;
  }
  
  //If the Lattitude of the user is larger than the drones then the drone should be facing East
  //Convert this to a negative number (details above)
  if(lat_user > lat_drone){
    desired_heading = desired_heading*(-1);
  }
  
  
  //Get Drone's current heading
  //First create the DCM matrix from the 3 euler angles and use them to calculate heading
  
  //dcm_matrix.from_euler(&roll, &pitch, &yaw);
  //dcm_matrix.rotation_matrix(dcm_matrix);
  hal.console->print(" Heading Angle: ");
  hal.console->println(compass.calculate_heading(dcm_matrix));
  
  
  
  
  
*/
