#include <AP_Compass.h>
#include <AP_Compass_HMC5843.h>
#include <Compass.h>

#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>
#include <PID.h>
#include <AP_Declination.h>
#include <AP_Math.h>
#include <AP_Buffer.h>
#include <Filter.h>
#include <AP_Baro.h>  //altitude
#include <AP_BattMonitor.h>  //battery Monitor
#include <AP_GPS.h>

//include uart messaging header file
#include <UartMessaging.h>

// ArduPilot Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

//Messaging Object
UartMessaging uartMessaging;

// MPU6050 accel/gyro chip
AP_InertialSensor_MPU6000 ins;

//Batery Monitor
AP_BattMonitor battery_mon;

//Gps Device
GPS         *gps;
AP_GPS_Auto GPS(&gps);




/*------------------------------------------------ SYSTEM DEFINITIONS ------------------------------------------------------*/
// Radio min/max values for each stick for my radio (worked out at beginning of article)
#define RC_THR_MIN   1107
#define RC_THR_MAX   1907
#define RC_YAW_MIN   1106
#define RC_YAW_MAX   1907
#define RC_PIT_MIN   1106
#define RC_PIT_MAX   1908
#define RC_ROL_MIN   1104
#define RC_ROL_MAX   1906
#define RC_ALT_MIN   0
#define RC_ALT_MAX   1

// Motor numbers definitions
#define MOTOR_FL   2    // Front left    
#define MOTOR_FR   0    // Front right
#define MOTOR_BL   1    // back left
#define MOTOR_BR   3    // back right

#define OFF_BUTTON 0
AP_HAL::AnalogSource* OFF_BUTTON_VALUE;

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
AP_Compass_PX4 compass;
#else
AP_Compass_HMC5843 compass;
#endif
uint32_t timer;
uint32_t interval;

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

// PID array (7 pids, two for each axis, 1 for altitude)
PID pids[8];
#define PID_PITCH_RATE 0
#define PID_ROLL_RATE 1
#define PID_PITCH_STAB 2
#define PID_ROLL_STAB 3
#define PID_YAW_RATE 4
#define PID_YAW_STAB 5
#define ALT_STAB 6
#define ALT_RATE 7

//Hover throttle
#define HOVER_THR 1300




/*------------------------------------------------ DECLARE GLOBAL VARIABLES ------------------------------------------------------*/
int startup = 0;
float originalOrientation = 0.0;
float currentOrientation = 0.0;
float alt = 0;
float last_alt = 0;
float climb_rate = 0;
float last_climb_rate = 0;
long rcthr = 1000;
int heightLock = 0;
Matrix3f dcm_matrix;


// Arduino map function
float map(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float calculateYaw(){ 
    static float min[3], max[3], offset[3];
    float heading;
    compass.accumulate();
    if((hal.scheduler->micros()- timer) > 100000L)
    {
      timer = hal.scheduler->micros();
      compass.read();
      unsigned long read_time = hal.scheduler->micros() - timer;
      

      if (!compass.healthy) {
          hal.console->println("not healthy");
          return 0;
      }
      dcm_matrix.to_euler(0,0, 0);
      heading = compass.calculate_heading(dcm_matrix); // roll = 0, pitch = 0 for this example
      compass.null_offsets();

      // capture min
      if( compass.mag_x < min[0] )
          min[0] = compass.mag_x;
      if( compass.mag_y < min[1] )
          min[1] = compass.mag_y;
      if( compass.mag_z < min[2] )
          min[2] = compass.mag_z;

      // capture max
      if( compass.mag_x > max[0] )
          max[0] = compass.mag_x;
      if( compass.mag_y > max[1] )
          max[1] = compass.mag_y;
      if( compass.mag_z > max[2] )
          max[2] = compass.mag_z;

      // calculate offsets
      offset[0] = -(max[0]+min[0])/2;
      offset[1] = -(max[1]+min[1])/2;
      offset[2] = -(max[2]+min[2])/2;
    } else {
	    hal.scheduler->delay(1);
    }  
   currentOrientation = ToDeg(heading);
   if(currentOrientation < 0){
     currentOrientation = 360 + currentOrientation;
   }
   
 if(startup == 1 ){
   originalOrientation = currentOrientation;
   startup = 2;
 }
 if(startup == 0)
   startup = 1;
 return wrap_180( currentOrientation - originalOrientation );   
}

AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);

float getAltitude(){
      baro.read();
      if (!baro.healthy) {
          hal.console->println("not healthy");
          return 0.0;
      }

      //hal.console->print(" Altitude:");
      //hal.console->println(baro.get_altitude());
      
      float a = baro.get_altitude();
      return a;
}  

float getClimbRate(){
      baro.read();
      if (!baro.healthy) {
          hal.console->println("not healthy");
          return 0.0;
      }
     // hal.console->print(" Climb Rate:");
      //hal.console->println(baro.get_climb_rate());
      return (baro.get_climb_rate());
}

//Exponential Moving Average  ->  http://en.wikipedia.org/wiki/Exponential_smoothing
float exponentialSmoother(float previous_value, float current_value, float a){
    //a is a constant the smoothing factor
    return (previous_value + a*(current_value-previous_value));
}


//Moving Average Filter
float movingAvg(float previous, float current, float a){
  //a is a constant smoothing factor. Large a means a lot of weight is put on the previous value
  //a must be between 0 and 1
  if(abs(a) > 0) {
    hal.console->println("Error: a must be between 0 and 1");
    a = 0;
  }
  return (previous*a + (1-a)*current);
}





/*---------------------------------------------------- SETUP ----------------------------------------------------------*/
void setup() 
{
  // Enable the motors and set at 490Hz update
  hal.rcout->set_freq(0xF, 490);
  hal.rcout->enable_mask(0xFF);

  // PID Configuration
  pids[PID_PITCH_RATE].kP(0.45);
  pids[PID_PITCH_RATE].kI(0.1);
  pids[PID_PITCH_RATE].imax(50);
  
  pids[PID_ROLL_RATE].kP(0.45);
  pids[PID_ROLL_RATE].kI(0.1);
  pids[PID_ROLL_RATE].imax(50);
  
  pids[PID_YAW_RATE].kP(0.7);
  pids[PID_YAW_RATE].kI(0.1);
  pids[PID_YAW_RATE].imax(50);
  
  pids[PID_PITCH_STAB].kP(4.5);
  pids[PID_ROLL_STAB].kP(4.5);
  pids[PID_YAW_STAB].kP(10);
  
  pids[ALT_RATE].kP(0.1);
  pids[ALT_RATE].kI(0.0);
  pids[ALT_RATE].imax(50);
  
  pids[ALT_STAB].kP(10.0);
  pids[ALT_STAB].kI(0.2);
  pids[ALT_STAB].imax(50);
  
  hal.console->println("Barometer Init");
  hal.gpio->pinMode(63, GPIO_OUTPUT);
  hal.gpio->write(63, 1);    
  baro.init();
  baro.calibrate();
  
  // Turn on MPU6050 - quad must be kept still as gyros will calibrate
  ins.init(AP_InertialSensor::COLD_START, 
			 AP_InertialSensor::RATE_100HZ,
                          NULL);

  // initialise sensor fusion on MPU6050 chip (aka DigitalMotionProcessing/DMP)
  hal.scheduler->suspend_timer_procs();  // stop bus collisions
  //ins.dmp_init();
  hal.scheduler->resume_timer_procs();
  
  OFF_BUTTON_VALUE = hal.analogin->channel(OFF_BUTTON);
  
  if (!compass.init()) {
      hal.console->println("compass initialisation failed!");
      while (1) ;
  }

  //compass.set_orientation(AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD); // set compass's orientation on aircraft.
  compass.set_offsets(0,0,0); // set offsets to account for surrounding interference
  compass.set_declination(ToRad(0.0)); // set local difference between magnetic north and true north

  hal.console->print("Compass auto-detected as: ");
  switch( compass.product_id ) {
  case AP_COMPASS_TYPE_HMC5843:
      hal.console->println("HMC5843");
      break;
  case AP_COMPASS_TYPE_HMC5883L:
      hal.console->println("HMC5883L");
      break;
  default:
      hal.console->println("unknown");
      break;
  }

  hal.scheduler->delay(1000);
  timer = hal.scheduler->micros();
  interval = timer;
  
  //Initializes the UART C bus (begin(baudrate, rx buffer, tx buffer)
  //See UARTDriver.h for more...
  hal.uartC->begin(115200, 16, 16); 
  hal.console->println("UARTC (UART2) Test");
  //Uart messaging
  uartMessaging.init(hal.uartC, hal.console);
  
  //SET UP UARTB CONNECTION FOR GPS
  hal.uartB->begin(38400);
  gps = &GPS;
  gps->init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_2G);       // GPS Initialization
  
  //Initialize the battery monitor
  battery_mon.init();
  battery_mon.set_monitoring(AP_BATT_MONITOR_VOLTAGE_AND_CURRENT);
  hal.console->println("Battery monitor initialized");
}




/*---------------------------------------------- LOOP -----------------------------------------------------*/
void loop() 
{
  //receive from uart
  uartMessaging.receive();
  
  static float yaw_target = 0;  
  // Wait until new orientation data (normally 5ms max)
  while (ins.num_samples_available() == 0);
  
  // Copy from channels array to something human readable - array entry 0 = input 1, etc.
  uint16_t channels[8];  // array for raw channel values
  hal.rcin->read(channels, 8);  
  long rcyaw, rcpit, rcroll, safety, last_rcthr;  // Variables to store radio in
  float rcalt;
  safety = channels[4];
  
  //Check off switch, kills motors otherwise
  float AVG_OFF_BUTTON_VALUE = OFF_BUTTON_VALUE->voltage_average();
  while( (AVG_OFF_BUTTON_VALUE < 1.0) || (safety < 1500)){
    hal.rcout->write(MOTOR_FL, 1000);
    hal.rcout->write(MOTOR_BL, 1000);
    hal.rcout->write(MOTOR_FR, 1000);
    hal.rcout->write(MOTOR_BR, 1000);    
    //hal.console->printf_P(PSTR("Voltage ch0:%.2f\n"), AVG_OFF_BUTTON_VALUE);
    //hal.scheduler->delay(500);
    hal.rcin->read(channels, 8);
    safety = channels[4];
    AVG_OFF_BUTTON_VALUE = OFF_BUTTON_VALUE->voltage_average();
    
    //GET BATTERY STATS
    // update voltage and current readings
    battery_mon.read();
    battery_mon.voltage(), //voltage
                            
    //GET GPS STATS
    gps->update();
    if (gps->new_data) {
      hal.console->print("Lat, ");
      hal.console->print(gps->latitude/10000000.0);
      hal.console->print(", Lon, ");
      hal.console->print(gps->longitude/10000000.0);
      hal.console->print(", g_speed, ");
      hal.console->println(gps->ground_speed/100.0);
      // hal.console->printf(" Alt: %.2fm GSP: %.2fm/s CoG: %d SAT: %d TIM: %lu STATUS: %u\n",
      //               (float)gps->altitude / 100.0,
      //               (float)gps->ground_speed / 100.0,
      //               (int)gps->ground_course / 100,
      //               gps->num_sats,
      //               gps->time,
      //               gps->status()
      //               );
    }
        
    for(int i=0; i<8; i++)
      pids[i].reset_I();
  }     
  
  
  rcthr = map(channels[2], RC_THR_MIN, RC_THR_MAX, RC_THR_MIN, 1500);
  rcalt = 1.0; //Hard code in RCALT
  rcyaw = map(channels[3], RC_YAW_MIN, RC_YAW_MAX, -180, 180);
  rcpit = map(channels[0], RC_ROL_MIN, RC_ROL_MAX, 45, -45);
  rcroll = map(channels[1], RC_PIT_MIN, RC_PIT_MAX, 45, -45); 
   
  // Ask MPU6050 for orientation
  ins.update();
  float roll, pitch, yaw, temp;  
  ins.quaternion.to_euler(&roll, &pitch, &yaw);
  
  roll = ToDeg(roll) ;
  pitch = ToDeg(pitch) ;
  yaw = ToDeg(yaw);


  
  /*------------------------ Autonomous Tracking. Overwrites RC input -----------------------------*/
  /*
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
  
  //COMPASS OPERATION: 
  //   True North is 0 (degrees)
  //   Eastern headings are negative numbers
  //   Western headings are positive numbers
  //   South is 180 or -180
  

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
  
  ins.quaternion.to_euler(&roll, &pitch, &yaw);
  //dcm_matrix.rotation_matrix(dcm_matrix);
  hal.console->print(" Heading Angle: ");
  hal.console->println(compass.calculate_heading(ahrs.get_dcm_matrix()));
  
 
  /*----------------------------------- Autonomous Tracking  END ------------------------------------*/






  
  //Automatic Altitude Holding
  if((hal.scheduler->micros() - interval) > 100000UL) {
    last_alt = alt;
    alt = getAltitude();
    
    //Smooth raw data (last parameter is the smoothing constant)
    temp = movingAvg(last_alt, alt, .5);
    alt = temp;
    
    //Verify that the altitude values are within scope
    if(abs(alt-last_alt) > 100){
      alt=last_alt;
    }

    //Get Climb Rate and smooth (last parameter is the smoothing constant)
    last_climb_rate = climb_rate;
    climb_rate = constrain(getClimbRate(), -15, 15);
    climb_rate = movingAvg(last_climb_rate, climb_rate, .9);
    
    //Verify that the climb rate values are within scope
    if(abs(climb_rate-last_climb_rate) > 100){
      climb_rate=last_climb_rate;
    }
    
    //Scheduling
    interval = hal.scheduler->micros();  
    
    //GET BATTERY STATS at the same frequency of the Altitude check
    // update voltage and current readings
    battery_mon.read();
    
    
    /*hal.console->printf("\nVoltage: %.2f \tCurrent: %.2f \tTotCurr:%.2f",
    			    battery_mon.voltage(),
      			    battery_mon.current_amps(),
                            battery_mon.current_total_mah());
    */
    //send alt and battery status
    uartMessaging.sendAltitude(alt);
    uartMessaging.sendBattery(battery_mon.voltage());
  }
  
  
  // Ask MPU6050 for gyro data
  Vector3f gyro = ins.get_gyro();
  float gyroPitch = ToDeg(gyro.y), gyroRoll = ToDeg(gyro.x), gyroYaw = ToDeg(gyro.z);
  
  // Do the magic
  if(rcalt >= 0) {  // Altitude raised, turn on stablisation.
    // Stablise PIDS
    float pitch_stab_output = constrain(pids[PID_PITCH_STAB].get_pid((float)rcpit - pitch, 1), -250, 250); 
    float roll_stab_output = constrain(pids[PID_ROLL_STAB].get_pid((float)rcroll - roll, 1), -250, 250);
    float yaw_stab_output = constrain(pids[PID_YAW_STAB].get_pid((float)yaw_target - yaw, 1), -360, 360);
      
    // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
    if(abs(rcyaw ) > 5) {
      yaw_stab_output = rcyaw;
      yaw_target = yaw;   // remember this yaw for when pilot stops
    }
    
    // rate PIDS
    long pitch_output =  (long) constrain(pids[PID_PITCH_RATE].get_pid(pitch_stab_output - gyroPitch, 1), -500, 500);  
    long roll_output =  (long) constrain(pids[PID_ROLL_RATE].get_pid(roll_stab_output - gyroRoll, 1), -500, 500);  
    long yaw_output =  (long) constrain(pids[PID_YAW_RATE].get_pid(yaw_stab_output - gyroYaw, 1), -500, 500);  


    //Feedback loop for altitude holding
    float alt_output = constrain(pids[ALT_STAB].get_pid((float)rcalt - alt, 1), -250, 250);
    //float alt_output = constrain(pids[ALT_RATE].get_pid(alt_stab_output - climb_rate, 1), -100, 100);
    
  
    //F.Mode is channels[5]
    //TOP = greater than 1100
    //MIDDLE = greater than 1500
    //BOTTOM = greater than 1900
      
    //Turn alt hold on / off
    if (channels[5] < 1200){
      //Autonomous Altitude hold
      
      //Map the Throttle
      rcthr = HOVER_THR + alt_output;
      rcthr = constrain(rcthr, 1200, 1400); 
      
    }else{
      //Manual alt
      
      //reset i
      // reset PID integrals while in manual mode
      pids[ALT_STAB].reset_I();
    }
  
    //hal.uartC->println(10);
    if (hal.uartC->available()) {
      hal.uartC->read();
    }
   
    
    hal.console->print("Desired Alt, ");
    hal.console->print(rcalt);
    hal.console->print(", alt, ");
    hal.console->print(alt);
    //hal.console->print(", Desired_climb_Rate, ");
    //hal.console->print(alt_stab_output);
    hal.console->print(", Climb Rate, ");
    hal.console->print(climb_rate); 
    hal.console->print(", alt_output, ");
    hal.console->print(alt_output); 
    hal.console->print(", THR, ");
    hal.console->println(rcthr); 
    

    
    // mix pid outputs and send to the motors.
    hal.rcout->write(MOTOR_FL, rcthr + roll_output + pitch_output - yaw_output);
    hal.rcout->write(MOTOR_BL, rcthr + roll_output - pitch_output + yaw_output);
    hal.rcout->write(MOTOR_FR, rcthr - roll_output + pitch_output + yaw_output);
    hal.rcout->write(MOTOR_BR, rcthr - roll_output - pitch_output - yaw_output);
    
    
  } else {
    // motors off
    hal.rcout->write(MOTOR_FL, 1000);
    hal.rcout->write(MOTOR_BL, 1000);
    hal.rcout->write(MOTOR_FR, 1000);
    hal.rcout->write(MOTOR_BR, 1000);
       
    // reset yaw target so we maintain this on takeoff
    yaw_target = yaw;
    
    // reset PID integrals whilst on the ground
    for(int i=0; i<8; i++)
      pids[i].reset_I();

  }
}

AP_HAL_MAIN();
