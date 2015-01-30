#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <PID.h>

#include <AP_Declination.h>
#include <AP_Compass.h> // Compass Library

#include <AP_Buffer.h>
#include <Filter.h>
#include <AP_Baro.h>

// ArduPilot Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

// MPU6050 accel/gyro chip
AP_InertialSensor_MPU6000 ins;

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
#define RC_ALT_MAX   50
#define THR_MIN 1100
#define THR_MAX 1800

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
uint32_t thr_interval;

// Arduino map function
long map(long x, long in_min, long in_max, long out_min, long out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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


//GLOBAL VARIABLES
int startup = 0;
float originalOrientation = 0.0;
float currentOrientation = 0.0;
float alt = 0;
float last_alt = 0;
float last_climb_rate = 0;
int thr_lock = 0;

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
      heading = compass.calculate_heading(0,0); // roll = 0, pitch = 0 for this example
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
      //baro.read();
      if (!baro.healthy) {
          hal.console->println("not healthy");
          return 0.0;
      }
      //hal.console->print(" Altitude:");
      //hal.console->println(baro.get_altitude());
      
      float a = baro.get_altitude();
      if(a < 0)
        return 0;
      else
        return a*100;
}  

float getClimbRate(){
      //baro.read();
      if (!baro.healthy) {
          hal.console->println("not healthy");
          return 0.0;
      }
     // hal.console->print(" Climb Rate:");
     // hal.console->println(baro.get_climb_rate());
	 
      return baro.get_climb_rate()*100;
}

float exponentialSmoother(float previous_value, float current_value, float a){
    //a is a constant the smoothing factor
    return (previous_value + a*current_value);
}

long rcthr = 1000;
int heightLock = 0;

void setup() 
{
  hal.console->println("Starting...");
  // Enable the motors and set at 490Hz update
  hal.rcout->set_freq(0xF, 490);
  hal.rcout->enable_mask(0xFF);

  // PID Configuration
  pids[PID_PITCH_RATE].kP(0.45);
  pids[PID_PITCH_RATE].kI(0.0);
  pids[PID_PITCH_RATE].imax(50);
  
  pids[PID_ROLL_RATE].kP(0.45);
  pids[PID_ROLL_RATE].kI(0.0);
  pids[PID_ROLL_RATE].imax(50);
  
  pids[PID_YAW_RATE].kP(0.7);
  pids[PID_YAW_RATE].kI(1.0);
  pids[PID_YAW_RATE].imax(50);
  
  pids[PID_PITCH_STAB].kP(4.5);
  pids[PID_ROLL_STAB].kP(4.5);
  pids[PID_YAW_STAB].kP(10);
  
  pids[ALT_RATE].kP(0.45);
  pids[ALT_RATE].kI(0.0);
  pids[ALT_RATE].imax(50);
  
  pids[ALT_STAB].kP(1.0);

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
  ins.dmp_init();
  hal.scheduler->resume_timer_procs();
  
  OFF_BUTTON_VALUE = hal.analogin->channel(OFF_BUTTON);
  
  hal.console->println("Compass library test");

  if (!compass.init()) {
      hal.console->println("compass initialisation failed!");
      while (1) ;
  }
  hal.console->println("init done");

  compass.set_orientation(AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD); // set compass's orientation on aircraft.
  compass.set_offsets(0,0,0); // set offsets to account for surrounding interference
  compass.set_declination(ToRad(0.0)); // set local difference between magnetic north and true north

  hal.console->print("Compass auto-detected as: ");
  switch( compass.product_id ) {
  case AP_COMPASS_TYPE_HIL:
      hal.console->println("HIL");
      break;
  case AP_COMPASS_TYPE_HMC5843:
      hal.console->println("HMC5843");
      break;
  case AP_COMPASS_TYPE_HMC5883L:
      hal.console->println("HMC5883L");
      break;
  case AP_COMPASS_TYPE_PX4:
      hal.console->println("PX4");
      break;
  default:
      hal.console->println("unknown");
      break;
  }
  
  hal.scheduler->delay(1000);
  timer = hal.scheduler->micros();
  interval = timer;
  thr_interval = timer;
}

void loop() 
{
  static float yaw_target = 0;  
  // Wait until new orientation data (normally 5ms max)
  while (ins.num_samples_available() == 0);
  
  // Copy from channels array to something human readable - array entry 0 = input 1, etc.
  uint16_t channels[8];  // array for raw channel values
  hal.rcin->read(channels, 8);  
  long rcyaw, rcpit, rcroll, safety, rcalt, thrmax;  // Variables to store radio in
  safety = channels[4];

  
  //Check off switch, kills motors otherwise
  float AVG_OFF_BUTTON_VALUE = OFF_BUTTON_VALUE->voltage_average();
  while( (AVG_OFF_BUTTON_VALUE < 1.0) || (safety < 1500) || (heightLock == 1)){
    hal.rcout->write(MOTOR_FL, 1000);
    hal.rcout->write(MOTOR_BL, 1000);
    hal.rcout->write(MOTOR_FR, 1000);
    hal.rcout->write(MOTOR_BR, 1000);    
    //hal.console->printf_P(PSTR("Voltage ch0:%.2f\n"), AVG_OFF_BUTTON_VALUE);
    //hal.scheduler->delay(500);
    hal.rcin->read(channels, 8);
    safety = channels[4];
    AVG_OFF_BUTTON_VALUE = OFF_BUTTON_VALUE->voltage_average();
    //hal.console->println("SAFE");
    //hal.scheduler->delay(1000);

    if( getAltitude() < 100)
      heightLock = 0;
  }     
  
  //rcalt = 10;
  thrmax = map(channels[2], RC_THR_MIN, RC_THR_MAX, THR_MIN, THR_MAX);
  rcalt = map(channels[2], RC_THR_MIN, RC_THR_MAX, RC_ALT_MIN, RC_ALT_MAX);
  rcyaw = map(channels[3], RC_YAW_MIN, RC_YAW_MAX, -180, 180);
  rcpit = map(channels[0], RC_ROL_MIN, RC_ROL_MAX, -45, 45);
  rcroll = map(channels[1], RC_PIT_MIN, RC_PIT_MAX, -45, 45);
  
  //hal.console->print("Desired Altitude : ");
  //hal.console->println(rcalt);
 
  
  // Ask MPU6050 for orientation
  ins.update();
  float roll, pitch, yaw, alt, climbRate;
  float alt_smoothing_constant, climb_rate_smoothing_constant, alt_temp, climb_rate_temp;  
  ins.quaternion.to_euler(&roll, &pitch, &yaw);
  roll = ToDeg(roll) ;
  pitch = ToDeg(pitch) ;
  yaw = calculateYaw();
  
  
  
 
  if((hal.scheduler->micros() - interval) > 330000UL) {
  
	baro.read();
  
    alt_smoothing_constant = 1.0;
	climb_rate_smoothing_constant = 1.0;
	
    last_alt = alt;
	last_climb_rate = climbRate;
	
    alt = getAltitude();
	climbRate = getClimbRate();
  
	alt_temp = exponentialSmoother(last_alt, alt, alt_smoothing_constant);
	climb_rate_temp = exponentialSmoother(last_climb_rate, climbRate, climb_rate_smoothing_constant);
  
	if(abs(alt-last_alt) > 100 )
		alt=last_alt;       
    alt = alt_temp;	
	
	if( abs(climbRate - last_climb_rate) > 15 )
		climbRate = last_climb_rate;
	climbRate = climb_rate_temp;
		
	climbRate = constrain(getClimbRate(), -15, 15);
	
    interval = hal.scheduler->micros();
	thr_lock = 1;
	// hal.console->print("Alt: ");
	// hal.console->println(alt);
	// hal.console->print("Climb Rate: ");
	// hal.console->println(climbRate);	
	// hal.console->print("Samples: ");
	// hal.console->println(baro.get_pressure_samples());
	// hal.console->println("");
  } 

  
  // Ask MPU6050 for gyro data
  Vector3f gyro = ins.get_gyro();
  float gyroPitch = ToDeg(gyro.y), gyroRoll = ToDeg(gyro.x), gyroYaw = ToDeg(gyro.z);
  
  // Do the magic
  if(rcalt > 0) {  // Altitude raised, turn on stablisation.
    // Stablise PIDS
    float pitch_stab_output = constrain(pids[PID_PITCH_STAB].get_pid((float)rcpit - pitch, 1), -250, 250); 
    float roll_stab_output = constrain(pids[PID_ROLL_STAB].get_pid((float)rcroll - roll, 1), -250, 250);
    float yaw_stab_output = constrain(pids[PID_YAW_STAB].get_pid(wrap_180(yaw_target - yaw), 1), -360, 360);
  
    // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
    if(abs(rcyaw ) > 5) {
      yaw_stab_output = rcyaw;
      yaw_target = yaw;   // remember this yaw for when pilot stops
    }
    
    // rate PIDS
    long pitch_output =  (long) constrain(pids[PID_PITCH_RATE].get_pid(pitch_stab_output - gyroPitch, 1), -500, 500);  
    long roll_output =  (long) constrain(pids[PID_ROLL_RATE].get_pid(roll_stab_output - gyroRoll, 1), -500, 500);  
    long yaw_output =  (long) constrain(pids[PID_YAW_RATE].get_pid(yaw_stab_output - gyroYaw, 1), -500, 500);  
    
    long alt_stab_output = constrain(pids[ALT_STAB].get_pid((float)rcalt - alt, 1), -50, 50);
    float alt_output = (long) constrain(pids[ALT_RATE].get_pid(alt_stab_output - climbRate, 1), -10, 10);
    
	
	//if( (hal.scheduler->micros() - thr_interval) > 67000UL){ 
	if( thr_lock == 1 ){	
		rcthr += alt_output;
		rcthr = constrain(rcthr, THR_MIN, THR_MAX);		
		thr_interval = hal.scheduler->micros();
		hal.console->print("RCALT: ");
		hal.console->println(rcalt);
		hal.console->print("ALT: ");
		hal.console->println(alt); 
		 hal.console->print("Alt Output: ");
		 hal.console->println(alt_output);
		 hal.console->print("Thr: ");
		 hal.console->println(rcthr);
		 hal.console->println("");
		thr_lock = 0;
	}
	
    // rcthr = constrain(map(alt_output, -100, 100, THR_MIN, THR_MAX), THR_MIN, thrmax);
	
    // hal.console->print("RCALT: ");
    // hal.console->println(rcalt);
    // hal.console->print("ALT: ");
    // hal.console->println(alt);    
    // hal.console->print("STAB: ");
    // hal.console->println(alt_stab_output);
    // hal.console->print("OUTPUT: ");
    // hal.console->println(alt_output);
//    
//    hal.console->print("FL: ");
//    hal.console->println(rcthr + roll_output + pitch_output - yaw_output);
//    hal.console->print("BL: ");
//    hal.console->println(rcthr + roll_output - pitch_output + yaw_output);
//    hal.console->print("FR: ");
//    hal.console->println(rcthr - roll_output + pitch_output + yaw_output);
//    hal.console->print("BR: ");
//    hal.console->println(rcthr - roll_output - pitch_output - yaw_output);    

    
    if(alt > 100)
      heightLock = 1;
    else 
      heightLock = 0;

    
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
