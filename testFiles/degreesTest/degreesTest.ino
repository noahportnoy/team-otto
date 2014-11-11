#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <PID.h>

// ArduPilot Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

// MPU6050 accel/gyro chip
AP_InertialSensor_MPU6000 ins;

// Radio min/max values for each stick for my radio (worked out at beginning of article)
#define RC_THR_MIN   1070
#define RC_YAW_MIN   1068
#define RC_YAW_MAX   1915
#define RC_PIT_MIN   1077
#define RC_PIT_MAX   1915
#define RC_ROL_MIN   1090
#define RC_ROL_MAX   1913

// Motor numbers definitions
#define MOTOR_FL   2    // Front left    
#define MOTOR_FR   0    // Front right
#define MOTOR_BL   1    // back left
#define MOTOR_BR   3    // back right

// Arduino map function
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

// PID array (6 pids, two for each axis)
PID pids[6];
#define PID_PITCH_RATE 0
#define PID_ROLL_RATE 1
#define PID_PITCH_STAB 2
#define PID_ROLL_STAB 3
#define PID_YAW_RATE 4
#define PID_YAW_STAB 5

float roll, yaw, pitch;

void setup() 
{
  
  // Enable the motors and set at 490Hz update
  hal.rcout->set_freq(0xF, 490);
  hal.rcout->enable_mask(0xFF);

  // PID Configuration
  pids[PID_PITCH_RATE].kP(0.7);
  pids[PID_PITCH_RATE].kI(1);
  pids[PID_PITCH_RATE].imax(50);
  
  pids[PID_ROLL_RATE].kP(0.7);
  pids[PID_ROLL_RATE].kI(1);
  pids[PID_ROLL_RATE].imax(50);
  
  pids[PID_YAW_RATE].kP(2.7);
  pids[PID_YAW_RATE].kI(1);
  pids[PID_YAW_RATE].imax(50);
  
  pids[PID_PITCH_STAB].kP(4.5);
  pids[PID_ROLL_STAB].kP(4.5);
  pids[PID_YAW_STAB].kP(10);

  // Turn off Barometer to avoid bus collisions
  hal.gpio->pinMode(40, GPIO_OUTPUT);
  hal.gpio->write(40, 1);
  
  // Turn on MPU6050 - quad must be kept still as gyros will calibrate
  ins.init(AP_InertialSensor::COLD_START, 
			 AP_InertialSensor::RATE_100HZ,
                        NULL);

  // initialise sensor fusion on MPU6050 chip (aka DigitalMotionProcessing/DMP)
  hal.scheduler->suspend_timer_procs();  // stop bus collisions
  ins.dmp_init();
  hal.scheduler->resume_timer_procs();
  
  
}

void loop(){

  while (ins.num_samples_available() == 0);
  
  
  ins.update(); 
  ins.quaternion.to_euler(&roll, &pitch, &yaw);
  roll = ToDeg(roll) ;
  pitch = ToDeg(pitch) ;
  yaw = ToDeg(yaw) ;
  hal.console->printf_P(
    PSTR("P:%4.1f  R:%4.1f Y:%4.1f\n"),
    pitch,
    roll,
    yaw);
    
  hal.scheduler->delay(2000);
    
}
 
AP_HAL_MAIN();



