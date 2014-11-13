#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;  // Hardware abstraction layer

#define RC_THR_MIN   1107
#define RC_YAW_MIN   1106
#define RC_YAW_MAX   1907
#define RC_PIT_MIN   1106
#define RC_PIT_MAX   1908
#define RC_ROL_MIN   1104
#define RC_ROL_MAX   1906

// Arduino map function
long map(long x, long in_min, long in_max, long out_min, long out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() 
{

}

void loop() 
{
  uint16_t channels[8];  // array for raw channel values
  
  // Read RC channels and store in channels array
  hal.rcin->read(channels, 8);
  
  // Copy from channels array to something human readable - array entry 0 = input 1, etc.
  uint16_t rcthr, rcyaw, rcpit, rcroll, safety;   // Variables to store rc input
  rcthr = channels[2];
  rcyaw = map(channels[3], RC_YAW_MIN, RC_YAW_MAX, -180, 180);
  rcpit = map(channels[0], RC_PIT_MIN, RC_PIT_MAX, -45, 45);
  rcroll = map(channels[1], RC_ROL_MIN, RC_ROL_MAX, -45, 45);
  safety = channels[4];
  
  hal.console->printf_P(
            PSTR("individual read THR %d YAW %d PIT %d ROLL %d, SAFE %d\r\n"),
            rcthr, rcyaw, rcpit, rcroll, safety);

  hal.scheduler->delay(500);  //Wait 50ms 
}

AP_HAL_MAIN();    // special macro that replace's one of Arduino's to setup the code (e.g. ensure loop() is called in a loop).
