#include <stdlib.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <UARTDriver.h>
#include <AP_HAL_Empty.h>
#include <AP_HAL_PX4.h>
#include <AP_GPS.h>
#include <AP_Math.h>
//include uart messaging header file
#include <UartMessaging.h>

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
UartMessaging uartMessaging;

int32_t lat, lon;
int seperationDistance;


unsigned long time = 0;

void setup()
{
    //Initializes the UART C bus (begin(baudrate, rx buffer, tx buffer)
    //See UARTDriver.h for more...
    hal.uartC->begin(115200, 32, 32); 
    hal.console->println("UARTC Test");
    //Uart messaging
    uartMessaging.init(hal.uartC, hal.console);
    
}
long temp = 521234567;

void loop()
{
  //receive from uart
  uartMessaging.receive();
  
  temp++;
  if((hal.scheduler->millis() - time) > 1000)
  {
    temp++;
    uartMessaging.sendAltitude(-25.21255f); //altitude of the drone as a float in meters
    uartMessaging.sendBattery(01.21255f);  //battery level of the drone as a float in volts
    uartMessaging.sendClimbRate(2.1);  //climb rate of drone as a float in meters/second
    uartMessaging.sendGPSStatus(2);  //GPS status of the drone an int(2 bytes)/long(4 bytes) or Fix_Status enum
    uartMessaging.sendDroneLat(temp);  //GPS latitude * 10 000 000 of the drone as a int32_t in degress
    uartMessaging.sendDroneLon(651234567);  //GPS longitude * 10 000 000 of the drone as a int32_t in degrees
    uartMessaging.sendGPSAccuracy(4.6);    //GPS accuracy of the drone as a float in meters
    uartMessaging.sendBearing(165.2);
    uartMessaging.sendSeperationhDistance(25.2);
    time = hal.scheduler->millis();
    
  }
  
  if(uartMessaging.isUserLonLatest())
  {
    uartMessaging.getUserLon(&lon);
    hal.console->println(lon);
  }
  
  if(uartMessaging.isUserLatLatest())
  {
    uartMessaging.getUserLat(&lat);
     hal.console->println(lat);
  }
  
  if(uartMessaging.isSeperationDistanceLatest())
  {
    uartMessaging.getSeperationDistance(&seperationDistance);
     hal.console->println(seperationDistance);
  }
  
  uartMessaging.isLand();
  uartMessaging.isTakeOff();
    
}

AP_HAL_MAIN();
