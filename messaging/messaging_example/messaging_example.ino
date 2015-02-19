// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
/*
 *       Example of GPS UBlox library.
 *       Code by Jordi Munoz and Jose Julio. DIYDrones.com
 *
 *       Works with Ardupilot Mega Hardware (GPS on Serial Port1)
 */

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
void setup()
{
    //Initializes the UART C bus (begin(baudrate, rx buffer, tx buffer)
    //See UARTDriver.h for more...
    hal.uartC->begin(115200, 32, 32); 
    hal.console->println("UARTC Test");
    //Uart messaging
    uartMessaging.init(hal.uartC, hal.console);
    
}
int32_t lat, lon;
unsigned long time = 0;

void loop()
{
  //receive from uart
  uartMessaging.receive();
  
  
  if((hal.scheduler->millis() - time) > 2000)
  {
    uartMessaging.sendAltitude(-25.21255f);
    uartMessaging.sendBattery(01.21255f);
    uartMessaging.sendClimbRate(2.1);
    time = hal.scheduler->millis();
  }
  
  if(uartMessaging.isUserLonLatest())
  {
    uartMessaging.getUserLon(&lon);
    hal.console->println(lon/1000000.0);
  }
  
  if(uartMessaging.isUserLatLatest())
  {
    uartMessaging.getUserLat(&lat);
     hal.console->println(lat/1000000.0);
  }
    
}

AP_HAL_MAIN();
