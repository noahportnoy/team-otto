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
    hal.uartC->begin(115200, 16, 16); 
    hal.console->println("UARTC (UART2) Test");
    //Uart messaging
    uartMessaging.init(hal.uartC, hal.console);
    
}
float lat, lon;
void loop()
{
  hal.scheduler->delay(1000);
  //receive from uart
  uartMessaging.receive();
  
  //send alt and battery status
  uartMessaging.sendAltitude(25.21255f);
  uartMessaging.sendBattery(01.21255f);
  uartMessaging.sendGPSLock(true);
  uartMessaging.sendSafetyStatus(false);
  
  if(uartMessaging.isUserLonLatest())
    uartMessaging.getUserLon(&lon);
  
  if(uartMessaging.isUserLatLatest())
    uartMessaging.getUserLat(&lat);
    
}

AP_HAL_MAIN();
