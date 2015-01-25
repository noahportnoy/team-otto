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
#include <AP_HAL_Empty.h>
#include <AP_HAL_PX4.h>
#include <AP_GPS.h>
#include <AP_Math.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

void setup()
{
    
    hal.uartC->begin(38400); //baudrate
    hal.console->println("UARTC (UART2) Test");
}

void loop()
{
    hal.uartC->println("hello");
    
    //I think this is how you read:
    //data = hal.uartBC>read();
    
    hal.scheduler->delay(20);
    
    /*Details here: http://diydrones.com/forum/topics/how-to-get-data-from-ardupilot-through-serial-port*/
    
}

AP_HAL_MAIN();
