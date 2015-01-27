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

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

void setup()
{
    //Initializes the UART C bus (begin(baudrate, rx buffer, tx buffer)
    //See UARTDriver.h for more...
    hal.uartC->begin(115200, 16, 16); 
    hal.console->println("UARTC (UART2) Test");
}
int a = 0;
void loop()
{
    hal.uartC->println(10);
    
    //Check to see if there is something on the buffer to read
    if (hal.uartC->available()) {
      hal.console->println("avaliable");
      //read data in buffer
      hal.console->println(hal.uartC->read());
      
      //Counter
      a++;
      hal.console->println(a);
    }
    
    //Write Hello to TX
    hal.uartC->write("Hello");
    hal.console->println("Sent hello");
    /*Details here: http://diydrones.com/forum/topics/how-to-get-data-from-ardupilot-through-serial-port*/
    
}

AP_HAL_MAIN();
