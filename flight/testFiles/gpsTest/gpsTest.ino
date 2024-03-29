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

AP_GPS_UBLOX gps;
#define T6 1000000
#define T7 10000000

void setup()
{
    hal.uartB->begin(38400);
    gps.print_errors = true;

    hal.console->println("GPS UBLOX library test");
    gps.init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_2G);       // GPS Initialization
}

void loop()
{
    hal.scheduler->delay(1000);
    gps.update();
    hal.console->println("test");
    if ( true ) {
        hal.console->print("gps:");
        hal.console->print(" Lat:");
        hal.console->print((float)gps.latitude / T7, DEC);
        hal.console->print(" Lon:");
        hal.console->print((float)gps.longitude / T7, DEC);
        hal.console->print(" Alt:");
        hal.console->print((float)gps.altitude / 100.0, DEC);
        hal.console->print(" GSP:");
        hal.console->print(gps.ground_speed / 100.0);
        hal.console->print(" COG:");
        hal.console->print(gps.ground_course / 100.0, DEC);
		Vector2f vel = Vector2f(gps.velocity_north(), gps.velocity_east());
        hal.console->printf(" VEL: %.2f %.2f %.2f",
							vel.x,
							vel.y,
							vel.length());
        hal.console->print(" SAT:");
        hal.console->print(gps.num_sats, DEC);
        hal.console->print(" FIX:");
        hal.console->print(gps.fix, DEC);
        hal.console->print(" TIM:");
        hal.console->print(gps.time, DEC);
        hal.console->println();
        gps.new_data = 0; // We have readed the data
    }
}

AP_HAL_MAIN();
