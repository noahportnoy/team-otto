#include <stdarg.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
AP_InertialSensor_MPU6000 ins;

void setup(void)
{
    hal.console->println("AP_InertialSensor startup...");


#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    // we need to stop the barometer from holding the SPI bus
    hal.gpio->pinMode(40, GPIO_OUTPUT);
    hal.gpio->write(40, 1);
#endif

    ins.init(AP_InertialSensor::COLD_START, 
			 AP_InertialSensor::RATE_100HZ,
			 NULL);

    hal.console->println("Complete. Reading:");
}

void loop(void)
{
    int16_t user_input;

	while( !hal.console->available() ) {
        hal.scheduler->delay(500);
	hal.console->println("Waiting for input");
    }
	
	while ( hal.console->available() ){
		user_input = hal.console->read();
		hal.console->print("Input received: ");
		hal.console->println(user_input);
	}
	
}

AP_HAL_MAIN();
