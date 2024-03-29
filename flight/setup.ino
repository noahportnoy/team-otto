
void setupMotors() {
	unsigned int escFreq;

	if(ESC_CALIBRATION) {
		escFreq = 50;
	} else {
		escFreq = 490;
	}

	// Enable the motors and set at 490Hz update
	hal.rcout->set_freq(0xF, escFreq);
	hal.rcout->enable_mask(0xFF);
}

void setupBarometer() {
	hal.console->println("Barometer Init");
	hal.gpio->pinMode(63, GPIO_OUTPUT);
	hal.gpio->write(63, 1);
	baro.init();
	baro.calibrate();
}

void setupMPU() {
	// Turn on MPU6050 - quad must be kept still as gyros will calibrate

	ins.init(AP_InertialSensor::COLD_START,
			 AP_InertialSensor::RATE_100HZ,
			 flash_leds);
	ins.init_accel(flash_leds);

	// initialise sensor fusion on MPU6050 chip (aka DigitalMotionProcessing/DMP)
	hal.scheduler->suspend_timer_procs();  // stop bus collisions
	ins.dmp_init();
	hal.scheduler->resume_timer_procs();
}

void setupOffButton() {
	OFF_BUTTON_VALUE = hal.analogin->channel(OFF_BUTTON);
}

void setupCompass() {
	if (!compass.init()) {
		hal.console->println("compass initialization failed!");
		while (1) ;
	}

	compass.set_orientation(ROTATION_ROLL_180);							// set compass's orientation on aircraft.

	//These offsets came from the ARDU_PILOT Compass calibration
	// compass.set_offsets(-42.642, 16.306, 18.598);			//Original but off from bearing
	
	//Cal in Engin quad on 4/19/2015
	// compass.set_offsets(-39.888, 9.952, -11.112);			//Off from bearing

	//Cal in Engin quad on 4/19/2015
	// compass.set_offsets(-43.586, 14.188, -5.152);			//Off from bearing by about 10

	// set offsets to account for surrounding interference
	// compass.set_offsets(-37, 7, 18);									// noah's offsets from mission planner //Off from bearing by about 20

	//Cal in Engin quad on 4/19/2015
	compass.set_offsets(-43.502, 25.224, -6.290);

	//Try this: https://www.youtube.com/watch?v=-_mjfPlHL9o
	//Try to get rid of declination for bearing...

	compass.set_declination(ToRad(-14.167));								// set local difference between magnetic north and true north

	//Otto uses the HMC5883L Compass
}

void setupTiming() {
	hal.scheduler->delay(1000);
	uint32_t timer = hal.scheduler->micros();
	altitude_timer = timer;
	send_to_phone_timer = timer;
	heading_timer = timer;
	hover_thr_timer = timer;
}

void setupRpi() {
	//Initializes the UART C bus (begin(baudrate, rx buffer, tx buffer)
	//See UARTDriver.h for more...
	hal.uartC->begin(115200, 32, 32);
	hal.console->println("UARTC (UART2) Test");
	//Uart messaging
	uartMessaging.init(hal.uartC, hal.console);
}

void setupGPS() {
	//SET UP UARTB CONNECTION FOR GPS
	hal.uartB->begin(38400);
	gps = &GPS;
	gps->init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_2G);       			// GPS Initialization
}

void setupBatteryMonitor() {
	//Initialize the battery monitor
	battery_mon.init();
	battery_mon.set_monitoring(AP_BATT_MONITOR_VOLTAGE_AND_CURRENT);
	hal.console->println("Battery monitor initialized");
}

void flash_leds(bool on) {
	hal.gpio->write(A_LED_PIN, on ? LED_OFF : LED_ON);
	hal.gpio->write(C_LED_PIN, on ? LED_ON : LED_OFF);
}
