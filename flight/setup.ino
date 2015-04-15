
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
	ahrs.init();
	float roll_trim, pitch_trim;
	ins.calibrate_accel( NULL , NULL , roll_trim , pitch_trim );
	ahrs.set_trim( Vector3f( roll_trim , pitch_trim , 0 ) );
	
	Vector3f test = ins.get_accel_offsets();
	Vector3f test_s = ins.get_accel_scale();
	Vector3f gyro_t = ins.get_gyro_offsets();
	
	hal.console->printf_P(
		PSTR("\nAccel Offsets X:%10.8f \t Y:%10.8f \t Z:%10.8f\n"),
				test.x,
				test.y,
				test.z);
				
	hal.console->printf_P(
		PSTR("\nAccel Scale	 X:%10.8f \t Y:%10.8f \t Z:%10.8f\n"),
				test_s.x,
				test_s.y,
				test_s.z);
	
	hal.console->printf_P(
		PSTR("\nGyro Offsets X:%10.8f \t Y:%10.8f \t Z:%10.8f\n"),
				gyro_t.x,
				gyro_t.y,
				gyro_t.z);
	
	// Vector3f accel_offset = Vector3f( 0.04854903 , 0.14138588 , 1.95303880 );
    // Vector3f accel_scale = Vector3f( 0.99788916 , 0.99240148 , 0.98475480 );
	
	//ins.set_accel_offsets( accel_offset );
	
	//test = ins.get_accel_offsets();
	
	// hal.console->printf_P(
		// PSTR("\nAccel Offsets X:%10.8f \t Y:%10.8f \t Z:%10.8f\n"),
				// test.x,
				// test.y,
				// test.z);
	
	

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

	//These offsets came from the ARDU_PILOT Compass calibratio
	compass.set_offsets(-42.642, 16.306, 18.598);							// set offsets to account for surrounding interference
	//compass.set_offsets(-37, 7, 18);									// noah's offsets from mission planner
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
