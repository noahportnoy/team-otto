#include <AP_Compass.h>
#include <AP_Compass_HMC5843.h>
#include <Compass.h>

#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>
#include <PID.h>
#include <AP_Declination.h>
#include <AP_Math.h>
#include <AP_Buffer.h>
#include <Filter.h>
#include <AP_Baro.h>  //altitude
//#include <AP_BattMonitor.h>  //battery Monitor
#include <AP_GPS.h>

// ArduPilot Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

// MPU6050 accel/gyro chip
AP_InertialSensor_MPU6000 ins;

//Batery Monitor
//AP_BattMonitor battery_mon;

//Gps Device
GPS         *gps;
AP_GPS_Auto GPS(&gps);

AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);


/*------------------------------------------------ SYSTEM DEFINITIONS ------------------------------------------------------*/
// Radio min/max values for each stick for my radio (worked out at beginning of article)
#define RC_THR_MIN   1107
#define RC_THR_MAX   1907
#define RC_YAW_MIN   1106
#define RC_YAW_MAX   1907
#define RC_PIT_MIN   1106
#define RC_PIT_MAX   1908
#define RC_ROL_MIN   1104
#define RC_ROL_MAX   1906
#define RC_ALT_MIN   0
#define RC_ALT_MAX   1

// Min/max throttle for autonomous takeoff
#define MAX_TAKEOFF_THR 1270
#define MIN_TAKEOFF_THR 1250

// Motor numbers definitions
#define MOTOR_FL   2    // Front left    
#define MOTOR_FR   0    // Front right
#define MOTOR_BL   1    // back left
#define MOTOR_BR   3    // back right

#define OFF_BUTTON 0
AP_HAL::AnalogSource* OFF_BUTTON_VALUE;

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
	AP_Compass_PX4 compass;
#else
	AP_Compass_HMC5843 compass;
#endif
uint32_t timer;
uint32_t interval;

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

// PID array (7 pids, two for each axis, 1 for altitude)
PID pids[8];
#define PID_PITCH_RATE 0
#define PID_ROLL_RATE 1
#define PID_PITCH_STAB 2
#define PID_ROLL_STAB 3
#define PID_YAW_RATE 4
#define PID_YAW_STAB 5
#define ALT_STAB 6
#define ALT_RATE 7

// switchStatus
#define OFF 0
#define AUTONOMOUS 1
#define MANUAL 2

// flightStatus
#define TAKEOFF 3
#define HOLD 4
#define LAND 5

// PID configurations
#define DEFAULT 0
#define CUSTOM 1

//Hover throttle
#define HOVER_THR 1290




/*------------------------------------------------ DECLARE GLOBAL VARIABLES ------------------------------------------------------*/
int startup = 0;
float originalOrientation = 0.0;
float currentOrientation = 0.0;
float alt = 0;
float last_alt = 0;
float climb_rate = 0;
float last_climb_rate = 0;
long rcthr = 1000;
int heightLock = 0;
int switchStatus = 0;
int flightStatus = 0;
Matrix3f dcm_matrix;


/*---------------------------------------------------- SETUP ----------------------------------------------*/
void setup() {
	setupMotors();
	setPidConstants(DEFAULT);
	setupBarometer();
	setupMPU();
	
	OFF_BUTTON_VALUE = hal.analogin->channel(OFF_BUTTON);
	
	setupCompass();

	hal.scheduler->delay(1000);
	timer = hal.scheduler->micros();
	interval = timer;
	
	setupRpi();
	setupGPS();
	setupBatteryMonitor();
}

/*---------------------------------------------- LOOP -----------------------------------------------------*/
void loop() {
	// Delay for console output
	// hal.scheduler->delay(20);

	static float yaw_target = 0;  

	// Wait until new orientation data (normally 5ms max)
	while (ins.num_samples_available() == 0);
	
	// Copy from channels array to something human readable - array entry 0 = input 1, etc.
	uint16_t channels[8];  // array for raw channel values
	hal.rcin->read(channels, 8);  

	long rcyaw, rcpit, rcroll, safety, last_rcthr;  // Variables to store radio in
	float rcalt;
	safety = channels[4];
	
	float AVG_OFF_BUTTON_VALUE = OFF_BUTTON_VALUE->voltage_average();
	while ( (AVG_OFF_BUTTON_VALUE < 1.0) || (safety < 1500) ) {			// Kill motors when [off switch] or [safety] is on
		droneOff();

		AVG_OFF_BUTTON_VALUE = OFF_BUTTON_VALUE->voltage_average();
		
		hal.rcin->read(channels, 8);
    	safety = channels[4];
	}
	
	rcalt = 1.0; //Hard code in RCALT

	// GET RC yaw, pitch, and roll
	rcyaw = map(channels[3], RC_YAW_MIN, RC_YAW_MAX, -180, 180);
	rcpit = map(channels[0], RC_ROL_MIN, RC_ROL_MAX, 45, -45);
	rcroll = map(channels[1], RC_PIT_MIN, RC_PIT_MAX, 45, -45); 

	float pitch, roll, yaw;
	getOrientation(pitch, roll, yaw);
	
	if( (hal.scheduler->micros() - interval) > 100000UL ) {				// Update altitude data on interval
		getAltitudeData();
	}
	
	float gyroPitch, gyroRoll, gyroYaw;
	getGyro(gyroPitch, gyroRoll, gyroYaw);
	
	if(rcalt >= 0) {  // Altitude raised, turn on stablisation.
		// Stablise PIDS
		float pitch_stab_output = constrain(pids[PID_PITCH_STAB].get_pid((float)rcpit - pitch, 1), -250, 250); 
		float roll_stab_output = constrain(pids[PID_ROLL_STAB].get_pid((float)rcroll - roll, 1), -250, 250);
		float yaw_stab_output = constrain(pids[PID_YAW_STAB].get_pid((float)yaw_target - yaw, 1), -360, 360);
			
		// is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
		if(abs(rcyaw ) > 5) {
			yaw_stab_output = rcyaw;
			yaw_target = yaw;   // remember this yaw for when pilot stops
		}
		
		// rate PIDS
		long pitch_output =  (long) constrain(pids[PID_PITCH_RATE].get_pid(pitch_stab_output - gyroPitch, 1), -500, 500);  
		long roll_output =  (long) constrain(pids[PID_ROLL_RATE].get_pid(roll_stab_output - gyroRoll, 1), -500, 500);  
		long yaw_output =  (long) constrain(pids[PID_YAW_RATE].get_pid(yaw_stab_output - gyroYaw, 1), -500, 500);  


		//Feedback loop for altitude holding
		float alt_output = constrain(pids[ALT_STAB].get_pid((float)rcalt - alt, 1), -250, 250);
		//float alt_output = constrain(pids[ALT_RATE].get_pid(alt_stab_output - climb_rate, 1), -100, 100);
		
		getSwitchPosition(channels);									// Sets switchStatus to: OFF, AUTONOMOUS, or MANUAL
																		// depending on RC top-right switch position

		if (switchStatus == AUTONOMOUS) {
			// hal.console->print("DRONE IN AUTONOMOUS MODE: ");

			if (flightStatus == TAKEOFF) {

				// hal.console->println("TAKEOFF");
				rcthr = autonomousTakeoff(rcalt);

			} else if (flightStatus == HOLD) {							// Autonomous altitude hold

				// hal.console->println("HOLD");
				rcthr = autonomousHold(alt_output);

			} else {
				hal.console->print("Error: flightStatus of ");
				hal.console->print(flightStatus);
				hal.console->println(" has not been configured");
				while(1);
			}

		} else if (switchStatus == MANUAL) {

			// hal.console->println("DRONE IN MANUAL MODE");
			rcthr = map(channels[2], RC_THR_MIN, RC_THR_MAX, RC_THR_MIN, 1500);
		
		} else if (switchStatus == OFF) {

			// hal.console->println("DRONE IN OFF MODE");
			rcthr = 1000;
		}

		if (hal.uartC->available()) {
			hal.uartC->read();
		}
		
		// hal.console->print("Desired Alt, ");
		// hal.console->print(rcalt);
		// hal.console->print(", alt, ");
		// hal.console->print(alt);
		// hal.console->print(", Climb Rate, ");
		// hal.console->print(climb_rate); 
		// hal.console->print(", alt_output, ");
		// hal.console->print(alt_output); 
		// hal.console->print(", THR, ");
		// hal.console->println(rcthr);

		// mix pid outputs and send to the motors.
		hal.rcout->write(MOTOR_FL, rcthr + roll_output + pitch_output - yaw_output);
		hal.rcout->write(MOTOR_BL, rcthr + roll_output - pitch_output + yaw_output);
		hal.rcout->write(MOTOR_FR, rcthr - roll_output + pitch_output + yaw_output);
		hal.rcout->write(MOTOR_BR, rcthr - roll_output - pitch_output - yaw_output);
			
	} else {														// motors off
		hal.rcout->write(MOTOR_FL, 1000);
		hal.rcout->write(MOTOR_BL, 1000);
		hal.rcout->write(MOTOR_FR, 1000);
		hal.rcout->write(MOTOR_BR, 1000);
		
		// TODO investigate 
		yaw_target = yaw;											// reset yaw target so we maintain this on takeoff
		
		// TODO remove, unnecessary?
		for(int i=0; i<8; i++) {									// reset PID integrals whilst on the ground
			pids[i].reset_I();
		}
	}
}

// Arduino map function
float map(float x, float in_min, float in_max, float out_min, float out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float calculateYaw(){ 
	static float min[3], max[3], offset[3];
	float heading;
	compass.accumulate();
	if((hal.scheduler->micros()- timer) > 100000L)
	{
		timer = hal.scheduler->micros();
		compass.read();
		unsigned long read_time = hal.scheduler->micros() - timer;
		
		if (!compass.healthy) {
			hal.console->println("not healthy");
			return 0;
		}
		dcm_matrix.to_euler(0,0, 0);
		heading = compass.calculate_heading(dcm_matrix); // roll = 0, pitch = 0 for this example
		compass.null_offsets();

		// capture min
		if( compass.mag_x < min[0] )
			min[0] = compass.mag_x;
		if( compass.mag_y < min[1] )
			min[1] = compass.mag_y;
		if( compass.mag_z < min[2] )
			min[2] = compass.mag_z;

		// capture max
		if( compass.mag_x > max[0] )
			max[0] = compass.mag_x;
		if( compass.mag_y > max[1] )
			max[1] = compass.mag_y;
		if( compass.mag_z > max[2] )
			max[2] = compass.mag_z;

		// calculate offsets
		offset[0] = -(max[0]+min[0])/2;
		offset[1] = -(max[1]+min[1])/2;
		offset[2] = -(max[2]+min[2])/2;
	} else {
		hal.scheduler->delay(1);
	}  
	
	currentOrientation = ToDeg(heading);
	if(currentOrientation < 0) {
		currentOrientation = 360 + currentOrientation;
	}
	 
	if(startup == 1) {
		originalOrientation = currentOrientation;
		startup = 2;
	}
	if(startup == 0)
	 	startup = 1;
 	return wrap_180( currentOrientation - originalOrientation );   
}

float getAltitude(){
	baro.read();
	if (!baro.healthy) {
		hal.console->println("not healthy");
		return 0.0;
	}

	float a = baro.get_altitude();
	return a;
}

float getClimbRate() {
	baro.read();
	if (!baro.healthy) {
		hal.console->println("not healthy");
		return 0.0;
	}
	
	return (baro.get_climb_rate());
}

void getAltitudeData() {
	last_alt = alt;
	alt = getAltitude();
	
	//Smooth raw data (last parameter is the smoothing constant)
	float temp = movingAvg(last_alt, alt, .8);
	alt = temp;
	
	//Verify that the altitude values are within scope
	if(abs(alt-last_alt) > 100){
		alt=last_alt;
	}

	//Get Climb Rate and smooth (last parameter is the smoothing constant)
	last_climb_rate = climb_rate;
	climb_rate = constrain(getClimbRate(), -15, 15);
	climb_rate = movingAvg(last_climb_rate, climb_rate, .9);
	
	//Verify that the climb rate values are within scope
	if(abs(climb_rate-last_climb_rate) > 100){
		climb_rate=last_climb_rate;
	}
	
	//Scheduling
	interval = hal.scheduler->micros();  
	
	//GET BATTERY STATS at the same frequency of the Altitude check
	// update voltage and current readings
	//battery_mon.read();
	
	
	/*hal.console->printf("\nVoltage: %.2f \tCurrent: %.2f \tTotCurr:%.2f",
					battery_mon.voltage(),
					battery_mon.current_amps(),
							battery_mon.current_total_mah());
	*/
}

void getOrientation(float &pitch, float &roll, float &yaw) {
	ins.update();											// Ask MPU6050 for orientation
	ins.quaternion.to_euler(&roll, &pitch, &yaw);
	
	pitch = ToDeg(pitch);
	roll = ToDeg(roll);
	yaw = ToDeg(yaw);
}

void getGyro(float &gyroPitch, float &gyroRoll, float &gyroYaw) {
	Vector3f gyro = ins.get_gyro();							// Ask MPU6050 for gyro data

	gyroPitch = ToDeg(gyro.y);
	gyroRoll = ToDeg(gyro.x);
	gyroYaw = ToDeg(gyro.z);
}

//Exponential Moving Average  ->  http://en.wikipedia.org/wiki/Exponential_smoothing
float exponentialSmoother(float previous_value, float current_value, float a){
	//a is a constant the smoothing factor
	return (previous_value + a*(current_value-previous_value));
}

//Moving Average Filter
float movingAvg(float previous, float current, float a){
	//a is a constant smoothing factor. Large a means a lot of weight is put on the previous value
	//a must be between 0 and 1
	if(abs(a) > 0) {
		hal.console->println("Error: a must be between 0 and 1");
		a = 0;
	}

	return (previous*a + (1-a)*current);
}

void setPidConstants(int config) {
	if (config == DEFAULT) {
		pids[PID_PITCH_RATE].kP(0.45);
		pids[PID_PITCH_RATE].kI(0.1);
		pids[PID_PITCH_RATE].imax(50);
		
		pids[PID_ROLL_RATE].kP(0.45);
		pids[PID_ROLL_RATE].kI(0.1);
		pids[PID_ROLL_RATE].imax(50);
		
		pids[PID_YAW_RATE].kP(0.7);
		pids[PID_YAW_RATE].kI(0.1);
		pids[PID_YAW_RATE].imax(50);
		
		pids[PID_PITCH_STAB].kP(4.5);
		pids[PID_ROLL_STAB].kP(4.5);
		pids[PID_YAW_STAB].kP(10);
		
		pids[ALT_RATE].kP(0.1);
		pids[ALT_RATE].kI(0.0);
		pids[ALT_RATE].imax(50);
		
		pids[ALT_STAB].kP(10.0);
		pids[ALT_STAB].kI(0.0);
		pids[ALT_STAB].imax(50);
	} else {
		hal.console->print("Error: PID constants not set for provided configuration ");
		hal.console->println(config);
		while(1);
	}
}

void droneOff() {

	flightStatus = OFF;

	// hal.console->println("DRONE OFF / safety");

	hal.rcout->write(MOTOR_FL, 1000);
	hal.rcout->write(MOTOR_BL, 1000);
	hal.rcout->write(MOTOR_FR, 1000);
	hal.rcout->write(MOTOR_BR, 1000);    
	
	//hal.console->printf_P(PSTR("Voltage ch0:%.2f\n"), AVG_OFF_BUTTON_VALUE);
	//hal.scheduler->delay(500);
	
	//GET BATTERY STATS
	// update voltage and current readings
	//battery_mon.read();
	//hal.console->printf("\nVoltage: %.2f \tCurrent: %.2f \tTotCurr:%.2f  ",
	//        battery_mon.voltage(), //voltage
	//        battery_mon.current_amps(), //Inst current
	//              battery_mon.current_total_mah()); //Accumulated current
							
	//GET GPS STATS
	gps->update();
	if (gps->new_data) {
		// hal.console->print("Lat, ");
		// hal.console->print(gps->latitude/10000000.0);
		// hal.console->print(", Lon, ");
		// hal.console->print(gps->longitude/10000000.0);
		// hal.console->print(", g_speed, ");
		// hal.console->println(gps->ground_speed/100.0);
		// hal.console->printf(" Alt: %.2fm GSP: %.2fm/s CoG: %d SAT: %d TIM: %lu STATUS: %u\n",
		//               (float)gps->altitude / 100.0,
		//               (float)gps->ground_speed / 100.0,
		//               (int)gps->ground_course / 100,
		//               gps->num_sats,
		//               gps->time,
		//               gps->status()
		//               );
	}
		
	for(int i=0; i<8; i++) {								// reset PID integrals
		pids[i].reset_I();
	}
}

long autonomousTakeoff(float rcalt) {
	if (alt < (rcalt/2)) {									// Otto is below rcalt/2
		rcthr = MAX_TAKEOFF_THR;
	} else if (alt < rcalt) {								// Otto is between rcalt/2 and rcalt
		rcthr = map(alt, rcalt/2, rcalt, MAX_TAKEOFF_THR,
					MIN_TAKEOFF_THR);
	} else {												// Otto is above rcalt
		for(int i=0; i<8; i++) {							// reset PID integrals for altitude hold
			pids[i].reset_I();
		}
		setPidConstants(DEFAULT);
		flightStatus = HOLD;
	}

	return rcthr;
}

long autonomousHold(float alt_output) {
	//Map the Throttle
	rcthr = HOVER_THR + alt_output;
	rcthr = constrain(rcthr, 1200, 1400);

	return rcthr;
}

void getSwitchPosition(uint16_t channels[]) {
	//F.Mode is channels[5]
	//TOP = greater than 1100
	//MIDDLE = greater than 1500
	//BOTTOM = greater than 1900

	if (channels[5] > 1700) {										// Otto off: switch is in BOTTOM position
		switchStatus = OFF;

	} else if ((1200 < channels[5]) && (channels[5] < 1700)) {		// Autonomous flight: switch is in MIDDLE position

		if (switchStatus == OFF || switchStatus == MANUAL) {		// If switching to AUTONOMOUS, reset PIDs and set flightStatus to TAKEOFF
			pids[ALT_STAB].reset_I();								// reset i; reset PID integrals while in manual mode
			setPidConstants(DEFAULT);								// PID Configuration for takeoff
			flightStatus = TAKEOFF;
		}

		if (flightStatus == OFF) {									// If safety was just turned off
			flightStatus = TAKEOFF;
		}

		switchStatus = AUTONOMOUS;

	} else {														// Manual control: switch is in BOTTOM position

		if (switchStatus == AUTONOMOUS || switchStatus == OFF) {	// If switching to manual control, reset PIDs
			pids[ALT_STAB].reset_I();								// reset i; reset PID integrals while in manual mode
			setPidConstants(DEFAULT);								// Set PIDs for manual control
		}

		switchStatus = MANUAL;
	}
}

void setupCompass() {
	if (!compass.init()) {
		hal.console->println("compass initialization failed!");
		while (1) ;
	}

	//compass.set_orientation(AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD); // set compass's orientation on aircraft.
	compass.set_offsets(0,0,0); // set offsets to account for surrounding interference
	compass.set_declination(ToRad(0.0)); // set local difference between magnetic north and true north

	hal.console->print("\nCompass auto-detected as: ");
	switch( compass.product_id ) {
		case AP_COMPASS_TYPE_HMC5843:
			hal.console->println("HMC5843");
			break;
		case AP_COMPASS_TYPE_HMC5883L:
			hal.console->println("HMC5883L");
			break;
		default:
			hal.console->println("unknown");
			break;
	}
}

void setupBatteryMonitor() {
	// Initialize the battery monitor
	//  battery_mon.init();
	//  battery_mon.set_monitoring(AP_BATT_MONITOR_VOLTAGE_AND_CURRENT);
	//  hal.console->println("Battery monitor initialized");
}

void setupMotors() {
	// Enable the motors and set at 490Hz update
	hal.rcout->set_freq(0xF, 490);
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
							NULL);

	// initialise sensor fusion on MPU6050 chip (aka DigitalMotionProcessing/DMP)
	hal.scheduler->suspend_timer_procs();  // stop bus collisions
	//ins.dmp_init();
	hal.scheduler->resume_timer_procs();
}

void setupGPS() {
	//SET UP UARTB CONNECTION FOR GPS
	hal.uartB->begin(38400);
	gps = &GPS;
	gps->init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_2G);       // GPS Initialization
}

void setupRpi() {
	//SET UP UARTC CONNECTION FOR RPI
	hal.uartC->begin(115200); //baudrate
}

AP_HAL_MAIN();
