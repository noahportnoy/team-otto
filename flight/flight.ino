#include <AP_Compass.h>
#include <AP_Compass_HMC5843.h>
#include <Compass.h>

#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <AP_AHRS.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>
#include <GCS_MAVLink.h>

#include <SITL.h>
#include <PID.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_Math.h>
#include <AP_Buffer.h>
#include <Filter.h>
#include <AP_Baro.h>  //altitude
#include <AP_BattMonitor.h>  //battery Monitor
#include <AP_GPS.h>

//include uart messaging header file
#include <UartMessaging.h>

/*------------------------------------------------ SYSTEM DEFINITIONS ------------------------------------------------------*/

/* 	------------------ Calibration Documentation ------------------
*
*
*	You have several options to modify Otto's throttle response.
*
* 	ESC CALIBRATION
*		To calibrate the ESCs, set ESC_CALIBRATION to 1 and specify ESC_CAL_MIN and ESC_CAL_MAX as you desire.
*
*
*	CALIBRATION COMPENSATION
*		1. Use type NONE when you are trying out a different ESC calibration and do not need compensation in software.
*
*		2. Use type PID_GAIN when you wish to compensate by scaling the PID output.
*		   Note: it is recommended that you use a calibration of 1107-1907 (ESC or SW) to maintain consistency.
*				REQUIRED: 	PID_GAIN_VAL
*							RC_THR_MIN_MAPPED
*							RC_THR_MAX_MAPPED
*
*		3. Use type SW_CAL when you would like to emulate an ESC calibration in software.
*				REQUIRED: 	SW_CAL_MIN
*							SW_CAL_MAX
*							RC_THR_MIN_MAPPED
*							RC_THR_MAX_MAPPED
*
*		4. Use type MIXED when you would like to use SW_CAL and PID_GAIN at the same time.
*		   Note: it is recommended that you use a SW_CAL of 1107-1907 to maintain consistency.
*				REQUIRED: 	SW_CAL_MIN
*							SW_CAL_MAX
*							PID_GAIN_VAL
*							RC_THR_MIN_MAPPED
*							RC_THR_MAX_MAPPED
*/

// ------------------------- Calibration Settings --------------------------
// Definitions for calibration compensation types. Do not change.
#define NONE		0
#define PID_GAIN 	1
#define SW_CAL 		2
#define MIXED 		3

// These MUST be kept up to date
#define ESC_CALIBRATION	0
#define ESC_CAL_MIN		1107
#define ESC_CAL_MAX		1460
#define CAL_COMP_TYPE	MIXED

// Change these to change the throttle response
#define PID_GAIN_VAL			2.859
#define SW_CAL_MIN				1107
#define SW_CAL_MAX				1907
#define RC_THR_MIN_MAPPED		1107
#define RC_THR_MAX_MAPPED		1907
// ---------------------- End of Calibration Settings ----------------------


// Radio min/max values for each stick for my radio (worked out at beginning of article)
#define RC_THR_MIN   1107
#define RC_THR_MAX   1907
#define RC_YAW_MIN   1106
#define RC_YAW_MAX   1908
#define RC_PIT_MIN   1106
#define RC_PIT_MAX   1908
#define RC_ROL_MIN   1104
#define RC_ROL_MAX   1906

//Set hover throttle definitions
#define Static_HOVER_THR		1620
unsigned int HOVER_THR = Static_HOVER_THR;

#define ADJ_THR_THRESHOLD 		Static_HOVER_THR-120
#define ADJ_THR_MIN				Static_HOVER_THR-15
#define ADJ_THR_MAX				Static_HOVER_THR+50
#define MAX_TAKEOFF_THR 		Static_HOVER_THR+15
#define MIN_TAKEOFF_THR 		Static_HOVER_THR-15
#define MIN_THR_CONSTRAINT		Static_HOVER_THR-320
#define MAX_THR_CONSTRAINT		Static_HOVER_THR+130

// Motor numbers definitions
#define MOTOR_FL   2    // Front left
#define MOTOR_FR   0    // Front right
#define MOTOR_BL   1    // back left
#define MOTOR_BR   3    // back right

#define OFF_BUTTON 0

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

// PID array (11 pids, two for each axis, 2 for altitude, 3 for AUTONOMOUS commands)
PID pids[11];
#define PID_PITCH_RATE 0
#define PID_ROLL_RATE 1
#define PID_PITCH_STAB 2
#define PID_ROLL_STAB 3
#define PID_YAW_RATE 4
#define PID_YAW_STAB 5
#define ALT_STAB 6
#define YAW_CMD 8
#define PITCH_CMD 9
#define ROLL_CMD 10

// switchState
#define MANUAL 0
#define AUTO_ALT_HOLD 1
#define AUTO_PERFORMANCE 2

// autopilotState
#define OFF 3
#define TAKEOFF 4
#define ALT_HOLD 5
#define LAND 6

// PID configurations
#define DEFAULT 0
#define CUSTOM 1

// Debug ON/OFF
#define PRINT_DEBUG 0

// Define the HW LED setup & Compass orientation
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
 # define A_LED_PIN        27
 # define C_LED_PIN        25
 # define LED_ON           0  //Low
 # define LED_OFF          1  //High
 # define MAG_ORIENTATION  AP_COMPASS_APM2_SHIELD
#else
 # define A_LED_PIN        37
 # define C_LED_PIN        35
 # define LED_ON           1    //High
 # define LED_OFF          0    //Low
 # define MAG_ORIENTATION  AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD
#endif

#define TARGET_PHONE 0
#define TARGET_FIXED 1

const float INT_LAT_TO_METER = 0.01110809;
const float INT_LONG_TO_METER = 0.00823380;

/*------------------------------------------------ DECLARE GLOBAL VARIABLES ------------------------------------------------------*/
// ArduPilot Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

//Messaging Object
UartMessaging uartMessaging;

// MPU6050 accel/gyro chip
AP_InertialSensor_MPU6000 ins;

//Batery Monitor
AP_BattMonitor battery_mon;

//Gps Device
GPS         *gps;
AP_GPS_Auto GPS(&gps);

//Otto uses the MS5611 Baro
AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);

// choose which AHRS system to use
//AP_AHRS_DCM  ahrs(&ins, gps);
AP_AHRS_MPU6000  ahrs(&ins, gps);		// only works with APM2

AP_HAL::AnalogSource* OFF_BUTTON_VALUE;

//Initialize the HMC5843 compass.
AP_Compass_HMC5843 compass;

uint32_t altitude_timer;
uint32_t send_to_phone_timer;
uint32_t heading_timer;
uint32_t hover_thr_timer;

float current_heading = 0;
float desired_heading = 0;
float climb_rate = 0;
int switchState = 0;
int autopilotState = 0;


/*---------------------------------------------------- SETUP ----------------------------------------------*/
void setup() {
	setupMotors();
	setPidConstants(DEFAULT);
	setupMPU();
	setupOffButton();
	setupCompass();
	setupTiming();
	setupRpi();
	setupBarometer();
	setupGPS();
	setupBatteryMonitor();
	//Initizlize the Altitude Hold Refernece System
	ahrs.init();
	// getGPSLock();
	hal.console->println("Otto Ready.");
}

/*---------------------------------------------- LOOP -----------------------------------------------------*/
void loop() {
	static long rcthr, rcpit, rcroll, rcyaw, safety;						 // Variables to store radio in
	static float yaw_target = 0;
	static float desired_alt, alt;
	static float accelPitch, accelRoll, accelYaw;
	static float gyroPitch, gyroRoll, gyroYaw;
	static long pitch_output, roll_output, yaw_output, alt_output;
	static uint16_t channels[8];  // array for raw channel values
	static float AVG_OFF_BUTTON_VALUE;

	updateReadings(channels, safety, accelPitch, accelRoll, accelYaw, gyroPitch, gyroRoll, gyroYaw, alt, AVG_OFF_BUTTON_VALUE);
	sendDataToPhone(alt, rcthr);
	desired_alt = 1.0; //Hard code in desired_alt

	while((AVG_OFF_BUTTON_VALUE < 1.0) || (safety < 1500)) {			// Kill motors when [off switch] or [safety] is on
		updateReadings(channels, safety, accelPitch, accelRoll, accelYaw, gyroPitch, gyroRoll, gyroYaw, alt, AVG_OFF_BUTTON_VALUE);
		sendDataToPhone(alt, rcthr);
		droneOff();
		yaw_target = accelYaw;											// reset yaw target so we maintain this on takeoff
	}

	controlRcInputs(rcthr, rcpit, rcroll, rcyaw, desired_alt, alt_output, alt, channels);
	runFeedback(pitch_output, roll_output, yaw_output, alt_output, yaw_target, rcpit, rcroll, rcyaw, accelPitch, accelRoll, accelYaw, gyroPitch, gyroRoll, gyroYaw, alt, desired_alt);
	writeToMotors(rcthr, pitch_output, roll_output, yaw_output, yaw_target, accelYaw);

	if (PRINT_DEBUG) {
		// hal.console->print("rcthr, ");
		// hal.console->print(rcthr);
		// hal.console->print(", hoverthr, ");
		// hal.console->print(HOVER_THR);
		// hal.console->print(", rcpitch, ");
		// hal.console->print(rcpit);
		// hal.console->print(", rcroll, ");
		// hal.console->print(rcroll);
		// hal.console->print(",  accelPitch, ");
		// hal.console->print(accelPitch);
		// hal.console->print(",  accelRoll, ");
		// hal.console->print(accelRoll);
		// hal.console->print(",  accelYaw, ");
		// hal.console->print(accelYaw);
		// hal.console->print(", pitch_out: ");
		// hal.console->print(pitch_output);
		// hal.console->print(", roll_out: ");
		// hal.console->print(roll_output);
		// hal.console->print(", yaw_out: ");
		// hal.console->print(yaw_output);

		// hal.console->print(", battery, ");
		// hal.console->print(battery_mon.voltage());

		// hal.console->print(", switch_position, ");
		// hal.console->print(channels[5]);
		// hal.console->print(", heading, ");
		// hal.console->print(current_heading);
		// hal.console->print(", t, ");
		// hal.console->print(hal.scheduler->millis());

		// hal.console->println("");
	}
}

// Arduino map function
float map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Moving Average Filter
float movingAvg(float previous, float current, float a) {
	//a is a constant smoothing factor. Large a means a lot of weight is put on the previous value
	//a must be between 0 and 1
	if(abs(a) > 0) {
		// hal.console->println("Error: a must be between 0 and 1");
		a = 0;
	}

	return (previous*a + (1-a)*current);
}

void setPidConstants(int config) {
	if (config == DEFAULT) {
		//Attidude PID's - uses MPU
		pids[PID_PITCH_STAB].kP(6);
		pids[PID_PITCH_STAB].kI(3);
		pids[PID_PITCH_STAB].imax(50);

		pids[PID_ROLL_STAB].kP(6);
		pids[PID_ROLL_STAB].kI(3);
		pids[PID_ROLL_STAB].imax(50);

		pids[PID_YAW_STAB].kP(10);
		pids[PID_YAW_STAB].kI(0);
		pids[PID_YAW_STAB].imax(10);

		//Rate PID's - uses Gyro
		pids[PID_PITCH_RATE].kP(0.2);
		pids[PID_PITCH_RATE].kI(0.0);
		pids[PID_PITCH_RATE].imax(50);

		pids[PID_ROLL_RATE].kP(0.2);
		pids[PID_ROLL_RATE].kI(0.0);
		pids[PID_ROLL_RATE].imax(50);

		pids[PID_YAW_RATE].kP(0.7);
		pids[PID_YAW_RATE].kI(0.1);
		pids[PID_YAW_RATE].imax(50);

		//Below are the PIDs for altitude hold
		pids[ALT_STAB].kP(8);
		pids[ALT_STAB].kI(1.5);
		pids[ALT_STAB].kD(9);
		pids[ALT_STAB].imax(20);

		//Below are the PIDs for autonomous control
		pids[PITCH_CMD].kP(0.4);
		pids[PITCH_CMD].kI(0.0);
		pids[PITCH_CMD].imax(50);

		pids[ROLL_CMD].kP(0.4);
		pids[ROLL_CMD].kI(0.0);
		pids[ROLL_CMD].imax(50);

		pids[YAW_CMD].kP(0.7);
		pids[YAW_CMD].kI(0.1);
		pids[YAW_CMD].imax(50);
	} else {
		hal.console->print("Error: PID constants not set for provided configuration");
		hal.console->println(config);
		while(1);
	}
}

float getHeading() {
	/*COMPASS OPERATION:
		 True North is 0 (degrees)
		 Eastern headings are negative numbers
		 Western headings are positive numbers
		 South is 180 or -180
	*/

	//Use AHRS for Heading
	ahrs.update();
	compass.read();
	float heading = compass.calculate_heading(ahrs.get_dcm_matrix());
	//Vector3f drift  = ahrs.get_gyro_drift();
	compass.null_offsets();

	//NOTE: AHRS can provide pitch, roll, and yaw angles

	current_heading = ToDeg(heading);
	current_heading = -current_heading; 	// correct for proper handling in the rotation matrix

	return current_heading;
}


void gpsTracking(long &rcpit, long &rcroll) {
	float current_heading_rad;
	//Vector format is x,y,z
	Vector3f lat_long_error, autonomous_pitch_roll;
	Matrix3f yaw_rotation_m;
	int32_t drone_coordinates[] = {0, 0};
	int32_t target_coordinates[] = {0, 0};

	if (gps->status() < 2) {
		///PID Feedback system for pitch and roll input 0 is bad GPS state
		rcpit = 0;
		rcroll = 0;
		return;
	}

	getTargetCoordinates(target_coordinates, TARGET_FIXED);
	getDroneCoordinates(drone_coordinates);

	//Get Lat and Long error
	lat_long_error.x = (float)((target_coordinates[0] - drone_coordinates[0])*INT_LONG_TO_METER);
	lat_long_error.y = (float)((target_coordinates[1] - drone_coordinates[1])*INT_LAT_TO_METER);
	lat_long_error.z = 0;

	/*
	Generic Matrix Setup
	----           ----
	|  a.x  a.y  a.z  |
	|  b.x  b.y  b.z  |
	|  c.x  c.y  c.z  |
	----           ----
	*/

	//Get the radian representation of current_heading
	current_heading_rad = current_heading*PI/180;

	yaw_rotation_m.a = Vector3f(cos(current_heading_rad), sin(current_heading_rad), 0);
	yaw_rotation_m.b = Vector3f(-sin(current_heading_rad), cos(current_heading_rad), 0);
	yaw_rotation_m.c = Vector3f(0, 0, 1);

	//Multiply the lat_long_error matrix by the yaw rotation matrix to get pitch / roll proportions
	autonomous_pitch_roll = yaw_rotation_m*lat_long_error;

	//PID Feedback system for pitch and roll.
	rcpit = constrain(pids[PITCH_CMD].get_pid(autonomous_pitch_roll.y, 1), -5, 5);
	rcpit = -rcpit;		// flip rcpit for proper mapping (neg pitch is forward)
	rcroll = constrain(pids[ROLL_CMD].get_pid(autonomous_pitch_roll.x, 1), -5, 5);

	if (PRINT_DEBUG) {
		// hal.console->print("Yaw Rotation Matrix:  ");
		// hal.console->printf("a: %f %f %f b: %f %f %f    ", yaw_rotation_m.a.x, yaw_rotation_m.a.y, yaw_rotation_m.a.z, yaw_rotation_m.b.x, yaw_rotation_m.b.y, yaw_rotation_m.b.z);
		// hal.console->printf(", p/r: %f %f  ", autonomous_pitch_roll.x, autonomous_pitch_roll.y);

		// hal.console->printf(", gps status: %d", gps->status());
		// hal.console->printf(", currheading, %f, ", current_heading);
		// hal.console->print(", desired_heading, ");
		// hal.console->print(desired_heading);
		// hal.console->printf(",  drone_long, %ld, drone_lat, %ld, ", drone_coordinates[0], drone_coordinates[1]);
		// hal.console->printf(", target_long, %ld, target_lat, %ld, ", target_coordinates[0], target_coordinates[1]);
		// hal.console->printf(",  diff_long, %f, diff_lat, %f, ", lat_long_error.x, lat_long_error.y);
		// hal.console->print(",  desired heading, ");
		// hal.console->print(desired_heading);
		// hal.console->print(", seperation, ");
		// hal.console->print(seperation_dist);
		// hal.console->print(", accuracy, ");
		// hal.console->print(gps->horizontal_accuracy/1000.0);
	}

	/* ~~~~~ USE FOR SEPERATION DISTANCE ~~~~~~~~
	float gps_distance = get_distance(&loc, &loc2);
	hal.console->printf("\narducopter bearing: %f and distance: %f\n", bearing_heading, gps_distance);
	*/
}

//Coordinate Arrays: [latitude, longitude]
void getDroneCoordinates(int32_t drone_coordinates[]) {
	gps->update();
	if (gps->new_data) {
		drone_coordinates[1] = gps->latitude;
		drone_coordinates[0] = gps->longitude;
	}
}

//Coordinate Arrays: [latitude, longitude]
void getTargetCoordinates(int32_t target_coordinates[], int gpsTarget) {
	if(gpsTarget == TARGET_PHONE) 		{getPhoneCoordinates(target_coordinates);}
	else if(gpsTarget == TARGET_FIXED) 	{getFixedCoordinates(target_coordinates);}
}

void getPhoneCoordinates(int32_t target_coordinates[]) {
	if(uartMessaging.isUserLonLatest() && uartMessaging.isUserLatLatest()) {
		uartMessaging.getUserLat(&target_coordinates[1]);
		uartMessaging.getUserLon(&target_coordinates[0]);
	}
}

void getFixedCoordinates(int32_t target_coordinates[]) {
	//Quad coordinates
	target_coordinates[1] = 423935390;
	target_coordinates[0] = -725294520;

	//target_coordinates[1] = 423945860;
	//target_coordinates[0] = -725291860;

	//target_coordinates[1] = 423942870;
	//target_coordinates[0] = -725294590;
}

void getGPSLock() {
	int counter=0;
	hal.console->println("getting GPS lock");
	gps->update();

	while (gps->status() < 2) {
		gps->update();

		flash_leds(true);
		hal.scheduler->delay(200);
		flash_leds(false);
		hal.scheduler->delay(200);

		//Counter to exit while loop if cannot get GPS coordinate
		counter++;
		hal.console->print(counter);

		//If we get to 20 attempts, then it probably cannot attain coordinates
		if(counter >= 50){
			hal.console->println("\n Cannot attain GPS coordinates. Status code: ");
			hal.console->println(gps->status());
			flash_leds(true);
			hal.scheduler->delay(50);
			flash_leds(false);
			hal.scheduler->delay(50);
			flash_leds(true);
			hal.scheduler->delay(50);
			flash_leds(false);
			hal.scheduler->delay(50);
		}
	}

	hal.console->println(gps->status());
}

float getAltitude() {
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

void getAltitudeData(float &alt) {
	if( (hal.scheduler->micros() - altitude_timer) > 100000UL ) {				// Update altitude data on altitude_timer
		float last_alt = alt;
		alt = getAltitude();

		//Smooth raw data (last parameter is the smoothing constant)
		//alt = movingAvg(last_alt, alt, .5);

		//Verify that the altitude values are within scope
		if(abs(alt-last_alt) > 10){
			alt=last_alt;
		}

		//Get Climb Rate and smooth (last parameter is the smoothing constant)
		float last_climb_rate = climb_rate;
		climb_rate = constrain(getClimbRate(), -15, 15);
		climb_rate = movingAvg(last_climb_rate, climb_rate, .9);

		//Verify that the climb rate values are within scope
		if(abs(climb_rate-last_climb_rate) > 100){
			climb_rate=last_climb_rate;
		}

		//Scheduling
		altitude_timer = hal.scheduler->micros();

		if(PRINT_DEBUG) {
			// hal.console->printf("\nVoltage: %.2f \tCurrent: %.2f \tTotCurr:%.2f",
			// battery_mon.voltage(),
			// battery_mon.current_amps(),
			// battery_mon.current_total_mah());
		}
	}
}

void updateReadings(uint16_t channels[], long &safety,
					float &accelPitch, float &accelRoll, float &accelYaw,
					float &gyroPitch, float &gyroRoll, float &gyroYaw,
					float &alt, float &AVG_OFF_BUTTON_VALUE) {

	// Wait until new orientation data (normally 5ms max)
	while(ins.num_samples_available() == 0);

	uartMessaging.receive();
	battery_mon.read();													// Get battery stats: update voltage and current readings
	hal.rcin->read(channels, 8);
	safety = channels[4];
	AVG_OFF_BUTTON_VALUE = OFF_BUTTON_VALUE->voltage_average();
	updateState(channels);

	getAltitudeData(alt);
	getAccel(accelPitch, accelRoll, accelYaw);
	getGyro(gyroPitch, gyroRoll, gyroYaw);

	if(PRINT_DEBUG) {
		// hal.console->printf("\nVoltage: %.2f \tCurrent: %.2f \tTotCurr:%.2f  ",
		// battery_mon.voltage(), //voltage
		// battery_mon.current_amps(), //Inst current
		// battery_mon.current_total_mah()); //Accumulated current
	}
}

void getAccel(float &accelPitch, float &accelRoll, float &accelYaw) {
	ins.update();
	ins.quaternion.to_euler(&accelRoll, &accelPitch, &accelYaw);		// Ask MPU6050 for orientation

	accelPitch = ToDeg(accelPitch);
	accelRoll = ToDeg(accelRoll);
	accelYaw = ToDeg(accelYaw);
}

void getGyro(float &gyroPitch, float &gyroRoll, float &gyroYaw) {
	Vector3f gyro = ins.get_gyro();										// Ask MPU6050 for gyro data

	gyroPitch = ToDeg(gyro.y);
	gyroRoll = ToDeg(gyro.x);
	gyroYaw = ToDeg(gyro.z);
}

void updateState(uint16_t channels[]) {
	//F.Mode is channels[5]
	//BOTTOM = greater than 1900	- MANUAL
	//MIDDLE = greater than 1500	- AUTO_ALT_HOLD
	//TOP    = greater than 1100	- AUTO_PERFORMANCE

	if (channels[5] > 1800) {

		if (switchState == AUTO_ALT_HOLD || switchState == AUTO_PERFORMANCE) {	// If switching to manual control, reset PIDs
			pids[ALT_STAB].reset_I();
		}

		switchState = MANUAL;

	} else if ((1300 < channels[5]) && (channels[5] < 1700)) {

		if (switchState == MANUAL || switchState == AUTO_PERFORMANCE) {			// If switching to AUTO_ALT_HOLD, reset PIDs and set autopilotState to ALT_HOLD
			pids[ALT_STAB].reset_I();
			autopilotState = ALT_HOLD;
			current_heading = getHeading();
			desired_heading = current_heading;
		}

		if (autopilotState == OFF) {											// If safety was just turned off
			autopilotState = ALT_HOLD;
			current_heading = getHeading();
			desired_heading = current_heading;
		}

		switchState = AUTO_ALT_HOLD;

	} else if (channels[5] < 1200) {

		if (switchState == MANUAL || switchState == AUTO_ALT_HOLD) {				// If switching to AUTO_PERFORMANCE, reset PIDs and set autopilotState to TAKEOFF
			pids[ALT_STAB].reset_I();
			autopilotState = TAKEOFF;
			current_heading = getHeading();
			desired_heading = current_heading;
		}

		if (autopilotState == OFF) {											// If safety was just turned off
			autopilotState = TAKEOFF;
			current_heading = getHeading();
			desired_heading = current_heading;
		}

		switchState = AUTO_PERFORMANCE;
	}
}

void controlRcInputs(long &rcthr, long &rcpit, long &rcroll, long &rcyaw, float &desired_alt,
					long alt_output, float alt, uint16_t channels[]) {

	// TODO when ready, use autonomousFollow(rcthr, rcpit, rcroll, rcyaw, alt_output)
	// TODO add switch functionality for autonomous land

	if 	(switchState == AUTO_PERFORMANCE) {
		if (autopilotState == TAKEOFF) {		autonomousTakeoff(rcthr, rcpit, rcroll, rcyaw, desired_alt, alt, channels);}
		else if (autopilotState == ALT_HOLD) {	semiautonomousAltitudeHold(rcthr, rcpit, rcroll, rcyaw, alt_output, channels);}
		else if (autopilotState == LAND) {		autonomousLand(rcthr, rcpit, rcroll, rcyaw, alt);}
	}

	else if (switchState == MANUAL) {			manualControl(rcthr, rcpit, rcroll, rcyaw, channels);}
	else if (switchState == AUTO_ALT_HOLD) {	semiautonomousAltitudeHold(rcthr, rcpit, rcroll, rcyaw, alt_output, channels);}
}

void runFeedback(long &pitch_output, long &roll_output, long &yaw_output, long &alt_output, float &yaw_target,
				 long rcpit, long rcroll, long rcyaw,
				 float accelPitch, float accelRoll, float accelYaw,
				 float gyroPitch, float gyroRoll, float gyroYaw,
				 float alt, float desired_alt) {
	// Stablize PIDS
	float pitch_stab_output = constrain(pids[PID_PITCH_STAB].get_pid((float)rcpit - accelPitch, 1), -250, 250);
	float roll_stab_output = constrain(pids[PID_ROLL_STAB].get_pid((float)rcroll - accelRoll, 1), -250, 250);
	float yaw_stab_output = constrain(pids[PID_YAW_STAB].get_pid(yaw_target - accelYaw, 1), -360, 360);
	float appliedPidGain;

	// is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
	if(abs(rcyaw ) > 5) {
		yaw_stab_output = rcyaw;
		yaw_target = accelYaw;   // remember this yaw for when pilot stops
	}

	if (CAL_COMP_TYPE == PID_GAIN || CAL_COMP_TYPE == MIXED)	{appliedPidGain = PID_GAIN_VAL;}
	else														{appliedPidGain = 1;}

	// rate PIDS
	pitch_output =  (long) appliedPidGain * constrain(pids[PID_PITCH_RATE].get_pid(pitch_stab_output - gyroPitch, 1), -500, 500);
	roll_output =  (long) appliedPidGain * constrain(pids[PID_ROLL_RATE].get_pid(roll_stab_output - gyroRoll, 1), -500, 500);
	yaw_output =  (long) appliedPidGain * constrain(pids[PID_YAW_RATE].get_pid(yaw_stab_output - gyroYaw, 1), -500, 500);
	alt_output = (long) appliedPidGain * constrain(pids[ALT_STAB].get_pid(desired_alt - alt, 1), -250, 250);
}

void writeToMotors(long &rcthr, long &pitch_output, long &roll_output, long &yaw_output, float &yaw_target,
				   float accelYaw) {
	if(ESC_CALIBRATION) {
		// Calibrate the ESCs using constant throttle across all four motors
		hal.rcout->write(MOTOR_FL, rcthr);
		hal.rcout->write(MOTOR_BL, rcthr);
		hal.rcout->write(MOTOR_FR, rcthr);
		hal.rcout->write(MOTOR_BR, rcthr);

	} else if (rcthr < RC_THR_MIN + 50) {
		droneOff();
		yaw_target = accelYaw;

	} else {
		// Throttle raised, turn on motors.

		// adjust HOVER_THR based on battery voltage
		if (rcthr > ADJ_THR_THRESHOLD) {adjustThrottle();}

		// mix pid outputs
		long MOTOR_FL_output = rcthr + roll_output + pitch_output - yaw_output;
		long MOTOR_BL_output = rcthr + roll_output - pitch_output + yaw_output;
		long MOTOR_FR_output = rcthr - roll_output + pitch_output + yaw_output;
		long MOTOR_BR_output = rcthr - roll_output - pitch_output - yaw_output;

		// gain to simulate an ESC calibration
		if (CAL_COMP_TYPE == SW_CAL || CAL_COMP_TYPE == MIXED) {
			MOTOR_FL_output = map(MOTOR_FL_output, SW_CAL_MIN, SW_CAL_MAX, ESC_CAL_MIN, ESC_CAL_MAX);
			MOTOR_BL_output = map(MOTOR_BL_output, SW_CAL_MIN, SW_CAL_MAX, ESC_CAL_MIN, ESC_CAL_MAX);
			MOTOR_FR_output = map(MOTOR_FR_output, SW_CAL_MIN, SW_CAL_MAX, ESC_CAL_MIN, ESC_CAL_MAX);
			MOTOR_BR_output = map(MOTOR_BR_output, SW_CAL_MIN, SW_CAL_MAX, ESC_CAL_MIN, ESC_CAL_MAX);
		}

		// send outputs to the motors
		hal.rcout->write(MOTOR_FL, MOTOR_FL_output);
		hal.rcout->write(MOTOR_BL, MOTOR_BL_output);
		hal.rcout->write(MOTOR_FR, MOTOR_FR_output);
		hal.rcout->write(MOTOR_BR, MOTOR_BR_output);
	}
}

void droneOff() {
	autopilotState = OFF;

	hal.rcout->write(MOTOR_FL, 1000);
	hal.rcout->write(MOTOR_BL, 1000);
	hal.rcout->write(MOTOR_FR, 1000);
	hal.rcout->write(MOTOR_BR, 1000);

	// reset PID integrals
	for(int i=0; i<11; i++) {
		pids[i].reset_I();
	}
}

void manualControl(long &rcthr, long &rcpit, long &rcroll, long &rcyaw,
				   uint16_t channels[]) {

	if(ESC_CALIBRATION) 		{rcthr = map(channels[2], RC_THR_MIN, RC_THR_MAX, ESC_CAL_MIN, ESC_CAL_MAX);}
	else 						{rcthr = map(channels[2], RC_THR_MIN, RC_THR_MAX, RC_THR_MIN_MAPPED, RC_THR_MAX_MAPPED);}

	// GET RC pitch, roll, and yaw
	rcpit = map(channels[0], RC_ROL_MIN, RC_ROL_MAX, 45, -45);
	rcroll = map(channels[1], RC_PIT_MIN, RC_PIT_MAX, 45, -45);
	rcyaw = map(channels[3], RC_YAW_MIN, RC_YAW_MAX, -180, 180);
}

void autonomousTakeoff(long &rcthr, long &rcpit, long &rcroll, long &rcyaw,
					   float desired_alt, float alt, uint16_t channels[]) {

	if (alt < (desired_alt/2)) {
		rcthr = MAX_TAKEOFF_THR;
	} else if (alt < desired_alt)	{
		rcthr = map(alt, desired_alt/2, desired_alt, MAX_TAKEOFF_THR, MIN_TAKEOFF_THR);
	} else {
		// Otto is above desired_alt. Reset PID integrals for altitude hold.
		for(int i=0; i<11; i++) {
			pids[i].reset_I();
		}
		autopilotState = ALT_HOLD;
	}

	// rcpit = 0;
	// rcroll = 0;
	rcpit = map(channels[0], RC_ROL_MIN, RC_ROL_MAX, 45, -45);
	rcroll = map(channels[1], RC_PIT_MIN, RC_PIT_MAX, 45, -45);
	headingHold(rcyaw);
}

void semiautonomousAltitudeHold(long &rcthr, long &rcpit, long &rcroll, long &rcyaw,
								long alt_output, uint16_t channels[]) {

	throttleControl(rcthr, alt_output);
	rcpit = map(channels[0], RC_ROL_MIN, RC_ROL_MAX, 45, -45);
	rcroll = map(channels[1], RC_PIT_MIN, RC_PIT_MAX, 45, -45);
	headingHold(rcyaw);
}

void autonomousFollow(long &rcthr, long &rcpit, long &rcroll, long &rcyaw,
					  long alt_output) {

	throttleControl(rcthr, alt_output);
	gpsTracking(rcpit, rcroll);
	headingHold(rcyaw);
}

//This function is not implemented in loop
void autonomousLand(long &rcthr, long &rcpit, long &rcroll, long &rcyaw,
					float alt) {

	if (alt > 1)														// Otto greater than 1 meter, 30us under hover throttle
		rcthr = HOVER_THR - 30;
	else if ( (alt <= 1) || (alt > 0.5) )								// Otto between a half and 1 meter, 20us under hover throttle
		rcthr = HOVER_THR - 20;
	else																// Otto under a half meter, 10us under hover throttle
		rcthr = HOVER_THR - 10;

	rcpit = 0;
	rcroll = 0;
	headingHold(rcyaw);
}

void headingHold(long &rcyaw) {
//Compass accumulate should be called frequently to accumulate readings from the compass
	compass.accumulate();

	if((hal.scheduler->micros() - heading_timer) > 100000L){		// Run loop @ 10Hz ~ 100ms
		heading_timer = hal.scheduler->micros();
		current_heading = getHeading();
	}

	//desired_heading = getBearing();

	//Calculate the Heading error and use the PID feedback loop to translate that into a yaw input
	float heading_error = wrap_180(desired_heading - current_heading);
	rcyaw = constrain(pids[YAW_CMD].get_pid(heading_error, 1), -10, 10);
	rcyaw = rcyaw * -1;
}

void throttleControl(long &rcthr,
					 long alt_output) {

	rcthr = HOVER_THR + alt_output;
	rcthr = constrain(rcthr, MIN_THR_CONSTRAINT, MAX_THR_CONSTRAINT);
}

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

void adjustThrottle() {
	uint32_t delay;
	if (autopilotState == TAKEOFF) {
		delay = 200000UL;												// delay every 0.2 seconds when performing autonomous takeoff
	} else {
		delay = 2000000UL;												// delay every 2 seconds under normal operating conditions
	}

	if((hal.scheduler->micros() - hover_thr_timer) > delay) {
		hover_thr_timer = hal.scheduler->micros();

		float voltage = battery_mon.voltage();
		float new_hover_thr = map(voltage, 10, 11.8, ADJ_THR_MAX, ADJ_THR_MIN);		// map HOVER_THR based on voltage of drone in flight
		HOVER_THR = constrain(new_hover_thr, MIN_THR_CONSTRAINT, MAX_THR_CONSTRAINT);			// constrain hover throttle for saftey
	}
}

void flash_leds(bool on) {
	hal.gpio->write(A_LED_PIN, on ? LED_OFF : LED_ON);
	hal.gpio->write(C_LED_PIN, on ? LED_ON : LED_OFF);
}

void sendDataToPhone(float alt, long rcthr) {
	//Send alt and battery info over UART to App every 1 second
	if((hal.scheduler->micros() - send_to_phone_timer) > 1000000UL) {
		//Scheduling
		send_to_phone_timer = hal.scheduler->micros();

		//send alt and battery status
		uartMessaging.sendAltitude(alt);
		uartMessaging.sendBattery(battery_mon.voltage());
		uartMessaging.sendDroneLat(gps->latitude);
		uartMessaging.sendDroneLon(gps->longitude);
		uartMessaging.sendGPSStatus((long)gps->status());
		// uartMessaging.sendClimbRate(climb_rate);
		uartMessaging.sendGPSAccuracy(gps->horizontal_accuracy);    //GPS accuracy of the drone as a float in meters
		uartMessaging.sendClimbRate(rcthr);
	}
}

float getBearing() {
	float bearing;
	int32_t drone_coordinates[] = {0, 0};
	int32_t target_coordinates[] = {0, 0};

	if (gps->status() < 2) {
		// Force the drone to north if there is a GPS loss
		bearing = current_heading;
		return desired_heading;
	}

	getDroneCoordinates(drone_coordinates);
	getTargetCoordinates(target_coordinates, TARGET_FIXED);

	/*COMPASS OPERATION:
		True North is 0 (degrees)
		Eastern headings are negative numbers
		Western headings are positive numbers
		South is 180 or -180
	*/

	struct Location loc = {0};
	loc.lat = drone_coordinates[1] * 1.0e7;
	loc.lng = drone_coordinates[0] * 1.0e7;

	struct Location loc2 = {0};
	loc2.lat = target_coordinates[1] * 1.0e7;
	loc2.lng = target_coordinates[0] * 1.0e7;

	bearing = 0.01 * get_bearing_cd(&loc, &loc2);

	if(PRINT_DEBUG) {
		// hal.console->printf("\n Drone bearing: %f\n", bearing);
	}

	return wrap_180(bearing);
}


AP_HAL_MAIN();