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

// Radio min/max values for each stick for my radio (worked out at beginning of article)
#define RC_THR_MIN   1107
#define RC_THR_MAX   1907
#define RC_YAW_MIN   1106
#define RC_YAW_MAX   1908
#define RC_PIT_MIN   1106
#define RC_PIT_MAX   1908
#define RC_ROL_MIN   1104
#define RC_ROL_MAX   1906

// Motor numbers definitions
#define MOTOR_FL   2    // Front left
#define MOTOR_FR   0    // Front right
#define MOTOR_BL   1    // back left
#define MOTOR_BR   3    // back right

#define OFF_BUTTON 0

// PID array (10 pids, two for each axis, 1 for altitude, 3 for AUTONOMOUS commands)
PID pids[10];
#define PID_PITCH_RATE 	0
#define PID_ROLL_RATE 	1
#define PID_PITCH_STAB 	2
#define PID_ROLL_STAB 	3
#define PID_YAW_RATE 	4
#define PID_YAW_STAB 	5
#define ALT_STAB 		6
#define YAW_CMD 		7
#define PITCH_CMD 		8
#define ROLL_CMD 		9

// switchState
#define MANUAL 				0
#define AUTO_TEST	 		1
#define AUTO_PERFORMANCE 	2

// autopilotState
#define OFF 			3
#define MANUAL_OVERRIDE	4
#define TAKEOFF 		5
#define ALT_HOLD 		6
#define LAND 			7
#define THROTTLE_ASSIST 8

// PID configurations
#define DEFAULT	0
#define CUSTOM 	1

// Definitions for calibration compensation types
#define NONE		0
#define PID_GAIN 	1
#define SW_CAL 		2
#define MIXED 		3

// Target GPS location
#define PHONE 		0
#define FIXED 		1

// Heading control definitions
#define TARGET	3
#define HOLD	4

// Define the HW LED setup & Compass orientation
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
 # define A_LED_PIN        27
 # define C_LED_PIN        25
 # define LED_ON           0  	//Low
 # define LED_OFF          1  	//High
#else
 # define A_LED_PIN        37
 # define C_LED_PIN        35
 # define LED_ON           1    //High
 # define LED_OFF          0    //Low
#endif

/*--------------------------------------- DECLARE GLOBAL VARIABLES ----------------------------------------*/
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
uint32_t land_timer;
uint32_t ground_timer;
uint32_t fall_timer;

float land_average = 0;
float land_total = 0;
unsigned int land_counter = 0;
int throttle_modifier = 10;
uint32_t land_interval = 2000000;

float current_heading = 0;
float desired_heading = 0;
float climb_rate = 0;
float distance_to_target = 0;
int switchState = 0;
int autopilotState = OFF;
long rcthrAtSwitch = 0;
float batteryVoltage = 10.9;

bool state_change = 0;
bool ground_flag = 0;

// Initialize drone and target coordinates to location in the engineering quad,
// in the middle of the farther grassy area (will be overwritten on update)
int32_t drone_coordinates[] = {423935750, -725293220};
int32_t drone_coordinates_to_hold[] = {423935750, -725293220};
int32_t target_coordinates[] = {423935750, -725293220};

struct Location drone_filtered = {0};

//initial values for the kalman filter
float x_est_last_1, x_est_last_2 = 0;
float P_last_1, P_last_2 = 0;

const float INT_LAT_TO_METER = 0.01110809;
const float INT_LONG_TO_METER = 0.00823380;



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

/*----------------------------------------------- SETTINGS ------------------------------------------------*/
// Calibration settings - These MUST be kept up to date
#define ESC_CALIBRATION	0
#define ESC_CAL_MIN		1107
#define ESC_CAL_MAX		1460
#define CAL_COMP_TYPE	MIXED

// Calibration settings - change these to change the throttle response
#define PID_GAIN_VAL			2.859
#define SW_CAL_MIN				1107
#define SW_CAL_MAX				1907
#define RC_THR_MIN_MAPPED		1107
#define RC_THR_MAX_MAPPED		1907

//Set hover throttle definitions
#define Static_HOVER_THR			1620
#define ADJ_THR_THRESHOLD 			Static_HOVER_THR-120
#define ADJ_THR_MIN					Static_HOVER_THR-15
#define ADJ_THR_MAX					Static_HOVER_THR+50
#define ADJ_THR_MIN_CONSTRAINT		Static_HOVER_THR-30
#define ADJ_THR_MAX_CONSTRAINT		Static_HOVER_THR+50
unsigned int HOVER_THR = Static_HOVER_THR;




/*-------------------------------------- PRE-FLIGHT CHECKLIST ---------------------------------------*/

// Debug ON/OFF
#define PRINT_DEBUG 1

// Control whether to perform GPS lock on startup
#define OUTDOORS 1

// Choose whether GPS tracking should follow the PHONE or a FIXED position
#define GPS_TRACKING_TARGET FIXED

// Choose whether GPS tracking should have heading TARGET or HOLD
#define GPS_TRACKING_HEADING TARGET

//Set the desired altitude in meters
const float DESIRED_ALTITUDE = 1.5;

//Set the desired seperation distance in meter
const int SEPERATION_DISTANCE = 15;

// Set the constraint on autonomous pitch and roll
const int PITCH_ROLL_CMD_CONSTRAINT = 8;


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
	// Initialize the Altitude Hold Reference System
	ahrs.init();
	if(OUTDOORS) {getGPSLock();}
	hal.console->println("Otto Ready.");
}

/*---------------------------------------------- LOOP -----------------------------------------------------*/
void loop() {
	static long rcthr, rcpit, rcroll, rcyaw, safety;					// Variables to store radio in
	static float yaw_target = 0;
	static float desired_alt, alt;
	static float accelPitch, accelRoll, accelYaw;
	static float gyroPitch, gyroRoll, gyroYaw;
	static float accelZ;
	static long pitch_output, roll_output, yaw_output, alt_output;
	static uint16_t channels[8];  // array for raw channel values
	static float AVG_OFF_BUTTON_VALUE;

	updateReadings(channels, safety, accelPitch, accelRoll, accelYaw, gyroPitch, gyroRoll, gyroYaw, alt,
		climb_rate, accelZ, AVG_OFF_BUTTON_VALUE);
	updateState(channels, rcthr);
	sendDataToPhone(alt, rcthr, accelZ);
	distance_to_target = getDistanceToUser();

	desired_alt = DESIRED_ALTITUDE; //Hard code in desired_alt

	while((AVG_OFF_BUTTON_VALUE < 1.0) || (safety < 1500)) {			// Kill motors when [off switch] or [safety] is on
		updateReadings(channels, safety, accelPitch, accelRoll, accelYaw, gyroPitch, gyroRoll, gyroYaw, alt,
			climb_rate, accelZ, AVG_OFF_BUTTON_VALUE);
		updateState(channels, rcthr);
		sendDataToPhone(alt, rcthr, accelZ);
		autopilotState = OFF;
		droneOff();
		yaw_target = accelYaw;											// reset yaw target so we maintain this on takeoff
	}

	runFlightControl(rcthr, rcpit, rcroll, rcyaw, desired_alt, alt_output, alt, climb_rate, accelZ, channels);
	runPidFeedback(pitch_output, roll_output, yaw_output, alt_output, yaw_target, rcpit, rcroll, rcyaw, accelPitch, accelRoll, accelYaw, gyroPitch, gyroRoll, gyroYaw, alt, desired_alt);
	writeToMotors(rcthr, pitch_output, roll_output, yaw_output, yaw_target, accelYaw);

	if (PRINT_DEBUG) {
		// hal.console->print("rcthr, ");
		// hal.console->print(rcthr);
		// hal.console->print(", hoverthr, ");
		// hal.console->print(HOVER_THR);
		hal.console->print(", rcpitch, ");
		hal.console->print(rcpit);
		hal.console->print(", rcroll, ");
		hal.console->print(rcroll);
		hal.console->print(",  rcyaw, ");
		hal.console->print(rcyaw);
		// hal.console->print(",  accelPitch, ");
		// hal.console->print(accelPitch);
		// hal.console->print(",  accelRoll, ");
		// hal.console->print(accelRoll);
		// hal.console->print(",  accelYaw, ");
		// hal.console->print(accelYaw);
		// hal.console->print(",  accelZ, ");
		// hal.console->print(accelZ);
		// hal.console->print(", pitch_out: ");
		// hal.console->print(pitch_output);
		// hal.console->print(", roll_out: ");
		// hal.console->print(roll_output);
		// hal.console->print(", yaw_out: ");
		// hal.console->print(yaw_output);

		// hal.console->printf_P(PSTR("Accel pitch:%4.2f \t roll:%4.2f \t yaw:%4.2f \t Gyro pitch:%4.2f \t roll:%4.2f \t yaw:%4.2f\n"),
		// 						  accelPitch, accelRoll, accelYaw,  gyroPitch, gyroRoll, gyroYaw);

		// hal.console->print(", battery, ");
		// hal.console->print(battery_mon.voltage());
		// hal.console->print(", averaged battery voltage: ");
		// hal.console->print(batteryVoltage);

		// hal.console->print(", switchState: ");

		// if (switchState == MANUAL) {
		// 	hal.console->print("MANUAL");
		// } else if (switchState == AUTO_TEST) {
		// 	hal.console->print("AUTO_TEST");
		// } else if (switchState == AUTO_PERFORMANCE) {
		// 	hal.console->print("AUTO_PERFORMANCE");
		// }

		hal.console->print(", autopilotState: ");

		if (autopilotState == OFF) {
			hal.console->print("OFF");
		} else if(autopilotState == MANUAL_OVERRIDE) {
			hal.console->print("MANUAL_OVERRIDE");
		} else if (autopilotState == TAKEOFF) {
			hal.console->print("TAKEOFF");
		} else if (autopilotState == ALT_HOLD) {
			hal.console->print("ALT_HOLD");
		} else if (autopilotState == LAND) {
			hal.console->print("LAND");
		} else if (autopilotState == THROTTLE_ASSIST) {
			hal.console->print("THROTTLE_ASSIST");
		}

		// hal.console->print(", desired_heading: ");
		// hal.console->print(desired_heading);
		// hal.console->print(", current_heading: ");
		// hal.console->print(current_heading);
		// hal.console->print(", accuracy: ");
		// hal.console->print(gps->horizontal_accuracy);

		hal.console->print(", distance, ");
		hal.console->print(distance_to_target);
		// hal.console->print(", t, ");
		// hal.console->print(hal.scheduler->millis());

		hal.console->println("");
	}
}

AP_HAL_MAIN();
