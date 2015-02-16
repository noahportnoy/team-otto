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
#define RC_ALT_MIN   0
#define RC_ALT_MAX   1

//Hover throttle
#define HOVER_THR 1330

// Min/max throttle for autonomous takeoff
#define MAX_TAKEOFF_THR 1340
#define MIN_TAKEOFF_THR 1320

// Motor numbers definitions
#define MOTOR_FL   2    // Front left    
#define MOTOR_FR   0    // Front right
#define MOTOR_BL   1    // back left
#define MOTOR_BR   3    // back right

#define OFF_BUTTON 0
AP_HAL::AnalogSource* OFF_BUTTON_VALUE;

//Initialize the HMC5843 compass.
AP_Compass_HMC5843 compass;

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
#define ALT_RATE 7
#define YAW_CMD 8
#define PITCH_CMD 9
#define ROLL_CMD 10

// switchState
#define MANUAL 0
#define AUTO_ALT_HOLD 1
#define AUTO_TAKEOFF 2

// autopilotState
#define OFF 3
#define TAKEOFF 4
#define ALT_HOLD 5
#define LAND 6

// PID configurations
#define DEFAULT 0
#define CUSTOM 1

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




/*------------------------------------------------ DECLARE GLOBAL VARIABLES ------------------------------------------------------*/
uint32_t timer;
uint32_t interval;
uint32_t send_interval;
uint32_t heading_timer;

int startup = 0;
float originalOrientation = 0.0;
float currentOrientation = 0.0;
float alt = 0;
float last_alt = 0;
float current_heading = 0, last_heading = 0;
float climb_rate = 0;
float last_climb_rate = 0;
long rcthr = 1000;
int heightLock = 0;
int switchState = 0;
int autopilotState = 0;

Matrix3f dcm_matrix;
Quaternion q;
float target_coordinates[2];


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
        
        //Get coordinates of takeoff point
        getTakeoffCoordinates(target_coordinates);
        
        hal.console->printf("target_long, %f, target_lat, %f,  ", target_coordinates[0], target_coordinates[1]);
        
        hal.console->println("Otto Ready.");
}

/*---------------------------------------------- LOOP -----------------------------------------------------*/
void loop() {
	//receive from uart
	uartMessaging.receive();
	
	static float yaw_target = 0;  

	// Wait until new orientation data (normally 5ms max)
	while (ins.num_samples_available() == 0);
	
	// Copy from channels array to something human readable - array entry 0 = input 1, etc.
	uint16_t channels[8];  // array for raw channel values
	hal.rcin->read(channels, 8);  

        long rcyaw, rcpit, rcroll, safety, last_rcthr;  // Variables to store radio in
	float rcalt;

	safety = channels[4];
	float pitch, roll, yaw;

	float pitch_stab_output, roll_stab_output, yaw_stab_output;
	float alt_output;
	long pitch_output, roll_output, yaw_output;
	
	float AVG_OFF_BUTTON_VALUE = OFF_BUTTON_VALUE->voltage_average();
	while ( (AVG_OFF_BUTTON_VALUE < 1.0) || (safety < 1500)) {			// Kill motors when [off switch] or [safety] is on
		droneOff();
		yaw_target = yaw;												// reset yaw target so we maintain this on takeoff
		sendDataToPhone();

		AVG_OFF_BUTTON_VALUE = OFF_BUTTON_VALUE->voltage_average();
		hal.rcin->read(channels, 8);
		safety = channels[4];
	}
	
	rcalt = 1.0; //Hard code in RCALT

	// GET RC yaw, pitch, and roll
	rcyaw = map(channels[3], RC_YAW_MIN, RC_YAW_MAX, -180, 180);
	rcpit = map(channels[0], RC_ROL_MIN, RC_ROL_MAX, 45, -45);
	rcroll = map(channels[1], RC_PIT_MIN, RC_PIT_MAX, 45, -45); 

	getOrientation(pitch, roll, yaw);
	
	if( (hal.scheduler->micros() - interval) > 100000UL ) {				// Update altitude data on interval
		getAltitudeData();
	}
	
	float gyroPitch, gyroRoll, gyroYaw;
	getGyro(gyroPitch, gyroRoll, gyroYaw);

    getSwitchPosition(channels);									// Sets switchStatus to: OFF, AUTONOMOUS, or MANUAL
																	// depending on RC top-right switch position

      //Autonomous 										// depending on RC top-right switch position  
      if (switchState == AUTO_ALT_HOLD) {
            float desired_heading, heading_error;
            desired_heading = -50; //This should be an input from autonomous SoftWare
            
		    /////////Autonomous YAW using the compass & GPS
		    
		    if((hal.scheduler->micros() - heading_timer) > 100000L){ //Run loop @ 10Hz ~ 100ms
		        current_heading = getHeading(last_heading);
		    }
		    
		    //Calculate the Heading error and use the PID feedback loop to translate that into a yaw input
		    heading_error = desired_heading - current_heading;
		    rcyaw = constrain(pids[YAW_CMD].get_pid(heading_error, 1), -180, 180);
		    rcyaw = rcyaw * -1;          
	}

    // Stablize PIDS
    pitch_stab_output = constrain(pids[PID_PITCH_STAB].get_pid((float)rcpit - pitch, 1), -250, 250); 
	roll_stab_output = constrain(pids[PID_ROLL_STAB].get_pid((float)rcroll - roll, 1), -250, 250);
	yaw_stab_output = constrain(pids[PID_YAW_STAB].get_pid((float)yaw_target - yaw, 1), -360, 360);
 
	// is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
	if(abs(rcyaw ) > 5) {
		yaw_stab_output = rcyaw;
		yaw_target = yaw;   // remember this yaw for when pilot stops
	}
		
	// rate PIDS
	pitch_output =  (long) constrain(pids[PID_PITCH_RATE].get_pid(pitch_stab_output - gyroPitch, 1), -500, 500);  
	roll_output =  (long) constrain(pids[PID_ROLL_RATE].get_pid(roll_stab_output - gyroRoll, 1), -500, 500);  
	yaw_output =  (long) constrain(pids[PID_YAW_RATE].get_pid(yaw_stab_output - gyroYaw, 1), -500, 500);  

      	//Feedback loop for altitude holding
      	alt_output = constrain(pids[ALT_STAB].get_pid((float)rcalt - alt, 1), -250, 250);
      	//float alt_output = constrain(pids[ALT_RATE].get_pid(alt_stab_output - climb_rate, 1), -100, 100);




	if (switchState == AUTO_TAKEOFF) {
		// hal.console->print("DRONE IN AUTOPILOT MODE: ");

		if (autopilotState == TAKEOFF) {							// Autonomous takeoff
			// hal.console->println("TAKEOFF");
			rcthr = autonomousTakeoff(rcalt);

		} else if (autopilotState == ALT_HOLD) {						// Autonomous altitude hold
			// hal.console->println("ALT_HOLD");
			rcthr = autonomousHold(alt_output);

		} else {
			hal.console->print("Error: autopilotState of ");
			hal.console->print(autopilotState);
			hal.console->println(" has not been configured");
			while(1);
		}

	} else if (switchState == MANUAL) {								// Manual mode
		// hal.console->println("DRONE IN MANUAL MODE");
		rcthr = map(channels[2], RC_THR_MIN, RC_THR_MAX, RC_THR_MIN, 1500);
	
	} else if (switchState == AUTO_ALT_HOLD) {							// Autonomous altitude hold
	
		// hal.console->println("DRONE IN AUTO_ALT_HOLD MODE");
		rcthr = autonomousHold(alt_output);
	
	} else {
		hal.console->print("Error: switchState of ");
		hal.console->print(switchState);
		hal.console->println(" has snot been configured");
		while(1);
	}       
        
    //Motor Control
    if(rcthr >= RC_THR_MIN+50) {  // Altitude raised, turn on motors.
    	// mix pid outputs and send to the motors.
    	hal.rcout->write(MOTOR_FL, rcthr + roll_output + pitch_output - yaw_output);
    	hal.rcout->write(MOTOR_BL, rcthr + roll_output - pitch_output + yaw_output);
    	hal.rcout->write(MOTOR_FR, rcthr - roll_output + pitch_output + yaw_output);
    	hal.rcout->write(MOTOR_BR, rcthr - roll_output - pitch_output - yaw_output);
    } else {
            droneOff();
            yaw_target = yaw;
    }

        //Send data to user App
	sendDataToPhone();
}


// Arduino map function
float map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
                //Below are the PIDs for drone stabilization
		pids[PID_PITCH_RATE].kP(0.2);
		pids[PID_PITCH_RATE].kI(0.08);
		pids[PID_PITCH_RATE].imax(50);
		
		pids[PID_ROLL_RATE].kP(0.2);
		pids[PID_ROLL_RATE].kI(0.08);
		pids[PID_ROLL_RATE].imax(50);
		
		pids[PID_YAW_RATE].kP(0.7);
		pids[PID_YAW_RATE].kI(0.1);
		pids[PID_YAW_RATE].imax(50);
		
		pids[PID_PITCH_STAB].kP(4.5);
		pids[PID_ROLL_STAB].kP(4.5);
		pids[PID_YAW_STAB].kP(10);
		
                //Below are the PIDs for altitude hold
		pids[ALT_RATE].kP(0.1);
		pids[ALT_RATE].kI(0.0);
		pids[ALT_RATE].imax(50);
		
		pids[ALT_STAB].kP(6.0);
		pids[ALT_STAB].kI(1.0);
		pids[ALT_STAB].imax(100);
                
                //Below are the PIDs for autonomous control
                pids[PITCH_CMD].kP(1.0);
		pids[PITCH_CMD].kI(0.0);
		pids[PITCH_CMD].imax(50);

                pids[ROLL_CMD].kP(1.0);
		pids[ROLL_CMD].kI(0.0);
		pids[ROLL_CMD].imax(50);

                pids[YAW_CMD].kP(0.7);
		pids[YAW_CMD].kI(0.0);
		pids[YAW_CMD].imax(50);

	} else {
		hal.console->print("Error: PID constants not set for provided configuration");
		hal.console->println(config);
		while(1);
	}
}

float calculateYaw() { 
	static float min[3], max[3], offset[3];
	float heading;
	compass.accumulate();
	if((hal.scheduler->micros()- timer) > 100000L)
	{
		timer = hal.scheduler->micros();
		compass.read();
		
		if (!compass.healthy) {
			hal.console->println("not healthy");
			return 0;
		}
		dcm_matrix.to_euler(0,0,0);
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


float getHeading(float last_heading){
        //Use AHRS for Heading
        static uint32_t last_t, last_print;
        uint32_t now = hal.scheduler->micros();
        float heading = 0;
    
        ahrs.update();
        if((hal.scheduler->micros() - heading_timer) > 100000L){ //Run loop @ 10Hz
            heading_timer = hal.scheduler->micros();
            
            compass.read();
            heading = compass.calculate_heading(ahrs.get_dcm_matrix());
            gps->update();
            
            Vector3f drift  = ahrs.get_gyro_drift();
            /*
            hal.console->printf_P(
                    PSTR("r:%4.1f  p:%4.1f y:%4.1f "
                        "drift=(%5.1f %5.1f %5.1f) hdg=%.1f\n"),
                            ToDeg(ahrs.roll),
                            ToDeg(ahrs.pitch),
                            ToDeg(ahrs.yaw),
                            ToDeg(drift.x),
                            ToDeg(drift.y),
                            ToDeg(drift.z),
                            compass.use_for_yaw() ? ToDeg(heading) : 2.67767789
            );
            */
            current_heading =  ToDeg(heading);
        }
        //current_heading = movingAvg(last_heading, current_heading, .5);
        return current_heading;
}


//Coordinate Arrays: [longitude, lattitude]
void getDroneCoordinates(float coords[]){
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

            coords[1] = gps->latitude/10000000.0;
            coords[0] = gps->longitude/10000000.0;
	} else {
            hal.console->print("~~~~~~~~~~~~~~~~  Error. NO NEW GPS DATA!  ~~~~~~~~~~~~~~");
        }
}

//Coordinate Arrays: [longitude, lattitude]
bool getTargetCoordinates(float coords[]){
        gps->update();
	if (gps->new_data) {
            coords[1] = gps->latitude/10000000.0;
            coords[0] = gps->longitude/10000000.0;
            hal.console->println(gps->status());
            return true;
	} 
        hal.console->print("~~~~~~~~~~~~~~~~  Error. NO NEW GPS DATA!  ~~~~~~~~~~~~~~");
        return false;
}

//Coordinate Arrays: [longitude, lattitude]
void getTakeoffCoordinates(float coords[]){
        int counter=0;
        hal.console->println("getting GPS lock");
        gps->update();
        
        while (gps->status() < 2){
              flash_leds(true);
              hal.scheduler->delay(50);
              flash_leds(false);
              hal.scheduler->delay(50);
              flash_leds(true);
              hal.scheduler->delay(50);
              
              //Counter to exit while loop if cannot get GPS coordinate
              counter++;
              hal.console->print(counter);
              
              //If we get to 20 attempts, then it probalby cannot attain coordinates
              if(counter >= 20){
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
                      flash_leds(true);
                      hal.scheduler->delay(50);     
              }
        }

        coords[1] = gps->latitude/10000000.0;
        coords[0] = gps->longitude/10000000.0;
        hal.console->println(gps->status());
        return;
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

void getAltitudeData() {
	last_alt = alt;
	alt = getAltitude();
	
	//Smooth raw data (last parameter is the smoothing constant)
	//alt = movingAvg(last_alt, alt, .5);
	
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
	
	/*hal.console->printf("\nVoltage: %.2f \tCurrent: %.2f \tTotCurr:%.2f",
					battery_mon.voltage(),
					battery_mon.current_amps(),
							battery_mon.current_total_mah());
	*/
}

void getOrientation(float &pitch, float &roll, float &yaw) {
	ins.update();	
        //q = ins.quaternion;										// Ask MPU6050 for orientation
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

void getSwitchPosition(uint16_t channels[]) {
	//F.Mode is channels[5]
	//BOTTOM = greater than 1900	- MANUAL
	//MIDDLE = greater than 1500	- AUTO_ALT_HOLD
	//TOP    = greater than 1100	- AUTO_TAKEOFF

	if (channels[5] > 1800) {
  
		if (switchState == AUTO_ALT_HOLD || switchState == AUTO_TAKEOFF) {	// If switching to manual control, reset PIDs
			pids[ALT_STAB].reset_I();											// reset i; reset PID integrals while in manual mode
			setPidConstants(DEFAULT);											// Set PIDs for manual control
		}

		switchState = MANUAL;

	} else if ((1300 < channels[5]) && (channels[5] < 1700)) {

		if (switchState == MANUAL || switchState == AUTO_TAKEOFF) {	// If switching to AUTO_ALT_HOLD, reset PIDs and set autopilotState to ALT_HOLD
			pids[ALT_STAB].reset_I();									// reset i; reset PID integrals while in manual mode
			setPidConstants(DEFAULT);									// PID Configuration for ALT_HOLD
			autopilotState = ALT_HOLD;
		}

		if (autopilotState == OFF) {										// If safety was just turned off
			autopilotState = ALT_HOLD;
		}

		switchState = AUTO_ALT_HOLD;

	} else if (channels[5] < 1200) {

		if (switchState == MANUAL || switchState == AUTO_ALT_HOLD) {		// If switching to AUTO_TAKEOFF, reset PIDs and set autopilotState to TAKEOFF
			pids[ALT_STAB].reset_I();										// reset i; reset PID integrals while in manual mode
			setPidConstants(DEFAULT);										// PID Configuration for takeoff
			autopilotState = TAKEOFF;
		}

		if (autopilotState == OFF) {											// If safety was just turned off
			autopilotState = TAKEOFF;
		}

		switchState = AUTO_TAKEOFF;

	}
}

//Saftey Switch Function
void droneOff() {

	autopilotState = OFF;

	//hal.console->println("DRONE OFF / safety");

	hal.rcout->write(MOTOR_FL, 1000);
	hal.rcout->write(MOTOR_BL, 1000);
	hal.rcout->write(MOTOR_FR, 1000);
	hal.rcout->write(MOTOR_BR, 1000);
	
	//hal.console->printf_P(PSTR("Voltage ch0:%.2f\n"), AVG_OFF_BUTTON_VALUE);
	//hal.scheduler->delay(500);
							
	//GET BATTERY STATS
	//Update voltage and current readings
	battery_mon.read();
	hal.console->printf("\nVoltage: %.2f \tCurrent: %.2f \tTotCurr:%.2f  ",
	        battery_mon.voltage(), //voltage
	        battery_mon.current_amps(), //Inst current
	        battery_mon.current_total_mah()); //Accumulated current
		
	for(int i=0; i<11; i++) {								// reset PID integrals
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
		for(int i=0; i<11; i++) {							// reset PID integrals for altitude hold
			pids[i].reset_I();
		}
		setPidConstants(DEFAULT);
		autopilotState = ALT_HOLD;
	}

	return rcthr;
}

long autonomousHold(float alt_output) {
	//Map the Throttle
	rcthr = HOVER_THR + alt_output;
	rcthr = constrain(rcthr, 1200, 1400);

	return rcthr;
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

	compass.set_orientation(MAG_ORIENTATION); // set compass's orientation on aircraft.
	compass.set_offsets(0,0,0); // set offsets to account for surrounding interference
	compass.set_declination(ToRad(0.0)); // set local difference between magnetic north and true north
        
        //Otto uses the HMC5883L Compass
}

void setupTiming() {
	hal.scheduler->delay(1000);
	timer = hal.scheduler->micros();
	interval = timer;
        send_interval = timer;
        heading_timer = timer;
}

void setupRpi() {
		//Initializes the UART C bus (begin(baudrate, rx buffer, tx buffer)
		//See UARTDriver.h for more...
		hal.uartC->begin(115200, 16, 16); 
		hal.console->println("UARTC (UART2) Test");
		//Uart messaging
		uartMessaging.init(hal.uartC, hal.console);
}

void setupGPS() {
	//SET UP UARTB CONNECTION FOR GPS
	hal.uartB->begin(38400);
	gps = &GPS;
	gps->init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_2G);       // GPS Initialization
}

void setupBatteryMonitor() {
	//Initialize the battery monitor
	battery_mon.init();
	battery_mon.set_monitoring(AP_BATT_MONITOR_VOLTAGE_AND_CURRENT);
	hal.console->println("Battery monitor initialized");
}

static void flash_leds(bool on)
{
    hal.gpio->write(A_LED_PIN, on ? LED_OFF : LED_ON);
    hal.gpio->write(C_LED_PIN, on ? LED_ON : LED_OFF);
}


void sendDataToPhone() {
	//Send alt and battery info over UART to App every 2 seconds
	if((hal.scheduler->micros() - send_interval) > 2000000UL) {
		//Scheduling
		send_interval = hal.scheduler->micros();  
		
		//GET BATTERY STATS
		// update voltage and current readings
		battery_mon.read();
		
		//send alt and battery status
		uartMessaging.sendAltitude(alt);
		uartMessaging.sendBattery(battery_mon.voltage());
	}
}

AP_HAL_MAIN();
