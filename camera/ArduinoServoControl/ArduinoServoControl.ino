#include <Servo.h>

#define X 0
#define Y 1

const int numServos = 1;

Servo servo[2];
int servoPin[2] = {9, 10};

int center_pos[2] = {90, 90};
int current_pos[2];
int error[2] = {0, 0};

void setup() {
	Serial.begin(9600);

	// Align the servos at startup to the center position
	for (int i = 0; i < numServos; i++) {
		current_pos[i] = center_pos[i];
		servo[i].attach(servoPin[i]);
		servo[i].write(current_pos[i]);
	}
}


long pmilUpdate = 0;
long cmilUpdate = 0;
long dlymilUpdate = 25;

void loop() {
	// Check the serial buffer for servo position error values:
	// 1st byte = x-servo sign [char: +/-], 2nd byte = x-servo error value [char: 0-9]
	// 3rd & 4th byte for y-servo sign & error when y-servo is enabled (i.e. numServos = 2)
	if (Serial.available()) {
		char errorSign[2];
		char errorVal[2];

		for (int i = 0; i < numServos; i++) {
			errorSign[i] = Serial.read();
			delay(50);
			errorVal[i] = Serial.read();

			// Store servo position error if sign is valid, otherwise ignore it
			if (errorSign[i] == '+') {
				error[i] = errorVal[i] - '0';
			} else if (errorSign[i] == '-') {
				error[i] = -(errorVal[i] - '0');
			} else {
				error[i] = 0;
			}

			// Ignore invalid errorVal values (outside range 0-9)
			if (errorVal[i] < '0' || errorVal[i] > '9') {
				errorVal[i] = '0';
			}
		}

	}

	// Update the servo's position, based on the error
	cmilUpdate = millis();
	if (dlymilUpdate < cmilUpdate - pmilUpdate) {
		for (int i = 0; i < numServos; i++) {
			current_pos[i] += error[i];
			error[i] = 0;	//Clear the error, so error is not carried forward
			current_pos[i] = constrain(current_pos[i], 20, 160);
			servo[i].write(180 - current_pos[i]);

			// Print the servo ID and it's position
			Serial.print("Servo[");
			Serial.print(i);
			Serial.print("] pos = ");
			Serial.println(current_pos[i]);
		}
		pmilUpdate = cmilUpdate;
	}

}

