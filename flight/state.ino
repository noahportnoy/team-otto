
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
