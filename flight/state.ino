
void updateState(uint16_t channels[], long rcthr) {
	//F.Mode is channels[5]
	//BOTTOM = greater than 1900	- MANUAL
	//MIDDLE = greater than 1500	- AUTO_ALT_HOLD
	//TOP    = greater than 1100	- AUTO_PERFORMANCE

	if (channels[5] > 1800) {

		if (autopilotState == OFF) {														// If safety was just turned off
			autopilotState = MANUAL_OVERRIDE;

		} else if (switchState == AUTO_ALT_HOLD || switchState == AUTO_PERFORMANCE) {		// If switching to MANUAL
			rcthrAtSwitch = getRcThrottle(channels);
			autopilotState = THROTTLE_ASSIST;
		}

		switchState = MANUAL;

	} else if (channels[5] < 1200) {

		if (autopilotState == OFF) {														// If safety was just turned off
			autopilotState = ALT_HOLD;
			current_heading = getHeading();
			desired_heading = current_heading;

		} else if (switchState == MANUAL || switchState == AUTO_PERFORMANCE) {				// If switching to AUTO_ALT_HOLD
			pids[ALT_STAB].reset_I();
			autopilotState = ALT_HOLD;
			current_heading = getHeading();
			desired_heading = current_heading;
		}

		switchState = AUTO_ALT_HOLD;
	} else if ((1300 < channels[5]) && (channels[5] < 1700)
) {

		if (autopilotState == OFF) {														// If safety was just turned off
			autopilotState = LAND;
			current_heading = getHeading();
			desired_heading = current_heading;

		} else if (switchState == MANUAL || switchState == AUTO_ALT_HOLD) {					// If switching to AUTO_PERFORMANCE
			//hal.console->println( " ENTERING LAND " );
			pids[ALT_STAB].reset_I();
			autopilotState = LAND;
			land_timer = hal.scheduler->micros();
			ground_timer = land_timer;
			fall_timer = land_timer;
			current_heading = getHeading();
			desired_heading = current_heading;
		}

		switchState = AUTO_PERFORMANCE;
	}
}
