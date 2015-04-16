
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

		// hal.console->print( "MANUAL - Current Heading : ");
		// hal.console->print( current_heading );
		// hal.console->print( ", Desired Heading : ");
		// hal.console->println( desired_heading );
		
		// hal.scheduler->delay(100);
		
		switchState = MANUAL;

	} else if ((1300 < channels[5]) && (channels[5] < 1700)) {

		if (autopilotState == OFF) {														// If safety was just turned off
			autopilotState = ALT_HOLD;
			state_change = true;					
		
		} else if (switchState == MANUAL || switchState == AUTO_PERFORMANCE) {				// If switching to AUTO_ALT_HOLD
			pids[ALT_STAB].reset_I();
			autopilotState = ALT_HOLD;
			state_change = true;
		}

		switchState = AUTO_ALT_HOLD;

	} else if (channels[5] < 1200) {

		if (autopilotState == OFF) {														// If safety was just turned off
			autopilotState = TAKEOFF;
			state_change = true;

		} else if (switchState == MANUAL || switchState == AUTO_ALT_HOLD) {					// If switching to AUTO_PERFORMANCE
			pids[ALT_STAB].reset_I();
			autopilotState = TAKEOFF;
			state_change = true;
		}

		switchState = AUTO_PERFORMANCE;
	}
}
