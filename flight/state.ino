
void updateState(uint16_t channels[], long rcthr) {
	//F.Mode is channels[5]
	//BOTTOM = greater than 1900	- MANUAL
	//MIDDLE = greater than 1500	- AUTO_TEST
	//TOP    = greater than 1100	- AUTO_PERFORMANCE

	if (channels[5] > 1800) {

		if (autopilotState == OFF) {													// If safety was just turned off
			autopilotState = MANUAL_OVERRIDE;

		} else if (switchState == AUTO_TEST || switchState == AUTO_PERFORMANCE) {		// If switching to MANUAL, apply throttle assistance
			rcthrAtSwitch = getRcThrottle(channels);
			autopilotState = THROTTLE_ASSIST;
		}

		switchState = MANUAL;

	} else if ((1300 < channels[5]) && (channels[5] < 1700)) {

		if (autopilotState == OFF) {														// PROTECT against safety going off
			// pids[ALT_STAB].reset_I();
			// autopilotState = ALT_HOLD;
			// state_change = true;

		} else if (switchState == MANUAL || switchState == AUTO_PERFORMANCE) {				// If switching to AUTO_TEST
			autopilotState = ALT_HOLD;
			state_change = true;
		}

		switchState = AUTO_TEST;

	} else if (channels[5] < 1200) {

		if (switchState == MANUAL || switchState == AUTO_TEST) {				// If switching into AUTO_PERFORMANCE
			// maintain last autopilot state
			uartMessaging.resetTakeOff();										// reset the isTakeoff/isLand booleans
			uartMessaging.resetLand();

		} else if (autopilotState == OFF && uartMessaging.isTakeOff()) {		// If user requests takeoff from phone and drone was last in autopilot OFF state
			pids[ALT_STAB].reset_I();											// reset I term because we're about to takeoff
			autopilotState = TAKEOFF;
			uartMessaging.resetTakeOff();										// reset the isTakeoff boolean
			state_change = true;

		} else if (autopilotState == ALT_HOLD && uartMessaging.isLand()) {		// If user requests land from phone and drone was last in autopilot ALT_HOLD state
			autopilotState = LAND;
			uartMessaging.resetLand();											// reset the isLand boolean
			land_timer = hal.scheduler->micros();
			ground_timer = land_timer;
			fall_timer = land_timer;
			state_change = true;
		}

		switchState = AUTO_PERFORMANCE;
	}
}
