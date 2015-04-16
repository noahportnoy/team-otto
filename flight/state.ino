
void updateState(uint16_t channels[], long rcthr) {
	//F.Mode is channels[5]
	//BOTTOM = greater than 1900	- MANUAL
	//MIDDLE = greater than 1500	- AUTO_FOLLOW_OR_ALT_HOLD
	//TOP    = greater than 1100	- AUTO_PERFORMANCE

	if (channels[5] > 1800) {

		if (autopilotState == OFF) {														// If safety was just turned off
			autopilotState = MANUAL_OVERRIDE;

		} else if (switchState == AUTO_FOLLOW_OR_ALT_HOLD || switchState == AUTO_PERFORMANCE) {		// If switching to MANUAL
			rcthrAtSwitch = getRcThrottle(channels);
			autopilotState = THROTTLE_ASSIST;
		}

		switchState = MANUAL;

	} else if ((1300 < channels[5]) && (channels[5] < 1700)) {

		if (autopilotState == OFF) {														// If safety was just turned off
			autopilotState = ALT_HOLD;
			state_change = true;

		} else if (switchState == MANUAL || switchState == AUTO_PERFORMANCE) {				// If switching to AUTO_FOLLOW_OR_ALT_HOLD
			pids[ALT_STAB].reset_I();
			autopilotState = ALT_HOLD;
			state_change = true;
		}

		switchState = AUTO_FOLLOW_OR_ALT_HOLD;

	} else if (channels[5] < 1200) {

		if (switchState == MANUAL || switchState == AUTO_FOLLOW_OR_ALT_HOLD) {	// If switching into AUTO_PERFORMANCE
			// maintain last autopilot state
			uartMessaging.resetTakeOff();										// reset the isTakeoff boolean. isTakeOff will become true again only if phone requests takeoff again

		} else if (autopilotState == OFF && uartMessaging.isTakeOff()) {		// If user requests takeoff from phone and drone was last in autopilot OFF state
			pids[ALT_STAB].reset_I();											// reset i; reset PID integrals while in manual mode
			autopilotState = TAKEOFF;
			uartMessaging.resetTakeOff();										// reset the isTakeoff boolean. isTakeOff will become true again only if phone requests takeoff again
			state_change = true;
		}

		switchState = AUTO_PERFORMANCE;
	}
}
