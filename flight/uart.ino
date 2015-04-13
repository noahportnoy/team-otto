
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
		uartMessaging.sendGPSAccuracy(gps->horizontal_accuracy);    //GPS accuracy of the drone as a float in meters
		// uartMessaging.sendClimbRate(climb_rate);
		uartMessaging.sendClimbRate(current_heading);
	}
}