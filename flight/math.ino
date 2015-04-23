
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

int wrap_180(float x) {
	if(x < -180) 		{return (x + 360);}
	else if(x > 180) 	{return (x - 360);}
	else 				{return x;}
}

void integrate( float accel, float &velocity  ){
	
	
	if( accel > -0.11 && accel < 0.11 )
		accel = 0;
		
		
	velocity = velocity - (accel* ( ((float)(hal.scheduler->micros() - integral_timer)) / 1000000) );
		
	
	integral_timer = hal.scheduler->micros();
}
