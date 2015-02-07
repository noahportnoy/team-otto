#pragma once
#include <stdlib.h>
#include <string.h>
#include "UartMessaging.h"
//#include <stdio.h>


const char* BatteryStatus = "$BTS";
const char* Altitude = "$ALT";
const char* GPSLock = "$GPL";
const char* Latitude = "$LAT";
const char* Longitude = "$LON";

void UartMessaging::init(AP_HAL::UARTDriver* _driver, AP_HAL::ConsoleDriver* _console)
{
	UARTdriver = _driver;
	idNumBytes = payloadNumBytes = 0;
	console =_console;
}

/*void generateMessage(const char* messageID, float payload, char* buffer)
{
	memcpy(buffer, messageID, 4);
	dtostrf(payload, 8, 4, &buffer[4]);
	memcpy(&buffer[12], "!", 1); //End of packet
	memcpy(&buffer[13], 0, 1); //Proper c-string termination
}*/

void UartMessaging::sendAltitude(float altitude)
{
	send(Altitude, altitude);	
}

void UartMessaging::sendBattery(float battery)
{
	send(BatteryStatus, battery);
}


void UartMessaging::send(const char* messageID, float f)
{
	char buffer[16];
	memcpy(buffer, messageID, 4);
	dtostrf(f, 7, 5, &buffer[4]);
	strcpy(&buffer[11], "!\n");
	send(buffer);
}

void UartMessaging::send(const char* messageID, bool b)
{
	char buffer[16];
	memcpy(buffer, messageID, 4);
	if(b)
	{
		memcpy(&buffer[4], "1", 1);
	}else
	{
		memcpy(&buffer[4], "0", 1);
	}
	strcpy(&buffer[12], "!\n");
	send(buffer);
}

void UartMessaging::send(const char* messageID, char* buffer)
{

}

void UartMessaging::send(char* buffer)
{
	console->println(buffer);
	UARTdriver->write(buffer);
}

void UartMessaging::receive()
{

	if(UARTdriver->available())
	  {
		char charReceive = UARTdriver->read();
		//hal->console->println(charReceive);
		switch(currentState)
		{
			//Detect start character
		  case START:
			  if(charReceive == '$');
				currentState = ID;
		  break;

		  case ID:
			  idReceived[idNumBytes] = charReceive;
			  idNumBytes++;

			  if(idNumBytes == 3)
			  {
				  currentState = PAYLOAD;
				  idNumBytes=0;
			  }
			  
			  break;

		  case PAYLOAD:
			  if(charReceive == '!')
			  {
				  currentState = END;
			  }else
			  {
				  payloadReceived[payloadNumBytes] = charReceive;
				  payloadNumBytes++;
			  }

			  
			  break;

		  default:
			 break;
      
		}
	  }
  
	if(currentState == END)
	{
	  
		if(idReceived[0] == '$' && idReceived[1] == 'T' && idReceived[2] == 'K' && idReceived[3] == 'F')
		{
		  //hal->console->println("Turn on");
		//  rcthr = RC_THR_MIN;
		}  
    
		else if(idReceived[0] == '$' && idReceived[1] == 'S' && idReceived[2] == 'T' && idReceived[3] == 'P')
		{
		  //hal->console->println("Turn off");
		  //rcthr = 0;
		} else
		{

		}
		
		currentState = START;
		
	  }
	}
