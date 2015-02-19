#pragma once
#include <stdlib.h>
#include <string.h>
#include "UartMessaging.h"
//#include <stdio.h>


const char* BatteryStatus = "$BTS";
const char* Altitude = "$ALT";
const char* Latitude = "$LAT";
const char* Longitude = "$LON";
const char* GPSLock = "$GPL";
const char* Safety = "$SFT";
const char* ClimbRate = "$CRS";


void UartMessaging::init(AP_HAL::UARTDriver* _driver, AP_HAL::ConsoleDriver* _console)
{
	_UARTdriver = _driver;
	_idNumBytes = _payloadNumBytes = 0;
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
void UartMessaging::sendGPSLock(bool isGPSLock)
{
	send(GPSLock, isGPSLock);
}
void UartMessaging::sendSafetyStatus(bool isSafety)
{
	send(Safety, isSafety);
}
void UartMessaging::sendClimbRate(float climbRate)
{
	send(ClimbRate, climbRate);
}

//Receive User Lattitude
void UartMessaging::getUserLat(int32_t* lat)
{
	_isUserLatLatest = false;
	*lat = _userLat;

}
bool UartMessaging::isUserLatLatest()
{
	return _isUserLatLatest;
}

//Receive User Longitude
void UartMessaging::getUserLon(int32_t* lon)
{
	_isUserLongLatest = false;
	*lon = _userLon;

}
bool UartMessaging::isUserLonLatest()
{
	return _isUserLongLatest;
}

//Generic send functions
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
	strcpy(&buffer[5], "!\n");
	send(buffer);
}

void UartMessaging::send(const char* messageID, char* buffer)
{

}

void UartMessaging::send(char* buffer)
{
	console->println(buffer);
	_UARTdriver->write(buffer);
}

//Receive loop and message interpeter
void UartMessaging::receive()
{

	if(_UARTdriver->available())
	  {
		char charReceive = _UARTdriver->read();
		//hal->console->println(charReceive);
		switch(_currentState)
		{
			//Detect start character
		  case START:
			  if(charReceive == '$');
				_currentState = ID;
		  break;

		  case ID:
			  _idReceived[_idNumBytes] = charReceive;
			  _idNumBytes++;

			  if(_idNumBytes == 3)
			  {
				  _currentState = PAYLOAD;
				  _idNumBytes=0;
			  }
			  
			  break;

		  case PAYLOAD:
			  if(charReceive == '!')
			  {
				  _payloadReceived[_payloadNumBytes] = '\0';
				  _currentState = END;
			  }else
			  {
				  _payloadReceived[_payloadNumBytes] = charReceive;
				  _payloadNumBytes++;
			  }
		  
			  break;

		  default:
			 break;
      
		}
	  }
  
	if(_currentState == END)
	{
	  
		if(_idReceived[0] == 'T' && _idReceived[1] == 'K' && _idReceived[2] == 'F')
		{
			console->println("Turn on");
		}  
    
		else if(_idReceived[0] == 'S' && _idReceived[1] == 'T' && _idReceived[2] == 'P')
		{
			console->println("Turn off");

		} else if(_idReceived[0] == 'L' && _idReceived[1] == 'A' && _idReceived[2] == 'T')
		{
			//Print the latitude
			console->print("User Latitude: ");
			console->println(_payloadReceived); 
			
			//convert latitude from string to double
			_userLat = atol(_payloadReceived);

			_isUserLatLatest = true;
			
		}else if(_idReceived[0] == 'L' && _idReceived[1] == 'O' && _idReceived[2] == 'N')
		{
			//Print the latitude
			console->print("User Longitude: ");
			console->println(_payloadReceived); 
			
			//convert latitude from string to double
			_userLon = atol(_payloadReceived);
			_isUserLongLatest = true;
			
		}
		
		_currentState = START;
		_idNumBytes = _payloadNumBytes = 0;
		
	  }
	}
