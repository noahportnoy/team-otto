#pragma once
#include <stdlib.h>
#include <string.h>
#include "UartMessaging.h"
//#include <stdio.h>

const char* StartCharacter = "$";
const char* EndCharacter = "!";

const char* BatteryStatus = "BTS";
const char* Altitude = "ALT";
const char* Latitude = "LAT";
const char* Longitude = "LON";
const char* GPSStatus = "GPS";
const char* GPSAccuracy = "GPA";
const char* Safety = "SFT";
const char* ClimbRate = "CRS";
const char* BearingToUser = "BRG";
const char* SeperationDistance = "SRD";


void UartMessaging::init(AP_HAL::UARTDriver* _driver, AP_HAL::ConsoleDriver* _console)
{
	_UARTdriver = _driver;
	_idNumBytes = _payloadNumBytes = 0;
	console =_console;

	_isUserLongLatest = _isUserLatLatest = false;
	_userLat = _userLon = 0;

	_isTakeOff = _isLand = false;

	_seperationDistance = 20;
	_isSeperationDistanceLatest = false;

}


void UartMessaging::sendAltitude(float altitude)
{
	send(Altitude, altitude);
}
void UartMessaging::sendBattery(float battery)
{
	send(BatteryStatus, battery);
}
void UartMessaging::sendGPSStatus(int gpsStatus)
{
	send(GPSStatus, (long)gpsStatus);
}
void UartMessaging::sendSafetyStatus(bool isSafety)
{
	send(Safety, isSafety);
}
void UartMessaging::sendClimbRate(float climbRate)
{
	send(ClimbRate, climbRate);
}
void UartMessaging::sendDroneLat(int32_t lat)
{
	send(Latitude, lat);
}
void UartMessaging::sendDroneLon(int32_t lon)
{
	send(Longitude, lon);
}
void UartMessaging::sendGPSAccuracy(float accuracy)
{
	send(GPSAccuracy, accuracy);
}

void UartMessaging::sendBearing(float bearing)
{
	send(BearingToUser, bearing);
}

void UartMessaging::sendSeperationDistance(float seperationDistance)
{
	send(SeperationDistance, seperationDistance);
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

bool UartMessaging::isTakeOff()
{
	return _isTakeOff;
}

bool UartMessaging::isLand()
{
	return _isLand;
}

void UartMessaging::resetTakeOff()
{
	_isTakeOff = false;
}

void UartMessaging::resetLand()
{
	_isLand = false;
}

bool UartMessaging::isSeperationDistanceLatest()
{
	return _isSeperationDistanceLatest;
}

void UartMessaging::getSeperationDistance(int* distance)
{
	_isSeperationDistanceLatest = false;
	*distance = _seperationDistance;
}
//Generic send functions
void UartMessaging::send(const char* messageID, float f)
{
	char buffer[16];
	memcpy(buffer, StartCharacter, 1);
	memcpy(&buffer[1], messageID, 3);
	dtostrf(f, 7, 5, &buffer[4]);
	memcpy(&buffer[11], EndCharacter, 1);
	memcpy(&buffer[12], "\n\0", 2);
	send(buffer);
}

void UartMessaging::send(const char* messageID, bool b)
{
	char buffer[16];
	memcpy(buffer, StartCharacter, 1);
	memcpy(&buffer[1], messageID, 4);
	if(b)
	{
		memcpy(&buffer[4], "1", 1);
	}else
	{
		memcpy(&buffer[4], "0", 1);
	}
	memcpy(&buffer[5], EndCharacter, 1);
	memcpy(&buffer[6], "\n\0", 2); //new line and properly terminate a c-string
	send(buffer);
}

void UartMessaging::send(const char* messageID, long iPayload)
{
	char buffer[16];
	memcpy(buffer, StartCharacter, 1);
	memcpy(&buffer[1], messageID, 3);
	ltoa(iPayload, &buffer[4], 10);

	int iPayloadSize = strlen(&buffer[4]);
	memcpy(&buffer[iPayloadSize + 4], EndCharacter, 1);
	memcpy(&buffer[iPayloadSize + 5], "\n\0", 2);
	send(buffer);
}

void UartMessaging::send(const char* messageID, char* buffer)
{

}

void UartMessaging::send(char* buffer)
{
	// console->println(buffer);
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
			_isTakeOff = true;
			_isLand = false;;
			// console->println("Take Off");
		}

		else if(_idReceived[0] == 'S' && _idReceived[1] == 'T' && _idReceived[2] == 'P')
		{
			_isLand = true;
			_isTakeOff = false;
			// console->println("Land");

		} else if(_idReceived[0] == 'L' && _idReceived[1] == 'A' && _idReceived[2] == 'T')
		{
			//Print the latitude
			// console->print("User Latitude: ");
			// console->println(_payloadReceived);

			//convert latitude from string to double
			_userLat = atol(_payloadReceived);

			_isUserLatLatest = true;

		}else if(_idReceived[0] == 'L' && _idReceived[1] == 'O' && _idReceived[2] == 'N')
		{
			//Print the latitude
			// console->print("User Longitude: ");
			// console->println(_payloadReceived);

			//convert latitude from string to double
			_userLon = atol(_payloadReceived);
			_isUserLongLatest = true;

		}else if(_idReceived[0] == 'S' && _idReceived[1] == 'R' && _idReceived[2] == 'D')
		{
			// console->print("Seperation Distance: ");
			// console->println(_payloadReceived);

			_seperationDistance = atoi(_payloadReceived);
			_isSeperationDistanceLatest = true;
		}

		_currentState = START;
		_idNumBytes = _payloadNumBytes = 0;

	  }
	}
