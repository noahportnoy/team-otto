#include <stdlib.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <UARTDriver.h>
#include <AP_HAL_Empty.h>
#include <AP_HAL_PX4.h>
#include <AP_GPS.h>
#include <AP_Math.h>

#define START	0 
#define ID		1
#define PAYLOAD	2
#define END		3

class UartMessaging{

private:
	AP_HAL::UARTDriver* UARTdriver;
	AP_HAL::ConsoleDriver* console;

	int currentState;//state of receiving the message
	char idReceived[4]; //actual ID received
	short idNumBytes; //number of bytes received on the ID state
	short payloadNumBytes; //number of bytes received on thepayload stage
	char payloadReceived[32]; //Actual payload receive

public:
	void init(AP_HAL::UARTDriver* driver, AP_HAL::ConsoleDriver* _console);

	void sendAltitude(float altitude);
	void sendBattery(float battery);

	void sendGPSLock(bool gpsLock);


	void send(const char* messageID, float fPayload);
	void send(const char* messageID, bool bPayload);
	void send(const char* messageID, char* strPayload);
	void send(char* buffer);
	void receive();

};

