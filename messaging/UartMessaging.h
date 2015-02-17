#include <stdlib.h>
#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <UARTDriver.h>

//state machine for receiving
#define START	0 
#define ID		1
#define PAYLOAD	2
#define END		3

class UartMessaging{

private:
	AP_HAL::UARTDriver* _UARTdriver;
	AP_HAL::ConsoleDriver* console;

	int _currentState;//state of receiving the message

	char _idReceived[4]; //actual ID received
	short _idNumBytes; //number of bytes received on the ID state

	char _payloadReceived[32]; //Actual payload receive
	short _payloadNumBytes; //number of bytes received on thepayload stage
	
	bool _isUserLatLatest, _isUserLongLatest;
	float _userLat, _userLong;

public:
	void init(AP_HAL::UARTDriver* driver, AP_HAL::ConsoleDriver* _console);

	void sendAltitude(float altitude);
	void sendBattery(float battery);
	void sendGPSLock(bool gpsLock);
	void sendSafetyStatus(bool isSafety);

	bool isUserLatLatest();
	void getUserLat(float* lat);

	bool isUserLonLatest();
	void getUserLon(float* lon);

	//generic send functions
	void send(const char* messageID, float fPayload);
	void send(const char* messageID, bool bPayload);
	void send(const char* messageID, char* strPayload);
	void send(char* buffer);

	void receive();

};

