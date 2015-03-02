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
	
	//User Lon and Lat
	bool _isUserLatLatest, _isUserLongLatest;
	int32_t _userLat, _userLon;

	//TakeOff and Land
	bool _isLand;
	bool _isTakeOff;

	int _seperationDistance;
	bool _isSeperationDistanceLatest;


	//generic send functions
	void send(const char* messageID, float fPayload);
	void send(const char* messageID, bool bPayload);
	void send(const char* messageID, long iPayload);
	void send(const char* messageID, char* strPayload);
	void send(char* buffer);


public:
	void init(AP_HAL::UARTDriver* driver, AP_HAL::ConsoleDriver* _console);

	void sendAltitude(float altitude);
	void sendBattery(float battery);
	void sendGPSStatus(int gpsStatus);
	void sendGPSAccuracy(float accuracy);
	void sendSafetyStatus(bool isSafety);
	void sendClimbRate(float climbRate);
	void sendDroneLat(int32_t lat);
	void sendDroneLon(int32_t lon);


	bool isUserLatLatest();
	void getUserLat(int32_t* lat);

	bool isUserLonLatest();
	void getUserLon(int32_t* lon);

	bool isTakeOff();
	bool isLand();
	void resetTakeOff();
	void resetLand();

	bool isSeperationDistanceLatest();
	void getSeperationDistance(int* distance);

	void receive();

};

