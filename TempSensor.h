#pragma once

#include <DallasTemperature.h> // install DallasTemperature by Miles Burton

#define MAX_TEMP_SENSORS (2) // maximum number of sensors used

extern float temperature[MAX_TEMP_SENSORS];
extern bool tempIsHigh[MAX_TEMP_SENSORS];
extern bool ErrorTempIsTooHigh[MAX_TEMP_SENSORS];

class TemperatureSensor {

private:
    int tempSensorNr;
	DeviceAddress tempDeviceAddress;
	bool tempSensorAvailable = false;
	float currentTemperature;
	float theTempIsHighLevel;
	float theTempIsTooHighLevel;
	float previousTemperature = -500;
	unsigned long conversionTime;
	unsigned long tempAvailableTime = 0;
	unsigned long tempIsTooHighStart = 0;
	int tryCount;

public:
  TemperatureSensor(float tempIsHighLevel, float tempIsTooHighLevel);

  void begin();
  
  void loop();
};

