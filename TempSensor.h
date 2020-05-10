#pragma once

#include <DallasTemperature.h> // install DallasTemperature by Miles Burton

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
	char labelTempSensor[20];

public:
  float temperature;
  bool tempIsHigh;
  bool ErrorTempIsTooHigh;

  TemperatureSensor(float tempIsHighLevel, float tempIsTooHighLevel, const char *tempLabel);

  void begin();
  
  void loop();
};

