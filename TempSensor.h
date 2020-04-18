#pragma once

extern float currentTemperature;
extern float temperature;
extern bool tempIsHigh;
extern bool ErrorTempIsTooHigh;

class TemperatureSensor {

private:
	float theTempIsHighLevel;
	float theTempIsTooHighLevel;
	float previousTemperature = -500;
	unsigned long conversionTime;
	unsigned long tempAvailableTime = 0;
	unsigned long tempIsTooHighStart = 0;

public:
  TemperatureSensor(float tempIsHighLevel, float tempIsTooHighLevel);

  void begin();
  
  void loop();
};

