#pragma once

extern bool oilLevelIsTooLow;
extern bool ErrorOilLevelIsTooLow;

class OilLevelSensor {
public:
	OilLevelSensor();

    void begin();
	
	void loop();
};