#include "TempSensor.h"
#include "OledDisplay.h"
#include <OneWire.h> 
#include <DallasTemperature.h> // install DallasTemperature by Miles Burton
#include <ACNode.h>

#ifndef TEMPSENSOR
#define TEMPSENSOR      ( 4)  // one wire digital input (GPIO4)
#endif

// One Wire input port for temperature sensor
#define ONE_WIRE_BUS (TEMPSENSOR)

#define TEMP_RESOLUTION (12) // 9, 10, 11 or 12 bit resolution of the ADC in the temperature sensor
#define MAX_TEMP_CONVERSIONTIME (750) // in ms

#define MAX_TEMP_IS_TOO_HIGH_WINDOW (10000) // in ms default 10000 = 10 seconds. Error is only signalled after this time window is passed

// temperature sensor
OneWire oneWire(ONE_WIRE_BUS); // used for the temperature sensor
DallasTemperature sensorTemp(&oneWire);

DeviceAddress tempDeviceAddress;

float currentTemperature;
float temperature;
bool tempIsHigh;
bool ErrorTempIsTooHigh;

TemperatureSensor::TemperatureSensor(float tempIsHighLevel, float tempIsTooHighLevel) {
	theTempIsHighLevel = tempIsHighLevel;
	theTempIsTooHighLevel = tempIsTooHighLevel;
  conversionTime = MAX_TEMP_CONVERSIONTIME / (1 << (12 - TEMP_RESOLUTION));
}

void TemperatureSensor::begin() {
  sensorTemp.begin();
  sensorTemp.getAddress(tempDeviceAddress, 0);
  sensorTemp.setResolution(tempDeviceAddress, TEMP_RESOLUTION);
  sensorTemp.setWaitForConversion(false);
  sensorTemp.requestTemperaturesByAddress(tempDeviceAddress);
  tempAvailableTime = millis() + conversionTime;
}

void TemperatureSensor::loop() {
  if (millis() > tempAvailableTime) {
    currentTemperature = sensorTemp.getTempC(tempDeviceAddress);

    if (currentTemperature == -127) {
      Log.println("Temperature sensor does not react, perhaps not available?");
    } else {
      if (currentTemperature != previousTemperature) {
        previousTemperature = currentTemperature;
        temperature = currentTemperature;
        Serial.print("Temp. changed, current temp. = ");
        Serial.print(temperature);
        Serial.println(" degrees C");
      }
    }
    sensorTemp.requestTemperaturesByAddress(tempDeviceAddress);
    tempAvailableTime = millis() + conversionTime;
    if (temperature <= theTempIsHighLevel) {
      if (tempIsHigh) {
        Log.println("Temperature is OK now (below warning threshold)");
      }
      tempIsHigh = false;
      if (ErrorTempIsTooHigh)
      {
        nextTimeDisplay = true;
      }
      ErrorTempIsTooHigh = false;
      tempIsTooHighStart = 0;
    } else {
      if (!tempIsHigh) {
        Log.println("WARNING: temperature is above warning level. Please check compressor");
      }
      tempIsHigh = true;
      if ((temperature > theTempIsTooHighLevel) && !ErrorTempIsTooHigh) {
        if (tempIsTooHighStart == 0) {
          tempIsTooHighStart = millis();
        } else {
          if (millis() > (tempIsTooHighStart + MAX_TEMP_IS_TOO_HIGH_WINDOW)) {
            nextTimeDisplay = true;
            ErrorTempIsTooHigh = true;
            if (!ERRORLOWOILLEVEL) {
              Log.println("ERROR: Temperature is too high, compressor will be disabled. Please check compressor!");
            } else {
              Log.println("ERROR: Temperature is too high, please check compressor!");
            }
          }
        }
      } else {
        if ((temperature <= theTempIsTooHighLevel) && ErrorTempIsTooHigh) {
          tempIsTooHighStart = 0;
          ErrorTempIsTooHigh = false;
          nextTimeDisplay = true;
          Log.println("WARNING: Temperature is below error level now, but still above warning level. Please check compressor!");
        }
      }
    }
  }
}

