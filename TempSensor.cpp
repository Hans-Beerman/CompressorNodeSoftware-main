#include "TempSensor.h"
#include "OledDisplay.h"
#include <OneWire.h> 
#include <ACNode.h>

#ifndef TEMPSENSOR
#define TEMPSENSOR      ( 4)  // one wire digital input (GPIO4)
#endif

// One Wire input port for temperature sensor
#define ONE_WIRE_BUS (TEMPSENSOR)

#define TEMP_RESOLUTION (12) // 9, 10, 11 or 12 bit resolution of the ADC in the temperature sensor
#define MAX_TEMP_CONVERSIONTIME (750) // in ms
#define MAX_NR_OF_TRIES 3

#define MAX_TEMP_IS_TOO_HIGH_WINDOW (10000) // in ms default 10000 = 10 seconds. Error is only signalled after this time window is passed

// temperature sensor
OneWire oneWire(ONE_WIRE_BUS); // used for the temperature sensor
DallasTemperature sensorTemp(&oneWire);

int currentTempSensor = 0;
float temperature[MAX_TEMP_SENSORS];
bool tempIsHigh[MAX_TEMP_SENSORS];
bool ErrorTempIsTooHigh[MAX_TEMP_SENSORS];
char labelTempSensor[20] [MAX_TEMP_SENSORS];

TemperatureSensor::TemperatureSensor(float tempIsHighLevel, float tempIsTooHighLevel, const char *tempLabel) {
  tempSensorNr = currentTempSensor++;
	theTempIsHighLevel = tempIsHighLevel;
	theTempIsTooHighLevel = tempIsTooHighLevel;
  conversionTime = MAX_TEMP_CONVERSIONTIME / (1 << (12 - TEMP_RESOLUTION));
  sprintf(labelTempSensor[tempSensorNr], "%s", tempLabel);
}

void TemperatureSensor::begin() {
  tempIsHigh[tempSensorNr] = false;
  ErrorTempIsTooHigh[tempSensorNr] = false;
  sensorTemp.begin();

  tempSensorAvailable = sensorTemp.getAddress(tempDeviceAddress, tempSensorNr);
  if (!tempSensorAvailable) {
    temperature[tempSensorNr] = -127;

    Log.print("Temperature sensor ");
    Log.print(tempSensorNr + 1);
    Log.print(" (");
    Log.print(labelTempSensor[tempSensorNr]);
    Log.println("): sensor not detected at init of node!");
    return;
  }
  sensorTemp.setResolution(tempDeviceAddress, TEMP_RESOLUTION);
  sensorTemp.setWaitForConversion(false);
  sensorTemp.requestTemperaturesByAddress(tempDeviceAddress);
  tempAvailableTime = millis() + conversionTime;
  tryCount = MAX_NR_OF_TRIES;
}

void TemperatureSensor::loop() {
  if (!tempSensorAvailable) {
    return;
  }
  if (millis() > tempAvailableTime) {
    currentTemperature = sensorTemp.getTempC(tempDeviceAddress);

    if (currentTemperature == -127) {
      if (tryCount > 0) {
        tryCount--;
        tempAvailableTime = millis() + conversionTime;
        return;
      } else {
        Log.print("Temperature sensor ");
        Log.print(tempSensorNr + 1);
        Log.print(" (");
        Log.print(labelTempSensor[tempSensorNr]);
        Log.println("): sensor does not react, perhaps not available?");
      }
    } else {
      if (currentTemperature != previousTemperature) {
        previousTemperature = currentTemperature;
        temperature[tempSensorNr] = currentTemperature;
/*        
        Serial.print("Temperature ");
        Serial.print(tempSensor + 1);
        Serial.print(" changed, current temperature = ");
        Serial.print(temperature[tempSensorNr]);
        Serial.println(" degrees C");
*/        
      }
    }
    tryCount = MAX_NR_OF_TRIES;
    sensorTemp.requestTemperaturesByAddress(tempDeviceAddress);
    tempAvailableTime = millis() + conversionTime;
    if (temperature[tempSensorNr] <= theTempIsHighLevel) {
      if (tempIsHigh[tempSensorNr]) {
        Log.print("Temperature sensor ");
        Log.print(tempSensorNr + 1);
        Log.print(" (");
        Log.print(labelTempSensor[tempSensorNr]);
        Log.println("): temperature is OK now (below warning threshold)");
      }
      tempIsHigh[tempSensorNr] = false;
      if (ErrorTempIsTooHigh[tempSensorNr])
      {
        nextTimeDisplay = true;
      }
      ErrorTempIsTooHigh[tempSensorNr] = false;
      tempIsTooHighStart = 0;
    } else {
      if (!tempIsHigh[tempSensorNr]) {
        Log.print("WARNING: temperature sensor ");
        Log.print(tempSensorNr + 1);
        Log.print(" (");
        Log.print(labelTempSensor[tempSensorNr]);
        Log.println("): temperature is above warning level. Please check the compressor");
      }
      tempIsHigh[tempSensorNr] = true;
      if ((temperature[tempSensorNr] > theTempIsTooHighLevel) && !ErrorTempIsTooHigh[tempSensorNr]) {
        if (tempIsTooHighStart == 0) {
          tempIsTooHighStart = millis();
        } else {
          if (millis() > (tempIsTooHighStart + MAX_TEMP_IS_TOO_HIGH_WINDOW)) {
            nextTimeDisplay = true;
            ErrorTempIsTooHigh[tempSensorNr] = true;
            Log.print("ERROR, sensor ");
            Log.print(tempSensorNr + 1);
            Log.print(" (");
            Log.print(labelTempSensor[tempSensorNr]);
            Log.println("): Temperature is too high, compressor is disabled. Please check the compressor!");
          }
        }
      } else {
        if ((temperature[tempSensorNr] <= theTempIsTooHighLevel) && ErrorTempIsTooHigh[tempSensorNr]) {
          tempIsTooHighStart = 0;
          ErrorTempIsTooHigh[tempSensorNr] = false;
          nextTimeDisplay = true;
          Log.print("WARNING, sensor ");
          Log.print(tempSensorNr + 1);
          Log.print(" (");
          Log.print(labelTempSensor[tempSensorNr]);
          Log.println("): Temperature is below error level now, but still above warning level. Please check the compressor!");
        }
      }
    }
  }
}

