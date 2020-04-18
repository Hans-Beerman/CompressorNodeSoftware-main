#include "OilLevelSensor.h"
#include "OledDisplay.h"
#include <ButtonDebounce.h>
#include <ACNode.h>

#ifndef OILLEVELSENSOR
#define OILLEVELSENSOR  (39)  // digital input
#endif

// oil level sensor
#define TO_LOW_OIL_LEVEL (LOW) // the input level of the GPIO port used for the oil level sensor signalling too low oil level
#define MAX_OIL_LEVEL_IS_TOO_LOW_WINDOW (10000) // in ms default 10000 = 10 seconds. Error is signalled after this time window is passed
#define OIL_LEVEL_LOG_WINDOW (60000) // in ms

ButtonDebounce oilLevel(OILLEVELSENSOR, 300 /* mSeconds */); // to signal if the oil level is too low (or not)

bool previousOilLevelIsTooLow = false;
bool oilLevelIsTooLow = false;
unsigned long oilLevelNextLoggingTime = 0;
unsigned long oilLevelIsTooLowStart = 0;
bool previousErrorOilLevelIsTooLow = false;
bool ErrorOilLevelIsTooLow = false;

OilLevelSensor::OilLevelSensor() {
  return;
}

void OilLevelSensor::begin() {
  pinMode(OILLEVELSENSOR, INPUT_PULLUP);

  oilLevel.setCallback([](int state) {
    Debug.printf("OilLevel sensor changed to %d\n", state);
  });
}

void OilLevelSensor::loop() {
  oilLevel.update();
  if (oilLevel.state() == TO_LOW_OIL_LEVEL) {
    oilLevelIsTooLow = true;

    if (oilLevelIsTooLowStart == 0) {
      oilLevelIsTooLowStart = millis();
    } else {
      if ((millis() - oilLevelIsTooLowStart) >= MAX_OIL_LEVEL_IS_TOO_LOW_WINDOW) {
        ErrorOilLevelIsTooLow = true;
        if ((ErrorOilLevelIsTooLow != previousErrorOilLevelIsTooLow) || (millis() > oilLevelNextLoggingTime)) {
          if (ErrorOilLevelIsTooLow != previousErrorOilLevelIsTooLow) {
            nextTimeDisplay = true;
          }
          oilLevelNextLoggingTime = millis() + OIL_LEVEL_LOG_WINDOW;
          previousErrorOilLevelIsTooLow = ErrorOilLevelIsTooLow;
          Log.println("ERROR: Oil level is too low; Compressor will be disabled; Please maintain the compressor by filling up the oil");
        }
      }
    }
    
    if ((oilLevelIsTooLow != previousOilLevelIsTooLow) || (millis() > oilLevelNextLoggingTime)) {
      oilLevelNextLoggingTime = millis() + OIL_LEVEL_LOG_WINDOW;
      Log.println("Oil level is too low!");
      previousOilLevelIsTooLow = oilLevelIsTooLow;
    }
  } else {
    if(ErrorOilLevelIsTooLow) {
       nextTimeDisplay = true;
       Log.println("SOLVED: Oil level error");
    }
    oilLevelIsTooLow = false;
    oilLevelIsTooLowStart = 0;
    ErrorOilLevelIsTooLow = false;
    previousErrorOilLevelIsTooLow = false;
    if (oilLevelIsTooLow != previousOilLevelIsTooLow) {
      Log.println("Oil level is OK now!");
      previousOilLevelIsTooLow = oilLevelIsTooLow;
    }
  }
}

