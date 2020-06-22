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

ButtonDebounce oilLevel(OILLEVELSENSOR, 300 /* mSeconds */); // to signal if the oil level is too low (or not)

bool oilLevelIsTooLow = false;
bool waitForError = false;
unsigned long oilLevelIsTooLowStart = 0;
bool ErrorOilLevelIsTooLow = false;

OilLevelSensor::OilLevelSensor() {
  return;
}

void OilLevelSensor::begin() {
  pinMode(OILLEVELSENSOR, INPUT_PULLUP);

  oilLevel.setCallback([](int state) {
//    Debug.printf("OilLevel sensor changed to %d\n", state);
    if (oilLevel.state() == TO_LOW_OIL_LEVEL) {
      nextTimeDisplay = true;
      oilLevelIsTooLow = true;
      oilLevelIsTooLowStart = millis();
      waitForError = true;
      Log.println("Warning: Oil level is too low; Compressor will be disabled soon if this issue is not solved; Please verify the oil level and fill up if needed");
    } else {
      if (ErrorOilLevelIsTooLow) {
        Log.println("SOLVED: Oil level error!");
        nextTimeDisplay = true;
      } else {
        if (oilLevelIsTooLow) {
          Log.println("Oil level OK now!");
          nextTimeDisplay = true;
      }
      }
      oilLevelIsTooLow = false;
      oilLevelIsTooLowStart = 0;
      ErrorOilLevelIsTooLow = false;
      waitForError = false;
    }
  });
}

void OilLevelSensor::loop() {
  oilLevel.update();
  
  if (waitForError) {
    if ((millis() - oilLevelIsTooLowStart) >= MAX_OIL_LEVEL_IS_TOO_LOW_WINDOW) {
      waitForError = false;
      ErrorOilLevelIsTooLow = true;
      nextTimeDisplay = true;
      Log.println("ERROR: Oil level is too low; Compressor will be disabled; Please maintain the compressor by filling up the oil");
    }
  }
}

