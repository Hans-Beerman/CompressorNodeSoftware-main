#include "PressureSensor.h"
#include <ACNode.h>

#ifndef PRESSURESENSOR
#define PRESSURESENSOR  (35)  // analog input
#endif

// pressure sensor
#define PRESSURE_SAMPLE_WINDOW (1000) // in ms
#define PRESSURE_CALIBRATE_VALUE_0_5V (144) // in measured bits
#define PRESSURE_CALIBRATE_VALUE_4_5V (2600) // in measured bits

int pressureADCVal = 0;
float pressureVoltage = 0;
float pressure = 0;
unsigned long pressureNextSampleTime = 0;
bool newCalibrationInfoAvailable = false;

PressureSensor::PressureSensor(float maxPressureLimit, float minPressureLimit) {
  pressureMaxLimit = maxPressureLimit;
  pressureMinLimit = minPressureLimit;
	return;
}

void PressureSensor::loop() {
  if (millis() > pressureNextSampleTime) {
    pressureNextSampleTime = millis() + PRESSURE_SAMPLE_WINDOW;
    pressureADCVal = analogRead(PRESSURESENSOR);
    if (pressureADCVal < PRESSURE_CALIBRATE_VALUE_0_5V) {
      pressureADCVal = PRESSURE_CALIBRATE_VALUE_0_5V;
    }
    pressureVoltage = (((float)pressureADCVal - (float)PRESSURE_CALIBRATE_VALUE_0_5V) * 4.0) / ((float)PRESSURE_CALIBRATE_VALUE_4_5V - (float)PRESSURE_CALIBRATE_VALUE_0_5V) + 0.5;
    pressure = (((pressureVoltage - 0.5) / 4.0) * 1.2) * 10; // pressure in bar
    newCalibrationInfoAvailable = true;
    pressureIsAboveMaximum = pressure > pressureMaxLimit;
    pressureIsBelowMinimum = pressure < pressureMinLimit;
  }
}

void PressureSensor::logInfoCalibration() {
  Log.print("Pressure ADC = ");
  Log.print(pressureADCVal);
  Log.println(" bits");
  Log.print("Pressure voltage = ");
  Log.print(pressureVoltage);
  Log.println(" V");
  Log.print("Pressure = ");
  Log.print(pressure);
  Log.println(" bar");
  newCalibrationInfoAvailable = false;
}

  bool PressureSensor::tooHighPressure() {
    return pressureIsAboveMaximum;
  }

  bool PressureSensor::lowPressure() {
    return pressureIsBelowMinimum;
  }


