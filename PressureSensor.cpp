#include "PressureSensor.h"
#include <ACNode.h>

#ifndef PRESSURESENSOR
#define PRESSURESENSOR  (35)  // analog input
#endif

// for calibrating the pressure sensor remove the comment statement in front of next #define
// #define CALIBRATE_PRESSURE

// pressure sensor
#define PRESSURE_SAMPLE_WINDOW (1000) // in ms
#define PRESSURE_LOG_WINDOW (60000) // in ms
#define PRESSURE_CALIBRATE_VALUE_0_5V (144) // in measured bits
#define PRESSURE_CALIBRATE_VALUE_4_5V (2600) // in measured bits

int pressureADCVal = 0;
float pressureVoltage = 0;
float pressure = 0;
unsigned long pressureNextSampleTime = 0;
unsigned long pressureNextLogTime = 0;

PressureSensor::PressureSensor() {
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
    pressure = ((pressureVoltage - 0.5) / 4.0) * 1.2;

    if (millis() > pressureNextLogTime) {
      pressureNextLogTime = millis() + PRESSURE_LOG_WINDOW;
      Log.print("Pressure = ");
      Log.print(pressure);
      Log.println(" MPa");
    }

    // for calibration
    #ifdef CALIBRATE_PRESSURE
    Serial.print("Pressure ADC = ");
    Serial.print(pressureADCVal);
    Serial.println(" bits");
    Serial.print("Pressure voltage = ");
    Serial.print(pressureVoltage);
    Serial.println(" V");
    Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.println(" MPa");
    #endif
    // end calibration
  }
}
