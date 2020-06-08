#pragma once

extern float pressure;

class PressureSensor {
private:
  float pressureMaxLimit;
  float pressureMinLimit;
  bool pressureIsAboveMaximum; // Compressor must be switched off above this limit for safety
  bool pressureIsBelowMinimum; // If compressor was switched off because pressure was to high, 
                               // compressor can be switched on again if pressure becomes below this limit

public:
  PressureSensor(float maxPressureLimit, float minPressureLimit);
  
  bool newCalibrationInfoAvailable;
  
  void loop();

  void logInfoCalibration();

  bool tooHighPressure();

  bool lowPressure();
};