#pragma once

extern float pressure;

class PressureSensor {
public:
  PressureSensor();
  
  bool newCalibrationInfoAvailable;
  
  void loop();

  void logInfoCalibration();
};