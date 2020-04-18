#pragma once

#include <MachState.h>

typedef enum {
  NOSTATUS,
  RELEASEBUTTON,
  NODEREBOOT,
  MANUALSWITCHON,
  MANUALOVERRIDE,
  MANUALSWITCHOFF,
  AUTOSWITCHON,
  AUTOONDENIED,
  AUTOSWITCHOFF,
  POWERONDISABLED,
  TIMEOUT,
  ERRORLOWOILLEVEL,
  NOLOWOILLEVEL,
  WARNINGHIGHTEMP,
  ERRORHIGHTEMP
} statusdisplay_t;

extern bool nextTimeDisplay;

class OledDisplay {

public:
	OledDisplay();

  void begin();

  void clearDisplay();

  void showStatus(statusdisplay_t statusMessage, float tempIsHighLevel, float tempIsTooHighLevel);

  void clearEEPromWarning();

  void clearEEPromMessage();

  void EEPromCleared();

  void cacheCleared();

  void loop(bool oilLevelIsTooLow, bool ErrorOilLevelIsTooLow, float temperature, bool tempIsHigh, 
            bool ErrorTempIsTooHigh, float pressure, machinestates_t machinestate,
            float tempIsHighLevel, float tempIsTooHighLevel,
            unsigned long powered_total, unsigned long powered_last,
            unsigned long running_total, unsigned long running_last);
};
