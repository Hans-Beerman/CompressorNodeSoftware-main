#pragma once

#include <MachState.h>

// software version
#define SOFTWARE_VERSION "  V0.8  "

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
  TIMEOUTEXTENDED,
  TIMEOUT,
  ERRORLOWOILLEVEL,
  NOLOWOILLEVEL,
  WARNINGHIGHTEMP1,
  ERRORHIGHTEMP1,
  WARNINGHIGHTEMP2,
  ERRORHIGHTEMP2
} statusdisplay_t;

extern bool nextTimeDisplay;

class OledDisplay {
private:
  float theTempIsHighLevel1;
  float theTempIsTooHighLevel1;
  float theTempIsHighLevel2;
  float theTempIsTooHighLevel2;
public:
	OledDisplay();

  void begin(float tempIsHighLevel1, float tempIsTooHighLevel1, float tempIsHighLevel2, float tempIsTooHighLevel2);

  void clearDisplay();

  void showStatus(statusdisplay_t statusMessage);

  void clearEEPromWarning();

  void clearEEPromMessage();

  void EEPromCleared();

  void cacheCleared();

  void loop(bool oilLevelIsTooLow, bool ErrorOilLevelIsTooLow, float temperature1, bool tempIsHigh1, 
            bool ErrorTempIsTooHigh1, float temperature2, bool tempIsHigh2, bool ErrorTempIsTooHigh2,
            float pressure, machinestates_t machinestate,
            unsigned long powered_total, unsigned long powered_last,
            unsigned long running_total, unsigned long running_last);
};
