#include "OledDisplay.h"

#include <U8x8lib.h> // install U8g2 library by oliver

// for I2C display
#ifndef I2C_SDA
#define I2C_SDA         (13)  // I2C SDA = GPIO 13
#endif
#ifndef I2C_SCL
#define I2C_SCL         (16)  // I2C SCL = GPIO 16
#endif

// oled display
#define DISPLAY_WINDOW (1000) // in ms, update display time

#define KEEP_STATUS_LINE_TIME (5000) // in ms, default = 5 s (5000), the time certain status messages are shown on the bottom line of the display

typedef enum {
  NORMALDISPLAY,        // Normal display shown when there is no error, showing current compressor state etc.
  ERRORDISPLAY
} displaystates_t;

struct {
  const char * statusmessage;
  int y;
  bool temporarily;
} dispstatus[ERRORHIGHTEMP2 + 1] = 
{
	{ "                ", 15, false },
	{ "Release button  ", 15, false },
	{ "Node will reboot", 15, false },
	{ "Manual poweron  ", 15, true },
	{ "Manual override ", 15, true },
	{ "Manual off      ", 15, true },
	{ "Auto Power on   ", 15, true },
	{ "Auto on denied  ", 15, true },
	{ "Automatic Stop  ", 15, true },
	{ "Poweron disabled", 15, true },
  { "Timout extended ", 15, true },
	{ "Timeout ==> off ", 15, true },
  { "Pressure toohigh", 15, false},
	{ "OilLevel too low", 7, false },
	{ "OilLevel OK!    ", 7, false },
	{ "Warning         ", 15, false },
	{ "ERROR           ", 15, false },
	{ "Warning         ", 15, false },
	{ "ERROR           ", 15, false },
};

bool nextTimeDisplay = true;

bool showStatusTemporarily = false;
unsigned long clearStatusLineTime = 0;

machinestates_t laststateDisplayed = BOOTING;

displaystates_t currentDisplayState = NORMALDISPLAY;
unsigned long updateDisplayTime = 0;

bool lastOilLevelDisplayed = false;

float lastTempDisplayed1 = -500;
float lastTempDisplayed2 = -500;

bool previousTempIsHigh1 = false;
bool previousErrorTempIsTooHigh1 = false;
bool previousTempIsHigh2 = false;
bool previousErrorTempIsTooHigh2 = false;

bool previousErrorOilLevelIsTooLow = false;
float lastPressureDisplayed = -1;

float poweredTime = 0.0;
float lastPoweredDisplayed = 0.0;
float runningTime = 0.0;
float lastRunningDisplayed = 0.0;

// for 1.5 inch OLED Display 128*128 pixels wit - I2C
U8X8_SSD1327_WS_128X128_SW_I2C u8x8(I2C_SCL, I2C_SDA, U8X8_PIN_NONE);

OledDisplay::OledDisplay() {
  return;
}

void OledDisplay::begin(float tempIsHighLevel1, float tempIsTooHighLevel1, float tempIsHighLevel2, float tempIsTooHighLevel2) {
// for 1.5 inch OLED Display 128*128 pixels wit - I2C
  pinMode(I2C_SDA, OUTPUT);
  pinMode(I2C_SCL, OUTPUT);
  digitalWrite(I2C_SDA, 0);
  digitalWrite(I2C_SCL, 0);

  theTempIsHighLevel1 = tempIsHighLevel1;
  theTempIsTooHighLevel1 = tempIsTooHighLevel1;
  theTempIsHighLevel2 = tempIsHighLevel2;
  theTempIsTooHighLevel2 = tempIsTooHighLevel2;

  u8x8.begin();
  u8x8.clear();
  u8x8.setCursor(0, 0);

  u8x8.setFont(u8x8_font_px437wyse700a_2x2_r);

  u8x8.drawString(0, 0, "CompNode");

  u8x8.setFont(u8x8_font_px437wyse700b_2x2_r);
  u8x8.drawString(0, 2, SOFTWARE_VERSION);

  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.drawString(0, 4, " c Hans Beerman ");
  u8x8.drawString(0, 6, "Booting, please ");
  u8x8.drawString(0, 7, "      wait      ");
}

void OledDisplay::clearDisplay() {
  u8x8.clearDisplay();
}

void OledDisplay::showStatus(statusdisplay_t statusMessage) {
  char outputStr[20];

  switch (statusMessage) {
    case NOSTATUS:
    case ERRORLOWOILLEVEL:
    case NOLOWOILLEVEL:
      u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
      u8x8.drawString(0, dispstatus[statusMessage].y, dispstatus[statusMessage].statusmessage);
      break;
    case WARNINGHIGHTEMP1:
      u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
      sprintf(outputStr, "WARNING >%4.0f %cC", theTempIsHighLevel1, 176);
      u8x8.drawString(0, dispstatus[statusMessage].y, outputStr);
      break;
    case ERRORHIGHTEMP1:
      u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
      sprintf(outputStr, "ERROR 1 >%4.0f %cC", theTempIsTooHighLevel1, 176);
      u8x8.drawString(0, dispstatus[statusMessage].y, outputStr);
      break;
    case WARNINGHIGHTEMP2:
      u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
      sprintf(outputStr, "WARNING >%4.0f %cC", theTempIsHighLevel2, 176);
      u8x8.drawString(0, dispstatus[statusMessage].y, outputStr);
      break;
    case ERRORHIGHTEMP2:
      u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
      sprintf(outputStr, "ERROR 2 >%4.0f %cC", theTempIsTooHighLevel2, 176);
      u8x8.drawString(0, dispstatus[statusMessage].y, outputStr);
      break;
    default:
      u8x8.setFont(u8x8_font_chroma48medium8_r);
      u8x8.drawString(0, dispstatus[statusMessage].y, dispstatus[statusMessage].statusmessage);
      break;
  }
  
  showStatusTemporarily = dispstatus[statusMessage].temporarily;
  clearStatusLineTime = millis() + KEEP_STATUS_LINE_TIME;
}

void OledDisplay::clearEEPromWarning() {
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.drawString(0, 9, "Keep Olimex BUT2");
  u8x8.drawString(0, 10, "pressed for at  ");
  u8x8.drawString(0, 11, "least 4 seconds ");
  u8x8.drawString(0, 12, "to clear EEProm ");
  u8x8.drawString(0, 13, "and cache memory");
}

void OledDisplay::clearEEPromMessage() {
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.drawString(0, 9, "EEProm and cache");
  u8x8.drawString(0, 10, "will be cleared ");
  u8x8.drawString(0, 11, "                ");
  u8x8.drawString(0, 12, "                ");
  u8x8.drawString(0, 13, "                ");
}

void OledDisplay::EEPromCleared() {
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.drawString(0, 12, "EEProm cleared  ");
}

void OledDisplay::cacheCleared() {
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.drawString(0, 13, "Cache cleared   ");
}

void OledDisplay::loop(bool oilLevelIsTooLow, bool ErrorOilLevelIsTooLow, float temperature1, bool tempIsHigh1, 
                      bool ErrorTempIsTooHigh1, float temperature2, bool tempIsHigh2, bool ErrorTempIsTooHigh2, bool ErrorPressureIsToHigh,
                      float pressure, machinestates_t machinestate,
                      unsigned long powered_total, unsigned long powered_last,
                      unsigned long running_total, unsigned long running_last) {

  char outputStr[20];
  bool temp1PrintAll = false;
  bool temp2PrintAll = false;

  if (!ErrorOilLevelIsTooLow && !ErrorTempIsTooHigh1 && !ErrorTempIsTooHigh2) {
    if (currentDisplayState == ERRORDISPLAY) {
      nextTimeDisplay = true;
      previousTempIsHigh1 = !tempIsHigh1;
      previousTempIsHigh2 = !tempIsHigh2;
      u8x8.clearDisplay();
      currentDisplayState = NORMALDISPLAY;
    }
  } else {
    if (currentDisplayState == NORMALDISPLAY) {
      nextTimeDisplay = true;
      u8x8.clearDisplay();
      currentDisplayState = ERRORDISPLAY;
    }
  }

  switch (currentDisplayState)
  {
    case NORMALDISPLAY:
      if (millis() > updateDisplayTime)
      {
        updateDisplayTime = millis() + DISPLAY_WINDOW;

        if ((pressure != lastPressureDisplayed) || nextTimeDisplay) {
          lastPressureDisplayed = pressure;
          u8x8.setFont(u8x8_font_px437wyse700b_2x2_f);
          if (nextTimeDisplay) {
            u8x8.drawString(0, 0, "Pressure");
            sprintf(outputStr, "%4.1f bar", pressure); 
          } else {
            sprintf(outputStr, "%4.1f", pressure); 
          }
          u8x8.drawString(0, 2, outputStr);
        }
        if ((temperature1 != lastTempDisplayed1) || nextTimeDisplay) {
          lastTempDisplayed1 = temperature1;
          u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
          if (nextTimeDisplay) {
            if (temperature1 < -100) {
              sprintf(outputStr, "Temp1: N.A.     ");
            } else {
              sprintf(outputStr, "Temp1:%7.2f %cC", temperature1, 176);
            }
            u8x8.drawString(0, 5, outputStr);
          } else {
            if (temperature1 < -100) {
              sprintf(outputStr, " N.A.     ");
              temp1PrintAll = true;
            } else {
              if (temp1PrintAll) {
                sprintf(outputStr, "%7.2f %cC", temperature1, 176);
              } else {
                sprintf(outputStr, "%7.2f", temperature1);
              }
              temp1PrintAll = false;
            }
            u8x8.drawString(6, 5, outputStr);
          }
          if ((previousTempIsHigh1 != tempIsHigh1) || (previousErrorTempIsTooHigh1 != ErrorTempIsTooHigh1) || nextTimeDisplay) {
            if (ErrorTempIsTooHigh1) {
              showStatus(ERRORHIGHTEMP1);
            } else {
              if (tempIsHigh1) {
                showStatus(WARNINGHIGHTEMP1);
              } else {
                if (!tempIsHigh2) {
                  showStatus(NOSTATUS);
                }
              }
            }
            previousErrorTempIsTooHigh1 = ErrorTempIsTooHigh1;  
            previousTempIsHigh1 = tempIsHigh1;
          }
        }

        if ((temperature2 != lastTempDisplayed2) || nextTimeDisplay) {
          lastTempDisplayed2 = temperature2;
          u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
          if (nextTimeDisplay) {
            if (temperature2 < -100) {
              sprintf(outputStr, "Temp2: N.A.     ");
            } else {
              sprintf(outputStr, "Temp2:%7.2f %cC", temperature2, 176);
            }
            u8x8.drawString(0, 6, outputStr);
          } else {
            if (temperature2 < -100) {
              sprintf(outputStr, " N.A.     ");
              temp2PrintAll = true;
            } else {
              if (temp2PrintAll) {
                sprintf(outputStr, "%7.2f %cC", temperature2, 176);
              } else {
                sprintf(outputStr, "%7.2f", temperature2);
              }
              temp2PrintAll = false;
            }
            u8x8.drawString(6, 6, outputStr);
          }
          if ((previousTempIsHigh2 != tempIsHigh2) || (previousErrorTempIsTooHigh2 != ErrorTempIsTooHigh2) || nextTimeDisplay) {
            if (ErrorTempIsTooHigh2) {
              showStatus(ERRORHIGHTEMP2);
            } else {
              if (tempIsHigh2) {
                showStatus(WARNINGHIGHTEMP2);
              } else {
                if (!tempIsHigh1) {
                  showStatus(NOSTATUS);
                }
              }
            }
            previousErrorTempIsTooHigh2 = ErrorTempIsTooHigh2;  
            previousTempIsHigh2 = tempIsHigh2;
          }
        }

        if ((machinestate != laststateDisplayed)  || nextTimeDisplay) {
          laststateDisplayed = machinestate;
          u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
          if (nextTimeDisplay) {
            u8x8.drawString(0, 9, "Machine state:  ");
          }
          switch (machinestate) {
            case BOOTING:
                sprintf(outputStr, "Booting         ");
              break;
            case OUTOFORDER:
                sprintf(outputStr, "Out of order    ");
              break;
            case REBOOT:
                sprintf(outputStr, "Reboot          ");
              break;
            case TRANSIENTERROR:
                sprintf(outputStr, "Transient error ");
              break;
            case NOCONN:
                sprintf(outputStr, "No connection   ");
              break;
            case SWITCHEDOFF:
                sprintf(outputStr, "Switched off    ");
              break;
            case POWERED:
                sprintf(outputStr, "On, motor off   ");
              break;
            case RUNNING:
                sprintf(outputStr, "Motor is running");
            break;
          }      
          u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
          u8x8.drawString(0, 10, outputStr);
        }

        if ((oilLevelIsTooLow != lastOilLevelDisplayed) || nextTimeDisplay) {
          lastOilLevelDisplayed = oilLevelIsTooLow;
          if (oilLevelIsTooLow) {
            showStatus(ERRORLOWOILLEVEL);
          } else {
            showStatus(NOLOWOILLEVEL);
          }
        }

        if ((machinestate >= POWERED) || nextTimeDisplay) {
          if (machinestate < POWERED) {
            poweredTime = (float)powered_total / 3600.0;
          } else {
            poweredTime = ((float)powered_total + ((float)millis() - float(powered_last)) / 1000.0) / 3600.0;
          }

          if ((poweredTime != lastPoweredDisplayed) || nextTimeDisplay) {
            lastPoweredDisplayed = poweredTime;
            u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
            if (nextTimeDisplay) {
              sprintf(outputStr, "On: %9.2f hr", poweredTime);
              u8x8.drawString(0, 12, outputStr);
            } else {
              sprintf(outputStr, "%9.2f", poweredTime);
              u8x8.drawString(4, 12, outputStr);
            }
          }
        }

        if ((machinestate == RUNNING) || nextTimeDisplay) {
          if (machinestate < RUNNING) {
            runningTime = (float)running_total / 3600.0;
          } else {
            runningTime = ((float)running_total + ((float)millis() - (float)running_last) / 1000.0) / 3600.0;
          }

          if ((runningTime != lastRunningDisplayed) || nextTimeDisplay) {
            lastRunningDisplayed = runningTime;
            u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
            if (nextTimeDisplay) {
              sprintf(outputStr, "Run:%9.2f hr", runningTime);
              u8x8.drawString(0, 13, outputStr);
            } else {
              sprintf(outputStr, "%9.2f", runningTime);
              u8x8.drawString(4, 13, outputStr);
            }
          }
        }
        nextTimeDisplay = false;
      }

      if (showStatusTemporarily && (millis() > clearStatusLineTime)) {
        if (tempIsHigh1 || tempIsHigh2)  {
          showStatus(WARNINGHIGHTEMP1);
        } else {
          if (ErrorTempIsTooHigh1) {
            showStatus(ERRORHIGHTEMP1);
          } else {
            if (ErrorTempIsTooHigh2) {
              showStatus(ERRORHIGHTEMP2);
            } else {
              if (ErrorPressureIsToHigh) {
                showStatus(ERRORPRESSUREISTOOHIGH);
              } else {
                showStatus(NOSTATUS);
              }
            }
          }
        }
        showStatusTemporarily = false;
      }

    break;
    case ERRORDISPLAY:
      if (millis() > updateDisplayTime)
      {
        updateDisplayTime = millis() + DISPLAY_WINDOW;
        if (nextTimeDisplay) {
          u8x8.setFont(u8x8_font_px437wyse700a_2x2_r);
          u8x8.drawString(0, 0, "MAINTAIN");
          u8x8.drawString(0, 2, "COMPRSR.");
          u8x8.setFont(u8x8_font_px437wyse700a_2x2_r);
          u8x8.drawString(0, 12, "COMPRSR.");  
          u8x8.drawString(0, 14, "DISABLED");
        }

        if ((previousErrorTempIsTooHigh1 != ErrorTempIsTooHigh1) || (previousErrorTempIsTooHigh2 != ErrorTempIsTooHigh2) || nextTimeDisplay) {
          previousErrorTempIsTooHigh1 = ErrorTempIsTooHigh1;
          previousErrorTempIsTooHigh2 = ErrorTempIsTooHigh2;
          if ((ErrorTempIsTooHigh1) || (ErrorTempIsTooHigh2)) {
            u8x8.setFont(u8x8_font_chroma48medium8_r);
            if (!ErrorTempIsTooHigh2) {
              u8x8.drawString(0, 8, "TEMPERATURE 1   ");
              u8x8.drawString(0, 9, "IS TOO HIGH     ");
            } else {
              if (!ErrorTempIsTooHigh1) {
                u8x8.drawString(0, 8, "TEMPERATURE 2   ");
                u8x8.drawString(0, 9, "IS TOO HIGH     ");
              } else {
                u8x8.drawString(0, 8, "TEMPERATURE 1+2 ");
                u8x8.drawString(0, 9, "ARE TOO HIGH    ");
              }
            }
          } else {
            u8x8.setFont(u8x8_font_chroma48medium8_r);
            u8x8.drawString(0, 8, "                ");
            u8x8.drawString(0, 9, "                ");
          }
        }

        if (ErrorTempIsTooHigh1) {
          if ((temperature1 != lastTempDisplayed1) || nextTimeDisplay) {
            lastTempDisplayed1 = temperature1;
            u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
            sprintf(outputStr, "Temp1:%7.2f %cC", temperature1, 176);
            u8x8.drawString(0, 10, outputStr);
          }
        } else {
          u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
          u8x8.drawString(0, 10, "                ");
        }

        if (ErrorTempIsTooHigh2) {
          if ((temperature2 != lastTempDisplayed2) || nextTimeDisplay) {
            lastTempDisplayed2 = temperature2;
            u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
            sprintf(outputStr, "Temp2:%7.2f %cC", temperature2, 176);
            u8x8.drawString(0, 11, outputStr);
          }
        } else {
          u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
          u8x8.drawString(0, 11, "                ");
        }

        if ((ErrorOilLevelIsTooLow != previousErrorOilLevelIsTooLow) || nextTimeDisplay) {
          previousErrorOilLevelIsTooLow = ErrorOilLevelIsTooLow;
          if (ErrorOilLevelIsTooLow) {
            u8x8.setFont(u8x8_font_chroma48medium8_r);
            u8x8.drawString(0, 5, "OIL LEVEL       ");
            u8x8.drawString(0, 6, "IS TOO LOW      ");
          } else {
            u8x8.setFont(u8x8_font_chroma48medium8_r);
            u8x8.drawString(0, 5, "                ");
            u8x8.drawString(0, 6, "                ");
          }
        }
        nextTimeDisplay = false;
      }
    break;
  }
}
