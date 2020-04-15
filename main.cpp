/*
      Copyright 2015-2018 Dirk-Willem van Gulik <dirkx@webweaving.org>
      Copyright 2020      Hans Beerman <hans.beerman@xs4all.nl>
                          Stichting Makerspace Leiden, the Netherlands.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#ifndef ESP32
#error "The CompressorNode uses an Olimex ESP32-PoE board only!"
#endif

#define ESP32_PoE
// #define OB_POLLING

// for calibrating the pressure sensor remove the comment statement in front of next #define
// #define CALIBRATE_PRESSURE

// The GPIO pins used on the Olimax ESP32PoE
#define RELAY_GPIO      (14)  // digital output
#define ON_BUTTON       (15)  // digital input
#define OFF_BUTTON      ( 5)  // digital input
#define OPTO1           (36)  // digital input
#define LED1            (32)  // digital output
#define LED2            (33)  // digital output
#define OILLEVELSENSOR  (39)  // digital input
#define PRESSURESENSOR  (35)  // analog input
#define TEMPSENSOR      ( 4)  // one wire digital input

// for I2C display
#define I2C_SDA         (13)  // I2C SDA
#define I2C_SCL         (16)  // I2C SCL

#include <Arduino.h>
#include <ACNode.h>
#include <WiredEthernet.h>
#include <SIG2.h>
#include <Cache.h>
#include <OptoDebounce.h>
#include <ButtonDebounce.h>
#include <OneWire.h> 
#include <DallasTemperature.h> // install DallasTemperature by Miles Burton
#include <U8x8lib.h> // install U8g2 library by oliver
#include <WiFiUdp.h>
#include <NTP.h> // install NTP by Stefan Staub
//
// information about NTP.h, see: https://platformio.org/lib/show/5438/NTP
//
WiFiUDP wifiUDP;
NTP ntp(wifiUDP);

// One Wire input port for temperature sensor
#define ONE_WIRE_BUS (TEMPSENSOR)

#define OTA_PASSWD "MyPassWoord"

// for test
#define MACHINE "test-compressor1"
// define MACHINE "compressor"

// button on and button off
#define BUTTON_ON_PRESSED (LOW) // the input level of the GPIO port used for button on, if button on is pressed
#define BUTTON_OFF_PRESSED (LOW) // the input level of te GPIO port used for button off. if button off is pressed

ButtonDebounce buttonOn(ON_BUTTON, 150 /* mSeconds */); // buttonOn is used to switch on the compressor
ButtonDebounce buttonOff(OFF_BUTTON, 150 /* mSeconds */); // buttonOff is used to switch off the compressor

// 230VAC optocoupler
OptoDebounce opto1(OPTO1); // wired to N0 - L1 of 3 phase compressor motor, to detect if the motor has power (or not)

// oil level sensor
#define TO_LOW_OIL_LEVEL (LOW) // the input level of the GPIO port used for the oil level sensor signalling too low oil level
#define MAX_OIL_LEVEL_IS_TOO_LOW_WINDOW (10000) // in ms default 10000 = 10 seconds. Error is signalled after this time window is passed
#define OIL_LEVEL_LOG_WINDOW (60000) // in ms

ButtonDebounce oilLevel(OILLEVELSENSOR, 300 /* mSeconds */); // to signal if the oil level is too low (or not)

// temperature sensor
OneWire oneWire(ONE_WIRE_BUS); // used for the temperature sensor
DallasTemperature sensorTemp(&oneWire);

#define TEMP_RESOLUTION (12) // 9, 10, 11 or 12 bit resolution of the ADC in the temperature sensor
#define MAX_TEMP_CONVERSIONTIME (750) // in ms
#define TEMP_IS_HIGH_LEVEL (40.0) // in degrees Celcius, used for temperature is high warning
#define TEMP_IS_TOO_HIGH_LEVEL (70.0) // in degrees Celcius, used to disable the compressor when temperature is too high
#define MAX_TEMP_IS_TOO_HIGH_WINDOW (10000) // in ms default 10000 = 10 seconds. Error is only signalled after this time window is passed

// For LED's showing node error
#define BLINKING_LED_PERIOD (600) // in ms

// pressure sensor
#define PRESSURE_SAMPLE_WINDOW (1000) // in ms
#define PRESSURE_LOG_WINDOW (60000) // in ms
#define PRESSURE_CALIBRATE_VALUE_0_5V (144) // in measured bits
#define PRESSURE_CALIBRATE_VALUE_4_5V (2600) // in measured bits

// oled display
#define DISPLAY_WINDOW (1000) // in ms, update display time

#define KEEP_STATUS_LINE_TIME (5000) // in ms, default = 5 s (5000), the time certain status messages are shown on the bottom line of the display

// for storage in EEProm of the duration counters
#define SAVE_DURATION_COUNTERS_WINDOW (86400) // in seconds (86400 = 24 hour)

#define DURATION_DIR_PREFIX "/init"
#define DURATION_FILE_PREFIX "/duration"

// for auto switch off of the compressor
#define AUTOTIMEOUT (30 * 60 * 1000) // default: in ms 30 * 60 * 1000 = 30 minutes

// Compressor disabled (or not) at late hours:
// IF the compressor is not allowed at late hours (DISABLE_COMPRESSOR AT LATE HOURS = true) the 
// compressor will not switch on automatically (or by hand if the on button is pressed normally) from
// DISABLED_TIME_START to DISABLED_START_END
// Pressing the on button longer than MAX_WAIT_TIME_BUTTON_ON_PRESSED will override this behaviour by
// switching on the compressor anyhow. In all cases the compressor will switch of after AUTOTIMEOUT (in ms)
#define DISABLE_COMPRESSOR_AT_LATE_HOURS (true)
#define DISABLED_TIME_START (19) // in hour, time from which the compressor is not automatically switched on
#define DISABLED_TIME_END (8) // in hour, time to which the compressor is not automatically switched on
#define MAX_WAIT_TIME_BUTTON_ON_PRESSED       (10000)  // in ms, time button on must be pressed to override late hour compressor disable
#define LED_DISABLE_DURATION                  (5000)  // in ms, the time LED1 will flash if button on is pressed during late hour
#define LED_DISABLE_PERIOD                    (200)  // in ms, the time LED1 will flash on/off

// Clear EEProm button
// Press BUT1 on Olimex ESP32 PoE module before (re)boot of node
// keep BUT1 pressed for at least 5 s
// After the release of BUT1 node will restart with empty EEProm
#define CLEAR_EEPROM_AND_CACHE_BUTTON         (34)
#define CLEAR_EEPROM_AND_CACHE_BUTTON_PRESSED (LOW)

#define MAX_WAIT_TIME_BUTTON_PRESSED          (4000)  // in ms

// for testing with WiFi
// ACNode node = ACNode(MACHINE, WIFI_NETWORK, WIFI_PASSWD);
ACNode node = ACNode(MACHINE);

MqttLogStream mqttlogStream = MqttLogStream();
TelnetSerialStream telnetSerialStream = TelnetSerialStream();

#ifdef OTA_PASSWD
OTA ota = OTA(OTA_PASSWD);
#endif

typedef enum {
  BOOTING, OUTOFORDER,      // device not functional.
  REBOOT,                   // forcefull reboot
  TRANSIENTERROR,           // hopefully goes away level error
  NOCONN,                   // sort of fairly hopless (though we can cache RFIDs!)
  SWITCHEDOFF,              // connected compressor switched off completely
  POWERED,                  // unit is powered on
  RUNNING,                  // unit is running (opto sees light).
} machinestates_t;

#define NEVER (0)

struct {
  const char * label;                   // name of this state
  LED::led_state_t ledState;            // flashing pattern for the aartLED. Zie ook https://wiki.makerspaceleiden.nl/mediawiki/index.php/Powernode_1.1.
  time_t maxTimeInMilliSeconds;         // how long we can stay in this state before we timeout.
  machinestates_t failStateOnTimeout;   // what state we transition to on timeout.
} state[RUNNING + 1] =
{
  { "Booting",              LED::LED_ERROR,           120 * 1000, REBOOT },
  { "Out of order",         LED::LED_ERROR,           120 * 1000, REBOOT },
  { "Rebooting",            LED::LED_ERROR,           120 * 1000, REBOOT },
  { "Transient Error",      LED::LED_ERROR,           120 * 1000, REBOOT },
  { "No network",           LED::LED_FLASH,           120 * 1000, REBOOT },
  { "Compressor switched off", LED::LED_IDLE,         NEVER, SWITCHEDOFF},
  { "Powered - motor off",  LED::LED_IDLE,            NEVER, POWERED },
  { "Powered - motor running", LED::LED_ON,           NEVER, RUNNING },
};

unsigned long laststatechange = 0;
static machinestates_t laststate = BOOTING;
machinestates_t machinestate = BOOTING;
machinestates_t laststateDisplayed = BOOTING;

unsigned long powered_total = 0, powered_last;
unsigned long running_total = 0, running_last;
float powered = 0.0;
float lastPoweredDisplayed = 0.0;
float running = 0.0;
float lastRunningDisplayed = 0.0;

DeviceAddress tempDeviceAddress;
float previousTemperature = -500;
float currentTemperature = -500;
float temperature = -500;
float lastTempDisplayed = -500;
unsigned long conversionTime = MAX_TEMP_CONVERSIONTIME / (1 << (12 - TEMP_RESOLUTION));
unsigned long tempAvailableTime = 0;
unsigned long tempIsTooHighStart = 0;
bool previousTempIsHigh = false;
bool tempIsHigh = false;
bool previousErrorTempIsTooHigh = false;
bool ErrorTempIsTooHigh = false;

bool previousOilLevelIsTooLow = false;
bool oilLevelIsTooLow = false;
bool lastOilLevelDisplayed = false;
unsigned long oilLevelNextLoggingTime = 0;
unsigned long oilLevelIsTooLowStart = 0;
bool previousErrorOilLevelIsTooLow = false;
bool ErrorOilLevelIsTooLow = false;

int pressureADCVal = 0;
float pressureVoltage = 0;
float pressure = 0;
float lastPressureDisplayed = -1;
unsigned long pressureNextSampleTime = 0;
unsigned long pressureNextLogTime = 0;

unsigned long blinkingLedNextTime = 0;
bool blinkingLedIsOn = false;
bool ledIsBlinking = false;

typedef enum {
  NORMALDISPLAY,        // Normal display shown when there is no error, showing current compressor state etc.
  ERRORDISPLAY
} displaystates_t;

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

struct {
  const char * statusmessage;
  int y;
  bool temporarily;
} dispstatus[ERRORHIGHTEMP + 1] = 
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
	{ "Timeout ==> off ", 15, true },
	{ "OilLevel too low", 10, false },
	{ "OilLevel OK!    ", 10, false },
	{ "Warning         ", 15, false },
	{ "ERROR           ", 15, false },
};

bool showStatusTemporarily = false;
unsigned long clearStatusLineTime = 0;


displaystates_t currentDisplayState = NORMALDISPLAY;
unsigned long updateDisplayTime = 0;
bool firstTimeDisplayed = true;

char reportStr[128];

unsigned long DurationCounterSave;

unsigned long buttonOnPressedTime;
bool buttonOnIsPressed = false;

unsigned long ledDisableTime = 0;
unsigned long nextLedDisableTime = 0;
bool showLedDisable = false;

unsigned long autoPowerOff;
bool compressorIsOn = false;

// for 1.5 inch OLED Display 128*128 pixels wit - I2C
U8X8_SSD1327_WS_128X128_SW_I2C u8x8(I2C_SCL, I2C_SDA,U8X8_PIN_NONE);

void showStatusOnDisplay(statusdisplay_t statusMessage) {
  char outputStr[20];

  switch (statusMessage) {
    case NOSTATUS:
    case ERRORLOWOILLEVEL:
    case NOLOWOILLEVEL:
      u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
      u8x8.drawString(0, dispstatus[statusMessage].y, dispstatus[statusMessage].statusmessage);
      break;
    case WARNINGHIGHTEMP:
      u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
      sprintf(outputStr, "WARNING >%4.0f %cC", TEMP_IS_HIGH_LEVEL, 176);
      u8x8.drawString(0, dispstatus[statusMessage].y, outputStr);
      break;
    case ERRORHIGHTEMP:
      u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
      sprintf(outputStr, "ERROR   >%4.0f %cC", TEMP_IS_TOO_HIGH_LEVEL, 176);
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

void checkClearEEPromAndCacheButtonPressed(void) {
  unsigned long ButtonPressedTime;
  unsigned long currentSecs;
  unsigned long prevSecs;
  bool firstTime = true;

  // check CLEAR_EEPROM_AND_CACHE_BUTTON pressed
  pinMode(CLEAR_EEPROM_AND_CACHE_BUTTON, INPUT);
  // check if button is pressed for at least 4 s
  Log.println("Checking if the button is pressed for clearing EEProm and cache");

  if (digitalRead(CLEAR_EEPROM_AND_CACHE_BUTTON) == CLEAR_EEPROM_AND_CACHE_BUTTON_PRESSED) {
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.drawString(0, 9, "Keep Olimex BUT2");
    u8x8.drawString(0, 10, "pressed for at  ");
    u8x8.drawString(0, 11, "least 4 seconds ");
    u8x8.drawString(0, 12, "to clear EEProm ");
    u8x8.drawString(0, 13, "and cache memory");

    ButtonPressedTime = millis();  
    prevSecs = MAX_WAIT_TIME_BUTTON_PRESSED / 1000;
    Log.print(prevSecs);
    Log.print(" s");
    while (digitalRead(CLEAR_EEPROM_AND_CACHE_BUTTON) == CLEAR_EEPROM_AND_CACHE_BUTTON_PRESSED) {
      if (millis() >= ButtonPressedTime + MAX_WAIT_TIME_BUTTON_PRESSED) {
        if (firstTime) {
          Log.print("\rPlease release button");
          showStatusOnDisplay(RELEASEBUTTON);
          firstTime = false;
        }
      } else {
        currentSecs = (MAX_WAIT_TIME_BUTTON_PRESSED - millis()) / 1000;
        if ((currentSecs != prevSecs) && (currentSecs >= 0)) {
          Log.print("\r");
          Log.print(currentSecs);
          Log.print(" s");
          prevSecs = currentSecs;
        }
      }
    }

    if (millis() >= (ButtonPressedTime + MAX_WAIT_TIME_BUTTON_PRESSED)) {
      Log.print("\rButton for clearing EEProm and cache was pressed for more than ");
      Log.print(MAX_WAIT_TIME_BUTTON_PRESSED / 1000);
      Log.println(" s, EEProm and Cache will be cleared!");
      u8x8.setFont(u8x8_font_chroma48medium8_r);
      u8x8.drawString(0, 9, "EEProm and cache");
      u8x8.drawString(0, 10, "will be cleared ");
      u8x8.drawString(0, 11, "                ");
      u8x8.drawString(0, 12, "                ");
      u8x8.drawString(0, 13, "                ");
      showStatusOnDisplay(NOSTATUS);
      // Clear EEPROM
      wipe_eeprom();
      Log.println("EEProm cleared!");
      u8x8.drawString(0, 12, "EEProm cleared  ");
      // Clear cache
      prepareCache(true);
      // Clear duration counter file
      String path = DURATION_DIR_PREFIX + (String)DURATION_FILE_PREFIX;
      if (SPIFFS.exists(path)) {
        SPIFFS.remove(path);
      } 
      u8x8.drawString(0, 13, "Cache cleared   ");
      Log.println("Cache cleared!");
      // wait until button is released, than reboot
      while (digitalRead(CLEAR_EEPROM_AND_CACHE_BUTTON) == CLEAR_EEPROM_AND_CACHE_BUTTON_PRESSED) {
        // do nothing here
      }
      Log.println("Node will be restarted");
      showStatusOnDisplay(NODEREBOOT);
      // restart node
      ESP.restart();
    } else {
      Log.println("\rButton was not (or not long enough) pressed to clear EEProm and cache");
    }
  }
}

void saveDurationCounters() {
  unsigned long tmpCounter;

  String path = DURATION_DIR_PREFIX + (String)DURATION_FILE_PREFIX;
  File durationFile;
  unsigned int writeSize;
  durationFile = SPIFFS.open(path, "wb");
  if(!durationFile) {
    Log.print("There was an error opening the ");
    Log.print(DURATION_DIR_PREFIX);
    Log.print(DURATION_FILE_PREFIX);
    Log.println(" file for writing");
    return;
  }
  if (machinestate >= POWERED) {
    tmpCounter = powered_total + (millis() - powered_last) / 1000;
  } else {
    tmpCounter = powered_total;
  }
  writeSize = durationFile.write((byte*)&tmpCounter, sizeof(tmpCounter));
  if (writeSize != sizeof(tmpCounter)) {
    Log.print("ERROR --> powered_total: ");
    Log.print(tmpCounter);
    Log.println(" NOT stored in SPIFFS");
    durationFile.close();
    return;
  }

  if (machinestate == RUNNING) {
    tmpCounter = running_total + (millis() - running_last) / 1000;
  } else {
    tmpCounter = running_total;
  }
  writeSize = durationFile.write((byte*)&tmpCounter, sizeof(tmpCounter));
  if (writeSize != sizeof(tmpCounter)) {
    Log.print("ERROR --> running_total: ");
    Log.print(tmpCounter);
    Log.println(" NOT stored in SPIFFS");
    durationFile.close();
    return;
  }

  durationFile.close();
}

void loadDurationCounters() {
  String path = DURATION_DIR_PREFIX + (String)DURATION_FILE_PREFIX;
  File durationFile;
  unsigned int readSize;

  if(!SPIFFS.begin(false)){
    Log.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  if (SPIFFS.exists(path)) {
    durationFile = SPIFFS.open(path, "rb");
    if(!durationFile) {
      Log.print("There was an error opening the ");
      Log.print(DURATION_DIR_PREFIX);
      Log.print(DURATION_FILE_PREFIX);
      Log.println(" file for reading");
      return;
    }
    durationFile.setTimeout(0);
    readSize = durationFile.readBytes((char*)&powered_total, sizeof(powered_total));
    if (readSize == sizeof(powered_total)) {
     powered = (float)powered_total / 3600.0;
    } else {
      Log.print("There was an error reading powered_total from ");
      Log.print(DURATION_DIR_PREFIX);
      Log.println(DURATION_FILE_PREFIX);
      durationFile.close();
      return;
    }
    readSize = durationFile.readBytes((char*)&running_total, sizeof(running_total));
    if (readSize == sizeof(running_total)) {
      running = (float)running_total / 3600.0;
    } else {
      Log.print("There was an error reading powered_total from ");
      Log.print(DURATION_DIR_PREFIX);
      Log.println(DURATION_FILE_PREFIX);
      durationFile.close();
      return;
    }
 
    durationFile.close();
    return;
  } else {
    Log.print("The file ");
    Log.print(DURATION_DIR_PREFIX);
    Log.print(DURATION_FILE_PREFIX);
    Log.println(" is not available!");
  }
}

bool compressorIsDisabeled() {
  int currentHour;

  currentHour = ntp.hours();
/* 
 * for test of NTP, check if time shown is correct 
  Serial.print("Current hour: ");
  Serial.println(currentHour);
*/  
  if (ErrorOilLevelIsTooLow || ErrorTempIsTooHigh || 
      (((currentHour >= DISABLED_TIME_START) || (currentHour <= DISABLED_TIME_END)) && DISABLE_COMPRESSOR_AT_LATE_HOURS)) {
    return true;
  } else {
    return false;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n\n");
  Serial.println("Booted: " __FILE__ " " __DATE__ " " __TIME__ );

  // Init the hardware and get it into a safe state.
  //
  pinMode(RELAY_GPIO, OUTPUT);
  digitalWrite(RELAY_GPIO, 0);

  pinMode(ON_BUTTON, INPUT_PULLUP);
  pinMode(OFF_BUTTON, INPUT_PULLUP);

  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, 0);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED2, 0);

  pinMode(OILLEVELSENSOR, INPUT_PULLUP);

  // for 1.5 inch OLED Display 128*128 pixels wit - I2C
  pinMode(I2C_SDA, OUTPUT);
  pinMode(I2C_SCL, OUTPUT);
  digitalWrite(I2C_SDA, 0);
  digitalWrite(I2C_SCL, 0);
  
  u8x8.begin();

  u8x8.setFont(u8x8_font_px437wyse700a_2x2_r);
  u8x8.drawString(0, 0, "CompNode");

  u8x8.setFont(u8x8_font_px437wyse700b_2x2_r);
  u8x8.drawString(0, 2, "  V0.1  ");

  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.drawString(0, 4, " c Hans Beerman ");
  u8x8.drawString(0, 6, "Booting, please ");
  u8x8.drawString(0, 7, "      wait      ");

  Serial.printf("Boot state: ButtonOn:%d ButtonOff:%d\n", digitalRead(ON_BUTTON), digitalRead(OFF_BUTTON));

  checkClearEEPromAndCacheButtonPressed();

  loadDurationCounters();
  DurationCounterSave = millis() / 1000 + SAVE_DURATION_COUNTERS_WINDOW;

  node.set_mqtt_prefix("ac");
  node.set_master("master");

  // the default is space.makerspaceleiden.nl, prefix test
  // node.set_mqtt_host("mymqtt-server.athome.nl");
  // node.set_mqtt_prefix("test-1234");

  // specify this when using your own `master'.
  //
  // node.set_master("test-master");

  // node.set_report_period(2 * 1000);

  // init temperature sensor and start reading first value
  sensorTemp.begin();
  sensorTemp.getAddress(tempDeviceAddress, 0);
  sensorTemp.setResolution(tempDeviceAddress, TEMP_RESOLUTION);
  sensorTemp.setWaitForConversion(false);
  sensorTemp.requestTemperaturesByAddress(tempDeviceAddress);
  tempAvailableTime = millis() + conversionTime;


  buttonOn.setCallback([](int state) {
    Debug.printf("Button On changed to %d\n", state);
  });
  buttonOff.setCallback([](int state) {
    Debug.printf("Button Off changed to %d\n", state);
  });

  oilLevel.setCallback([](int state) {
    Debug.printf("OilLevel sensor changed to %d\n", state);
  });


  node.onConnect([]() {
    machinestate = SWITCHEDOFF;
  });
  node.onDisconnect([]() {
    machinestate = NOCONN;
  });
  node.onError([](acnode_error_t err) {
    Log.printf("Error %d\n", err);
    machinestate = TRANSIENTERROR;
  });

  node.onValidatedCmd([](const char *cmd, const char * rest) -> ACBase::cmd_result_t  {
    if (!strcasecmp(cmd, "stop")) {
      Log.println("Request received to stop the compressor");
      machinestate = SWITCHEDOFF;
      digitalWrite(RELAY_GPIO, 0);
      digitalWrite(LED1, 0);
      digitalWrite(LED2, 0);
      compressorIsOn = false;
      Log.println("Compressor stopped");
      showStatusOnDisplay(AUTOSWITCHOFF);
      return ACNode::CMD_CLAIMED;
    };

    if (!strcasecmp(cmd, "poweron")) {
      Log.println("Request received to power on the compressor");
      if (!compressorIsDisabeled()) {
        if (machinestate < POWERED) {
          digitalWrite(RELAY_GPIO, 1);
          digitalWrite(LED1, 1);
          digitalWrite(LED2, 0);
          compressorIsOn = true;
          machinestate = POWERED;
          Log.println("Compressor powered on");
          showStatusOnDisplay(AUTOSWITCHON);
        };
        autoPowerOff = millis() + AUTOTIMEOUT;
      } else {
        Log.println("Request denied to power on the compressor. Reason: late hours/night!");
        showStatusOnDisplay(AUTOONDENIED);
      }
      return ACBase::CMD_CLAIMED;
    };
    return ACBase::CMD_DECLINE;
  });

  node.onReport([](JsonObject  & report) {
    report["state"] = state[machinestate].label;

    report["powered_time"] = powered_total + ((machinestate == POWERED) ? ((millis() - powered_last) / 1000) : 0);
    report["running_time"] = running_total + ((machinestate == RUNNING) ? ((millis() - running_last) / 1000) : 0);

    if (temperature == -127) {
      sprintf(reportStr, "Error reading temperature sensor, perhaps not connected?");
    } else {
      if (ErrorTempIsTooHigh) {
        sprintf(reportStr, "ERROR: Temperature is too high, compressor is disabled!");
        report["temperature_sensor_error"] = reportStr;
      } else {
        if (tempIsHigh) {
          sprintf(reportStr, "WARNING: Temperature is very high!");
          report["temperature_sensor_warning"] = reportStr;
        }
      }
      sprintf(reportStr, "%f degrees Celcius", temperature);
    }
    report["temperature_sensor"] = reportStr;

    if (!oilLevelIsTooLow)
    {
      report["oil_level_sensor"] = "oil level is OK!";
    } else {
      if (ErrorOilLevelIsTooLow) {
        report["oil)level_sensor_error"] = "ERROR: Oil level is too low, compressor is disabled";
      } else {
        report["oil_level_sensor_warning"] = "WARNING: Oil level is too low!";
      }
    }
    sprintf(reportStr, "%5.3f MPa", pressure);
    report["pressure_sensor"] = reportStr;

#ifdef OTA_PASSWD
    report["ota"] = true;
#else
    report["ota"] = false;
#endif
    report["opto1"] = opto1.state();
  });

  Log.addPrintStream(std::make_shared<MqttLogStream>(mqttlogStream));

  auto t = std::make_shared<TelnetSerialStream>(telnetSerialStream);
  Log.addPrintStream(t);
  Debug.addPrintStream(t);

#ifdef OTA_PASSWD
  node.addHandler(&ota);
#endif

  // node.set_debug(true);
  // node.set_debugAlive(true);
  Log.println("Booted: " __FILE__ " " __DATE__ " " __TIME__ );

  // Olimex ESP32-PoE board is used
  node.begin(BOARD_OLIMEX);
  u8x8.clearDisplay();

  ntp.ruleDST("CEST", Last, Sun, Mar, 2, 120); // last sunday in march 2:00, timetone +120min (+1 GMT + 1h summertime offset)
  ntp.ruleSTD("CET", Last, Sun, Oct, 3, 60); // last sunday in october 3:00, timezone +60min (+1 GMT)
  ntp.begin();
  ntp.update();
}

void buttons_optocoupler_loop() {

  opto1.loop();

  buttonOn.update();
  buttonOff.update();

  if (opto1.state()) {
    if (machinestate == POWERED) {
      machinestate = RUNNING;
      digitalWrite(LED2, 1);
    }
  } else {
    if (machinestate > POWERED) {
      machinestate = POWERED;
      digitalWrite(LED2, 0);
    }
  }

  if ((buttonOn.state() == BUTTON_ON_PRESSED && machinestate == SWITCHEDOFF) && (buttonOff.state() != BUTTON_ON_PRESSED)) {
    if (ErrorOilLevelIsTooLow || ErrorTempIsTooHigh) {
      // Compressor must remain disabled, until these errors, which can demage the compressor, are solved
      return;
    }
    if (!compressorIsDisabeled()) {
      Log.printf("Compressor switched on with button\n");
      digitalWrite(RELAY_GPIO, 1);
      digitalWrite(LED1, 1);
      digitalWrite(LED2, 0);
      compressorIsOn = true;
      machinestate = POWERED;
      autoPowerOff = millis() + AUTOTIMEOUT;
      showStatusOnDisplay(MANUALSWITCHON);
    } else {
      if (!buttonOnIsPressed) {
        buttonOnPressedTime = millis();
        showStatusOnDisplay(POWERONDISABLED);
        Log.println("Power on denied!");
        Log.println("Power on is disabled during evening/night window");
        // flash LED to show that function is disabled
        ledDisableTime = millis() + LED_DISABLE_DURATION;
        nextLedDisableTime = millis() + LED_DISABLE_PERIOD;
        showLedDisable = true;
        digitalWrite(LED1, 1);
      }
      buttonOnIsPressed = true;
    }
  }

  if (buttonOnIsPressed) {
    if ((digitalRead(ON_BUTTON) != BUTTON_ON_PRESSED) && (millis() < (buttonOnPressedTime + MAX_WAIT_TIME_BUTTON_ON_PRESSED))) {
      buttonOnIsPressed = false;
    } else {
      if ((digitalRead(ON_BUTTON) == BUTTON_ON_PRESSED) && (millis() >= (buttonOnPressedTime + MAX_WAIT_TIME_BUTTON_ON_PRESSED))) {
        buttonOnIsPressed = false;
        digitalWrite(RELAY_GPIO, 1);
        digitalWrite(LED1, 1);
        digitalWrite(LED2, 0);
        compressorIsOn = true;
        machinestate = POWERED;
        autoPowerOff = millis() + AUTOTIMEOUT; 
        showStatusOnDisplay(MANUALOVERRIDE);
        Log.println("Warning: compressor was switched on using manual override!");
      }
    }
  }

  if ((buttonOff.state() == BUTTON_OFF_PRESSED && machinestate >= POWERED) && (buttonOn.state() != BUTTON_ON_PRESSED)) {
    Log.printf("Compressor switched off with button\n");
    digitalWrite(RELAY_GPIO, 0);
    digitalWrite(LED1, 0);
    digitalWrite(LED2, 0);
    compressorIsOn = false;
    machinestate = SWITCHEDOFF;
    showStatusOnDisplay(MANUALSWITCHOFF);
  };
  if (showLedDisable) {
    if (millis() > ledDisableTime) {
      showLedDisable = false;
      digitalWrite(LED1, 0);
    } else {
      if (millis() > nextLedDisableTime) {
        nextLedDisableTime = millis() + LED_DISABLE_PERIOD;
        digitalWrite(LED1, !digitalRead(LED1));
      }
    }
  }
}

void temp_sensor_loop() {
  if (millis() > tempAvailableTime) {
    currentTemperature = sensorTemp.getTempC(tempDeviceAddress);

    if (currentTemperature == -127) {
      Log.println("Temperature sensor does not react, perhaps not available?");
    } else {
      if (currentTemperature != previousTemperature) {
        previousTemperature = currentTemperature;
        temperature = currentTemperature;
        Serial.print("Temp. changed, current temp. = ");
        Serial.print(temperature);
        Serial.println(" degrees C");
      }
    }
    sensorTemp.requestTemperaturesByAddress(tempDeviceAddress);
    tempAvailableTime = millis() + conversionTime;
    if (temperature <= TEMP_IS_HIGH_LEVEL) {
      if (tempIsHigh) {
        Log.println("Temperature is OK now (below warning threshold)");
      }
      tempIsHigh = false;
      if (ErrorTempIsTooHigh)
      {
        firstTimeDisplayed = true;
      }
      ErrorTempIsTooHigh = false;
      tempIsTooHighStart = 0;
    } else {
      if (!tempIsHigh) {
        Log.println("WARNING: temperature is above warning level. Please check compressor");
      }
      tempIsHigh = true;
      if ((temperature > TEMP_IS_TOO_HIGH_LEVEL) && !ErrorTempIsTooHigh) {
        if (tempIsTooHighStart == 0) {
          tempIsTooHighStart = millis();
        } else {
          if (millis() > (tempIsTooHighStart + MAX_TEMP_IS_TOO_HIGH_WINDOW)) {
            firstTimeDisplayed = true;
            ErrorTempIsTooHigh = true;
            if (!ERRORLOWOILLEVEL) {
              Log.println("ERROR: Temperature is too high, compressor will be disabled. Please check compressor!");
            } else {
              Log.println("ERROR: Temperature is too high, please check compressor!");
            }
          }
        }
      } else {
        if ((temperature <= TEMP_IS_TOO_HIGH_LEVEL) && ErrorTempIsTooHigh) {
          tempIsTooHighStart = 0;
          ErrorTempIsTooHigh = false;
          firstTimeDisplayed = true;
          Log.println("WARNING: Temperature is below error level now, but still above warning level. Please check compressor!");
        }
      }
    }
  }
}

void pressure_sensor_loop() {
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

void display_loop() {
  char outputStr[20];

  if (!ErrorOilLevelIsTooLow && !ErrorTempIsTooHigh) {
    if (currentDisplayState == ERRORDISPLAY) {
      firstTimeDisplayed = true;
      previousTempIsHigh = !tempIsHigh;
      u8x8.clearDisplay();
      currentDisplayState = NORMALDISPLAY;
    }
  } else {
    if (currentDisplayState == NORMALDISPLAY) {
      firstTimeDisplayed = true;
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

        if ((pressure != lastPressureDisplayed) || firstTimeDisplayed) {
          lastPressureDisplayed = pressure;
          u8x8.setFont(u8x8_font_px437wyse700b_2x2_f);
          sprintf(outputStr, "%4.2f MPa", pressure); 
          u8x8.drawString(0, 0, "Pressure");
          u8x8.drawString(0, 2, outputStr);
        }
        if ((temperature != lastTempDisplayed) || firstTimeDisplayed) {
          lastTempDisplayed = temperature;
          u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
          if (temperature < -100) {
            sprintf(outputStr, "Temp.: N.A.     ");
          } else {
            sprintf(outputStr, "Temp.:%7.2f %cC", temperature, 176);
          }
          u8x8.drawString(0, 5, outputStr);
          if ((previousTempIsHigh != tempIsHigh) || (previousErrorTempIsTooHigh != ErrorTempIsTooHigh)) {
            if (ErrorTempIsTooHigh) {
              showStatusOnDisplay(ERRORHIGHTEMP);
            } else {
              if (tempIsHigh) {
                showStatusOnDisplay(WARNINGHIGHTEMP);
              } else {
                showStatusOnDisplay(NOSTATUS);
              }
            }
            previousErrorTempIsTooHigh = ErrorTempIsTooHigh;  
            previousTempIsHigh = tempIsHigh;
          }
        }
        if ((machinestate != laststateDisplayed)  || firstTimeDisplayed) {
          laststateDisplayed = machinestate;
          u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
          u8x8.drawString(0, 7, "Machine state:  ");
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
          u8x8.drawString(0, 8, outputStr);
        }
        if ((oilLevelIsTooLow != lastOilLevelDisplayed)  || firstTimeDisplayed) {
          lastOilLevelDisplayed = oilLevelIsTooLow;
          if (oilLevelIsTooLow) {
            showStatusOnDisplay(ERRORLOWOILLEVEL);
          } else {
            showStatusOnDisplay(NOLOWOILLEVEL);
          }
        }

        if ((machinestate == POWERED) || firstTimeDisplayed) {
          if (machinestate < POWERED) {
            powered = (float)powered_total / 3600.0;
          } else {
            powered = ((float)powered_total + ((float)millis() - float(powered_last)) / 1000.0) / 3600.0;
          }

          if ((powered != lastPoweredDisplayed) || firstTimeDisplayed) {
            lastPoweredDisplayed = powered;
            u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
            sprintf(outputStr, "On: %9.2f hr", powered);
            u8x8.drawString(0, 12, outputStr);
          }
        }

        if ((machinestate == RUNNING) || firstTimeDisplayed) {
          if (machinestate < RUNNING) {
            running = (float)running_total / 3600.0;
          } else {
            running = ((float)running_total + ((float)millis() - (float)running_last) / 1000.0) / 3600.0;
          }

          if ((running != lastRunningDisplayed) || firstTimeDisplayed) {
            lastRunningDisplayed = running;
            u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
            sprintf(outputStr, "Run:%9.2f hr", running);
            u8x8.drawString(0, 13, outputStr);
          }
        }
        firstTimeDisplayed = false;
      }

      if (showStatusTemporarily && (millis() > clearStatusLineTime)) {
        if (tempIsHigh) {
          showStatusOnDisplay(WARNINGHIGHTEMP);
        } else {
          if (ErrorTempIsTooHigh) {
            showStatusOnDisplay(ERRORHIGHTEMP);
          } else {
            showStatusOnDisplay(NOSTATUS);
          }
        }
        showStatusTemporarily = false;
      }

    break;
    case ERRORDISPLAY:
      if (firstTimeDisplayed) {
        u8x8.setFont(u8x8_font_px437wyse700a_2x2_r);
        u8x8.drawString(0, 0, "MAINTAIN");
        u8x8.drawString(0, 2, "COMPRSR.");
        if (ErrorOilLevelIsTooLow) {
          u8x8.setFont(u8x8_font_chroma48medium8_r);
          u8x8.drawString(0, 5, "OIL LEVEL       ");
          u8x8.drawString(0, 6, "TOO LOW         ");
        } else {
          u8x8.setFont(u8x8_font_chroma48medium8_r);
          u8x8.drawString(0, 5, "                ");
          u8x8.drawString(0, 6, "                ");
        }                     
        if (ErrorTempIsTooHigh) {
          u8x8.setFont(u8x8_font_chroma48medium8_r);
          u8x8.drawString(0, 8, "TEMPERATURE     ");
          u8x8.drawString(0, 9, "TOO HIGH        ");
        } else {
          u8x8.setFont(u8x8_font_chroma48medium8_r);
          u8x8.drawString(0, 8, "                ");
          u8x8.drawString(0, 9, "                ");
        }
        u8x8.setFont(u8x8_font_px437wyse700a_2x2_r);
        u8x8.drawString(0, 12, "COMPRSR.");  
        u8x8.drawString(0, 14, "DISABLED");
      }
      if (ErrorTempIsTooHigh) {
        if ((temperature != lastTempDisplayed) || firstTimeDisplayed) {
          lastTempDisplayed = temperature;
          u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
          sprintf(outputStr, "Temp.:%7.2f %cC", temperature, 176);
          u8x8.drawString(0, 10, outputStr);
        }
      } else {
        u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
        u8x8.drawString(0, 10, "                ");
      }

      firstTimeDisplayed = false;
    break;
  }
}

void oil_level_sensor_loop() {
  if (oilLevel.state() == TO_LOW_OIL_LEVEL) {
    oilLevelIsTooLow = true;

    if (oilLevelIsTooLowStart == 0) {
      oilLevelIsTooLowStart = millis();
    } else {
      if ((millis() - oilLevelIsTooLowStart) >= MAX_OIL_LEVEL_IS_TOO_LOW_WINDOW) {
        ErrorOilLevelIsTooLow = true;
        if ((ErrorOilLevelIsTooLow != previousErrorOilLevelIsTooLow) || (millis() > oilLevelNextLoggingTime)) {
          if (ErrorOilLevelIsTooLow != previousErrorOilLevelIsTooLow) {
            firstTimeDisplayed = true;
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
       firstTimeDisplayed = true;
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

void compressorLoop() {
 
  ntp.update();

  if (machinestate > SWITCHEDOFF) {
    // check if compressor must be switched off
    if (ErrorOilLevelIsTooLow || ErrorTempIsTooHigh || (millis() > autoPowerOff)) {
      digitalWrite(RELAY_GPIO, 0);
      digitalWrite(LED1, 0);
      digitalWrite(LED2, 0);
      compressorIsOn = false;
      machinestate = SWITCHEDOFF;
      if (ErrorOilLevelIsTooLow || ErrorTempIsTooHigh) {
        Log.println("Compressor is disabled now due to error(s). Please check compressor!");
      }
      if (millis() > autoPowerOff) {
        Log.println("Timeout: compressor automatically switched off");
        showStatusOnDisplay(TIMEOUT);
      }
    }
  } else {
    if (ErrorOilLevelIsTooLow || ErrorTempIsTooHigh) {
      ledIsBlinking = true;
      if (millis() > blinkingLedNextTime) {
        if (blinkingLedIsOn) {
          digitalWrite(LED1, 0);
          digitalWrite(LED2, 0);
        } else {
          digitalWrite(LED1, 1);
          digitalWrite(LED2, 1);
        }
        blinkingLedIsOn = !blinkingLedIsOn;
        blinkingLedNextTime = millis() + BLINKING_LED_PERIOD;
      }
    }
  }

  if (!ErrorOilLevelIsTooLow && !ErrorTempIsTooHigh && ledIsBlinking) {
    digitalWrite(LED1, 0);
    digitalWrite(LED2, 0);
    ledIsBlinking = false;
  }

  // save duration counters in EEProm every SAVE_DURATION_COUNTERS_WINDOW number of seconds
  if ((millis() / 1000) > DurationCounterSave) {
    DurationCounterSave = millis() / 1000 + SAVE_DURATION_COUNTERS_WINDOW;
    
    Log.print("powered_total = ");
    Log.println(powered_total);
    Log.print("running_total = ");
    Log.println(running_total);

    saveDurationCounters();
  }
}

void loop() {
  node.loop();

  temp_sensor_loop();
  oilLevel.update();
  pressure_sensor_loop();

  display_loop();

  compressorLoop();

  if (laststate != machinestate) {
    Log.printf("Changed from state <%s> to state <%s>\n",
               state[laststate].label, state[machinestate].label);

    if (machinestate == POWERED && laststate < POWERED) {
      powered_last = millis();
    } else if (laststate >= POWERED && machinestate < POWERED) {
      powered_total += (millis() - powered_last) / 1000;
      powered = (float)powered_total / 3600.0;
    };
    if (machinestate == RUNNING && laststate < RUNNING) {
      running_last = millis();
    } else if (laststate == RUNNING && machinestate < RUNNING) {
      running_total += (millis() - running_last) / 1000;
      running = (float)running_total / 3600.0;
    };
    laststate = machinestate;
    laststatechange = millis();
  }

  if (state[machinestate].maxTimeInMilliSeconds != NEVER &&
      (millis() - laststatechange > state[machinestate].maxTimeInMilliSeconds)) {
    laststate = machinestate;
    machinestate = state[machinestate].failStateOnTimeout;
    Debug.printf("Time-out; transition from %s to %s\n",
               state[laststate].label, state[machinestate].label);
    return;
  };

  buttons_optocoupler_loop();

  oil_level_sensor_loop();

  switch (machinestate) {
    case REBOOT:
      saveDurationCounters();
      node.delayedReboot();
      break;

    case SWITCHEDOFF:
      // Compressor switched off completely.
      if (compressorIsOn) {
        digitalWrite(RELAY_GPIO, 0);
        digitalWrite(LED1, 0);
        digitalWrite(LED2, 0);
        compressorIsOn = false;
        Log.printf("Compressor switched off\n");
      }
      break;

    case POWERED:
      // Compressor switched on, but motor is off
      static unsigned long lastpowered = 0;

      if (!compressorIsOn) {
        digitalWrite(RELAY_GPIO, 1);
        digitalWrite(LED1, 1);
        digitalWrite(LED2, 0);
        compressorIsOn = true;
        Log.printf("Compressor switched on, motor is off\n");
      } else {
        if (millis() - lastpowered > 10 * 1000 || lastpowered == 0) {
          Log.printf("Compressor switched on, motor is off\n");
          lastpowered = millis();
        };
      }
      break;
    case RUNNING:
      // Compressor switched on and motor is running
      {
        static unsigned long lastrunning = 0;
        if (millis() - lastrunning > 10 * 1000 || lastrunning == 0) {
          Log.printf("Compressor switched on, motor is running\n");
          lastrunning = millis();
        };
      }
      break;

    case TRANSIENTERROR:
    case OUTOFORDER:
    case NOCONN:
    case BOOTING:
      break;
  };
}

