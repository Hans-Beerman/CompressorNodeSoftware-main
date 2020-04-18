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

#include <Arduino.h>
#include <MachState.h>
#include <ACNode.h>
#include <WiredEthernet.h>
#include <SIG2.h>
#include <Cache.h>
#include <OptoDebounce.h>
#include <ButtonDebounce.h>
#include <WiFiUdp.h>
#include <NTP.h> // install NTP by Stefan Staub
//
// information about NTP.h, see: https://platformio.org/lib/show/5438/NTP
//
#include "TempSensor.h"
#include "OledDisplay.h"


WiFiUDP wifiUDP;
NTP ntp(wifiUDP);

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

// temperature sensor
#define TEMP_IS_HIGH_LEVEL (40.0) // in degrees Celcius, used for temperature is high warning
#define TEMP_IS_TOO_HIGH_LEVEL (70.0) // in degrees Celcius, used to disable the compressor when temperature is too high

// oil level sensor
#define TO_LOW_OIL_LEVEL (LOW) // the input level of the GPIO port used for the oil level sensor signalling too low oil level
#define MAX_OIL_LEVEL_IS_TOO_LOW_WINDOW (10000) // in ms default 10000 = 10 seconds. Error is signalled after this time window is passed
#define OIL_LEVEL_LOG_WINDOW (60000) // in ms

ButtonDebounce oilLevel(OILLEVELSENSOR, 300 /* mSeconds */); // to signal if the oil level is too low (or not)

// For LED's showing node error
#define BLINKING_LED_PERIOD (600) // in ms

// pressure sensor
#define PRESSURE_SAMPLE_WINDOW (1000) // in ms
#define PRESSURE_LOG_WINDOW (60000) // in ms
#define PRESSURE_CALIBRATE_VALUE_0_5V (144) // in measured bits
#define PRESSURE_CALIBRATE_VALUE_4_5V (2600) // in measured bits

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
  { "Waiting for card",     LED::LED_IDLE,            NEVER, REBOOT },
  { "Checking card",        LED::LED_IDLE,            NEVER, REBOOT },
  { "Compressor switched off", LED::LED_IDLE,         NEVER, SWITCHEDOFF},
  { "Powered - motor off",  LED::LED_IDLE,            NEVER, POWERED },
  { "Powered - motor running", LED::LED_ON,           NEVER, RUNNING },
};

unsigned long laststatechange = 0;
static machinestates_t laststate = BOOTING;
machinestates_t machinestate = BOOTING;

unsigned long powered_total = 0, powered_last;
unsigned long running_total = 0, running_last;

float powered = 0.0;
float running = 0.0;

// temperature sensor
TemperatureSensor theTempSensor(TEMP_IS_HIGH_LEVEL, TEMP_IS_TOO_HIGH_LEVEL);

// OledDisplay
OledDisplay theOledDisplay;

bool previousOilLevelIsTooLow = false;
bool oilLevelIsTooLow = false;
unsigned long oilLevelNextLoggingTime = 0;
unsigned long oilLevelIsTooLowStart = 0;
bool previousErrorOilLevelIsTooLow = false;
bool ErrorOilLevelIsTooLow = false;

int pressureADCVal = 0;
float pressureVoltage = 0;
float pressure = 0;
unsigned long pressureNextSampleTime = 0;
unsigned long pressureNextLogTime = 0;

unsigned long blinkingLedNextTime = 0;
bool blinkingLedIsOn = false;
bool ledIsBlinking = false;

char reportStr[128];

unsigned long DurationCounterSave;

unsigned long buttonOnPressedTime;
bool buttonOnIsPressed = false;

unsigned long ledDisableTime = 0;
unsigned long nextLedDisableTime = 0;
bool showLedDisable = false;

unsigned long autoPowerOff;
bool compressorIsOn = false;

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
    theOledDisplay.clearEEPromWarning();
    ButtonPressedTime = millis();  
    prevSecs = MAX_WAIT_TIME_BUTTON_PRESSED / 1000;
    Log.print(prevSecs);
    Log.print(" s");
    while (digitalRead(CLEAR_EEPROM_AND_CACHE_BUTTON) == CLEAR_EEPROM_AND_CACHE_BUTTON_PRESSED) {
      if (millis() >= ButtonPressedTime + MAX_WAIT_TIME_BUTTON_PRESSED) {
        if (firstTime) {
          Log.print("\rPlease release button");
          theOledDisplay.showStatus(RELEASEBUTTON, TEMP_IS_HIGH_LEVEL, TEMP_IS_TOO_HIGH_LEVEL);
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
      theOledDisplay.clearEEPromMessage();
      theOledDisplay.showStatus(NOSTATUS, TEMP_IS_HIGH_LEVEL, TEMP_IS_TOO_HIGH_LEVEL);
      // Clear EEPROM
      wipe_eeprom();
      Log.println("EEProm cleared!");
      theOledDisplay.EEPromCleared();      
      // Clear cache
      prepareCache(true);
      // Clear duration counter file
      String path = DURATION_DIR_PREFIX + (String)DURATION_FILE_PREFIX;
      if (SPIFFS.exists(path)) {
        SPIFFS.remove(path);
      } 
      theOledDisplay.cacheCleared();
      Log.println("Cache cleared!");
      // wait until button is released, than reboot
      while (digitalRead(CLEAR_EEPROM_AND_CACHE_BUTTON) == CLEAR_EEPROM_AND_CACHE_BUTTON_PRESSED) {
        // do nothing here
      }
      Log.println("Node will be restarted");
      theOledDisplay.showStatus(NODEREBOOT, TEMP_IS_HIGH_LEVEL, TEMP_IS_TOO_HIGH_LEVEL);
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

  theOledDisplay.begin();

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
  theTempSensor.begin();

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
      theOledDisplay.showStatus(AUTOSWITCHOFF, TEMP_IS_HIGH_LEVEL, TEMP_IS_TOO_HIGH_LEVEL);
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
          theOledDisplay.showStatus(AUTOSWITCHON, TEMP_IS_HIGH_LEVEL, TEMP_IS_TOO_HIGH_LEVEL);
        };
        autoPowerOff = millis() + AUTOTIMEOUT;
      } else {
        Log.println("Request denied to power on the compressor. Reason: late hours/night!");
        theOledDisplay.showStatus(AUTOONDENIED, TEMP_IS_HIGH_LEVEL, TEMP_IS_TOO_HIGH_LEVEL);
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
  
  theOledDisplay.clearDisplay();

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
      theOledDisplay.showStatus(MANUALSWITCHON, TEMP_IS_HIGH_LEVEL, TEMP_IS_TOO_HIGH_LEVEL);
    } else {
      if (!buttonOnIsPressed) {
        buttonOnPressedTime = millis();
        theOledDisplay.showStatus(POWERONDISABLED, TEMP_IS_HIGH_LEVEL, TEMP_IS_TOO_HIGH_LEVEL);
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
        theOledDisplay.showStatus(MANUALOVERRIDE, TEMP_IS_HIGH_LEVEL, TEMP_IS_TOO_HIGH_LEVEL);
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
    theOledDisplay.showStatus(MANUALSWITCHOFF, TEMP_IS_HIGH_LEVEL, TEMP_IS_TOO_HIGH_LEVEL);
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
        theOledDisplay.showStatus(TIMEOUT, TEMP_IS_HIGH_LEVEL, TEMP_IS_TOO_HIGH_LEVEL);
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

  theTempSensor.loop();
  oilLevel.update();
  pressure_sensor_loop();

  theOledDisplay.loop(oilLevelIsTooLow, ErrorOilLevelIsTooLow, temperature, tempIsHigh, 
                      ErrorTempIsTooHigh, pressure, machinestate, 
                      TEMP_IS_HIGH_LEVEL, TEMP_IS_TOO_HIGH_LEVEL,
                      powered_total, powered_last,
                      running_total, running_last);

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

    case WAITINGFORCARD:
    case CHECKINGCARD:
      // should not come here, so bail out
      machinestate = REBOOT;
      break;
    case TRANSIENTERROR:
    case OUTOFORDER:
    case NOCONN:
    case BOOTING:
      break;
  };
}

