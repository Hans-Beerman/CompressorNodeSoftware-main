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
#error "The space deur uses an Olimex ESP32-PoE board only!"
#endif

#define ESP32_PoE
// #define OB_POLLING

// #define CALIBRATE_PRESSURE

#define RELAY_GPIO      (14)  // digital output
#define ON_BUTTON      (15)  // digital input
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
// #include <PowerNodeV11.h>
#include <WiredEthernet.h>
#include <SIG2.h>
#include <OptoDebounce.h>
#include <ButtonDebounce.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#include <U8x8lib.h>


// One Wire input port for temperature sensor
#define ONE_WIRE_BUS (TEMPSENSOR)

/* Voor testen thuis
 *
 */
#define OTA_PASSWD "MyPassWoord"
#define WIFI_NETWORK "WLAN-HB-24"
#define WIFI_PASSWD "C7F382D940BBAB5"
/* Einde testen thuis
 *
 */

/* Voor testen makerspace
 *
 */
// #define OTA_PASSWD "MyPassWoord"
// #define WIFI_NETWORK "MakerSpaceLeiden_deelnemers"
// #define WIFI_PASSWD "M@@K1234"

#define MACHINE "test-compressor1"
// define MACHINE "compressor"
/* Einde voor testen makerspace
 *
 */

ButtonDebounce buttonOn(ON_BUTTON, 150 /* mSeconds */);
ButtonDebounce buttonOff(OFF_BUTTON, 150 /* mSeconds */);
OptoDebounce opto1(OPTO1); // wired to N0 - L1 of 3 phase compressor motor

ButtonDebounce oilLevel(OILLEVELSENSOR, 300 /* mSeconds */);

OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensorTemp(&oneWire);

#define TEMP_RESOLUTION (12) // 9, 10, 11 or 12 bit resolution
#define MAX_TEMP_CONVERSIONTIME (750) // in ms
#define MAX_TEMP_ERROR_COUNT (30)

#define TO_LOW_OIL_LEVEL (LOW)
#define OIL_LEVEL_LOG_WINDOW (60000) // in ms

#define PRESSURE_SAMPLE_WINDOW (1000) // in ms
#define PRESSURE_LOG_WINDOW (60000) // in ms
#define PRESSURE_CALIBRATE_VALUE_0_5V (144) // in measured bits
#define PRESSURE_CALIBRATE_VALUE_4_5V (2600) // in measured bits

#define DISPLAY_WINDOW (1000) // in ms, update display time

// Clear EEProm button
// Press BUT1 on Olimex ESP32 PoE module before (re)boot of node
// keep BUT1 pressed for at least 5 s
// After the release of BUT1 node will restart with empty EEProm
#define CLEAR_EEPROM_BUTTON                   (34)
#define CLEAR_EEPROM_BUTTON_PRESSED           (LOW)
#define MAX_WAIT_TIME_BUTTON_PRESSED          (4000)  // in ms

/* Voor testen thuis
 *
 */ 
ACNode node = ACNode(MACHINE, WIFI_NETWORK, WIFI_PASSWD);
// ACNode node = ACNode(MACHINE);
/* Einde voor testen thuis
 *
 */ 

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
int tempErrorCount = 0;
unsigned long conversionTime = MAX_TEMP_CONVERSIONTIME / (1 << (12 - TEMP_RESOLUTION));
unsigned long tempAvailableTime = 0;

bool previousOilLevelIsTooLow = false;
bool oilLevelIsTooLow = false;
bool lastOilLevelDisplayed = false;
unsigned long oilLevelNextLoggingTime = 0;

int pressureADCVal = 0;
float pressureVoltage = 0;
float pressure = 0;
float lastPressureDisplayed = -1;
unsigned long pressureNextSampleTime = 0;
unsigned long pressureNextLogTime = 0;

unsigned long updateDisplayTime = 0;
bool firstTimeDisplayed = true;

// for 1.5 inch OLED Display 128*128 pixels wit - I2C
U8X8_SSD1327_WS_128X128_SW_I2C u8x8(I2C_SCL, I2C_SDA,U8X8_PIN_NONE);

void checkClearEEPromAndCacheButtonPressed(void) {
  unsigned long ButtonPressedTime;
  unsigned long currentSecs;
  unsigned long prevSecs;
  bool firstTime = true;

  // check CLEAR_EEPROM_BUTTON pressed
  pinMode(CLEAR_EEPROM_BUTTON, INPUT);
  // check if button is pressed for at least 3 s
  Log.println("Checking if the button is pressed for clearing EEProm");
  ButtonPressedTime = millis();  
  prevSecs = MAX_WAIT_TIME_BUTTON_PRESSED / 1000;
  Log.print(prevSecs);
  Log.print(" s");
  while (digitalRead(CLEAR_EEPROM_BUTTON) == CLEAR_EEPROM_BUTTON_PRESSED) {
    if (millis() >= MAX_WAIT_TIME_BUTTON_PRESSED) {
      if (firstTime == true) {
        Log.print("\rPlease release button");
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
    Log.print("\rButton for clearing EEProm was pressed for more than ");
    Log.print(MAX_WAIT_TIME_BUTTON_PRESSED / 1000);
    Log.println(" s, EEProm will be cleared!");
    // Clear EEPROM
    wipe_eeprom();
    Log.println("EEProm cleared!");
    // wait until button is released, than reboot
    while (digitalRead(CLEAR_EEPROM_BUTTON) == CLEAR_EEPROM_BUTTON_PRESSED) {
      // do nothing here
    }
    Log.println("Node will be restarted");
    // restart node
    ESP.restart();
  } else {
    Log.println("\rButton was not (or not long enough) pressed to clear EEProm");
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

  // the default is space.makerspaceleiden.nl, prefix test
  // node.set_mqtt_host("mymqtt-server.athome.nl");
  // node.set_mqtt_prefix("test-1234");

  // specify this when using your own `master'.
  //
  // node.set_master("test-master");

  // node.set_report_period(2 * 1000);

  // init temperature sensor and read first value
  sensorTemp.begin();
  sensorTemp.getAddress(tempDeviceAddress, 0);
  sensorTemp.setResolution(tempDeviceAddress, TEMP_RESOLUTION);
  sensorTemp.setWaitForConversion(false);
  sensorTemp.requestTemperatures();
  tempAvailableTime = millis() + conversionTime;

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

  buttonOn.setCallback([](int state) {
    Debug.printf("Button On changed to %d\n", state);
  });
  buttonOff.setCallback([](int state) {
    Debug.printf("Button Off changed to %d\n", state);
  });

  oilLevel.setCallback([](int state) {
    Debug.printf("OilLevel sensor changed to %d\n", state);
  });

  node.onReport([](JsonObject  & report) {
    report["state"] = state[machinestate].label;

    report["powered_time"] = powered_total + ((machinestate == POWERED) ? ((millis() - powered_last) / 1000) : 0);
    report["running_time"] = running_total + ((machinestate == RUNNING) ? ((millis() - running_last) / 1000) : 0);

    if (currentTemperature == -127) {
      report["temperature_sensor"] = "Error reading temperature sensor, perhaps not connected?";
    } else {
      report["temperature_sensor"] = currentTemperature;
    }

    if (oilLevelIsTooLow == false)
    {
      report["oil_level_sensor"] = "oil level is OK!";
    } else {
      report["oil_level_sensor"] = "oil level too low!";
    }
    
    report["pressure_sensor"] = pressure;

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
}

void temp_sensor_loop() {
  if (millis() > tempAvailableTime) {
    currentTemperature = sensorTemp.getTempCByIndex(0);
    sensorTemp.requestTemperatures();
    tempAvailableTime = millis() + conversionTime;

    if (currentTemperature == -127) {
      if ((currentTemperature != previousTemperature) && (tempErrorCount == 0)) {
        tempErrorCount = 1;
      } else {
        previousTemperature = currentTemperature;
        tempErrorCount++;
        if (tempErrorCount > MAX_TEMP_ERROR_COUNT) {
          Log.println("Temperature sensor does not react, perhaps not available?");
          tempErrorCount = 0;
          temperature = -127;
        }
      }
    } else {
      tempErrorCount = 0;

      if (currentTemperature != previousTemperature) {
        previousTemperature = currentTemperature;
        temperature = currentTemperature;
        Serial.print("Temp. changed, current temp. = ");
        Serial.print(currentTemperature);
        Serial.println(" degrees C");
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

  if (millis() > updateDisplayTime)
  {
    updateDisplayTime = millis() + DISPLAY_WINDOW;

    if ((pressure != lastPressureDisplayed) || (firstTimeDisplayed == true)) {
      lastPressureDisplayed = pressure;
      u8x8.setFont(u8x8_font_px437wyse700b_2x2_f);
      sprintf(outputStr, "%4.2f MPa", pressure); 
      u8x8.drawString(0, 0, "Pressure");
      u8x8.drawString(0, 2, outputStr);
    }
    if ((temperature != lastTempDisplayed) || (firstTimeDisplayed == true)) {
      lastTempDisplayed = temperature;
      u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
      if (temperature < -100) {
        sprintf(outputStr, "Temp.: N.A.");
      } else {
        sprintf(outputStr, "Temp.:%7.2f %cC", temperature, 176);
      }
      u8x8.drawString(0, 5, outputStr);
    }
    if ((machinestate != laststateDisplayed)  || (firstTimeDisplayed == true)) {
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
    if ((oilLevelIsTooLow != lastOilLevelDisplayed)  || (firstTimeDisplayed == true)) {
      lastOilLevelDisplayed = oilLevelIsTooLow;
      u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
      
      if (oilLevelIsTooLow == true) {
        u8x8.drawString(0, 10, "OilLevel too low");      
      } else {
        u8x8.drawString(0, 10, "OilLevel OK!    ");      
      }
    }

    if ((machinestate == POWERED) || (firstTimeDisplayed == true)) {
      if (machinestate < POWERED) {
        powered = (float)powered_total / 3600.0;
      } else {
        powered = ((float)powered_total + ((float)millis() - float(powered_last)) / 1000.0) / 3600.0;
      }

      if ((powered != lastPoweredDisplayed) || (firstTimeDisplayed == true)) {
        lastPoweredDisplayed = powered;
        u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
        sprintf(outputStr, "On: %9.2f hr", powered);
        u8x8.drawString(0, 12, outputStr);
      }
    }

    if ((machinestate == RUNNING) || (firstTimeDisplayed == true)) {
      if (machinestate < RUNNING) {
        running = (float)running_total / 3600.0;
      } else {
        running = ((float)running_total + ((float)millis() - (float)running_last) / 1000.0) / 3600.0;
      }

      if ((running != lastRunningDisplayed) || (firstTimeDisplayed == true)) {
        lastRunningDisplayed = running;
        u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
        sprintf(outputStr, "Run:%9.2f hr", running);
        u8x8.drawString(0, 13, outputStr);
      }
    }



    firstTimeDisplayed = false;
  }
}

void loop() {
  node.loop();
  opto1.loop();

  buttonOn.update();
  buttonOff.update();

  temp_sensor_loop();
  oilLevel.update();
  pressure_sensor_loop();

  display_loop();

  if (laststate != machinestate) {
    Log.printf("Changed from state <%s> to state <%s>\n",
               state[laststate].label, state[machinestate].label);

    if (machinestate == POWERED && laststate < POWERED) {
      powered_last = millis();
    } else if (laststate == POWERED && machinestate < POWERED) {
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

  
   if (opto1.state() == true) {
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

  if (buttonOn.state() == LOW && machinestate == SWITCHEDOFF) {
    Log.printf("Compressor switched on with button\n");
    digitalWrite(RELAY_GPIO, 1);
    digitalWrite(LED1, 1);
    digitalWrite(LED2, 0);
    machinestate = POWERED;
  };

  if (buttonOff.state() == LOW && machinestate >= POWERED) {
    Log.printf("Compressor switched off with button\n");
    digitalWrite(RELAY_GPIO, 0);
    digitalWrite(LED1, 0);
    digitalWrite(LED2, 0);
    machinestate = SWITCHEDOFF;
  };

  if (oilLevel.state() == TO_LOW_OIL_LEVEL) {
    oilLevelIsTooLow = true;
    
    if ((oilLevelIsTooLow != previousOilLevelIsTooLow) || (millis() > oilLevelNextLoggingTime)) {
      oilLevelNextLoggingTime = millis() + OIL_LEVEL_LOG_WINDOW;
      Log.println("Oil level is too low!");
      previousOilLevelIsTooLow = oilLevelIsTooLow;
    }
  } else {
    oilLevelIsTooLow = false;
    if (oilLevelIsTooLow != previousOilLevelIsTooLow) {
      Log.println("Oil level is OK now!");
      previousOilLevelIsTooLow = oilLevelIsTooLow;
    }
  }

  switch (machinestate) {
    case REBOOT:
      node.delayedReboot();
      break;

    case SWITCHEDOFF:
      // Compressor switched off completely.
      break;

    case POWERED:
      // Normal state -- PoE power is always on.
      static unsigned long lastpowered = 0;
        if (millis() - lastpowered > 10 * 1000 || lastpowered == 0) {
          Log.printf("Compressor switched on, motor is off\n");
          lastpowered = millis();
        };
      break;
    case RUNNING:
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

