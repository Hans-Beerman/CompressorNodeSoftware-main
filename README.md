**Source code CompressorNode Makerspace Leiden**

Current version: V0.9 Concept

This repository contains the source code for the CompressorNode used in the Makerspace Leiden.

This CompressorNode is based on the use of the Olimex ESP32\_PoE. This small processor board is plugged on a backplane, details about this backplane can be found here:

[https://github.com/Hans-Beerman/CompressorNode](https://github.com/Hans-Beerman/CompressorNode)

This software is developed with Visual Studio Code in combination with the extension PlatformIO.

**This CompressorNode (the hardware in combination with the software) has the following features:**

- _Button On_: to manual switch on the compressor;
- _Button Off_: to manual switch off the compressor;
- _Button On + Button Off_: if both buttons are pressed together for more than 5 seconds, while calibration mode is disabled, calibration mode will be disabled. Pressing both buttons together while calibration mode is enabled, calibration mode is disabled immediately. Also the duratution counters are saved in flash, every time calibration mode is enabled;
- _but1_: also used to toggle info / calibration mode on or off. but1 is the second button of the ESP32-PoE module. Press this button, when the node is running (the boot screen is not shown anymore). When info calibration mode is active each second e.g. the IP address and calibration info is logged to MQTT and telnet etc. Also the duratution counters are saved in flash, every time this button is pressed;
- _Automatic switch on_: switch the compressor on by means of dedicated MQTT messages
- _Automatic switch off_: switch the compressor off by means of dedicated MQTT mesages
- _Timeout_: the compressor will automatically switch off after a certain (in source code) configured timeout, currently after 30 minutes. Pressing Button On again (or sending MQTT command), while the compressor is switched on, will extend the timeout with 30 minutes;
- _Late hour disable_: after a configured time in the evening (currently 22:00 h) and before a configured time in the morning (currently 08:00 h) the compressor is disabled to prevent too much noise for our neighbors. This means the compressor will not start automatically by means of MQTT messages. Also the compressor will not start if the Button On is pressed normally;
- _Overrule late hour disable_: the late hour disable function can be overruled by pressing the Button On continuously for more than 10 seconds;
- _230VAC relais output:_ to switch the compressor on or off;
- _230VAC opto coupler input:_ This input is connected to one of the phases of the 3-phase motor of the compressor to detect if the motor is running or not;
- _2 LED&#39;s_: for signaling purposes. LED1 is on continuously when the compressor is switched on. LED2 is on continuously when the motor in the compressor is switched on. LED1 will flash for 5 s with 200 ms intervals if Button On is pressed during late hour disable. Both LED&#39;s will flash simultaneously with 600 ms intervals if there is an error which make it impossible to operate the compressor. The LED&#39;s will flash until the error is solved. The following two errors will disable de compressor:
  - Oil level too low;
  - Temperature to high;
- _Measurement of the machine temperature_: the temperature of the compressor is measured and reported via MQTT. The temperature is also shown on the display of the node. There are 2 temperature sensors, one is measuring the temperture of the motor, the other measures the temperature of the compressor;
- _Measurement of the oil level_: The oil level of the compressor is measured and reported via MQTT. This pressure is also shown on the display of the node;
- _Measurement of air pressure_: The air pressure, as produced by the compressor is measured and reported via MQTT. This pressure is also shown on the display of the node;
- _Status show on display_: There is a small Oled display (128x128 pixels) which shows status information about the node and the compressor.

**Setup of the software development environment**

See [https://github.com/dirkx/AccesSystem/tree/master/lib-arduino/ACNode](https://github.com/dirkx/AccesSystem/tree/master/lib-arduino/ACNode) for an introduction of how the software development environment can be setup. The description here however is based on the use of the Arduino software development tool. It will give you information about the libraries needed to compile the source code of the compressor node.

In addition to this description the following libraries are needed:

- _NTP_: NTP by Stefan Staub (#include <NTP.h>;) current version 1.4.

For more information about this library see:

[https://platformio.org/lib/show/5438/NTP](https://platformio.org/lib/show/5438/NTP)

This library is used to collect the correct local time via NTP;

- _DallasTemperature_: DallasTemperature by Miles Burton (#include <DallasTemperature.h>), current version 3.6.1, this library is used for the one wire temperature sensor based on the DS18B20;
- _U8x8lib_: U8g2 library by oliver (#include <U8x8lib.h>), current version 2.28.2, this library is used for the 128x128 pixels Oled display used.

**Configuratie PlatformIO**

The configuration for this Project in PlatformIO is stored in the platformio.ini file, the content of this file is shown next, the IP address (192.168.5.95) is the current IP address of the node in the Makerspace, please change this to the correct IP address, given by the network to which the node is connected:

_;PlatformIO Project Configuration File_

_;_

_; Build options: build flags, source filter_

_; Upload options: custom upload port, speed and extra flags_

_; Library options: dependencies, extra library storages_

_; Advanced options: extra scripting_

_;_

_; Please visit documentation for the other options and examples_

_; https://docs.platformio.org/page/projectconf.html_

_[env:esp32-poe]_

_platform = espressif32_

_board = esp32-poe_

_framework = arduino_

_; enable ota_

_upload\_protocol = espota_

_upload\_port = 192.168.5.95_

_upload\_flags =_

_--port=8266_

_--auth=MyPassWoord_

_; evaluate C/C++ Preprocessor conditional syntax_

_lib\_ldf\_mode = deep+_

_;upload\_port = /dev/ttyUSB0_

_monitor\_speed = 115200_

_board\_build.partitions = huge\_app.csv_

**Configuration of the behaviour of the Node**

With the following parameters in the source code the behaviour of the node can be controlled:

- _Temperature limits:_

In main.cpp:

// temperature sensor

#define TEMP\_SENSOR\_LABEL1 ("Compressor") // label used in logging for temp. sensor 1

#define TEMP\_SENSOR\_LABEL2 ("Motor") // label used in logging for temp. sensor 2

#define TEMP\_REPORT\_ERROR1 ("temperature_sensor_1_(compressor)_error") // label used in reporting for temp. sensor 1

#define TEMP\_REPORT\_WARNING1 ("temperature_sensor_1_(compressor)_warning") // label used in reporting for temp. sensor 1

#define TEMP\_REPORT\1 ("temperature_sensor_1_(compressor)") // label used in reporting for temp. sensor 1

#define TEMP\_REPORT\_ERROR2 ("temperature_sensor_2_(motor)_error") // label used in reporting for temp. sensor 2

#define TEMP\_REPORT\_WARNING2 ("temperature_sensor_2_(motor)_warning") // label used in reporting for temp. sensor 2

#define TEMP\_REPORT2 ("temperature_sensor_2_(motor)") // label used in reporting for temp. sensor 2

#define TEMP\_IS\_HIGH\_LEVEL\_1 (60.0) // in degrees Celcius, used for temperature is high warning of sensor 1

#define TEMP_IS\_TOO\_HIGH\_LEVEL\_1 (90.0) // in degrees Celcius, used to disable the compressor when temperature is too high of sensor 1

#define TEMP_IS\_HIGH\_LEVEL\_2 (60.0) // in degrees Celcius, used for temperature is high warning of sensor 2

#define TEMP_IS\_TOO\_HIGH\_LEVEL\_2 (90.0) // in degrees Celcius, used to disable the compressor when temperature is too high of sensor 2

- _The LED's dim value:_
In main.cpp:

#define LED1\_DIM\_VALUE (50) // 0 - 255, 0 = LED1 is off
#define LED2\_DIM\_VALUE (50) // 0 - 255, 0 = LED2 is off

- _The time interval of the LED&#39;s in case of an error:_

In main.cpp:

// For LED&#39;s showing node error

#define BLINKING\_LED\_PERIOD (600) // in ms

- The time window between saves of the duration counters:

In main.cpp:

// for storage in EEProm of the duration counters

#define SAVE\_DURATION\_COUNTERS\_WINDOW (86400) // in seconds (86400 = 24 hour)

- _For the automatic timeout of the compressor:_

In main.cpp:

// for auto switch off of the compressor

#define AUTOTIMEOUT (30 \* 60 \* 1000) // default: in ms 30 \* 60 \* 1000 = 30 minutes

- _For the late hour disable function:_

In main.cpp:

// Compressor disabled (or not) at late hours:

// IF the compressor is not allowed at late hours (DISABLE\_COMPRESSOR AT LATE HOURS = true) the

// compressor will not switch on automatically (or by hand if the on button is pressed normally) from

// DISABLED\_TIME\_START to DISABLED\_START\_END

// Pressing the on button longer than MAX\_WAIT\_TIME\_BUTTON\_ON\_PRESSED will override this behaviour by

// switching on the compressor anyhow. In all cases the compressor will switch of after AUTOTIMEOUT (in ms)

// unless the button on is pressed again or a new auto on command is received while the compressor is

// already switched on. In both cases the time will be extended by AUTOTIMEOUT ms.

#define DISABLE\_COMPRESSOR\_AT\_LATE\_HOURS (true)

#define DISABLED\_TIME\_START (19) // in hour, time from which the compressor is not automatically switched on

#define DISABLED\_TIME\_END (8) // in hour, time to which the compressor is not automatically switched on

#define MAX\_WAIT\_TIME\_BUTTON\_ON\_PRESSED (10000) // in ms, time button on must be pressed to override late hour compressor disable

#define LED\_DISABLE\_DURATION (5000) // in ms, the time LED1 will flash if button on is pressed during late hour

#define LED\_DISABLE\_PERIOD (200) // in ms, the time LED1 will flash on/off

- _Time window for calibration buttons_

In main.cpp:

#define CALIB\_WINDOW\_TIME                     (5000) // in ms

- _Clear EEProm and cache, by pressing but1 of the ESP32-PoE module during boot for some time:_

In main.cpp:

#define MAX\_WAIT\_TIME\_BUTTON\_PRESSED (4000) // in ms

- _For logging to MQTT etc.:_

In main.cpp:

#define LOGGING\_ENABLED                       (true)  // to enable/disable logging

#define LOGGING\_TIME_\WINDOW                   (20000)  // in ms

- _The time between updates of the display:_

In OledDisplay.cpp:

// oled display

#define DISPLAY\_WINDOW (1000) // in ms, update display time

- _The time short status messages are shown on the bottom line of the display:_

In OledDisplay.cpp

#define KEEP\_STATUS\_LINE\_TIME (5000) // in ms, default = 5 s (5000), the time certain status messages are shown on the bottom line of the display

- _The time the oil level is too low, before an error is signaled:_

In OilLevelSensor.cpp:

#define MAX\_OIL\_LEVEL\_IS\_TOO\_LOW\_WINDOW (10000) // in ms default 10000 = 10 seconds. Error is signalled after this time window is passed

- _For calibration of the air pressure sensor:_

See the following parameters:

- Pressure calibrating values:

In PressureSensor.cpp:

#define PRESSURE\_CALIBRATE\_VALUE\_0\_5V (144) // in measured bits

#define PRESSURE\_CALIBRATE\_VALUE\_4\_5V (2600) // in measured bits

The air pressure sensor output is 0.5 V when the pressure is 0 bar.

The output is 4.5 V when the pressure is 12 bar

- _The time between samples of the air pressure:_

In PressureSensor.cpp:

#define PRESSURE\_SAMPLE\_WINDOW (1000) // in ms

- _The time the temperature is too high, before an error is signaled:_

In TempSensor.cpp:

#define MAX\_TEMP\_IS\_TOO\_HIGH\_WINDOW (10000) // in ms default 10000 = 10 seconds. Error is only signalled after this time window is passed



