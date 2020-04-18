**Source code CompressorNode Makerspace Leiden**

Current version: V0.1 Concept

This repository contains the source code for the CompressorNode used in the Makerspace Leiden.

This CompressorNode is based on the use of the Olimex ESP32\_PoE. This small processor board is plugged on a backplane, details about this backplane can be found here:

[https://github.com/Hans-Beerman/CompressorNode](https://github.com/Hans-Beerman/CompressorNode)

This software is developed with Visual Studio Code in combination with the extension PlatformIO.

**This CompressorNode (the hardware in combination with the software) has the following features:**

- _Button On_: to manual switch on the compressor;
- _Button Off_: to manual switch off the compressor;
- _Automatic switch on_: switch the compressor on by means of dedicated MQTT messages
- _Automatic switch off_: switch the compressor off by means of dedicated MQTT mesages
- _Timeout_: the compressor will automatically switch off after a certain (in source code) configured timeout, currently after 30 minutes;
- _Late hour disable_: after a configured time in the evening (currently 19:00 h) and before a configured time in the morning (currently 08:00 h) the compressor is disabled to prevent too much noise for our neighbors. This means the compressor will not start automatically by means of MQTT messages. Also the compressor will not start if the Button On is pressed normally;
- _Overrule late hour disable_: the late hour disable function can be overruled by pressing the Button On continuously for more than 10 seconds;
- _230VAC relais output:_ to switch the compressor on or off;
- _230VAC opto coupler input:_ This input is connected to one of the phases of the 3-phase motor of the compressor to detect if the motor is running or not;
- _2 LED&#39;s_: for signaling purposes. LED1 is on continuously if the compressor is switched on. LED2 is on continuously if the motor in the compressor is switched on. LED1 will flash for 5 s with 200 ms intervals if Button On is pressed during late hour disable. Both LED&#39;s will flash simultaneously with 600 ms intervals if there is an error which make it impossible to operate the compressor. The LED&#39;s will flash until the error is solved. The following two errors will disable de compressor:
  - Oil level too low;
  - Temperature to high;
- _Measurement of the machine temperature_: the temperature of the compressor is measured and reported via MQTT. The temperature is also shown on the display of the node;
- _Measurement of the oil level_: The oil level of the compressor is is measured and reported via MQTT. This pressure is also shown on the display of the node;
- _Measurement of air pressure_: The air pressure, as produced by the compressor is measured and reported via MWTT. This pressure is also shown on the display of the node;
- _Status show on display_: There is a small Oled display (128x128 pixels) which shows status information about the node and the compressor.

**Setup of the software development environment**

See [https://github.com/dirkx/AccesSystem/tree/master/lib-arduino/ACNode](https://github.com/dirkx/AccesSystem/tree/master/lib-arduino/ACNode) for an introduction of how the software development environment can be setup. The description here however is based on the use of the Arduino software development tool. It will give you information about the libraries needed to compile the source code of the compressor node.

In addition to this description the following libraries are needed:

- _NTP_: NTP by Stefan Staub (#include \&lt;NTP.h\&gt;) current version 1.4.

For more information about this library see:

[https://platformio.org/lib/show/5438/NTP](https://platformio.org/lib/show/5438/NTP)

This library is used to collect via NTP the correct local time;

- _DallasTemperature_: DallasTemperature by Miles Burton (#include \&lt;DallasTemperature.h\&gt;), current version 3.6.1;
- _U8x8lib_: U8g2 library by oliver (#include \&lt;U8x8lib.h\&gt;), current version 2.28.2.

**Configuratie PlatformIO**

The configuration for this Project in PlatformIO is stored in the platformio.ini file, the content of this file is file is shown next, the IP address (10.0.0.127) shown in this config will be different, dependent of the network to which the node is connected:

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

_upload\_port = 10.0.0.127_

_upload\_flags =_

_--port=8266_

_--auth=MyPassWoord_

_; evaluate C/C++ Preprocessor conditional syntax_

_lib\_ldf\_mode = deep+_

_;upload\_port = /dev/ttyUSB0_

_monitor\_speed = 115200_

_board\_build.partitions = huge\_app.csv_

**Configuration of the behavior of the Node**

With the following parameters in the source code the behavior of the node can be controlled:

- _Temperature limits:_

In main.cpp:

// temperature sensor

#define TEMP\_IS\_HIGH\_LEVEL (40.0) // in degrees Celcius, used for temperature is high warning

#define TEMP\_IS\_TOO\_HIGH\_LEVEL (70.0) // in degrees Celcius, used to disable the compressor when temperature is too high

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

#define DISABLE\_COMPRESSOR\_AT\_LATE\_HOURS (true)

#define DISABLED\_TIME\_START (19) // in hour, time from which the compressor is not automatically switched on

#define DISABLED\_TIME\_END (8) // in hour, time to which the compressor is not automatically switched on

#define MAX\_WAIT\_TIME\_BUTTON\_ON\_PRESSED (10000) // in ms, time button on must be pressed to override late hour compressor disable

#define LED\_DISABLE\_DURATION (5000) // in ms, the time LED1 will flash if button on is pressed during late hour

#define LED\_DISABLE\_PERIOD (200) // in ms, the time LED1 will flash on/off

- _Clear EEProm and cache, by pressing but1 of the ESP32-PoE module during boot some time:_

In main.cpp:

#define MAX\_WAIT\_TIME\_BUTTON\_PRESSED (4000) // in ms

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

- _The time between log messages about the oil level:_

In OilLevelSensor.cpp:

#define OIL\_LEVEL\_LOG\_WINDOW (60000) // in ms

- _For calibration of the air pressure sensor, remove the comments before the define CALIBRATE\_PRESSURE:_

In PressureSensor.cpp:

// for calibrating the pressure sensor remove the comment statement in front of next #define

// #define CALIBRATE\_PRESSURE

Also see the following in the source code:

// for calibration

#ifdef CALIBRATE\_PRESSURE

Serial.print(&quot;Pressure ADC = &quot;);

Serial.print(pressureADCVal);

Serial.println(&quot; bits&quot;);

Serial.print(&quot;Pressure voltage = &quot;);

Serial.print(pressureVoltage);

Serial.println(&quot; V&quot;);

Serial.print(&quot;Pressure = &quot;);

Serial.print(pressure);

Serial.println(&quot; MPa&quot;);

#endif

// end calibration

See also the following parameters:

- Pressure calibrating values:

In PressureSensor.cpp:

#define PRESSURE\_CALIBRATE\_VALUE\_0\_5V (144) // in measured bits

#define PRESSURE\_CALIBRATE\_VALUE\_4\_5V (2600) // in measured bits

The air pressure sensor output is 0.5 V when the pressure is 0 MPa.

The output is 4.5 V when the pressure is 1.2 MPa

- _The time between samples of the air pressure:_

In PressureSensor.cpp:

#define PRESSURE\_SAMPLE\_WINDOW (1000) // in ms

- _The time between log messages of the air pressure:_

In PressureSensor.cpp:

#define PRESSURE\_LOG\_WINDOW (60000) // in ms

- _The time the temperature is too high, before an error is signaled:_

In OilLevelSensor.cpp:

#define MAX\_TEMP\_IS\_TOO\_HIGH\_WINDOW (10000) // in ms default 10000 = 10 seconds. Error is only signalled after this time window is passed
