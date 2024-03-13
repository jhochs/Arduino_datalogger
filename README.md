# Arduino code for pressure/wind datalogger

This repository contains .ino files for data collection with three separate purposes:
1. `mote_` files collect pressure data from up to three BMP388 sensors on a mote at 12.5 Hz
2. `ref_mote_` files collect pressure data from two BMP388 sensors at 1 Hz
3. `wind_mote_` files collect wind data from a Gill WindSonic anemometer

For each category, there are multiple .ino files for different logging types:
1. `_MKR1500` is for use with the Arduino MKR 1500, to log data over the cellular network
2. `_MKR1010` is for use with the Arduino MKR 1010, to log data over Wifi
3. `_disconnected` is to log data to a SD card connected to the Arduino

Note that the `_2BMP` files provide code for acquiring from only two BMP388 sensors on one mote. The code can easily be altered to acquire from one BMP388 sensor.

For connected motes (cellular/wifi), the motes are configured to connect to AWS to check shadow state (which tells sensors whether to sleep or wake) and stream data to a DynamoDB table. For disconnected motes, the motes either wake when P_rms exceeds a threshold value (for `mote_`) or are constantly measuring (for `ref_mote_`, since these are plugged into power so consumption is not a concern).

## User changes
### Variables that must be set
| Variable | Mote connectivity type(s) | Description |
| -------- | ------------------------- | ----------- |
| HOSTNAME | All | Should be set to the name of the mote that is labeled on the Arudino + housing |
| TOPIC_OUTGOING | Connected | This is the AWS IoT topic to which measurements are streamed, mote name should match HOSTNAME |
| TOPIC_SHADOW_UPDATE | Connected | This is the AWS IoT topic to which the updated state is reported by the Arduino, mote name should match HOSTNAME |
| TOPIC_SHADOW_OUT | Connected | Arduino asks AWS to publish shadow state on this topic, mote name should match HOSTNAME |
| TOPIC_SHADOW_IN | Connected | Arduino asks AWS to publish shadow state on this topic, mote name should match HOSTNAME |
| CERT | Connected | X.509 certificate used by the Arduino to securely connect to AWS |
| BROKER | Connected | The AWS MQTT endpoint |
| SSID, PASS | Wifi | Network SSID and password |
| SD_CS | Disconnected | This is the pin to which the CS pin is connected |
| BMP_388 bmpA(CS) | All | The argument to the bmpA,B,C declarations is the CS pin to which each sensor is connected |

### Parameters that can be tuned
| Variable | Mote connectivity type(s) | Description |
| -------- | ------------------------- | ----------- |
| MS_DT | All | Number of milliseconds to add to the wait period, should be tuned to get to measurements as close to 12.5 Hz as possible |
| PRMS_THRESH | Disconnected | This is the P_rms threshold at which motes start acquiring data
| SLEEP_CHECK_PERIOD | All | When sleeping, how often the mote wakes to check whether to start acquiring data
| WAKE_CHECK_PERIOD | All | When awake, how often the mote checks whether to sleep


## Dependent libraries
The following Arduino libraries must be installed. The codes may work with later versions, but it is not guaranteed.

Since some changes were made to the libraries, the changed versions are all included in the [libraries/](libraries/) directory.

| Library | Version | Reason |
| ------- | ------- | ------ | 
| Adafruit SleepyDog | 1.6.3 | Enables watchdog timer to handle hang |
| ArduinoBearSSL | 1.7.3 | Enables SSL connection required for MQTT |
| ArduinoECCX08 | 1.3.7 | Enables use of the crypto chip for HTTPS connection |
| Arduino Low Power | 1.2.2 | Enables low power deep sleep, used for pressure motes |
| ArduinoMqttClient | 0.1.7 | Enables MQTT communication with AWS |
| BMP388_DEV | 1.0.3 | Enables communication with BMP388 sensors |
| RTClib | 1.12.4 | Enables communication with DS3231 |
| SDI-12 | 2.1.4 | For logging data from the anemometer |

## Utilities
Two types of code (with slightly different versions) are included in the [utilities/](utilities/) directory:
- `set_RTC` is used to set the DS3231 RTC in the lab, either using NB, GSM (3G network), Wifi, or manually (by entering the time to set into the .ino code)
- `three_BMP` streams measurements live from three BMP388 sensors to the command window
