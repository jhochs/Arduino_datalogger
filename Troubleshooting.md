# Mote operation troubleshooting
**Wifi mote won't connect to the internet:**
- SSID and/or password are entered incorrectly/wrong network
- WiFi.begin(SSID) is being called when WiFI.begin(SSID,PASS) should be called when connecting to a network which requires a password
- Watchdog is activated but watchdog is not being reset in WIFININA > WiFi.cpp, prompting persistent timeout

**Cellular mote won't connect to the internet:**
- SIM card is not installed all the way
- Antenna not connected properly
- SIM card does not have an active data plan
- Local LTE band available is not supported by SARA R410M-02B (see datasheet)

**Wifi/cellular mote won't connect to AWS:**
- Wrong authentication certificate being used
- Certificate was not activated in AWS upon creation

**Mote connects but won't take measurements, hangs on setup:**
- Likely problem with the milliseconds counter; check RTC and interrupt on D0

**Data is streaming to MQTT topic but not entering DynamoDB database:**
- Rule is deactivated
- Rule is ill-defined (e.g. wrong table)
- Mote timestamp is wrong (e.g. local time not UTC) meaning the rule is ignoring each packet
