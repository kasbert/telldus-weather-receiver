# telldus-weather-receiver
Receive and store Telldus weather station radio messages

This project is still work-in-progress.

Telldus weather station consists of
 * Outdoor unit, which is like SwitchDoc Labs WeatherRack2 without light and UV sensors
 * Indoor unit Telldus-FT0385R with pressure, humidity and temperature sensor and a display

Outdoor unit sends Manchester encoded message with 433Mhz radio OOK.

As Telldus weather station indoor unit receives a message from the outdoor unit,
 it sends 3 radio messages
 * Oregon-WGR800
 * Oregon-THGR810 or Oregon-PCR800
 * Telldus-FT0385R

 This program receives messages and writes weather data to pywws database (CSV files).

 Requires
 * Arduino Nano
 * 433Mhz Receiver
 * Arduino IDE for compiling and installing "telldus-weather-receiver.ino"
 * Python3
    * sudo pip install pywws

Create a data dir with weather.ini (see https://pywws.readthedocs.io/en/latest/guides/weather_ini.html)

See also
- https://github.com/merbanan/rtl_433/ has a good set of decoders, if you own RTL-SDR USB stick
- Tellstick
- https://github.com/pimatic/RFControl
- https://github.com/roccomuso/iot-433mhz/
