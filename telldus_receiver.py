#!/usr/bin/env python3
# Receive wireless messages from Weatherstation sensors
# and write pywws database.
# 433Mhz receiver is connected using arduino.
#

from __future__ import print_function
import sys
import serial
import time
import argparse
import logging

from datetime import datetime

sys.path.append("../pywws/src")
import pywws.storage
import pywws.process
import pywws.logger
import pywws.localisation
import pywws.regulartasks

logger = logging.getLogger(__name__)

parser = argparse.ArgumentParser()
parser.add_argument("-V", "--version", help="version", action="store_true")
parser.add_argument("-d", "--debug", help="debug", action="store_true")
parser.add_argument("-p", "--port", help="device, default /dev/ttyUSB0", default='/dev/ttyUSB0')
parser.add_argument("-W", "--initial-wait", type=float, help="Initial wait before first command", default='1.0')
parser.add_argument('--verbose', '-v', action='count', default=0)
parser.add_argument('args', nargs=argparse.REMAINDER, help="datadir")

def crc8(message, polynomial, init):
    crc = init
    for byte in message:
        crc ^= byte
        for bit in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ polynomial
            else:
                crc = (crc << 1)
        crc &= 0xff
    return crc

def reflect4(x):
    x = (x & 0xCC) >> 2 | (x & 0x33) << 2
    x = (x & 0xAA) >> 1 | (x & 0x55) << 1
    return x

def reflect_nibbles(message):
    b = bytearray()
    for byte in message:
        b.push(reflect4(byte))
    return b

TELLDUS_MSGLEN = 37
SWITCHDOCLABS_MSGLEN = 14

class WeatherReceiver:
  def __init__(self, serialclient = None, **kwargs):
    ''' Initialize a serial client instance
    '''
    self.debug = kwargs.get('debug', False)
    if serialclient == None:
        port = kwargs.get('port', '/dev/ttyUSB0')
        baudrate = kwargs.get('baudrate', 115200)
        self.ser = serial.Serial(port, baudrate, timeout=2.5)
        self.ser.flushInput()
    else:
        self.ser = serialclient
    time.sleep(kwargs.get("initial_wait")) # Opening serial port boots the Arduino

  # Return received message as a bit string "10111000100..."
  def recv(self):
    s = self.ser.readline()
    # print('<', repr(s))
    if s == b'':
        return
    s = s.decode('ISO8859-1')
    if self.debug:
        logger.debug('<%s', repr(s))
    s = s.strip()
    if s.startswith('Manchester receiver'):
        logger.info ("Version:%s", s)
        return ''
    toks = s.split(' ')
    if len(toks) != 3 or len(toks[0]) != 4 or len(toks[2]) != 2:
        logger.error ("error, invalid serial message: %s", repr(s))
        return ''
    bitcount = int(toks[0], 16)
    if len(toks[1]) * 4 != (bitcount + 7) & 0xfff8:
        logger.error ("error, invalid serial message length %d %d %d : %s", len(toks[1]) * 4, bitcount, (bitcount + 7) & 0xfff8, repr(s))
        return ''
    data = bytes.fromhex(toks[1])
    crc = crc8(data, 0x8c, 0)
    if crc != int(toks[2], 16):
        logger.error ("error, invalid serial message crc %d: %s", crc, repr(s))
        return ''
    #print('<', repr(data))
    bytes_as_bits = ''.join(format(byte, '08b') for byte in data)
    logger.debug('<%s', repr(bytes_as_bits[0:bitcount]))
    return bytes_as_bits[0:bitcount]

  def parse_switchdoclabs(self, b):
    myDevice = (b[0] & 0xf0)>>4
    if myDevice != 0x0c:
	    return 0 # not my device

    mySerial = (b[0]&0x0f)<<4 & (b[1] % 0xf0)>>4
    myFlags  = b[1] & 0x0f
    myBatteryLow = (myFlags & 0x08) >> 3
    myAveWindSpeed = b[2] | ((myFlags & 0x01)<<8)
    myGust         = b[3] | ((myFlags & 0x02)<<7)
    myWindDirection= b[4] | ((myFlags & 0x04)<<6)
    myCumulativeRain=(b[5]<<8) + b[6]
    mySecondFlags  = (b[7] & 0xf0)>>4
    myTemperature = ((b[7] & 0x0f)<<8) + b[8]
    myHumidity = b[9]
    myLight = (b[10]<<8) + b[11] + ((mySecondFlags & 0x08)<<13)
    myUV = b[12]
    myCRC = b[13]

    myTemperatureC = round((((myTemperature-400)/10)-32)/1.8, 1)
    myAveWindSpeed *= 0.1
    myGust *= 0.1
    myCumulativeRain *= 0.1

    if self.debug:
        print("myDevice", myDevice)
        print("mySerial", mySerial)
        print("myFlags", myFlags)
        print("myBatteryLow", myBatteryLow)
        print("myAveWindSpeed", myAveWindSpeed)
        print("myGust", myGust)
        print("myWindDirection", myWindDirection)
        print("myCumulativeRain", myCumulativeRain)
        print("mySecondFlags", mySecondFlags)
        print("myTemperature", myTemperature)
        print("myHumidity", myHumidity)
        print("myLight", myLight)
        print("myUV", myUV)
        print("myCRC", myCRC)
        print("myTemperatureC", myTemperatureC)

    if myLight == 0x1fffb:
        myLight = None
    if myUV == 0xfb:
        myUV = None

    data = {
        'delay': 0,
        'status': 0
    }
    data['hum_in'] = None
    data['temp_in'] = None
    data['hum_out'] = myHumidity
    data['temp_out'] = myTemperatureC
    data['abs_pressure'] = None
    data['wind_ave'] = myAveWindSpeed
    data['wind_gust'] = myGust
    data['wind_dir'] = int(myWindDirection / 22.5) % 16
    data['rain'] = myCumulativeRain
    data['illuminance'] = myLight
    data['uv'] = myUV
    if crc8(b, 0x31, 0xc0) != 0:
        logger.debug ("Invalid switchdoclabs CRC! %d != %d", crc8(b[0:13], 0x31, 0xc0), b[13])
        return None # Invalid CRC
    return data

  def parse_telldus(self, b):
    header = b[0] >> 4
    serial = ((b[0] & 0x0f) << 4) | ((b[1] & 0xf0) >> 4)
    flags = b[1] & 0x0f
    battery_low     = (flags & 0x8) >> 3                     # FIXME just a guess
    msb = b[2]
    wind_speed_avg = b[3] | ((msb & 0x01) << 8)
    gust           = b[4] | ((msb & 0x02) << 7)
    wind_direction = b[5] | ((msb & 0x04) << 6)
    unk6           = b[6] | ((msb & 0x08) << 5)   # FIXME wind speed ?
    unk7           = b[7] | ((msb & 0x10) << 4)   # FIXME wind direction ?
    unk8           = b[8] | ((msb & 0x20) << 3)   # FIXME wind speed ?
    unk9           = b[9] | ((msb & 0x40) << 2)   # FIXME wind direction ?
    rain_rate      = (b[10] << 8) | b[11] # FIXME Just a guess
    rain_1h        = (b[12] << 8) | b[13]
    rain_24h       = (b[14] << 8) | b[15]
    rain_week      = (b[16] << 8) | b[17]
    rain_month     = (b[18] << 8) | b[19]
    rain_total     = (b[20] << 8) | b[21]
    rain_total2    = (b[22] << 8) | b[23] # FIXME this or previous ?
    temperature    = ((b[24] & 0x0f) << 8) | b[25]
    humidity       = b[26]
    temp_indoor    = ((b[24] & 0xf0) << 4) | b[27]
    humidity_indoor = b[28]
    pressure_abs   = (b[29] << 8) | b[30]
    pressure_rel   = (b[31] << 8) | b[32]
    light          = (b[33] << 8) | b[34] # fffa FIXME there is one bit somewhere
    uv             = b[35]                # fa
    crc            = b[36]

    rain_rate *= 0.1
    rain_1h *= 0.1
    rain_24h *= 0.1
    rain_week *= 0.1
    rain_month *= 0.1
    pressure_abs *= 0.1
    temp_c = round((((temperature - 400) / 10) - 32) / 1.8, 1)
    temp_in_c = round((((temp_indoor - 400) / 10) - 32) / 1.8, 1)

    if humidity == 0xfb:
        humidity = None
    if temperature == 0x7fb:
        temp_c = None
    if wind_speed_avg == 0x1fb:
        wind_speed_avg = None
    if wind_direction == 0x1fb:
        wind_direction = None
    if rain_total == 0xfffb:
        rain_total = None
    else:
        rain_total *= 0.1
    if light == 0xfffb or light == 0xfffa:
        light = None
    if uv == 0xfa:
        uv = None

    if self.debug:
        print ("header = ", header)
        print ("serial = ", serial)
        print ("flags = ", flags)
        print ("battery_low  = ", battery_low)
        print ("wind_speed_avg = ", wind_speed_avg)
        print ("gust = ", gust)
        print ("wind_direction = ", wind_direction)
        print ("Unknown6 = ", unk6)
        print ("Unknown7 = ", unk7)
        print ("Unknown8 = ", unk8)
        print ("Unknown9 = ", unk9)
        print ("rain_rate = mm", rain_rate)
        print ("rain_1h = mm", rain_1h)
        print ("rain_24h = mm", rain_24h)
        print ("rain_week = mm", rain_week)
        print ("rain_month = mm", rain_month)
        print ("rain_total = mm", rain_total)
        print ("rain_total2 = ", rain_total2)
        print ("temperature = ", temperature)
        print ("humidity = ", humidity)
        print ("temp_in_c = ", temp_in_c)
        print ("humidity_indoor = ", humidity_indoor)
        print ("pressure_abs = ", pressure_abs)
        print ("pressure_rel = ", pressure_rel)
        print ("light = ", light)
        print ("uv = ", uv)
        print ("crc = ", crc)
        print ("temp_c = ", temp_c)

    data = {
        'delay': 0,
        'status': 0
    }
    data['hum_in'] = humidity_indoor
    data['temp_in'] = temp_in_c
    data['hum_out'] = humidity
    data['temp_out'] = temp_c
    data['abs_pressure'] = pressure_abs
    data['wind_ave'] = wind_speed_avg
    data['wind_gust'] = wind_speed_avg
    data['wind_dir'] = int(wind_direction / 22.5) % 16
    data['rain'] = rain_total
    data['illuminance'] = None
    data['uv'] = None

    if crc8(b, 0x31, 0xc0) != 0:
        logger.debug ("Invalid telldus CRC! %d != %d", crc8(b[0:36], 0x31, 0xc0), b[36])
        return None # Invalid CRC
    return data

  def parse_message(self, message):
    global indoor_data
    if message and "101001110" in message:
        idx = message.index("101001110")
        idx += 5
        if len(message) - idx  + 1 >= TELLDUS_MSGLEN * 8:
            s = message[idx: idx + TELLDUS_MSGLEN * 8]
            msg = int(s, 2).to_bytes(TELLDUS_MSGLEN, byteorder='big')
            logger.debug ("TELLDUS %s", bytes(msg).hex())
            data = self.parse_telldus(msg)
            if data is not None:
                indoor_data = data
                logger.info (data)
    if message and "101001100" in message:
        idx = message.index("101001100")
        idx += 5
        if len(message) - idx >= SWITCHDOCLABS_MSGLEN * 8:
            s = message[idx: idx + SWITCHDOCLABS_MSGLEN * 8 ]
            msg = int(s, 2).to_bytes(SWITCHDOCLABS_MSGLEN, byteorder='big')
            logger.debug ("SWITCHDOCLABS %s", bytes(msg).hex())
            data = self.parse_switchdoclabs(msg)

            if data is not None:
                logger.debug(data)
            if data is not None and indoor_data is not None:
                # FIXME expire indoor data after a while
                # Enrich the message from previous indoor sensor data
                if 'temp_in' in indoor_data:
                    data['temp_in'] = indoor_data['temp_in']
                if 'hum_in' in indoor_data:
                    data['hum_in'] = indoor_data['hum_in']
                if 'abs_pressure' in indoor_data:
                    data['abs_pressure'] = indoor_data['abs_pressure']
                if 'rain' in indoor_data:
                    data['rain'] = indoor_data['rain']
                return data
    return None

indoor_data = None

def main():
    opts = parser.parse_args()
    pywws.logger.setup_handler(opts.verbose)
    wr = WeatherReceiver(port = opts.port, debug = opts.debug, initial_wait = opts.initial_wait)
    if opts.version:
        # FIXME implement this.
        # Receive version string from ardiono.
        sys.exit()
    if len(opts.args) > 0:
        datadir = opts.args[0]
    else:
        datadir = 'weather_data'
    with pywws.storage.pywws_context(datadir, live_logging=True) as context:
        # localise application
        pywws.localisation.set_application_language(context.params)
        # create a RegularTasks object
        tasks = pywws.regulartasks.RegularTasks(context)
        if context.status.get('fixed', 'fixed block') is None:
            context.status.set('fixed', 'fixed block', {})

        while True:
            message = wr.recv()
            data = wr.parse_message(message)
            if data is None:
                continue
            # Store data
            d = datetime.utcnow().replace(microsecond=0)
            context.raw_data[d] = data
            context.raw_data.flush()
            logger.info(context.raw_data[d])
            context.status.set('data', 'last', data)

            # process new data
            try:
                logger.debug('context', context)
                pywws.process.process_data(context)
            except Exception as error:
                logger.exception(error)
            try:
                tasks.do_tasks()
            except Exception as error:
                logger.exception(error)
            try:
                tasks.do_live(data)
            except Exception as error:
                logger.exception(error)

if __name__ == "__main__":
    main()
