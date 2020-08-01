import sys
import spidev
import time

class MCP3008(object):

    def __init__(self, bus, device):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device) # bus = 0, device = 0
        self.spi.max_speed_hz = 1000000

    def readIRAdc(self, channel = 0):
        # analog to digital conversion
        # http://arduinomega.blogspot.com/2011/05/infrared-long-range-sensor-gift-of.html
        adc = self.spi.xfer2([1,(8 + channel) << 4, 0])
        data = ((adc[1] & 3) << 8) + adc[2]

        return data

    def close(self):
        self.spi.close()