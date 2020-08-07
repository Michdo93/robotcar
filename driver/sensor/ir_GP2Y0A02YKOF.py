import os
import sys
import getpass
import time

env=os.path.expanduser(os.path.expandvars('/home/' + getpass.getuser() + '/robotcar/driver'))
sys.path.insert(0, env)

from mcp3008 import MCP3008

class IRGP2Y0A02YKOF(object):

    def __init__(self, channel):
        self.mcp = MCP3008(0, 0)
        self.channel = channel

    def distance(self):
        read = self.mcp.readIRAdc(self.channel)

        if read > 0:
            # 10650.08 * x ^ (-0.935)
            distance = 10650.08 * pow(read,-0.935)
        else:
            distance = 0

        if distance >= 0:
            return distance
        else:
            return -1

    def speed(self):
        start_time = time.time()

        start_distance = self.distance() * 0.01     # to m conversion
        end_distance = self.distance() * 0.01       # to m conversion

        end_time = time.time()

        speed = (end_distance - start_distance) / (end_time - start_time)   # m/s

        return speed