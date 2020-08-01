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
        # 10650.08 * x ^ (-0.935)
        distance = 10650.08 * pow(self.mcp.readIRAdc(self.channel),-0.935)

        return distance

    def speed(self):
        start_time = time.time()

        start_distance = self.distance() * 0.01     # to m conversion
        end_distance = self.distance() * 0.01       # to m conversion

        end_time = time.time()

        speed = (end_distance - start_distance) / (end_time - start_time)   # m/s

        return speed