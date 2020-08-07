#!/usr/bin/env python
import os
import sys
import re
import socket
import getpass
import time

env=os.path.expanduser(os.path.expandvars('/home/' + getpass.getuser() + '/robotcar/driver/sensor'))
sys.path.insert(0, env)

from ultrasonic_hc_sr04 import UltrasonicHCSR04


ultrasonicFrontLeft = UltrasonicHCSR04(23, 24)
ultrasonicFrontRight = UltrasonicHCSR04(22, 5)
ultrasonicRearLeft = UltrasonicHCSR04(19, 26)
ultrasonicRearRight = UltrasonicHCSR04(16, 20)

while True:
	#print("front-left: %s" % ultrasonicFrontLeft.distance())
	#print("front-right: %s" % ultrasonicFrontRight.distance())
	#print("rear-left: %s" % ultrasonicRearLeft.distance())
	#print("rear-right: %s" % ultrasonicRearRight.distance())

	print("front-left: %s" % ultrasonicFrontLeft.speed())
	print("front-right: %s" % ultrasonicFrontRight.speed())
	print("rear-left: %s" % ultrasonicRearLeft.speed())
	print("rear-right: %s" % ultrasonicRearRight.speed())

	time.sleep(0.5)