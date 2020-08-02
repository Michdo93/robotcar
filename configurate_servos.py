#!/usr/bin/env python
import os
import sys

from config.servo_config import ServoConfig

# Main function.
if __name__ == "__main__":
    conf = ServoConfig()

    conf.setServo('steering')
    conf.setServo('pan')
    conf.setServo('tilt')

    conf.writeConifgFile()