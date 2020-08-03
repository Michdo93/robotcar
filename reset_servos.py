#!/usr/bin/env python
import os
import sys
import getpass

from config.servo_config import ServoConfig

env=os.path.expanduser(os.path.expandvars('/home/' + getpass.getuser() + '/robotcar/driver/actor'))
sys.path.insert(0, env)

from servo import ServoMotor

# Main function.
if __name__ == "__main__":
    conf = ServoConfig()

    steering = ServoMotor(0, conf.getNeutral('steering'), conf.getMin('steering'), conf.getMax('steering'), 5)
    pan = ServoMotor(4, conf.getNeutral('pan'), conf.getMin('pan'), conf.getMax('pan'), 5)
    tilt = ServoMotor(3, conf.getNeutral('tilt'), conf.getMin('tilt'), conf.getMax('tilt'), 5)

    steering.reset()
    pan.reset()
    tilt.reset()    