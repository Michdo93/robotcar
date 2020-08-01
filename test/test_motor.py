import os
import sys
import getpass

env=os.path.expanduser(os.path.expandvars('/home/' + getpass.getuser() + '/robotcar'))
sys.path.insert(0, env)

from servo_config import ServoConfig

env2=os.path.expanduser(os.path.expandvars('/home/' + getpass.getuser() + '/robotcar/driver/actor'))
sys.path.insert(1, env2)

from servo import ServoMotor

if __name__ == "__main__":
	conf = ServoConfig()

	# channel, pwm_neutral, pwm_min, pwm_max, intervall
	mg996r = ServoMotor(0, conf.getNeutral('steering'), conf.getMin('steering'), conf.getMax('steering'), 5)

	print("a/d: turn left / right")
	print("x:   Exit program")

	mg996r.test_servo()
