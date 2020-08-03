import os
import sys
import tty
import termios
import readchar
import getpass

env=os.path.expanduser(os.path.expandvars('/home/' + getpass.getuser() + '/robotcar/config'))
sys.path.insert(0, env)

from servo_config import ServoConfig

env2=os.path.expanduser(os.path.expandvars('/home/' + getpass.getuser() + '/robotcar/driver/actor'))
sys.path.insert(1, env2)

from servo import ServoMotor

def getch():
   ch = readchar.readchar()
   return ch

def printscreen():
   # The call os.system('clear') clears the screen.
   os.system('clear')
   print("w/s: turn up / down")
   print("a/d: turn left / right")
   print("x:   Exit program")
   print("=== PWM value of the servo motors ===")
   print("PWM value pan servo: ", pan.get_current_pwm())
   print("PWM value tilt servo: ", tilt.get_current_pwm())

   #servo2 pan
   #servo1 tilt

if __name__ == "__main__":
	conf = ServoConfig()

	# channel, pwm_neutral, pwm_min, pwm_max, intervall
	mg996r = ServoMotor(0, conf.getNeutral('steering'), conf.getMin('steering'), conf.getMax('steering'), 5)

	pan = ServoMotor(4, conf.getNeutral('pan'), conf.getMin('pan'), conf.getMax('pan'), 5, -90.0, 90.0)
	tilt = ServoMotor(3, conf.getNeutral('tilt'), conf.getMin('tilt'), conf.getMax('tilt'), 5, -45.0, 45.0)

	print("w/s: turn up / down")
	print("a/d: turn left / right")
	print("x:   Exit program")

	while True:
		char = getch()

		# The Pan-Tilt-Kit turns up.
		if(char == "w"):
			if tilt.get_current_pwm() > tilt.get_servo_min():
				tilt.set_current_pwm(tilt.get_current_pwm() - tilt.get_intervall_pwm())
			printscreen()

		# The Pan-Tilt-Kit turns down.
		if(char == "s"):
			if tilt.get_servo_max() > tilt.get_current_pwm():
				tilt.set_current_pwm(tilt.get_current_pwm() + tilt.get_intervall_pwm())
			printscreen()

		# The Pan-Tilt-Kit turns to the right.
		if(char == "d"):
			if pan.get_current_pwm() > pan.get_servo_min():
				pan.set_current_pwm(pan.get_current_pwm() - pan.get_intervall_pwm())    
			printscreen()
			
		# The Pan-Tilt-Kit turns to the left.
		if(char == "a"):
			if pan.get_servo_max() > pan.get_current_pwm():
				pan.set_current_pwm(pan.get_current_pwm() + pan.get_intervall_pwm())
			printscreen()
			
		# By pressing the key "x" the endless loop for reading the 
		# keyboard inputs stopps and the program ends.
		if(char == "x"):
			pan.reset()
			tilt.reset()
			print("Exit program")
			break

		# The variable char is cleared after each loop run to be able to 
		# read the next key pressed by the user.
		char = ""