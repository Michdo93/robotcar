#!/usr/bin/env python

import sys, tty, termios, os, readchar
import Adafruit_PCA9685

# Initialization of the servo controller object pwm
pwm = Adafruit_PCA9685.PCA9685()

# With a frequency of 60 HZ the RC model servo motors I use work 
# very well.
pwm.set_pwm_freq(60)

# Here the starting positions for the two servo motors are defined
servo_pwm = 400
servo_min = 300
servo_max = 500

#pwm.set_pwm(0, 0, servo_pwm)

# The variable interval defines the step size for the servo motors.
intervall = 5

# The menu that the user sees after starting the program explains 
# to him with which keys he has to control the Pan-Tilt-Kit.
print("a/d: turn left / right")
print("x:   Exit program")

def reset():
   pwm.set_pwm(0, 0, 400)

def getch():
   ch = readchar.readchar()
   return ch


# The menu for the user when running the program.
# The menu explains which buttons are used to control the 
# Pan-Tilt-Kit and shows the PWM value of each servo motor.
def printscreen():
   # The call os.system('clear') clears the screen.
   os.system('clear')
   print("a/d: turn left / right")
   print("x:   Exit program")
   print("=== PWM value of the servo motors ===")
   print("PWM value servo motor: ", servo_pwm)

while True:
    # The getch() functions reads the pressed key by the user and
   # stores the value in the variable char for futher processing
   char = getch()

   # The Pan-Tilt-Kit turns to the right.
   if(char == "d"):
      if servo_pwm > servo_min:
         servo_pwm = servo_pwm - intervall      
         pwm.set_pwm(0, 0, servo_pwm)
      printscreen()
      
   # The Pan-Tilt-Kit turns to the left.
   if(char == "a"):
      if servo_max > servo_pwm:
         servo_pwm = servo_pwm + intervall          
         pwm.set_pwm(0, 0, servo_pwm)
      printscreen()
      
   # By pressing the key "x" the endless loop for reading the 
   # keyboard inputs stopps and the program ends.
   if(char == "x"):
      reset()
      print("Exit program")
      break
   
   # The variable char is cleared after each loop run to be able to 
   # read the next key pressed by the user.
   char = ""