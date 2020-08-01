#!/usr/bin/env python

import sys
import tty
import termios
import os
import getpass
import readchar
import Adafruit_PCA9685

class ServoMotor(object):

    def __init__(self, channel, pwm_neutral, pwm_min, pwm_max, intervall = 5, min_degree = None, max_degree = None):
        self.channel = channel
        self.min_degree = min_degree
        self.max_degree = max_degree

        # Initialization of the servo controller object pwm
        self.pwm = Adafruit_PCA9685.PCA9685()

        # With a frequency of 60 HZ the RC model servo motors I use work 
        # very well.
        self.pwm.set_pwm_freq(60)

        # Here the starting positions for the two servo motors are defined
        self.servo_neutral = pwm_neutral
        self.servo_min = pwm_min
        self.servo_max = pwm_max

        self.servo_pwm = self.servo_neutral # start with neutral postion

        # The variable interval defines the step size for the servo motors.
        self.intervall = intervall

    def reset(self):
        self.pwm.set_pwm(self.channel, 0, self.servo_neutral)

    def get_intervall_pwm(self):
        return self.intervall

    def set_intervall_pwm(self, intervall):
        self.intervall = intervall

    def get_intervall_degree(self):
        if (self.get_servo_min_degree() and self.get_servo_max_degree()) != None:
            intervall = self.get_servo_range_pwm() / self.get_servo_range_degree()
            return intervall
        else:
            return None

    def get_channel(self):
        return self.channel

    def set_channel(self, channel):
        self.channel = channel

    def get_servo_min_degree(self):
        return self.min_degree

    def set_servo_min_degree(self, min_degree):
        self.min_degree = min_degree

    def get_servo_max_degree(self):
        return self.max_degree

    def set_servo_max_degree(self, max_degree):
        self.max_degree = max_degree

    def get_servo_min(self):
        return self.servo_min

    def get_servo_max(self):
        return self.servo_max

    def get_servo_neutral(self):
        return self.servo_neutral

    def get_servo_range_pwm(self):
        servo_range = self.get_servo_max() - self.get_servo_min()
        return servo_range

    def get_servo_range_degree(self):
        if (self.get_servo_min_degree() and self.get_servo_max_degree()) != None:
            servo_range = self.get_servo_max_degree() + self.get_servo_min_degree()
            return servo_range
        else:
            return None

    def set_servo_pwm(self, pwm):
        if pwm > self.get_servo_max():
            self.servo_pwm = self.get_servo_max()
        elif pwm < self.get_servo_min():
            self.servo_pwm = self.get_servo_min()
        else:
            self.servo_pwm = pwm
        self.pwm.set_pwm(self.channel, 0, self.servo_pwm)

    def get_servo_pwm(self):
        return self.servo_pwm

    # alias get_servo_pwm
    def get_current_pwm(self):
        return self.get_servo_pwm()

    # alias set_servo_pwm
    def set_current_pwm(self, pwm):
        self.set_servo_pwm(pwm)

    # alias get_servo_degree
    def get_current_degree(self):
        if (self.get_servo_min_degree() and self.get_servo_max_degree()) != None:
            return self.get_servo_degree()
        else:
            return None

    # alias get_servo_degree
    def set_current_degree(self, degree):
        self.set_servo_degree(degree)

    def get_servo_degree(self):
        if (self.get_servo_min_degree() and self.get_servo_max_degree()) != None:
            servo_range_pwm = self.get_servo_range_pwm()
            servo_range_degree = self.get_servo_range_degree()
            servo_neutral = self.get_servo_neutral()
            servo_pwm = self.get_servo_pwm()

            degree = (servo_pwm - servo_neutral) / (servo_range_pwm / servo_range_degree)

            return degree
        else:
            return None

    def set_servo_degree(self, degree):
        if (self.get_servo_min_degree() and self.get_servo_max_degree()) != None:
            servo_range_pwm = self.get_servo_range_pwm()
            servo_range_degree = self.get_servo_range_degree()
            servo_neutral = self.get_servo_neutral()

            servo_pwm = servo_neutral + ((servo_range_pwm * degree) / servo_range_degree)
            self.set_servo_pwm(servo_pwm)

    def __getch(self):
        ch = readchar.readchar()
        return ch

    # The menu for the user when running the program.
    # The menu explains which buttons are used to control the 
    # Pan-Tilt-Kit and shows the PWM value of each servo motor.
    def __printscreen(self):
        # The call os.system('clear') clears the screen.
        os.system('clear')
        print("a/d: turn left / right")
        print("x:   Exit program")
        print("=== PWM value of the servo motors ===")
        print("PWM value servo motor: ", self.get_current_pwm())
        print("Degree value servo motor: ", self.get_current_degree())

    def test_servo(self):
        while True:
            # The getch() functions reads the pressed key by the user and
            # stores the value in the variable char for futher processing
            char = self.__getch()

            # The Pan-Tilt-Kit turns to the right.
            if(char == "d"):
                if self.get_current_pwm() > self.get_servo_min():
                    self.set_current_pwm((self.get_current_pwm() - self.get_intervall_pwm()))
                self.__printscreen()
            
            # The Pan-Tilt-Kit turns to the left.
            if(char == "a"):
                if self.get_servo_max() > self.get_current_pwm():
                    self.set_current_pwm((self.get_current_pwm() + self.get_intervall_pwm()))
                self.__printscreen()
            
            # By pressing the key "x" the endless loop for reading the 
            # keyboard inputs stopps and the program ends.
            if(char == "x"):
                self.reset()
                print("Exit program")
                break

            # The variable char is cleared after each loop run to be able to 
            # read the next key pressed by the user.
            char = ""
            
#if __name__ == "__main__":
#    # channel, pwm_neutral, pwm_min, pwm_max, intervall
#    mg996r = ServoMotor(0, 400, 300, 500, 5)

#    # The menu that the user sees after starting the program explains 
#    # to him with which keys he has to control the Pan-Tilt-Kit.
#    print("a/d: turn left / right")
#    print("x:   Exit program")

#    mg996r.test_servo()