#!/usr/bin/env python
import configparser
import io
import os
import getpass
import sys
import tty
import termios
import readchar
import shutil

env=os.path.expanduser(os.path.expandvars('/home/' + getpass.getuser() + '/robotcar/driver/actor'))
sys.path.insert(0, env)

from servo import ServoMotor

class ServoConfig:

    def __init__(self):
        #make a copy of the invoice to work with
        src = '/home/' + getpass.getuser() + '/robotcar/config/servo_config.ini'
        dst = '/home/' + getpass.getuser() + '/robotcar/config/servo_config.bak.ini'
        shutil.copy(src, dst)

        self.config = configparser.ConfigParser()
        self.config.read(src)
        self.config.sections()

    # List all contents
    def showSections(self):
        print(self.config.sections())

    def getServo(self, servoName):
        return self.config[servoName]

    def getNeutral(self, servoName):
        return int(self.config[servoName]['neutral'])

    def getMin(self, servoName):
        return int(self.config[servoName]['min'])
    
    def getMax(self, servoName):
        return int(self.config[servoName]['max'])

    def setNeutral(self, servoName, value):
        if value % 5 == 0:
            self.config[servoName]['neutral'] = str(value)
        else:
            self.config[servoName]['neutral'] = str(5 * round(value/5))

    def setMin(self, servoName, value):
        if value % 5 == 0:
            self.config[servoName]['min'] = str(value)
        else:
            self.config[servoName]['min'] = str(5 * round(value/5))
    
    def setMax(self, servoName, value):
        if value % 5 == 0:
            self.config[servoName]['max'] = str(value)
        else:
            self.config[servoName]['max'] = str(5 * round(value/5))

    def setServoValue(self, servoName, valueName, value):
        if value % 5 == 0:
            self.config[servoName][valueName] = str(value)
        else:
            self.config[servoName][valueName] = str(5 * round(value/5))

    def __getch(self):
        ch = readchar.readchar()
        return ch

    def __printScreen(self, charMin, charMax, turn, servoName, current, servo_pwm):
        # The call os.system('clear') clears the screen.
        os.system('clear')
        print(charMin, "/", charMax, ": ", turn)
        print("x: finish " , servoName, " ", current, " configuration")
        print("=== PWM value of the servo motor " , servoName, " ===")
        print("PWM value servo motor: ", servo_pwm)

    def setServo(self, servoName):
        servo_neutral = self.getNeutral(servoName)
        servo_min = self.getMin(servoName)
        servo_max = self.getMax(servoName)

        servo = self.getServo(servoName)

        servo_pwm = servo_neutral

        intervall = 5
        
        if servoName == "steering":
            charMax = "d"
            charMin = "a"
            turn = "turn right / left"
            servoChannel = 0
        elif servoName == "pan":
            charMax = "a"
            charMin = "d"
            turn = "turn left / right"
            servoChannel = 4
        elif servoName == "tilt":
            charMax = "s"
            charMin = "w"
            turn = "turn down / up"
            servoChannel = 3

        # channel, pwm_neutral, pwm_min, pwm_max, intervall
        servoMotor = ServoMotor(servoChannel, servo_neutral, servo_min, servo_max, intervall)
        servoMotor.set_current_pwm(servo_neutral)

        for current in servo:
            if(current == 'neutral'):
                servo_pwm = servo_neutral

            elif(current == 'min'):
                servo_pwm = servo_min

            elif(current == 'max'):
                servo_pwm = servo_max

            self.__printScreen(charMin, charMax, turn, servoName, current, servo_pwm)

            # pwm.set_pwm(servoChannel, 0, servo_pwm)
            servoMotor.set_current_pwm(servo_pwm)

            while True:
                char = self.__getch()    

                if(char == charMin):
                    servo_pwm = servo_pwm - intervall
                    servoMotor.set_current_pwm(servo_pwm)

                    self.__printScreen(charMin, charMax, turn, servoName, current, servo_pwm)

                if(char == charMax):
                    servo_pwm = servo_pwm + intervall
                    servoMotor.set_current_pwm(servo_pwm)

                    self.__printScreen(charMin, charMax, turn, servoName, current, servo_pwm)

                if(char == "x"):
                    print("x pressed")
                    self.setServoValue(servoName, current, servo_pwm)
                    self.writeConifgFile()
                    servoMotor.set_current_pwm(servo_neutral)
                    break


        print("Exit program...")

        char = ""
    
    def writeConifgFile(self):
        with open('/home/' + getpass.getuser() + '/robotcar/config/servo_config.ini', 'w') as configfile:
            self.config.write(configfile)