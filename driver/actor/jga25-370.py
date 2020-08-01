#!/usr/bin/env python
import RPi.GPIO as GPIO

class JGA25_370(object):

    def __init__(self, ena, in1, in2, dc_max = 100):
        self.ENA = ena
        self.IN1 = in1
        self.IN2 = in2

        self.DC_MAX = dc_max

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.ENA, GPIO.OUT)
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)

        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)

        self.motorpwm = GPIO.PWM(self.ENA, 100)

        self.dutycycle = 0
        self.pwm = 0

        self.motorpwm.start(self.pwm)
        self.motorpwm.ChangeDutyCycle(self.dutycycle)

        self.rate = 0
        self.direction = "stopped"

    def getMotorDirection(self):
        return self.direction

    def getRate(self):
        return self.rate

    def getDCMax(self):
        return self.DC_MAX

    def getDutyCycle(self):
        return self.dutycycle

    def getMotorPWM(self):
        return self.pwm
    
    def setMotorDirection(self, direction):
        self.direction = direction

        if self.direction == "backward":
            GPIO.output(self.IN1, GPIO.HIGH)
            GPIO.output(self.IN2, GPIO.LOW)
        elif self.direction == "forward":
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.HIGH)
        else:
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.LOW)

    def setMotorRate(self, rate):
        self.rate = int(rate)

        if self.rate < 0:
            # reverse mode for the motor
            self.setMotorDirection("backward")
            self.pwm = -int(self.DC_MAX * self.rate)
            if self.pwm > self.DC_MAX:
                self.pwm = self.DC_MAX
        elif self.rate > 0:
            # forward mode for the motor
            self.setMotorDirection("forward")
            self.pwm = int(self.DC_MAX * self.rate)
            if self.pwm > self.DC_MAX:
                self.pwm = self.DC_MAX
        else:
            # the motor is stopped
            self.setMotorDirection("stopped")
            self.pwm = 0

        self.dutycycle = self.pwm
        self.motorpwm.ChangeDutyCycle(self.pwm)

    def engineStop(self):
        self.setMotorDirection("stopped")
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)

        GPIO.cleanup()