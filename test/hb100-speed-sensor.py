import sys
import os
import RPi.GPIO as GPIO
import time

#### limpkin.fr

class HB100(object):

    def __init__(self, freq_out, average_examples = 4):
        self.FREQ_OUT = freq_out
        self.AVERAGE = average_examples

        self.doppler_div = 19
        self.samples = [None] * self.AVERAGE # [None, None, None, None]

        self.x = None

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.FREQ_OUT, GPIO.IN)

    def __noInterrupts(self):
        return None

    def __interrupts(self):
        return None

    def __pulseIn(self, pin_number, pegel_level, timeout = 0):
        GPIO.setup(pin_number, GPIO.OUT)

        # if value is HIGH, pulseIn() waits for the pin to go HIGH, starts timing, then waits for the pin to go LOW and stops timing
        if pegel_level == GPIO.HIGH:   
            GPIO.output(pin_number, False)
            
            # Set to input
            GPIO.setup(pin_number, GPIO.IN)

            # Count microseconds
            while GPIO.input(pin_number) == GPIO.HIGH:
                starttime = time.time()

            while GPIO.input(pin_number) == GPIO.LOW:
                endtime = time.time()
        elif pegel_level == GPIO.LOW:
            GPIO.output(pin_number, True)

            # Set to input
            GPIO.setup(pin_number, GPIO.IN)

            # Count microseconds
            while GPIO.input(pin_number) == GPIO.LOW:
                starttime = time.time()

            while GPIO.input(pin_number) == GPIO.HIGH:
                endtime = time.time()

        duration = endtime - starttime

        return duration

    def interruptCallback(self):
        self.__pulseIn(self.FREQ_OUT, GPIO.HIGH)

        pulse_length = 0

        for x in range(0, self.AVERAGE):
            pulse_length = self.__pulseIn(self.FREQ_OUT, GPIO.HIGH)
            pulse_length += self.__pulseIn(self.FREQ_OUT, GPIO.LOW)
            self.samples[x] = pulse_length

    def measureSpeed(self):
        # self.__noInterrupts()

        # stuff inside interruptCallback
        GPIO.wait_for_edge(self.FREQ_OUT, GPIO.RISING)

        # self.__interrupts()

        samples_ok = True

        nbPulsesTime = self.samples[0]

        for y in range(1, self.AVERAGE):
            nbPulsesTime += self.samples[y]

            if (self.samples[y] > self.samples[0] * 2) or (self.samples[y] < self.samples[0] / 2):
                samples_ok = False

        if samples_ok:
            Ttime = nbPulsesTime / self.AVERAGE
            Freq = 1000000 / Ttime

            kmh = Freq / self.doppler_div
        else:
            kmh = None

        return kmh