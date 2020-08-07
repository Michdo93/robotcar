import sys
import os
import RPi.GPIO as GPIO
import time

#### limpkin.fr

class IPM165(object):

    def __init__(self, freq_out, average_examples = 8):
        self.FREQ_OUT = freq_out
        self.AVERAGE = average_examples

        # 71.0 Hz to MPH according to sensor datasheet
        self.doppler_div = 44.0 # 44.0 Hz to KMH according to sensor datasheet
        self.samples = [None] * self.AVERAGE # [None, None, None, None]

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.FREQ_OUT, GPIO.IN)

    def __pulseIn(self, pin_number, value, timeout = 1):
        GPIO.setup(pin_number, GPIO.OUT)

        # if value is HIGH, pulseIn() waits for the pin to go HIGH, starts timing, then waits for the pin to go LOW and stops timing
        if value == GPIO.HIGH:   
            GPIO.output(pin_number, False)
        elif value == GPIO.LOW:
            GPIO.output(pin_number, True)
            
        # Set to input
        GPIO.setup(pin_number, GPIO.IN)

        # Count microseconds
        starttime = time.time()

        while GPIO.input(pin_number) == value:
            if time.time() - starttime > 1:
                return 0

        starttime = time.time()

        while GPIO.input(pin_number) != value:
            if time.time() - starttime > 1:
                return 0

        starttime = time.time()

        while GPIO.input(pin_number) == value:
            if time.time() - starttime > 1:
                return 0

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
        channel = GPIO.wait_for_edge(self.FREQ_OUT, GPIO.FALLING)

        # self.__interrupts()

        if channel:
            self.interruptCallback()

            samples_ok = True

            nbPulsesTime = self.samples[0]

            for y in range(1, self.AVERAGE):
                nbPulsesTime += self.samples[y]

                if (self.samples[y] > self.samples[0] * 2) or (self.samples[y] < self.samples[0] / 2):
                    samples_ok = False

            if samples_ok:
                Ttime = nbPulsesTime / self.AVERAGE
                Freq = 1 / Ttime

                #print("Ttime: %s" % Ttime)
                #print("Freq: %s" % Freq)

                kmh = (Freq*1e6) / self.doppler_div
            else:
                kmh = None

            return kmh

    def measureSpeedAlternative(self):
        NUM_CYCLES = self.AVERAGE
        #GPIO.wait_for_edge(self.FREQ_OUT, GPIO.FALLING)
        
        start = time.time()
        
        for impulse_count in range(NUM_CYCLES):
            GPIO.wait_for_edge(self.FREQ_OUT, GPIO.FALLING)

        duration = time.time() - start      # seconds to run for loop

        frequency = NUM_CYCLES / duration   # in Hz

        speed = (frequency*1e6) / self.doppler_div    

        return speed