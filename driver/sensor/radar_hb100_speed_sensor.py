import sys
import os
import RPi.GPIO as GPIO
import time

#### limpkin.fr

class HB100(object):

    def __init__(self, freq_out, average_examples = 4):
        self.FREQ_OUT = freq_out
        self.AVERAGE = average_examples

        # 9.35 GHz Germany
        # 9.9 GHz France, Italy
        # 10.525 GHz USA, Belgium, Netherlands
        # 10.587 GHz UK Outdoor applications
        # 10.687 GHz UK Indoor applications

        #9.35 GHz 17.31V 27.85V
        #9.9 GHz 18.33V 29.49V
        #10.525 GHz 19.49V 31.36V
        #10.587 GHz 19.60V 31.54V
        #10.687 GHz 19.79V 31.84V
        #24.125 GHz 44.68V 71.89V

        self.doppler_div = 19.49 # 19.49 Hz to KMH according to sensor datasheet
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

                kmh = Freq / self.doppler_div
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

        speed = frequency / self.doppler_div    

        return speed