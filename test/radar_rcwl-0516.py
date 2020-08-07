import os
import sys
import time
import RPi.GPIO as GPIO

PIN_RADAR = 4 # #define PIN_RADAR 2

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_RADAR, GPIO.IN)

try:
	# void loop()
	while True:
		NUM_CYCLES = 2

		start = time.time()

		for impulse_count in range(NUM_CYCLES):
			GPIO.wait_for_edge(PIN_RADAR, GPIO.FALLING)
			
		duration = time.time() - start      # seconds to run for loop
		
		frequency = NUM_CYCLES / duration   # in Hz

        #speed = frequency / float(31.36)   # 31.36 Hz to MPH according to sensor datasheet
		speed = frequency / float(19.49)    # 19.49 Hz to KMH according to sensor datasheet

		print(speed)
		
except KeyboardInterrupt: # press Ctrl + C to stop
	GPIO.cleanup()
