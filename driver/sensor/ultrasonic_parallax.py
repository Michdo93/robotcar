import sys
import RPi.GPIO as GPIO
import time

class UltrasonicParallax(object):
    timeout = 0.05

    def __init__(self, pin):
        self.pin = pin

        GPIO.setmode(GPIO.BCM)

    def distance(self):
        pulse_end = 0
        pulse_start = 0

        GPIO.setup(self.pin, GPIO.OUT) # Set to low
        GPIO.output(self.pin, False)

        time.sleep(0.01)

        GPIO.output(self.pin, True) # Set high

        time.sleep(0.00001)

        GPIO.output(self.pin, False) # Set low
        GPIO.setup(self.pin, GPIO.IN) # Set to input

        timeout_start = time.time()

        # Count microseconds that SIG was high
        while GPIO.input(self.pin) == 0:
            pulse_start = time.time()

            if pulse_start - timeout_start > self.timeout:
                return -1

        while GPIO.input(self.pin) == 1:
            pulse_end = time.time()
        
            if pulse_start - timeout_start > self.timeout:
                return -1

        if pulse_start != 0 and pulse_end != 0:
            pulse_duration = pulse_end - pulse_start
            # The speed of sound is 340 m/s or 29 microseconds per centimeter.
            # The ping travels out and back, so to find the distance of the
            # object we take half of the distance travelled.
            # distance = duration / 29 / 2
            distance = pulse_duration * 100 * 343.0 / 2
            distance = int(distance)

            #print('start = %s'%pulse_start,)
            #print('end = %s'%pulse_end)
            if distance >= 0:
                return distance
            else:
                return -1
        else :
            #print('start = %s'%pulse_start,)
            #print('end = %s'%pulse_end)
            return -1

    def get_distance(self, mount = 5):
        sum = 0

        for i in range(mount):
            a = self.distance()
            sum += a

        return int(sum/mount)

    def less_than(self, alarm_gate):
        dis = self.get_distance()
        status = 0

        if dis >= 0 and dis <= alarm_gate:
            status = 1
        elif dis > alarm_gate:
            status = 0
        else:
            status = -1

        return status

    def speed(self):
        start_time = time.time()

        start_distance = self.distance() * 0.01     # to m conversion
        end_distance = self.distance() * 0.01       # to m conversion

        end_time = time.time()

        speed = (end_distance - start_distance) / (end_time - start_time)   # m/s

        return speed
        