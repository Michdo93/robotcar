import os
import sys
from sense_hat import SenseHat

sense = SenseHat()

humidity = sense.get_humidity()
print("Humidity: %s %%rH" % humidity)

temp = sense.get_temperature()
print("Temperature: %s C" % temp)

temp = sense.get_temperature_from_humidity()
print("Temperature: %s C" % temp)

temp = sense.get_temperature_from_pressure()
print("Temperature: %s C" % temp)

pressure = sense.get_pressure()
print("Pressure: %s Millibars" % pressure)

# sense.set_imu_config(False, True, False)  # gyroscope only

orientation_rad = sense.get_orientation_radians()
print("p: {pitch}, r: {roll}, y: {yaw}".format(**orientation_rad))

orientation = sense.get_orientation_degrees()
print("p: {pitch}, r: {roll}, y: {yaw}".format(**orientation))

orientation = sense.get_orientation()   # degrees
print("p: {pitch}, r: {roll}, y: {yaw}".format(**orientation))

north = sense.get_compass()
print("North: %s" % north)

raw = sense.get_compass_raw()
print("x: {x}, y: {y}, z: {z}".format(**raw))

gyro_only = sense.get_gyroscope()
print("p: {pitch}, r: {roll}, y: {yaw}".format(**gyro_only))

raw = sense.get_gyroscope_raw()
print("x: {x}, y: {y}, z: {z}".format(**raw))

accel_only = sense.get_accelerometer()
print("p: {pitch}, r: {roll}, y: {yaw}".format(**accel_only))

raw = sense.get_accelerometer_raw()
print("x: {x}, y: {y}, z: {z}".format(**raw))

