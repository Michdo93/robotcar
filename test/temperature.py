import sys
from sense_hat import SenseHat

sense = SenseHat()

temp = sense.get_temperature()
tempFeucht = sense.get_temperature_from_humidity()
tempDruck = sense.get_temperature_from_pressure()



print("Temperatur Temperatur-Sensor	:	%2.1f °C " %temp)
print("Temperatur Feuchtigkeitssensor	:	%2.1f °C " %tempFeucht)
print("Temperatur Drucksensor		:	%2.1f °C " %tempDruck)
