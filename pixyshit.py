#!/usr/bin/env python3


from ev3dev.ev3 import *
pixy = Sensor(address=INPUT_1)
assert pixy.connected, "Error while connecting Pixy camera to port1"

pixy = Sensor(address = INPUT_1)
#assert pixy.connected, "Error while connecting Pixy camera"



#from ev3dev2.sensor import INPUT_2
#from ev3dev2.sensor import Sensor
#from ev3dev2 import *

inf = Sensor(address=INPUT_2)
assert inf.address, "Error while connecting Infraredto port2"
#pixy.mode = 'SIG1'
sound = Sound()
#sound.beep()

print("messi")
print(inf.address)
count = pixy.value(0)  # The number of objects that match signature 1
#x = pixy.value(1)      # X-centroid of the largest SIG1-object
#y = pixy.value(2)      # Y-centroid of the largest SIG1-object
#w = pixy.value(3)      # Width of the largest SIG1-object
#h = pixy.value(4)      # Height of the largest SIG1-object