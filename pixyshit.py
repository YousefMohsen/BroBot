#!/usr/bin/env python3

from ev3dev.ev3 import *
pixy = Sensor(address=INPUT_2)
#assert pixy.connected, "Error while connecting Pixy camera to port1"
pixy.mode = 'ALL'

sig = pixy.value(1)*256 + pixy.value(0) # Signature of largest object
x_centroid = pixy.value(2)    # X-centroid of largest SIG1-object
y_centroid = pixy.value(3)    # Y-centroid of largest SIG1-object
width = pixy.value(4)         # Width of the largest SIG1-object
height = pixy.value(5)        # Height of the largest SIG1-object
print("messi")
print(sig)