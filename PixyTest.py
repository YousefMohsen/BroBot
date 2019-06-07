#!/usr/bin/env python3

from ev3dev.ev3 import *
from smbus import SMBus
sound = Sound()
sound.beep()
sound.speak("broooooo")

#sound.speak("David, you are a douchebag")


# get a Port object to control the input port
in1 = LegoPort(address=INPUT_1)
# force the port into i2c-other mode so that the default driver is not automatically loaded
in1.mode = 'other-i2c'

# might need a short delay here (e.g. time.sleep(0.5)) to wait for the I2C port to be setup
# note: this delay should only be needed in ev3dev-jessie (4.4.x kernel), not ev3dev-stretch (4.9.x kernel)


bus = SMBus(3)  # bus number is input port number + 2
I2C_ADDRESS = 0x01  # the default I2C address of the sensor
while True:
    # read the number of object detected
  # num = bus.read_byte_data(I2C_ADDRESS, 0x42)
   # if num:
      #  print(bus.read_i2c_block_data(I2C_ADDRESS, 0x42))
     #   print("NUM",num)
     #   print("printing x in nums")
        #sound.beep()
      #  x1 = bus.read_byte_data(I2C_ADDRESS, 0x42)
      #  y1 = bus.read_byte_data(I2C_ADDRESS, 0x07)
      #  x2 = bus.read_byte_data(I2C_ADDRESS, 0x5E)
      #  y2 = bus.read_byte_data(I2C_ADDRESS, 0x5A)

        ignore1, x, y, width, height,ignore2, ignore3 = bus.read_i2c_block_data(I2C_ADDRESS, 0x51, 7)
        # do stuff with coordinates
        print("x", x)
        print("y", y)
        print("width", width)
        print("height", height)
        print("-----------\n")
        time.sleep(2)
      

        #time.sleep(1)
    # for x in range(150):
     #    res = bus.read_byte_data(I2C_ADDRESS, int(hex(x),16  )) 
      #   print(x,"res: ",res)
       #  time.sleep(1)


#inf = Sensor(address=INPUT_2)
#assert inf.address, "Error while connecting Infraredto port2"


#pixy = Sensor(address = INPUT_1)
#assert pixy.connected, "Error while connecting Pixy camera"


#from ev3dev2.sensor import INPUT_2
#from ev3dev2.sensor import Sensor
#from ev3dev2 import *


#pixy.mode = 'SIG1'
#sound = Sound()
# sound.beep()

# print("messi")
# count = pixy.value(0)  # The number of objects that match signature 1
# x = pixy.value(1)      # X-centroid of the largest SIG1-object
# y = pixy.value(2)      # Y-centroid of the largest SIG1-object
# w = pixy.value(3)      # Width of the largest SIG1-object
# h = pixy.value(4)      # Height of the largest SIG1-object
