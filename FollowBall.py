#!/usr/bin/env python3
from ev3dev.ev3 import *
from smbus import SMBus



def limit_speed(speed):
    """ Limit speed in range [-900,900] """
    if speed > 900:
        speed = 900
    elif speed < -900:
        speed = -900
    return -speed

in1 = LegoPort(address=INPUT_1)
# force the port into i2c-other mode so that the default driver is not automatically loaded
in1.mode = 'other-i2c'

# might need a short delay here (e.g. time.sleep(0.5)) to wait for the I2C port to be setup
# note: this delay should only be needed in ev3dev-jessie (4.4.x kernel), not ev3dev-stretch (4.9.x kernel)


bus = SMBus(3)  # bus number is input port number + 2
I2C_ADDRESS = 0x01  # the default I2C address of the sensor

# Connect Pixy camera and set mode
#pixy = Sensor(address=INPUT_1)
#assert pixy.connected, "Error while connecting Pixy camera"
#pixy.mode = 'SIG1'

# Connect TouchSensor (to stop script)
#ts = TouchSensor(INPUT_4)
#assert ts.connected, "Error while connecting TouchSensor"

# Connect LargeMotors
rmotor = LargeMotor(OUTPUT_A)
assert rmotor.connected, "Error while connecting right motor"
lmotor = LargeMotor(OUTPUT_D)
assert lmotor.connected, "Error while connecting left motor"

# Defining constants
X_REF = 128  # X-coordinate of referencepoint
Y_REF = 150  # Y-coordinate of referencepoint
KP = 0.4     # Proportional constant PID-controller
KI = 0.01    # Integral constant PID-controller
KD = 0.005   # Derivative constant PID-controller
GAIN = 10   # Gain for motorspeed

# Initializing PID variables
integral_x = 0
derivative_x = 0
last_dx = 0
integral_y = 0
derivative_y = 0
last_dy = 0

while True:
    signatureType, ignore, x, y, width, height = bus.read_i2c_block_data(I2C_ADDRESS, 0x50, 6)

    if signatureType > 0:
       # x = pixy.value(1)             # X-centroid of largest SIG1-object
       # y = pixy.value(2)             # Y-centroid of largest SIG1-object
# Initializing PID variables
integral_x = 0
derivative_x = 0
last_dx = 0
integral_y = 0
derivative_y = 0
last_dy = 0

while True:
    signatureType, ignore, x, y, width, height = bus.read_i2c_block_data(I2C_ADDRESS, 0x50, 6)

    if signatureType > 0:
       # x = pixy.value(1)             # X-centroid of largest SIG1-object
       # y = pixy.value(2)             # Y-centroid of largest SIG1-object
# Initializing PID variables
integral_x = 0
derivative_x = 0
last_dx = 0
integral_y = 0
derivative_y = 0
last_dy = 0

while True:
    signatureType, ignore, x, y, width, height = bus.read_i2c_block_data(I2C_ADDRESS, 0x50, 6)

    if signatureType > 0:
       # x = pixy.value(1)             # X-centroid of largest SIG1-object
       # y = pixy.value(2)             # Y-centroid of largest SIG1-object
# Initializing PID variables
integral_x = 0
derivative_x = 0
last_dx = 0
integral_y = 0
derivative_y = 0
last_dy = 0

while True:
    signatureType, ignore, x, y, width, height = bus.read_i2c_block_data(I2C_ADDRESS, 0x50, 6)

    if signatureType > 0:
       # x = pixy.value(1)             # X-centroid of largest SIG1-object
       # y = pixy.value(2)             # Y-centroid of largest SIG1-object
        dx = X_REF - x                # Error in reference to X_REF
        integral_x = integral_x + dx  # Calculate integral for PID
        derivative_x = dx - last_dx   # Calculate derivative for PID
        speed_x = KP*dx + KI*integral_x + KD*derivative_x  # Speed in X-direction
        dy = Y_REF - y       # Error in reference to Y_REF
        integral_y = integral_y + dy  # Calculate integral for PID
        derivative_y = dy - last_dy   # Calculate derivative for PID
        speed_y = KP*dy + KI*integral_y + KD*derivative_y  # Speed in Y-direction
        # Calculate motorspeed out of speed_x and speed_y
        # Use GAIN otherwise speed will be to slow, but limit in range [-900,900]
        rmotor.run_forever(speed_sp=limit_speed(GAIN*(speed_y + speed_x)))
        lmotor.run_forever(speed_sp=limit_speed(GAIN*(speed_y - speed_x)))
        last_dx = dx     # Set last error for x
        last_dy = dy     # Set last error for y
    else:
        rmotor.stop()    # SIG1 not detected, stop motors
        lmotor.stop()

# End of script, stop motors
rmotor.stop()
lmotor.stop()
