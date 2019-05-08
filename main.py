#!/usr/bin/env pybricks-micropython
# So program can be run from Brickman

from ev3dev.ev3 import *
from time import sleep


from pybricks import ev3brick as brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
# Play a sound.
brick.sound.beep()
# Initialize a motor at port B.
test_motor = Motor(Port.B)
# Run the motor up to 500 degrees per second. To a target angle of 90 degrees.
test_motor.run_target(500, 90)
# Play another beep sound.
# This time with a higher pitch (1000 Hz) and longer duration (500 ms).
brick.sound.beep(1000, 500)