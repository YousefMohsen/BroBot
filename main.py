#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds
from ev3dev2.sound import Sound

sound = Sound()
sound.beep()

#move forward for 2 seconds
tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
tank_drive.on_for_seconds(SpeedPercent(-20),SpeedPercent(-20), 2)

