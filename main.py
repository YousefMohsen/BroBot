#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_4, Sensor, INPUT_3
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds
from ev3dev2.sound import Sound

sound = Sound()
sound.beep()
    
def moveForward (speed):
    print("messi") 
    tank_drive = MoveTank(OUTPUT_D, OUTPUT_A)
    tank_drive.on(SpeedPercent(-speed), SpeedPercent(-speed), 1)
def stopMotor (motor):
    motor.off()
# move forward for 2 seconds
#tank_drive = MoveTank(OUTPUT_D, OUTPUT_A)
#tank_drive.on_for_seconds(SpeedPercent(-20), SpeedPercent(-20), 1)
move_forward()
