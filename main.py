#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent, MoveTank
from ev3dev2.motor import MoveSteering, OUTPUT_A, OUTPUT_D

from ev3dev2.sensor import INPUT_1, INPUT_4, INPUT_2, Sensor, INPUT_3
from ev3dev2.sensor.lego import TouchSensor, GyroSensor, UltrasonicSensor
from ev3dev2.led import Leds
from ev3dev2.sound import Sound
import time

sound = Sound()
sound.beep()


gyro = GyroSensor(INPUT_2)
motors = MoveTank(OUTPUT_D, OUTPUT_A)
steering_drive = MoveSteering(OUTPUT_A, OUTPUT_D)
us = UltrasonicSensor(INPUT_3)
# tank_pair = MoveTank(OUTPUT_D, OUTPUT_A)

def resetGyroAngle():
    gyro.mode = 'GYRO-RATE'
    gyro.mode = 'GYRO-ANG'

def moveForward(speed):
    print("messi")
    motors.on(SpeedPercent(-speed), SpeedPercent(-speed))


def stopMotor(motor):
    motor.off()

# move forward for 2 seconds
#tank_drive = MoveTank(OUTPUT_D, OUTPUT_A)
#tank_drive.on_for_seconds(SpeedPercent(-20), SpeedPercent(-20), 1)


def turnRightByDegrees(degrees):
    print("moveForwardGyro")
    turnSpeed = -40
    # Connect gyro sensor.

    # Start the left motor with speed 40% to initiate a medium turn right.
    motors.on(left_speed=turnSpeed, right_speed=0)

    # Wait until the gyro sensor detects that the robot has turned
    # (at least) 90 deg in the positive direction (to the right)
    gyro.wait_until_angle_changed_by(degrees)
    motors.off()

    # Robot moves straight ahead with speed 50% until the wheels
    # have turned through one rotation
    #tank_pair.on_for_rotations(left_speed=50, right_speed=50, rotations=1)


def turnLeftByDegrees(degrees):
    print("moveForwardGyro")
    turnSpeed = -40
    # Connect gyro sensor.

    # Start the left motor with speed 40% to initiate a medium turn right.
    motors.on(left_speed=0, right_speed=turnSpeed)

    # Wait until the gyro sensor detects that the robot has turned
    # (at least) 90 deg in the positive direction (to the right)
    gyro.wait_until_angle_changed_by(degrees)
    motors.off()

    # Robot moves straight ahead with speed 50% until the wheels
    # have turned through one rotation
    #tank_pair.on_for_rotations(left_speed=50, right_speed=50, rotations=1)


def driveStraightGyro(power):
    error = gyro.angle * -10
    print("error ",error)
    print("gyro.angle ",gyro.angle)


    '''if(error == 0):
        moveForward(power)
    else:
        steering_drive.on_for_degrees(-100, -power, abs(error))
    '''

def turnBack(left):
    if(left):
        turnLeftByDegrees(90)
        motors.on_for_rotations(10, 10, -0.5)
        turnLeftByDegrees(90)
    else:
        turnRightByDegrees(90)
        motors.on_for_rotations(10, 10, -0.5)
        turnRightByDegrees(90)


def main():
    turnRight = True
    while True:
        while(us.distance_centimeters > 20):
            driveStraightGyro(50)

        if(turnRight):
            turnBack(False)
            turnRight = False
        else:
            turnBack(True)
            turnRight = True



#main()
