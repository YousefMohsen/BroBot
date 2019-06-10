#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent, MoveTank, MoveSteering

from ev3dev2.sensor import INPUT_1, INPUT_4, INPUT_2, Sensor, INPUT_3
from ev3dev2.sensor.lego import TouchSensor, GyroSensor, UltrasonicSensor
from ev3dev2.led import Leds
from ev3dev2.sound import Sound
import time

sound = Sound()
sound.beep()
sound.speak("David please fuck off")
gyro = GyroSensor(INPUT_2)
motors = MoveTank(OUTPUT_D, OUTPUT_A)
steering_drive = MoveSteering(OUTPUT_D, OUTPUT_A)
us = UltrasonicSensor(INPUT_3)

# tank_pair = MoveTank(OUTPUT_D, OUTPUT_A)

def resetGyroAngle():
    motors.off()
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
    resetGyroAngle()

    turnSpeed = -20
    # Connect gyro sensor.

    # Start the left motor with speed 40% to initiate a medium turn right.
    motors.on(left_speed=turnSpeed, right_speed=-turnSpeed)

    # Wait until the gyro sensor detects that the robot has turned
    # (at least) 90 deg in the positive direction (to the right)
    gyro.wait_until_angle_changed_by(degrees)
    motors.off()

    # Robot moves straight ahead with speed 50% until the wheels
    # have turned through one rotation
    #tank_pair.on_for_rotations(left_speed=50, right_speed=50, rotations=1)


def turnLeftByDegrees(degrees):
    print("moveForwardGyro")
    resetGyroAngle()
    turnSpeed = -20
    # Connect gyro sensor.

    # Start the left motor with speed 40% to initiate a medium turn right.
    motors.on(left_speed=-turnSpeed, right_speed=turnSpeed)

    # Wait until the gyro sensor detects that the robot has turned
    # (at least) 90 deg in the positive direction (to the right)
    gyro.wait_until_angle_changed_by(degrees)
    motors.off()

    # Robot moves straight ahead with speed 50% until the wheels
    # have turned through one rotation
    #tank_pair.on_for_rotations(left_speed=50, right_speed=50, rotations=1)


def driveStraightGyro(power):
    resetGyroAngle()
    #error = (gyro.angle / 360) * 100
    error =  -1 * gyro.angle
    #print("error ",error)
    print("gyro.angle ",gyro.angle)
    time.sleep(1)
    #if(error == 0):
    #    moveForward(power)
    #else:
    steering_drive.on(error, -power)
    


def turnBack(left,turnDegrees):
    motors.off()
    if(left):
        turnLeftByDegrees(turnDegrees)
        motors.on_for_rotations(10, 10, -0.5)
        turnLeftByDegrees(turnDegrees)
    else:
        turnRightByDegrees(turnDegrees)
        motors.on_for_rotations(10, 10, -0.5)
        turnRightByDegrees(turnDegrees)

    if(turnDegrees==90):
        return 85
    else:
       return 90



def main():
    turnDegrees = 90
    resetGyroAngle()
    turnRight = True
    while True:
        while(us.distance_centimeters > 20):
            driveStraightGyro(30)
          #  moveForward(30)

        if(turnRight):
          turnDegrees =  turnBack(False, turnDegrees)
          turnRight = False
        else:
           turnDegrees = turnBack(True, turnDegrees)
           turnRight = True



#main()
#turnBack(True)
#turnRightByDegrees(90)
#turnLeftByDegrees(90)
#turnRightByDegrees(90)
#turnLeftByDegrees(90)
while True
    driveStraightGyro(30)