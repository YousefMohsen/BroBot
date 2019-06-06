#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_4, INPUT_2, Sensor, INPUT_3
from ev3dev2.sensor.lego import TouchSensor, GyroSensor, UltrasonicSensor
from ev3dev2.led import Leds
from ev3dev2.sound import Sound

sound = Sound()
sound.beep()
sound.speak("Whats up my niggas")
gyro = GyroSensor(INPUT_2)
motors = MoveTank(OUTPUT_D, OUTPUT_A)
us = UltrasonicSensor(INPUT_3)
# tank_pair = MoveTank(OUTPUT_D, OUTPUT_A)


def moveForward(speed):
    print("messi")
    motors.on(SpeedPercent(-10), SpeedPercent(-10))


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
    turnSpeed = 40
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


# turnByDegrees(5)
# dosen't work
""" def driveStraightGyro(power):
    error = -gyro.angle
    print("error",error)
    motors.on_for_rotations(SpeedPercent(-10), SpeedPercent(-10), error)

 """
# driveStraightGyro(3)


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
            moveForward(50)

        if(turnRight):
            turnBack(False)
            turnRight = False
        else:
            turnBack(True)
            turnRight = True



main()
