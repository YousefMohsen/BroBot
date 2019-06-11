#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent, MoveTank, MoveSteering

from ev3dev2.sensor import INPUT_1, INPUT_4, INPUT_2, Sensor, INPUT_3
from ev3dev2.sensor.lego import TouchSensor, GyroSensor, UltrasonicSensor, InfraredSensor
from ev3dev2.led import Leds
from ev3dev2.sound import Sound
from ev3dev2.port import LegoPort

import time
from smbus import SMBus


sound = Sound()
sound.beep()
gyro = GyroSensor(INPUT_2)
motors = MoveTank(OUTPUT_D, OUTPUT_A)
steering_drive = MoveSteering(OUTPUT_D, OUTPUT_A)
us = UltrasonicSensor(INPUT_3)

infraSensor = InfraredSensor(INPUT_4)
infraSensor.mode = 'IR-PROX'


in1 = LegoPort(address=INPUT_1)
in1.mode = 'other-i2c'

bus = SMBus(3)  # bus number is input port number + 2
I2C_ADDRESS = 0x01  # the default I2C address of the sensor

def resetGyroAngle():
    motors.off()
    gyro.mode = 'GYRO-RATE'
    gyro.mode = 'GYRO-ANG'

def drive(speed):
    motors.on(SpeedPercent(-speed), SpeedPercent(-speed))


def stopMotor(motor):
    motor.off()


def turnRightByDegrees(degrees):
    resetGyroAngle()

    turnSpeed = -20
    
    motors.on(left_speed=turnSpeed, right_speed=-turnSpeed)

    gyro.wait_until_angle_changed_by(degrees)
    motors.off()


def turnLeftByDegrees(degrees):
    resetGyroAngle()
    turnSpeed = -20

    motors.on(left_speed=-turnSpeed, right_speed=turnSpeed)

    gyro.wait_until_angle_changed_by(degrees)
    motors.off()

def driveStraightGyro(power):
    resetGyroAngle()
    error =  -1 * gyro.angle
    time.sleep(1)
    steering_drive.on(error, -power)
    


def turnBack(left,turnDegrees, tracks):
    motors.off()
    if(left):
        turnLeftByDegrees(turnDegrees)
        motors.on_for_rotations(10, 10, -0.85)
        turnLeftByDegrees(turnDegrees)
    else:
        turnRightByDegrees(turnDegrees)
        if(tracks == 5):
            motors.on_for_rotations(10, 10, -0.65)
        else:
            motors.on_for_rotations(10, 10, -0.85)
        turnRightByDegrees(turnDegrees)


   # motors.on_for_rotations(left_speed=-25, right_speed=-15, rotations=0.5)
    """if(turnDegrees==90):
        return 85
    else:
       return 90"""


def lookForObstacle(signatureToFind,callback):
       # signatureToFind = 1
        signatureType, ignore, x, y, width, height = bus.read_i2c_block_data(I2C_ADDRESS, 0x50, 6)
        # do stuff with coordinates
        print("Signature ", signatureType)
        print("x", x)
        print("y", y)
        print("width", width)
        print("height", height)

        print("-----------\n")
        time.sleep(2)
        if(signatureType==signatureToFind):
            callback(x, y, width, height)



def obstacleFound(x, y, width, height):
        motors.off()
        motors.on_for_rotations(10,10,0.5) #move 2 rotations back
 
        
        turnLeftByDegrees(50) #turn left
        motors.on_for_rotations(10,10,-2) 
        turnRightByDegrees(100) #turn right
        motors.on_for_rotations(10,10,-2) 
        turnLeftByDegrees(50) #turn left


def driveAlongWall(dist):
    #dist = 20
    #while(True): 
        while(25 < (infraSensor.proximity/100)*70):
            print("us.distance_centimeters",us.distance_centimeters)
            time.sleep(1)
            if(us.distance_centimeters < dist):
                motors.on(-30, -25)
            else:
                if(us.distance_centimeters > dist):
                    motors.on(-25, -30)
                else:
                    motors.on(-30, -30)

def sweep():
    tracksDistance = [8,80,55,55,80,8]

    turnRight = True
    for index, track in enumerate(tracksDistance, start=0):
       # print("track",track)
        driveAlongWall(track)
        if(len(tracksDistance)-1!=index ):
            if(turnRight):
                turnBack(False, 90, index+1)
                turnRight = False
            else:
                turnBack(True, 90, index+1)
                turnRight = True
        else: #find goal
            motors.off()
            sound.beep()
        
        print("Counter", index+1)
   
sweep()
