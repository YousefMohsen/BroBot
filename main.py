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
#sound.speak("Boris shut up")
gyro = GyroSensor(INPUT_2)
motors = MoveTank(OUTPUT_D, OUTPUT_A)
steering_drive = MoveSteering(OUTPUT_D, OUTPUT_A)
us = UltrasonicSensor(INPUT_3)

infraSensor = InfraredSensor(INPUT_4)
infraSensor.mode = 'IR-PROX'

# tank_pair = MoveTank(OUTPUT_D, OUTPUT_A)
#init pixy 
# get a Port object to control the input port
in1 = LegoPort(address=INPUT_1)
# force the port into i2c-other mode so that the default driver is not automatically loaded
in1.mode = 'other-i2c'

# might need a short delay here (e.g. time.sleep(0.5)) to wait for the I2C port to be setup
# note: this delay should only be needed in ev3dev-jessie (4.4.x kernel), not ev3dev-stretch (4.9.x kernel)


bus = SMBus(3)  # bus number is input port number + 2
I2C_ADDRESS = 0x01  # the default I2C address of the sensor


def resetGyroAngle():
    motors.off()
    gyro.mode = 'GYRO-RATE'
    gyro.mode = 'GYRO-ANG'

def drive(speed):
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
    #    drive(power)
    #else:
    steering_drive.on(error, -power)
    
def adjustDistanceToWall():

    #If too far from wall, drive closer
    if(15 < (infraSensor.proximity/100)*70):
        while( 15 < (infraSensor.proximity/100)*70):
            motors.on(-10, -10)
    #if too close to wall, reverse.
    else:
        while(15 > (infraSensor.proximity/100)*70):
            motors.on(10, 10)

def turnBack(left,turnDegrees,trackNumber):
    motors.off()
    if(left):
        turnLeftByDegrees(turnDegrees)
        motors.on_for_rotations(10, 10, -0.7)
        turnLeftByDegrees(turnDegrees)
    else:
        turnRightByDegrees(turnDegrees)
        
        #At the last track, drive until only 20 cm from wall
        if(trackNumber == 5):
            adjustDistanceToWall()
        else:
            motors.on_for_rotations(10, 10, -0.7)
        turnRightByDegrees(turnDegrees)       
        '''
        if(trackNumber == 5) and (40 > (infraSensor.proximity/100)*70): 
            sound.beep()
            motors.on_for_rotations(10, 10, -0.3)
        else:
            motors.on_for_rotations(10, 10, -0.7)
        turnRightByDegrees(turnDegrees)
        '''

    motors.on_for_rotations(left_speed=-50, right_speed=-50, rotations=1) 





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





def main():
    turnDegrees = 90
    turnRight = True
    resetGyroAngle()
    lookForObstacle()
    while True:
        while(us.distance_centimeters > 20):
           # driveStraightGyro(30)
            drive(30)

        if(turnRight):
          turnDegrees =  turnBack(False, turnDegrees)
          turnRight = False
        else:
           turnDegrees = turnBack(True, turnDegrees)
           turnRight = True



def obstacleFound(x, y, width, height):
        motors.off()
        motors.on_for_rotations(10,10,0.5) #move 2 rotations back
        #turnLeftByDegrees(90) #turn left
        #motors.on_for_rotations(-10,-10,1) #move 6 rotations back
        #turnRightByDegrees(90) #turn right
        #motors.on_for_rotations(-10,-10,2.9) #move 6 rotations back
        #turnRightByDegrees(90) #turn right
        #motors.on_for_rotations(-10,-10,0.5) #move 6 rotations back
        #turnLeftByDegrees(90) #turn right
        
        turnLeftByDegrees(50) #turn left
        motors.on_for_rotations(10,10,-2) 
        turnRightByDegrees(100) #turn right
        motors.on_for_rotations(10,10,-2) 
        turnLeftByDegrees(50) #turn left







      # motors.on_for_rotations(-10,-10,2) #move 6 rotations back

             

#while True:
 #    lookForObstacle(1,obstacleFound)
#obstacleFound(1,2,3,4)


#def findGoal():
    #find

#main()
#turnBack(True,90)
#turnRightByDegrees(90)
#turnLeftByDegrees(90)
#turnRightByDegrees(90)
#turnLeftByDegrees(90)
#while True:
 #   """cm = (infraSensor.proximity/100)*70
  #  print("cm:",cm)
   # print("cm shit",us.distance_centimeters)
  #  time.sleep(1)"""
 #   drive(30)

#driveAlongWall(10)
#steering_drive.on_for_rotations(-20, -10, 0.5)

def driveAlongWall(dist):

    speed = -50

    #dist = 20
    #while(True): 
    while(25 < (infraSensor.proximity/100)*70):
        print("us.distance_centimeters",us.distance_centimeters)
        if(us.distance_centimeters < dist):
            motors.on(speed, speed+5) #if too close to wall, drive away
        else:
            if(us.distance_centimeters > dist):
                motors.on(speed+5, speed) #if too far from wall, drive closer
            else:
                motors.on(speed, speed)

def sweep():
    
    tracksDistance = [8,80,45,45,80,8]
    
    turnRight = True
    for index, track in enumerate(tracksDistance, start=0):
       # print("track",track)
        driveAlongWall(track)
        if(len(tracksDistance)-1!=index ):
            if(turnRight):
                turnBack(False, 90,index+1)
                turnRight = False
            else:
                turnBack(True, 90,index+1)
                turnRight = True
        else: #find goal
            motors.off()
            sound.beep()
        



#while True:
    #motors.on(left_speed=-40, right_speed=-10)
  # print("us.distance_centimeters", us.distance_centimeters)
 #  time.sleep(2)
   
sweep()
