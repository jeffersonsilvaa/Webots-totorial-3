# **************************************************************
# Project 03 - Disciplina de robótica Móvel UFC / IFCE / LAPISCO
#       Simulação 01 com robô Pioneer 3AT - Webots R2020a
#              Distance sensors - Lidar RangeImage
#        Python 3.6 na IDE Pycharm - controller <extern>
#                By: Jefferson Silva Almeida
#                       Data: 23/01/2020
# **************************************************************

from controller import Robot
from controller import Motor
from controller import DistanceSensor
from controller import Lidar
import cv2
import matplotlib.pyplot as plt
import numpy as np
import math
import time

# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 6.28

# maximal value returned by the sensors
MAX_SENSOR_VALUE = 1024
# minimal distance, in meters, for an obstacle to be considered
MIN_DISTANCE = 1.0

# create the robot instance
robot = Robot()

# init lidar lms291
lms291 = robot.getLidar('Sick LMS 291')
print(lms291)
Lidar.enable(lms291, TIME_STEP)
Lidar.enablePointCloud(lms291)
print('Lidar enabled')
lms291_width = Lidar.getHorizontalResolution(lms291)
print(lms291_width)
#half_width = lms291_width / 2
#max_range = Lidar.getMaxRange(lms291)
#num_points = Lidar.getNumberOfPoints(lms291)

# Lidar.disablePointCloud(lms291)
# print(map)
# cv2.imshow(map)
# cv2.waitKey(100)


# Lidar.disable(lms291)
print('Lidar disabled')

# get a handler to the motors and set target position to infinity (speed control)
leftMotorFront = robot.getMotor('front left wheel')
rightMotorFront = robot.getMotor('front right wheel')
leftMotorBack = robot.getMotor('back left wheel')
rightMotorBack = robot.getMotor('back right wheel')

leftMotorFront.setPosition(float('inf'))
rightMotorFront.setPosition(float('inf'))
leftMotorBack.setPosition(float('inf'))
rightMotorBack.setPosition(float('inf'))

# initialize devices
ps = []
psNames = [
    'so0', 'so1', 'so2', 'so3',
    'so4', 'so5', 'so6', 'so7'
]

for i in range(8):
    ps.append(robot.getDistanceSensor(psNames[i]))
    ps[i].enable(TIME_STEP)

while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
        # print(psValues[i])

    # detect obstacles
    right_obstacle = psValues[0] > 70.0 or psValues[1] > 70.0 or psValues[2] > 70.0
    left_obstacle = psValues[5] > 70.0 or psValues[6] > 70.0 or psValues[7] > 70.0
    front_obstacle = psValues[3] > 50.0 or psValues[4] > 50.0
    # print(right_obstacle)
    # print(left_obstacle)

    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed = 0.9 * MAX_SPEED
    rightSpeed = 0.9 * MAX_SPEED

    # modify speeds according to obstacles
    if front_obstacle:
        leftSpeed -= 0.5 * MAX_SPEED
        rightSpeed += 0.5 * MAX_SPEED
        print("front_obstacle")
    elif left_obstacle:
        leftSpeed -= 0.5 * MAX_SPEED
        rightSpeed += 0.5 * MAX_SPEED
        print("left_obstacle")
    elif right_obstacle:
        leftSpeed += 0.5 * MAX_SPEED
        rightSpeed -= 0.5 * MAX_SPEED
        print("right_obstacle")

    # set up the motor speeds at x% of the MAX_SPEED.
    leftMotorFront.setVelocity(leftSpeed)
    rightMotorFront.setVelocity(rightSpeed)
    leftMotorBack.setVelocity(leftSpeed)
    rightMotorBack.setVelocity(rightSpeed)

    # read Lidar
    lms291_values = []
    lms291_values = Lidar.getRangeImage(lms291)
    # print(lms291_values)

    # plot polar map
    y = lms291_values
    x = np.linspace(math.pi, 0, np.size(y))
    plt.polar(x, y)
    plt.pause(0.0001)
    plt.clf()

plt.show()
plt.savefig('mapa.png')


