#!/usr/bin/env python
import math, time
import roslib
import rospy
from std_msgs.msg import Int32
import os
import numpy as np
from std_msgs.msg import Int32MultiArray
#initialize all variables

mapping_rob4= rospy.Publisher('rob4_map', Int32MultiArray, queue_size=10)

#initialize all variables


# robot 1 current_position
xR1=0
yR1=0

#robot 2 current_position
xR2=0
yR2=0

#robot 3 current_position
xR3=0
yR3=0

#robot 4 current_pposition
xR4=0
yR4=0

#obstacle 1 position
xO1=0
yO1=0

#obstacle 2 position
xO2=0
yO2=0

rob1_pose=[]

#get all data from callbacks(positions of all robots and obstacles)
def callback1(data):
    global xR1,yR1
    xR1=data.data[0]
    yR1=data.data[1]
def callback2(data):
    global xR2,yR2
    xR2=data.data[0]
    yR2=data.data[1]
def callback3(data):
    global xR3,yR3
    xR3=data.data[0]
    yR3=data.data[1]
def callback4(data):
    global xR4,yR4
    xR4=data.data[0]
    yR4=data.data[1]
def callback5(data):
    global xO1,yO1
    xO1=data.data[0]
    yO1=data.data[1]
def callback6(data):
    global xO2,yO2
    xO2=data.data[0]
    yO2=data.data[1]
def callback7(data):
    global rob1_pose
    rob1_pose=data.data
def callback8(data):
    global rob2_pose
    rob2_pose=data.data
def callback9(data):
    global rob3_pose
    rob3_pose=data.data
def callback10(data):
    global rob4_pose
    rob4_pose=data.data
    #global mapping
    rob4_map()
    rospy.sleep(1)

#initialize thethe_map[yR3][xR3]=-1 subscribers
def listener():
    #global the_map
    rospy.init_node('Tracking5')
    rospy.Subscriber('robot1',Int32MultiArray,callback1)
    rospy.Subscriber('robot2',Int32MultiArray,callback2)
    rospy.Subscriber('robot3',Int32MultiArray,callback3)
    rospy.Subscriber('robot4',Int32MultiArray,callback4)
    rospy.Subscriber('obst1',Int32MultiArray,callback5)
    rospy.Subscriber('obst2',Int32MultiArray,callback6)

    rospy.Subscriber('rob1_CurrentPose',Int32MultiArray,callback7)
    rospy.Subscriber('rob2_CurrentPose',Int32MultiArray,callback8)
    rospy.Subscriber('rob3_CurrentPose',Int32MultiArray,callback9)
    rospy.Subscriber('rob4_CurrentPose',Int32MultiArray,callback10)

    while not rospy.is_shutdown():
	rospy.spin()

#define a map(2D array of ones and zeros)

def rob4_map():
   the_map = np.full( (7,7), 2)
   print the_map
   the_map[yR4][xR4]= 1
   the_map[yR3][xR3]=-1
   the_map[yR2][xR2]=-1
   the_map[yR1][xR1]=-1
   the_map[yO1][xO1]=-1
   the_map[yO2][xO2]=-1

   print the_map

   sendable_map = np.reshape(the_map, (49))
   mapping_rob4.publish(Int32MultiArray(data = sendable_map))

if __name__ == '__main__':

	listener()
