#!/usr/bin/env python
import math, time
import roslib
import rospy
import os
import numpy as np
from std_msgs.msg import Int32MultiArray, Int32, Byte


#### Publishers ####

mapping_rob1= rospy.Publisher('rob1_map', Int32MultiArray, queue_size=10)
mapping_rob2= rospy.Publisher('rob2_map', Int32MultiArray, queue_size=10)
mapping_rob3= rospy.Publisher('rob3_map', Int32MultiArray, queue_size=10)
mapping_rob4= rospy.Publisher('rob4_map', Int32MultiArray, queue_size=10)


#### initialize all variables ####

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

#robot to be moved id from formation
rob_id=0

rob2_pose=[]
rob1_pose=[]
rob3_pose=[]
rob4_pose=[]


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
def callback11(data):
    global rob_id
    rob_id=data.data

    rob_map()



#### initialize the node and set the subscribers ####

def listener():
    the_map = Int32MultiArray()
    rospy.init_node('Tracking')
    rospy.Subscriber('robot1', Int32MultiArray, callback1)
    rospy.Subscriber('robot2', Int32MultiArray, callback2)
    rospy.Subscriber('robot3', Int32MultiArray, callback3)
    rospy.Subscriber('robot4', Int32MultiArray, callback4)
    rospy.Subscriber('obst1', Int32MultiArray, callback5)
    rospy.Subscriber('obst2', Int32MultiArray, callback6)
    rospy.Subscriber('rob1_CurrentPose', Int32MultiArray, callback7)
    rospy.Subscriber('rob2_CurrentPose', Int32MultiArray, callback8)
    rospy.Subscriber('rob3_CurrentPose', Int32MultiArray, callback9)
    rospy.Subscriber('rob4_CurrentPose', Int32MultiArray, callback10)
    #rospy.Subscriber('obst1_CurrentPose',Int32MultiArray, callback11)
    #rospy.Subscriber('obst2_CurrentPose',Int32MultiArray, callback12)
    rospy.Subscriber('robot_to_be_moved', Int32, callback11)

    while not rospy.is_shutdown():
      rob_map()
      rospy.sleep(1)



#define the map

def rob_map():
      printable_map = [] #The List of Lists
      n=7  #no.of columns
      m=7  #no.of rows
      the_map = []
      row = [0] *n
      for i in range(m):
          the_map.append(list(row))

      print 'Map size (X,Y):', n, m
      print "robot1 on map(x,y):" , xR1 , yR1
      print "robot2 on map(x,y):" , xR2 , yR2
      print "robot3 on map(x,y):" , xR3 , yR3
      print "robot4 on map(x,y):" , xR4 , yR4
      print "obst 1 on map(x,y):", xO1 , yO1
      print "obst 2 on map(x,y):", xO2 , yO2
      print 'rob1_current position(x,y,w):' ,rob1_pose
      print 'rob2_current position(x,y,w):' ,rob2_pose
      print 'rob3_current position(x,y,w):' ,rob3_pose
      print 'rob4_current position(x,y,w):' ,rob4_pose
      #print 'obstacle 1  posistion (x,y):' , obst1_pose
      #print 'obstacle 2 position (x,y):', obst2_pose

##############set the robots and the obstacles positions on the map#############

      if (rob_id == -1):
          print('No robots to be moved')

########################### Mapping robot 1 ####################################

      if 1:
          the_map = np.zeros((7,7))
          printable_map = [] #The List of Lists
          the_map[0][0] = 4
          the_map[yR1][xR1] = 1
          the_map[yR2][xR2] = 4
          the_map[yR3][xR3] = 4
          the_map[yR4][xR4] = 4
          the_map[yO1][xO1] = 4
          the_map[yO2][xO2] = 4

          t = time.time()
          print 'Time=', time.time() - t
          # display the map with the route added
          print 'Map robot 1:'
          for y in range(m):
                 #list1= [] #Temp list that is appeneded
                 for x in range(n):
                       xy = the_map[y][x]
                       if xy == 0:
                          #print '2', # space
                          #list1.append(2)
                          printable_map.append(2)
                       elif xy == 1:
                          #print '1', # robot1
                          #list1.append(1)
                          printable_map.append(1)
                       elif xy == 4:
                          #print '-1', # ref,obstacles & the rest of robots
                          #list1.append(-1)
                          printable_map.append(-1)

                          #print 'the map',the_map
                 #printable_map.append(list1)
          print printable_map
          sendable_map = Int32MultiArray()
          sendable_map.data = printable_map
          mapping_rob1.publish(sendable_map)


########################### Mapping robot 2 ####################################
      if 1:
          the_map = np.zeros((7,7))
          printable_map = [] #The List of Lists
          the_map[0][0] = 4
          the_map[yR1][xR1] = 4
          the_map[yR2][xR2] = 1
          the_map[yR3][xR3] = 4
          the_map[yR4][xR4] = 4
          the_map[yO1][xO1] = 4
          the_map[yO2][xO2] = 4

          t = time.time()
          print 'Time=', time.time() - t
          # display the map with the route added
          print 'Map robot 2:'
          for y in range(m):
                 #list1= [] #Temp list that is appeneded
                 for x in range(n):
                       xy = the_map[y][x]
                       if xy == 0:
                          #print '2', # space
                          #list1.append(2)
                          printable_map.append(2)
                       elif xy == 1:
                          #print '1', # robot1
                          #list1.append(1)
                          printable_map.append(1)
                       elif xy == 4:
                          #print '-1', # ref,obstacles & the rest of robots
                          #list1.append(-1)
                          printable_map.append(-1)

                          #print 'the map',the_map
                 #printable_map.append(list1)
          print printable_map
          sendable_map = Int32MultiArray()
          sendable_map.data = printable_map
          mapping_rob2.publish(sendable_map)


########################### Mapping robot 3 ####################################

      if 1:
          the_map = np.zeros((7,7))
          printable_map = [] #The List of Lists
          the_map[0][0] = 4
          the_map[yR1][xR1] = 4
          the_map[yR2][xR2] = 4
          the_map[yR3][xR3] = 1
          the_map[yR4][xR4] = 4
          the_map[yO1][xO1] = 4
          the_map[yO2][xO2] = 4

          t = time.time()
          print 'Time=', time.time() - t
          # display the map with the route added
          print 'Map robot 3 :'
          for y in range(m):
                 #list1= [] #Temp list that is appeneded
                 for x in range(n):
                       xy = the_map[y][x]
                       if xy == 0:
                          #print '2', # space
                          #list1.append(2)
                          printable_map.append(2)
                       elif xy == 1:
                          #print '1', # robot1
                          #list1.append(1)
                          printable_map.append(1)
                       elif xy == 4:
                          #print '-1', # ref,obstacles & the rest of robots
                          #list1.append(-1)
                          printable_map.append(-1)

                          #print 'the map',the_map
                 #printable_map.append(list1)
          print printable_map
          sendable_map = Int32MultiArray()
          sendable_map.data = printable_map
          mapping_rob3.publish(sendable_map)


########################### Mapping robot 4 ####################################

      if 1:
          the_map = np.zeros((7,7))
          printable_map = [] #The List of Lists
          the_map[0][0] = 4
          the_map[yR1][xR1] = 4
          the_map[yR2][xR2] = 4
          the_map[yR3][xR3] = 4
          the_map[yR4][xR4] = 1
          the_map[yO1][xO1] = 4
          the_map[yO2][xO2] = 4

          t = time.time()
          print 'Time=', time.time() - t
          # display the map with the route added
          print 'Map robot 4:'
          for y in range(m):
                 #list1= [] #Temp list that is appeneded
                 for x in range(n):
                       xy = the_map[y][x]
                       if xy == 0:
                          #print '2', # space
                          #list1.append(2)
                          printable_map.append(2)
                       elif xy == 1:
                          #print '1', # robot1
                          #list1.append(1)
                          printable_map.append(1)
                       elif xy == 4:
                          #print '-1', # ref,obstacles & the rest of robots
                          #list1.append(-1)
                          printable_map.append(-1)

                          #print 'the map',the_map
                 #printable_map.append(list1)
          print printable_map
          sendable_map = Int32MultiArray()
          sendable_map.data = printable_map
          mapping_rob4.publish(sendable_map)

################################################################################



      os.system('clear')

if __name__ == '__main__':

        listener()
