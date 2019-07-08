#!/usr/bin/env python
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import MultiArrayDimension
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import tf
import math
import time
import numpy as np
import os

# square side length in cm
grid_dim = 0

# reference april tag position
x_ref=0
y_ref=0

# initialize and define publishers
# MAP publishers that send positions in terms of squares (pixels)
Robot1 = rospy.Publisher('robot1', Int32MultiArray, queue_size=10)
Robot2 = rospy.Publisher('robot2', Int32MultiArray, queue_size=10)
Robot3 = rospy.Publisher('robot3', Int32MultiArray, queue_size=10)
Robot4 = rospy.Publisher('robot4', Int32MultiArray, queue_size=10)
obstacle1 = rospy.Publisher('obst1', Int32MultiArray, queue_size=10)
obstacle2 = rospy.Publisher('obst2', Int32MultiArray, queue_size=10)

# DISTANCE publishers that send positions in terms of cm wrt reference id
Robot1_current = rospy.Publisher('rob1_CurrentPose', Int32MultiArray, queue_size=10)
Robot2_current = rospy.Publisher('rob2_CurrentPose', Int32MultiArray, queue_size=10)
Robot3_current = rospy.Publisher('rob3_CurrentPose', Int32MultiArray, queue_size=10)
Robot4_Current = rospy.Publisher('rob4_CurrentPose', Int32MultiArray, queue_size=10)

# POSES publisher that publishes all positions in one array for the formation algorithm
pub_robots_current_poses = rospy.Publisher('robots_current_poses',Int32MultiArray,queue_size = 10)


# variables that need to be stored globally
tag_quaternion_x = tag_quaternion_y = tag_quaternion_z = tag_quaternion_w = 0

counter = 0
index = 0

def callback(data):
	try:
		global x_ref, y_ref,tag_quaternion_x, tag_quaternion_y, tag_quaternion_z, tag_quaternion_w, all_robots_positions
		global index, counter
		try:
			for i in range(0,7):
				length = len(data.detections[i].id)
		except IndexError:
			rospy.logwarn('Starting from tag {} missing'.format(i))

		all_robots_positions = [ [0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0] ]

		# looping over each ID AprilTagDetection Array and extracting required info
		for i in range ( len(data.detections) ):
			tag_id = data.detections[i].id
			tag_id = int (tag_id[0])

			# Absolute X and Y Positions
			tag_x  = int((data.detections[i].pose.pose.pose.position.x)*100)
			tag_y = int((data.detections[i].pose.pose.pose.position.y)*100)

			# Quaternion data of the current April Tag being processed
			get_quaternion_x = data.detections[i].pose.pose.pose.orientation.x
			get_quaternion_y = data.detections[i].pose.pose.pose.orientation.y
			get_quaternion_z = data.detections[i].pose.pose.pose.orientation.z
			get_quaternion_w = data.detections[i].pose.pose.pose.orientation.w

			# transformation to euler angles from quaternions
			explicit_quat = [get_quaternion_x,get_quaternion_y,get_quaternion_z,get_quaternion_w]
			euler = tf.transformations.euler_from_quaternion(explicit_quat)

			# calculating orientation
			tag_w = int(euler[2]*180/math.pi)

			all_robots_positions[tag_id]= [tag_x, tag_y,tag_w]

		#############################################################################################
		# Absolute Positions W.R.T Camera
		#the reference position w.r.t the camera
		x_ref=all_robots_positions [0][0]
		y_ref=all_robots_positions [0][1]
		w_ref=all_robots_positions [0][2]

		#the position of robot 1 w.r.t the camera
		x_1=all_robots_positions [1][0]
		y_1=all_robots_positions [1][1]
		w_1=all_robots_positions [1][2]

		#the position of robot 2 w.r.t the camera
		x_2=all_robots_positions [2][0]
		y_2=all_robots_positions [2][1]
		w_2=all_robots_positions [2][2]

		#the position of robot 3 w.r.t the camera
		x_3=all_robots_positions [3][0]
		y_3=all_robots_positions [3][1]
		w_3=all_robots_positions [3][2]

		#the position of robot 4 w.r.t the camera
		x_4=all_robots_positions [4][0]
		y_4=all_robots_positions [4][1]
		w_4=all_robots_positions [4][2]

		#the position of obstacle 1 w.r.t the camera
		Ob1_x = all_robots_positions [5][0]
		Ob1_y = all_robots_positions [5][1]

		#the position of obstacle 2 w.r.t the camera
		Ob2_x=all_robots_positions [6][0]
		Ob2_y=all_robots_positions [6][1]
		#############################################################################################

		#############################################################################################
		# ROBOT 1 POSITION CALCULATIONS
		# positions are calculated w.r.t to reference april tag
		X_r1_current= (x_1 - x_ref) # x-position of robot 1 wrt ref /current position in cm
		Y_r1_current= (y_1 - y_ref) # y-position of robot 1 wrt ref

		#convert the positions into pixels for the map
		X_r1 = round((X_r1_current) /grid_dim) #pixels
		Y_r1 = round(((Y_r1_current) /grid_dim))  #pixels

		# Capping the output to [0,9] interval
		X_r1 = max(min(9, X_r1), 0)
		Y_r1 = max(min(9, Y_r1), 0)

		# replaced these if conditions with built in capping functions
		# if X_r1 <= 0:
		# 	X_r1=0
		# if X_r1 >= 9:
		# 	X_r1=9

		# if Y_r1 <= 0:
		# 	Y_r1=0
		# if Y_r1 >= 9:
		# 	Y_r1=9

		# range of theta from (0 to 180) and from (-180 to 0 )
		W_1= (w_1 - w_ref ) # theta 1 in degrees
		W_r1= 100*((W_1)*math.pi/180)  # theta in radians
		#############################################################################################

		#############################################################################################
		# ROBOT 2 POSITION CALCULATIONS
		# positions are calculated w.r.t to reference april tag

		X_r2_current= (x_2 - x_ref) # x-position of robot 2 wrt ref
		Y_r2_current= (y_2 - y_ref) # y-position of robot 2 wrt ref

		X_r2 =round((X_r2_current) /grid_dim)  #pixels
		Y_r2 = round(((Y_r2_current) /grid_dim) )  #pixels

		# Capping the output to [0,9] interval
		X_r2 = max(min(9, X_r2), 0)
		Y_r2 = max(min(9, Y_r2), 0)

		# replaced these if conditions with built in capping functions
		# if X_r2	<=0:
		# 	X_r2=0
		# if X_r2 >=9:
		# 	X_r2=9
		# if Y_r2 <=0:
		# 	Y_r2= 0
		# if Y_r2 >9:
		# 	Y_r2 =9

		W_2= (w_2 - w_ref ) # theta 2 in degrees
		W_r2= 100*((W_2)*math.pi/180)  # theta 2 in radians
		#############################################################################################

		#############################################################################################
		# ROBOT 3 POSITION CALCULATIONS
		# positions are calculated w.r.t to reference april tag

		X_r3_current= (x_3 - x_ref) # x-position of robot 3 wrt ref
		Y_r3_current= (y_3 - y_ref) # y-position of robot 3 wrt ref

		X_r3 = round((X_r3_current) /grid_dim)  #pixels
		Y_r3 = round(((Y_r3_current) /grid_dim) ) #pixels

		# Capping the output to [0,9] interval
		X_r3 = max(min(9, X_r3), 0)
		Y_r3 = max(min(9, Y_r3), 0)

		# replaced these if conditions with built in capping functions
		# if X_r3 <=0:
		# 	X_r3=0
		# if Y_r3 <=0:
		# 	Y_r3= 0
		# if Y_r3 >=9:
		# 	Y_r3 =9
		# if X_r3 >=9:
		# 	X_r3=9

		W_3= (w_3 - w_ref ) # theta 3 in degrees
		W_r3= 100*((W_3)*math.pi/180)  # theta in radians
		#############################################################################################

		#############################################################################################
		# ROBOT 4 POSITION CALCULATIONS
		# positions are calculated w.r.t to reference april tag

		X_r4_current= (x_4 - x_ref) # x-position of robot 4 wrt ref
		Y_r4_current= (y_4 - y_ref) # y-position of robot 4 wrt ref

		X_r4 = round((X_r4_current) /grid_dim)  #pixels
		Y_r4 = round((Y_r4_current) /grid_dim)  #pixels

		# Capping the output to [0,9] interval
		X_r4 = max(min(9, X_r4), 0)
		Y_r4 = max(min(9, Y_r4), 0)

		# Capping the output to [0,9] interval
		# if X_r4 <=0:
		# 	X_r4=0
		# if X_r4>=9:
		# 	X_r4=9
		# if Y_r4 <=0:
		# 	Y_r4= 0
		# if Y_r4 >=9:
		# 	Y_r4 =9

		W_4= (w_4 - w_ref ) # theta 1 in degrees
		W_r4= 100*((W_4)*math.pi/180)  # theta in radians
		#############################################################################################

		#############################################################################################
		# OBSTACLE 1 POSITION CALCULATIONS
		# positions are calculated w.r.t to reference april tag

		Ob1_x_current= (Ob1_x - x_ref) # x-position of obstacle 1 wrt ref
		Ob1_y_current= (Ob1_y - y_ref) # y-position of obstacle 1 wrt

		ob1_x = round((Ob1_x_current) /grid_dim)  #pixels
		ob1_y = round(((Ob1_y_current)/grid_dim) ) #pixels
		#############################################################################################

		#############################################################################################
		# OBSTACLE 2 POSITION CALCULATIONS
		# positions are calculated w.r.t to reference april tag

		Ob2_x_current= (Ob2_x - x_ref) #x-position of obstacle 2 wrt ref
		Ob2_y_current= (Ob2_y - y_ref) #y-position of obstacle 2 wrt ref

		ob2_x = round((Ob2_x_current) /grid_dim ) #pixels
		ob2_y = round(((Ob2_y_current) /grid_dim) ) #pixels
		#############################################################################################

		# creating arrays for positions to publish them
		# cm arrays
		robot1_xyw_current= Int32MultiArray(data = [X_r1_current, Y_r1_current, W_r1])
		robot2_xyw_current= Int32MultiArray(data = [X_r2_current, Y_r2_current, W_r2])
		robot3_xyw_current= Int32MultiArray(data = [X_r3_current, Y_r3_current, W_r3])
		robot4_xyw_current= Int32MultiArray(data = [X_r4_current, Y_r4_current, W_r4])

		# map arrays
		robot1_xyw= Int32MultiArray(data=[int(X_r1) , int(Y_r1) , (W_r1)])
		robot2_xyw= Int32MultiArray(data=[int(X_r2) , int(Y_r2) , (W_r2)])
		robot3_xyw= Int32MultiArray(data=[int(X_r3) , int(Y_r3) , (W_r3)])
		robot4_xyw= Int32MultiArray(data=[int(X_r4) , int(Y_r4) , (W_r4)])
		obst1_xy= Int32MultiArray(data=[int(ob1_x) , int(ob1_y)])
		obst2_xy= Int32MultiArray(data=[int(ob2_x) , int(ob2_y)])

		# publish cm topics
		Robot1.publish(robot1_xyw)
		Robot2.publish(robot2_xyw)
		Robot3.publish(robot3_xyw)
		Robot4.publish(robot4_xyw)
		obstacle1.publish(obst1_xy)
		obstacle2.publish(obst2_xy)

		# publish px topics
		Robot1_current.publish(robot1_xyw_current)
		Robot2_current.publish(robot2_xyw_current)
		Robot3_current.publish(robot3_xyw_current)
		Robot4_Current.publish(robot4_xyw_current)

		#publish formation array
		pub_robots_current_poses.publish( Int32MultiArray( data=[X_r1_current, Y_r1_current,X_r2_current, Y_r2_current,X_r3_current, Y_r3_current,X_r4_current, Y_r4_current] ) )

		Rate.sleep()

	except rospy.ROSInterruptException:
		rospy.logwarn('NOT ALL TAGS DETECTED')
		pass

if __name__ == '__main__':
	try:
		# initializing the ROS node
		rospy.init_node('localization')
		rospy.loginfo('%s started.' % rospy.get_name())

		# get global grid parameters
		grid_dim = rospy.get_param('/grid_dimension')

		# Defining Publishing Rate
		Rate = rospy.Rate(30)
		
		# initialize the subscriber to AprilTagDetections
		rospy.Subscriber('tag_detections',AprilTagDetectionArray,callback)

		while not rospy.is_shutdown():
			rospy.spin()
	except rospy.ROSInterruptException:
		pass
	finally:
		rospy.loginfo('%s is closed' % rospy.get_name() )
