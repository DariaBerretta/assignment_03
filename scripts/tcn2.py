#! /usr/bin/env python
"""
.. module:: teleopContrNode
	:platform: Unix
	:synopsis: Python module that the implement the server for the 'reachCoordinateService'
	
.. moduleauthor:: Daria Berretta daria.berretta@gmail.com

This node implements the server for the 'reachCoordinateService'

"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

#If in the vector walls one element is true, the robot can't go in that
#direction:
#first element -> right corner
#second element -> front 
#third element -> left corner
walls = [False, False, False]
"""If in the vector walls one element is true, the robot can't go in that direction
"""
min_dist = 0.5
"""Minimum possible distance between robots and obstacles
"""

linear = Vector3(0, 0, 0)
"""Vector of linear velocities
"""
angular = Vector3(0, 0, 0)
"""Vector of angular velocities
"""

repost = Twist(linear, angular)

def compute_min_dist(ranges):
	#Compute the min distance of the robot from the walls
    #first element -> right corner
    #second element -> front
    #third element -> left corner
    distance = [0,0,0]
    subarrays = [ ranges[0:240], ranges[241:480], ranges[481:721] ]
    distance = [ min(subarrays[0]), min(subarrays[1]), min(subarrays[2]) ]
    
    return distance
        
  
def callback_scan(data):
	#Eventually redefine repost if the robot have to avoid a wall
	global repost
	global walls
	
	pub = rospy.Publisher('cmd_vel',Twist, queue_size=10)
	
	#copy the ranges array
	ranges = data.ranges
	distances = compute_min_dist(ranges)
	
	#set boolean values for walls vector
	walls[0] = ( distances[0] < min_dist )
	walls[1] = ( distances[1] < min_dist )
	walls[2] = ( distances[2] < min_dist )
	
	#take the value inside the param "selection_param" 
	#to know if the user want to use some "warnings"
	#to drive the robot.
	toggle = rospy.get_param('/selection_param')
	
	if toggle:
		if walls[0]:
			#cannot turn right
			if repost.angular.z <0:
				#print("cannot turn right")
				repost.angular.z = 0
						
		if walls[1]:
			if repost.linear.x > 0:
				#print("cannot go ahead")
				repost.linear.x = 0
		
		if walls[2]:
			#cannot turn left
			if repost.angular.z > 0:
				#print("cannot go ahead")
				repost.angular.z = 0 
				
	#pubblic on topic cmd_vel
	pub.publish(repost)

def callback_remap(data):
	"""
	Copy the variable data on the global variable repost
	
	Args
		data(Twist): contains linear and angular velocities
		
	"""
	
	#copy remap_cmd_vel on repost -> ready to be:
	#modified by the controller or reposted as it was
	global repost
	repost = data
    
def keyboard_remap():
	"""
	Creates two different subscriber to two different topic ``/remap_cmd_vel`` 
	and ``/scan``
	
	"""
	
	#Creation of two different subscriber to two different topic
	rospy.Subscriber("/remap_cmd_vel", Twist, callback_remap)
	rospy.Subscriber("/scan", LaserScan, callback_scan)
	
	rospy.spin()
    
 
if __name__=="__main__":
	rospy.init_node('keyboard_remap_node')
	keyboard_remap()
