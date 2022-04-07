#!/usr/bin/env python
"""
.. module:: reachCoordinateNode
	:platform: Unix
	:synopsis: Python module that the implement the server for the 'reachCoordinateService'
	
.. moduleauthor:: Daria Berretta <daria.berretta@gmail.com>

This node implements the server for the 'reachCoordinateService'

Service:
	/reachCoordinateService

ActionClient:
	/move_base

"""

import rospy
import os
import actionlib
from assignment_3.srv import reachCoordinateService 
from move_base_msgs.msg import *
from actionlib_msgs.msg import *

def reachCoordinate_handler(req):
	"""
	This function obtain from the arguments the coordinate to be reached,
	creates a simple client to the ``MoveBaseAction``, setup the goal, and
	then wait for the results of the goal: if it is 1 the goal is reached, 
	otherwise if it is 0 the goal is not reached.
	
	Args:
		req(): rapresents the (x,y) coordinate that the robot has to reach.
		
	Returns:
		0 if the goal is not reached
		
		1 if the goal is reached
	
	"""
	#Obtain the coordinate to reach
	x = req.x
	y = req.y
	
	#Creation of a Simple Client to the MoveBaseAction
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()
	
	#Setup of the goal
	goal = MoveBaseGoal()
	
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.pose.orientation.w = 1
	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y= y
	client.send_goal(goal)
	
	#wait for the result of the goal
	state_goal = client.wait_for_result(timeout=rospy.Duration(50.0))
	"""Variable that rapresent the state of the goal.
	
	1 -> means goal reached
	
	0 -> means goal not reached, so the goal il cancelled
	
	"""
	
	#return the state of the goal
	# 1 -> means goal reached
	# 0 -> means goal not reached, so the goal il cancelled
	if not state_goal:
		client.cancel_goal()
		return 0

	return 1

def reachCoordinate_server():
	"""
	Define a server for the ``reachCoordinateService``
	
	"""
	#define a server for the reachCoordinateService
	srv = rospy.Service('reachCoordinateService', reachCoordinateService, reachCoordinate_handler)
	print("Service ready")
	rospy.spin()


if __name__ == "__main__":
	rospy.init_node('reachCoordinateNode')
	reachCoordinate_server()
