#!/usr/bin/env python
"""
.. module:: reachCoordinateNode
	:platform: Unix
	:synopsis: Python module that the implement the server for the 'reachCoordinateService'
	
.. moduleauthor:: Daria Berretta <daria.berretta@gmail.com>

This node implements the server for the "reachCoordinateService".
It uses the function "reachCoordinate_handler" to read the request made by the user,
creates a SimpleCLient to the "MoveBaseAction" and set the specific position goal for the robot.

Implemented service:
	* ``/reachCoordinateService``

ActionClient:
	* ``/move_base``

"""

import rospy
import os
import actionlib
from assignment_3.srv import reachCoordinateService 
from move_base_msgs.msg import *
from actionlib_msgs.msg import *

def reachCoordinate_handler(req):
	"""
	The coordinates to be reached by the robot are obtained from the argument of the function.
	Here is created the simple client to the ``MoveBaseAction``, the goal is
	set up, and then the function waits for the results of the goal.
	If the variable ``state_goal`` has a value '1' the goal is reached and
	the function returns '1', otherwise, if it has a value '0' the goal 
	is not reached, so it is canceled and the function returns '0'.
	
	Args:
		req(float): rapresents the (x,y) coordinate that the robot has to reach.
		
	Returns:
		state_goal(int): 0 means the goal is not reached, 1 means that goal has been reached.
	
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
		return state_goal

	return state_goal

def reachCoordinate_server():
	"""
	In this function is simply defined a server for the ``/reachCoordinateService``
	
	"""
	#define a server for the reachCoordinateService
	srv = rospy.Service('reachCoordinateService', reachCoordinateService, reachCoordinate_handler)
	print("Service ready")
	rospy.spin()


if __name__ == "__main__":
	rospy.init_node('reachCoordinateNode')
	reachCoordinate_server()
