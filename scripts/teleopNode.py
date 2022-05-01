#!/usr/bin/env python
"""
.. module:: teleopNode
	:platform: Unix
	:synopsis: Python module that the implement the server for the 'keyboardService'
	
.. moduleauthor:: Daria Berretta <daria.berretta@gmail.com>

This node implements the server for the ``keyBoardService``

Implemented Service:
	* ``/keyBoardService``

"""

import rospy
import os
from assignment_3.srv import keyboardService

def teleop_handler(req):
	"""
	This function read the request of the user and implement the correct 
	actions.
	
	If ``req.action == 0`` an os call is made to start the ``teleope_twist_keybord``
	node, otherwise if ``req.action == 1`` an os call is made to launch the 
	``option3.launch``
	
	Args:
		req(int):0 means that the user want to ride the robot without help, 1 means that the user want to ride the robor with help
					
	"""
	#req.action = 0 -> means that the user want to ride the robot without help
	#req.action = 1 -> means that the user want to ride the robor with help
	
	if req.action == 0:
		print("Driving the robot without help")
		os.system("rosrun teleop_twist_keyboard teleop_twist_keyboard.py")
	elif req.action == 1:
		print("Driving the robot with help")
		os.system("roslaunch assignment_3 option3.launch")
	else:
		print("error")
	
	return 0

def teleop_server():
	"""
	This function crates the server for the ``keyBoardService``
	
	"""
	#Creation of the server for the keyboardService
	srv = rospy.Service('keyboardService', keyboardService, teleop_handler)
	print("Service ready")
	rospy.spin()


if __name__ == "__main__":
	rospy.init_node('teleopNode')
	teleop_server()
