#!/usr/bin/env python
"""
.. module:: userInteractionNode
	:platform: Unix
	:synopsis: Python module for the user Interface
	
.. moduleauthor:: Daria Berretta <daria.berretta@gmail.com>

This node implements an user interface

Service:
	* /reachCoordinateService
	* /keyboardService

"""

import rospy
from assignment_3.srv import *

def calluser():
	"""	
	Function used to interact with the user.
		
	Ask to the user what action would complete.
		
	Returns:
			ans (int): the option selected by the user
	
	"""
	
	print("What do you want to do?")
	print("1. Reach a (x,y) coordinate")
	print("2. Drive the robot with the keybord")
	print("3. Drive the robot with some assistance")
	print("0. Exit")
	ans = input('Insert the number corresponding to the action you would do:')
	return ans
	
def opt1():
	"""
	Function called when option1 is selected, it define a client for the 
	``/reachCoordinateService``.
	
	It also allows the user to enter the coordinates to reach the desired point.
	Finally, it informs the user about the fulfillment of the goal with 
	a printout on the terminal.
	
	"""
	print("Option 1")
	x = float(input("Insert coordinate x: "))
	y = float(input("Insert coordinate y: "))
				
	print("You have insert the coordinate: (",x,",",y,")")
				
	#define a client for the service reachCoordinateService
	rospy.wait_for_service("/reachCoordinateService")
	client = rospy.ServiceProxy("/reachCoordinateService", reachCoordinateService)
	#store and interpretation of the return
	rt = client(x,y)
	if rt.ret == 0:
		print("Target not reached, goal cancelled")
	elif rt.ret == 1:
		print("Target reached!")
		
def opt2():
	"""
	Function called when option2 is selected, it define a client for the 
	``/keyboardService``
	
	"""
	print("Option 2")
	#define a client for the service keyboardService
	rospy.wait_for_service("/keyboardService")
	client = rospy.ServiceProxy("/keyboardService", keyboardService)
	client(0)
	
def opt3():
	"""
	Function called when option3 is selected, it define a client for the 
	``/keyboardService``
	
	"""
	#define a client for the service keyboardService
	print("Option 3")
	rospy.wait_for_service("/keyboardService")
	client = rospy.ServiceProxy("/keyboardService", keyboardService)
	client(1)
	

if __name__ == "__main__":
	"""
	This function initializes the ROS node and call the ``calluser()``
	function to	choose to send to the robot a position goal or
	to control the robot.
	
	"""
	
	#Initialization of the userInteractionNode
	rospy.init_node('userInteractionNode')
	
	
	while(1):
		ans = calluser();
		if ans.isnumeric():
			ans = int(ans)
			
			if ans==1:
				opt1()
					
			elif ans==2:
				opt2()
					
			elif ans==3:
				opt3()
				
			elif ans == 0:
				sys.exit()
	
			else:
				print("error")
		else:
			print("Invalid input")		

