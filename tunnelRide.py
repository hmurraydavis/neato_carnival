#!/usr/bin/env python
import rospy
from math import *
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import cv2

class DominoRide(Ride):

	STATE_LOOK = 0
	STATE_KNOCK_DOMINO = 1
	STATE_DONE = 2

	def __init__(self):
		self.state = 0
		self.target = None

	def run(self):
		if self.state == STATE_LOOK:
			self.analyze_frame()

		elif self.state == STATE_KNOCK_DOMINO:
			self.knock_target()

	def analyze_frame(self):
		pass
		#TODO: get frame from camera
		# run image filtering and color filters
		# identify spot of correct color
			# if no spots of color consider this ride STATE_DONE
		# set self.target to that location
		# set state to KNOCK DOMINO once target is set
		# Halie knows how to do this? it was her CV project

	def knock_target(self):
		pass
		# identify motor commands
		# go towards target
		# run over target
		# back up to "reset"
		# set state to LOOK

class TunnelRide(Ride):

	STATE_FIND_FRONT = 0
	STATE_NAVIGATE_TO_FRONT = 1
	STATE_DRIVE_THRU = 2
	STATE_DONE = 3

	def __init__(self):
		self.state = 0
		self.direction_towards_fiducial = (0,0) #distance, angle in degrees
		self.sub = rospy.Subscriber('scan', LaserScan, self.scan_received)

	def run(self):
		# check state and run appropriate command
		if self.state == STATE_FIND_FRONT:
			self.find_front_of_tunnel()

		elif self.state == STATE_DRIVE_THRU:
			self.drive_thru_tunnel_with_lidar()

	def drive_thru_tunnel_with_lidar(self):
		pass


	def find_front_of_tunnel(self):
		img = self.get_frame() # get image from robot cam



	def get_frame(self):
		#TODO: get image from robot cam
		pass

	def process_frame(self, frame):
		#TODO: color processing on frame
		pass

	def scan_received(self, msg):
        # processes data from laser scan, msg is of type sensor_msgs/LaserScan
	    if self.state == STATE_NAVIGATE_TO_FRONT:
	        angle_to_fid = self.direction_towards_fiducial[1]
	        view_env = []
	        for i in range(angle_to_fid-60, angle_to_fid+60): #slice of view around fiducial
	            if msg.ranges[i] > .5 and msg.range[i] < 5: #check if object exists at a certain degree
	            	view_env.append(1)
	            else:
	            	view_env.append(0)

	        # now we have an array of ones and zeros based on whether it sees an object
	        # we want to see something like 0000011100000000000111000000 
	        # where two sections of ones mean front of tunnel
	        # 0000011111111111111111111111100000 would mean it sees side of tunnel, for example

	        while self.number_walls_seen(view_env) < 2:
	        	#TODO navigate towards front
	        	pass
	    
	    elif self.state == STATE_DRIVE_THRU:
	    	view_env = []
	    	for reading in msg.ranges:
	    		if reading > .1 and reading < 5:
	    			view_env.append(1)
	    		else:
	    			view_env.append(0)
	    	#view-env is a 360-degree view of 0's and 1's.  we wanna go down the middle, straight ahead, towards 0's
	    	hit_one_at_left = 180
	    	hit_one_at_right = 181
	    	for i in range(0,180):
	    		if view_env[i] ==1:
	    			hit_one_at_left = i
	    			break
	    	for j in range(359,181):
	    		if view_env[j] == 1:
	    			hit_one_at_right = j
	    			break
	    	right_val = -1*(359-j)
	    	window = i - right_val
	    	if window > 270:
	    		self.state = STATE_DONE
	    	window_middle = (i + right_val)/2.0
	    	angle_thru_tunnel = 0
	    	if window_middle > 0:
	    		angle_thru_tunnel = window_middle
	    	elif window_middle < 0:
	    		#do we turn telling it to turn to degree 350, or degree -10?
	    		#change this based on that
	    		#I think it needs 350, hence this code
	    		#of not, just set angle_thru_tunnel to window_middle
	    		angle_thru_tunnel = 359 + window_middle


    def determine_turn_direction(self,view_env):
    	angle_to_fid = self.direction_towards_fiducial[1]
    	if angle_to_fid > 0 and angle_to_fid < 180: # angle on the left
    		return "RIGHT_TURN"
    	else:
    		return "LEFT_TURN"

    def 

    def number_walls_seen(self,view_env):
    	num_walls = 0
    	for i in range(1,len(view_env)):
    		if view_env[i]==1 and view_env[i-1]==0:
    			num_walls +=1
    	return num_walls
                

'''
Steps it has to take once it identifies as DominoRide:

- locate dominos and identify color
- make list of dominos in "knockover" color and their locations
- while still sees dominos to be knocked over:
	- navigate to domino (know its location)
	- hit domino
	- back up and re-check system

Steps it has to take once it identifies as TunnelRide:

- find front of tunnel (use computer vision)
- navigate to front of tunnel
	- be sure point into tunnel
- drive through tunnel without touching walls
- understand when it has completed tunnel
'''