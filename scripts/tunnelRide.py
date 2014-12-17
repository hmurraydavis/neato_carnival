#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import cv2

class TunnelRide():

	def __init__(self):
		#rospy.init_node('tunnel_ride', anonymous=True)
		self.STATE_DRIVE_THRU = 1
		self.STATE_DONE = 2
		self.state = self.STATE_DRIVE_THRUd2
		self.sub = rospy.Subscriber('scan', LaserScan, self.scan_received)
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)


	def do(self):
		r = rospy.Rate(10)
		while not rospy.is_shutdown() and self.state != self.STATE_DONE:
			print self.state
			# check state and run appropriate command
			if self.state == self.STATE_NAVIGATE_TO_FRONT:
				vel, ang_vel = self.find_front_of_tunnel()
				self.publish_twist_message(vel, ang_vel)

			elif self.state == self.STATE_DRIVE_THRU:
				pass
				# 	vel, ang_vel = self.drive_thru_tunnel_with_lidar()
			elif self.state == self.STATE_DONE:
				self.publish_twist_message(0,0)

			r.sleep ()
		return True

	def publish_twist_message(self, vel, ang_vel):
		msg = Twist (Vector3 (vel, 0, 0), Vector3 (0, 0, ang_vel))
		self.pub.publish (msg)

	def drive_thru_tunnel_with_lidar(self):
		pass

	def find_front_of_tunnel(self):
		#TODO: we have 2 fiducials 
		distance_to_fid1 = fid1[0]
		distance_to_fid2 = fid2[0]
		angle_to_fid1 = fid1[1]
		angle_to_fid2 = fid2[1]
		distance_epsilon = .25 #tweak this
		angle_epsilon = math.pi/8
		if abs(distance_to_fid1 - distance_to_fid2) > distance_epsilon: #one is closer than the other
			if distance_to_fid1 == 0: #we only see fid2
				vel = 0.2
				ang_vel = 0.5
				return vel, ang_vel
			elif distance_to_fid2 == 0: #we only see fid1
				vel = 0.2
				ang_vel = -0.5
				return vel, ang_vel
			else: #we see both
				if distance_to_fid2 > distance_to_fid1:
					return 0.2, angle_to_fid2*.5 # proportional control on angle
				else:
					return 0.2, angle_to_fid1*.5 # proportional control on angle
		else: # they are about equidistant
			#make sure angle is right
			if abs(angle_to_fid1 - angle_to_fid2) > angle_epsilon:
				desired_angle = (angle_to_fid1 + angle_to_fid2)/2.0
				return 0, desired_angle*.5 # proportional control
			else:	
				self.state = self.STATE_DRIVE_THRU

	def scan_received(self, msg):

		print "scanning"
		if self.state == self.STATE_DRIVE_THRU:
			view_env = []
			for reading in msg.ranges:
				if reading > .1 and reading < 2:
					view_env.append(1)
				else:
					view_env.append(0)
			print view_env
			#view-env is a 360-degree view of 0's and 1's.  we wanna go down the middle, straight ahead, towards 0's
			window = 0
			window_middle = 0
			if view_env[0] == 0:
				hit_one_at_left = 0
				hit_one_at_right = 181
				for i in range(0,180):
					if view_env[i] ==1:
						hit_one_at_left = i
						break
				j=359
				while j >=181:
					if view_env[j] == 1:
						hit_one_at_right = j
						break
					j-=1
				right_val = -1*(359-hit_one_at_right)
				window = hit_one_at_left - right_val
				window_middle = (hit_one_at_left + right_val)/2.0
			else:
				longest_opening = 0
				prev = 1
				start = 0
				current_streak=0
				for i in range(-60,60):
					if view_env[i]==0 and prev==0:
						if current_streak == 0:
							start = i
						current_streak +=1
					elif view_env[i]==1 and prev==0:
						if current_streak > longest_opening:
							longest_opening = current_streak
						current_streak = 0
					prev = view_env[i]
				end = start + longest_opening
				print "lo",longest_opening,start,end
				window_middle = (end - start)/2.0
			if window > 200:
				self.state = self.STATE_DONE			
			angle_thru_tunnel = window_middle
			print "att",angle_thru_tunnel
			self.publish_twist_message(.15, angle_thru_tunnel*.025) # proportional control



if __name__ == '__main__':
	try:
		node = TunnelRide()
		node.do()
	except rospy.ROSInterruptException: pass
