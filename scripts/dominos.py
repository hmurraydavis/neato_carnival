#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan, Image
import cv2
import cv2.cv as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

STATE_DONE = 0
STATE_LOCATE_TARGETS = 1
STATE_KNOCK_TARGET = 2
STATE_RETURN = 3

class DominoRide():

	def __init__(self):
		rospy.init_node('domino_ride', anonymous=True)
		self.state = STATE_LOCATE_TARGETS
		#self.sub = rospy.Subscriber('scan', LaserScan, self.scan_received)
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.cam = rospy.Subscriber('camera/image_raw',Image, self.image_recieved)
		self.bridge = CvBridge()
		self.targets = []

	def do(self):
		r = rospy.Rate(10)
		while not rospy.is_shutdown():

			if self.state == STATE_LOCATE_TARGETS:
				pass

			elif self.state == STATE_KNOCK_TARGET:
				pass

			elif self.state == STATE_RETURN:
				pass
			
			elif self.state == STATE_DONE:
				self.publish_twist_message(0,0)
			
			r.sleep ()


	def publish_twist_message(self, vel, ang_vel):
		msg = Twist (Vector3 (vel, 0, 0), Vector3 (0, 0, ang_vel))
		self.pub.publish (msg)

	def image_recieved(self,msg):
		img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
		cv2.imshow('test',img)
		cv2.waitKey(3)
		if self.state == STATE_LOCATE_TARGETS:
			red_thresh = self.preprocess_img(img)
			centers = self.locate_red(red_thresh, img)
			self.targets = centers
			#self.state = STATE_KNOCK_TARGET
		elif self.state == STATE_KNOCK_TARGET:

			rows,cols,stuff = np.shape(img)
			middle = cols/2
			target = self.targets[0]
			print target
			if target[0] < middle:
				self.publish_twist_message(0,.5)
			else:
				self.publish_twist_message(0,-.5)


	def preprocess_img(self,cimg):
		img_HSV = cv2.cvtColor(cimg, cv.CV_BGR2HSV)
		
		red_gaussian = cv2.GaussianBlur(img_HSV, (9,9), 2, 2)

		## TODO: FIND REAL RED VALUES!
		red_Threshed = cv2.inRange(red_gaussian, np.array((0,50,50)), np.array((10,255,255)))

		cv2.imshow("threshed", red_Threshed)
		cv2.waitKey(3)
		return red_Threshed

	def locate_red(self,red_thresh, cimg):
		centers = []
		areas = []
		ret,gray = cv2.threshold(red_thresh,127,255,0)
		gray2 = gray.copy()
		mask = np.zeros(gray.shape,np.uint8)
		contours, hier = cv2.findContours(gray,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
		for cnt in contours:
			if 200<cv2.contourArea(cnt)<5000:
				cv2.drawContours(cimg,[cnt],0,(0,255,0),2)
				cv2.drawContours(mask,[cnt],0,255,-1)
				M = cv2.moments(cnt)
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				area = cv2.contourArea(cnt)
				areas.append(area)
				centers.append((cx,cy))
				cv2.circle(cimg,(cx,cy),2,(0,0,255),3)
		cv2.imshow('contors',cimg)
		cv2.waitKey(3)
		return centers


if __name__ == '__main__':
	try:
		node = DominoRide()
		node.do()
	except rospy.ROSInterruptException: pass
