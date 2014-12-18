#!/usr/bin/env python
import math
import numpy as np

import cv2
import cv2.cv as cv
from cv_bridge import CvBridge, CvBridgeError

import roslib
import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Pose, Point, Quaternion
import tf
import tf.transformations as transform
roslib.load_manifest('ar_pose')
roslib.load_manifest('actionlib_msgs')
from ar_pose.msg import ARMarkers
from actionlib_msgs.msg import GoalStatusArray

from betweenRides import RideFinder


class DominoRide():

    def __init__(self,robo_pub):
        # rospy.init_node('domino_ride', anonymous=True)
        self.STATE_DONE = 0
        self.STATE_LOCATE_TARGETS = 1
        self.STATE_KNOCK_TARGET = 2
        self.STATE_RETURN = 3
        self.STATE_SEND_ONE = 4
        self.STATE_WAITING = 5
        self.STATE_FOUND_TARGET = 6
        self.state = self.STATE_WAITING
        self.red_fiducial_names = []
        self.red_centers = []
        self.targets = []
        self.poses = []
        self.pub = robo_pub
        self.goal_pub = rospy.Publisher('/move_base_simple/goal',PoseStamped)
        self.listener = tf.TransformListener()
        self.cam = rospy.Subscriber('camera/image_raw',Image, self.image_recieved)
        self.bridge = CvBridge()
        self.origin = PoseStamped()

    def reset(self):
        self.state = self.STATE_WAITING
        self.red_fiducial_names = []
        self.red_centers = []
        self.targets = []

    def do(self):
        self.state = self.STATE_LOCATE_TARGETS
        self.getOrigin()
        fiducial_sub = rospy.Subscriber('ar_pose_marker', ARMarkers, self.getFiducials)
        goal_sub = rospy.Subscriber('move_base/status',GoalStatusArray,self.toFiducial)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():

            print self.state
            if self.state == self.STATE_LOCATE_TARGETS:
                for name in self.red_fiducial_names:
                    n = self.IDtoName(name)
                    self.poses.append(self.getPose(n))
                print self.poses
                if len(self.poses)>0:
                    pose = self.poses[0]
                    self.toPose(pose)
                    self.state = self.STATE_KNOCK_TARGET
            elif self.state == self.STATE_FOUND_TARGET:
                print "wooo"
                for i in range(5):
                    self.publish_twist_message(-.5,-.5)
                if len(self.poses) > 0:
                    pose = self.poses[0]
                    self.toPose(pose)
                    self.state = self.STATE_KNOCK_TARGET
                else:
                    self.toOrigin()
                    self.state = self.STATE_RETURN

            elif self.state == self.STATE_DONE:
                self.publish_twist_message(0,0)
                return True
            
            r.sleep()
    '''
    def sendFiducial(self, name):
        print "sending"
        sent = False
        while not sent:
            try:
                (g_trans,g_rot) = self.listener.lookupTransform('/map',
                                                                '/'+name, 
                                                                rospy.Time(0))
                euler = transform.euler_from_quaternion(g_rot)
                rot = transform.quaternion_from_euler(0,0,euler[2]+3.14)
                point = Point(g_trans[0],g_trans[1],0 )
                quat = Quaternion(rot[0],rot[1],rot[2],rot[3])
                goal = PoseStamped(header = Header(stamp=rospy.Time.now(),
                                                   frame_id="/map"), 
                                   pose = Pose(point,quat))
                self.goal_pub.publish(goal)
                sent=True

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException),e:
                print e
        print "sent"
    '''
    def toFiducial(self,data):
        #print self.state
        if self.state == self.STATE_KNOCK_TARGET:
            if data.status_list:
                goal_status = data.status_list[-1].status
                #print "goal",goal_status
                if goal_status == 3:
                    print "GOAL"
                    self.state = self.STATE_FOUND_TARGET
                    if self.poses:
                        self.poses.pop(0)
        if self.state == self.STATE_RETURN:
            if data.status_list:
                goal_status = data.status_list[-1].status
                if goal_status == 3:
                    self.state == self.STATE_DONE
        

    def getFiducials(self,data):
        ''' IN: an array of markers from ar_pose
            OUT: publishes the pose of the closest fiducial to pub
        '''
        # Find fiducials
        if self.state == self.STATE_LOCATE_TARGETS:
            markers = data.markers
            allFiducals = []
            print "find fiducials"
            for fiducial in markers:
                markderID = fiducial.id
                fpose = fiducial.pose.pose
                x = fpose.position.x
                allFiducals.append((x, fpose, markderID))
            # If found any:
            if allFiducals:
                self.publish_twist_message(0,0)
                allFiducals.sort(key=lambda tup: tup[0])
                if len(allFiducals) == len(self.targets):
                    for i in range(len(allFiducals)):
                        if self.targets[i] in self.red_centers:
                            self.red_fiducial_names.append(allFiducals[i][2])
            # if len(self.red_fiducial_names) > 0:
            #     self.state = self.STATE_LOCATE_TARGETS

    def getOrigin(self):
        saved = False
        while not saved:
            try:
                (trans,rot) = self.listener.lookupTransform('/map','/base_link',rospy.Time(0))
                point = Point(trans[0],trans[1],trans[2])
                quat = Quaternion(rot[0],rot[1],rot[2],rot[3])
                self.origin = PoseStamped(header = Header(stamp=rospy.Time.now(),
                                                       frame_id="/map"), 
                                       pose = Pose(point,quat))
                saved = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException),e:
                print e
    def toOrigin(self):
        self.goal_pub.publish(self.origin)
        #self.state = self.STATE_RETURN

    def getPose(self,name):
        print "getting ", name
        while True:
            try:
                (trans,rot) = self.listener.lookupTransform('/map',
                                                                '/'+name, 
                                                                rospy.Time(0))
                euler = transform.euler_from_quaternion(rot)
                rot = transform.quaternion_from_euler(0,0,euler[2]+3.14)
                point = Point(trans[0],trans[1],0 )
                quat = Quaternion(rot[0],rot[1],rot[2],rot[3])
                pose = PoseStamped(header = Header(stamp=rospy.Time.now(),
                                                   frame_id="/map"), 
                                   pose = Pose(point,quat))
                return pose
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException),e:
                print e
    def toPose(self,pose):
        self.goal_pub.publish(pose)
        #self.state = self.STATE_RETURN

    def publish_twist_message(self, vel, ang_vel):
        msg = Twist (Vector3 (vel, 0, 0), Vector3 (0, 0, ang_vel))
        self.pub.publish (msg)

    def image_recieved(self,msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow('test',img)
        cv2.waitKey(3)
        if self.state == self.STATE_LOCATE_TARGETS:
            red_thresh, blue_thresh = self.preprocess_img(img)
            self.red_centers = self.locate_color(red_thresh, img)
            blue_centers = self.locate_color(blue_thresh, img)          
            self.targets = self.red_centers + blue_centers
            self.targets.sort(key=lambda tup: tup[1])
            #self.state = STATE_KNOCK_TARGET

    def preprocess_img(self,cimg):
        img_HSV = cv2.cvtColor(cimg, cv.CV_BGR2HSV)
        
        red_gaussian = cv2.GaussianBlur(img_HSV, (9,9), 2, 2)

        blue_gaussian = cv2.GaussianBlur(img_HSV, (9,9), 2, 2)

        red_Threshed = cv2.inRange(red_gaussian, np.array((0,0,0)), np.array((10,255,255)))

        blue_Threshed = cv2.inRange(blue_gaussian, np.array((60,0,30)), np.array((70,255,255)))

        cv2.imshow("threshed", blue_Threshed)
        cv2.waitKey(3)
        return red_Threshed, blue_Threshed

    def locate_color(self,thresh, cimg):
        centers = []
        areas = []
        ret,gray = cv2.threshold(thresh,127,255,0)
        gray2 = gray.copy()
        mask = np.zeros(gray.shape,np.uint8)
        contours, hier = cv2.findContours(gray,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            #if 200<cv2.contourArea(cnt)<5000:
            M = cv2.moments(cnt)
            if M['m00'] != 0 and 1000<cv2.contourArea(cnt):
                cv2.drawContours(cimg,[cnt],0,(0,255,0),2)
                cv2.drawContours(mask,[cnt],0,255,-1)
                M = cv2.moments(cnt)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                area = cv2.contourArea(cnt)
                areas.append(area)
                tooClose = False
                for center in centers:
                    if center[0] - cx < 5 and center[1] - cy < 5:
                        tooClose = True
                if not tooClose:        
                    centers.append((cx,cy))
                    cv2.circle(cimg,(cx,cy),2,(0,0,255),3)
        cv2.imshow('contors',cimg)
        cv2.waitKey(3)
        return centers

    def IDtoName(self,markerID):
        d = {0: 'a', 1:'b', 2:'c', 3:'d', 4:'f', 5:'g'}
        # for IDs: 0 -> a, 1 -> b, ...
        # In ascii: a -> 97, b -> 98, ...
        #return chr(markerID+97)
        return d[markerID]

if __name__ == '__main__':
    try:
        rospy.init_node('between', anonymous=True)
        robo_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        node = DominoRide(robo_pub)
        node.state = node.STATE_LOCATE_TARGETS
        node.do()
    except rospy.ROSInterruptException: pass
