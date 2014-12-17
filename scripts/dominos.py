#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, Image
import cv2
import cv2.cv as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from betweenRides import RideFinder

from geometry_msgs.msg import Twist, Vector3, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import tf
import roslib
roslib.load_manifest('ar_pose')
roslib.load_manifest('actionlib_msgs')
from ar_pose.msg import ARMarkers
from actionlib_msgs.msg import GoalStatusArray
import tf.transformations as transform



class DominoRide(RideFinder):

    def __init__(self):
        rospy.init_node('domino_ride', anonymous=True)
        self.STATE_DONE = 0
        self.STATE_LOCATE_TARGETS = 1
        self.STATE_KNOCK_TARGET = 2
        self.STATE_RETURN = 3
        self.fiducial = Pose(orientation=Quaternion(w=1))
        self.state = self.STATE_LOCATE_TARGETS
        self.red_fiducial_names = []
        #self.sub = rospy.Subscriber('scan', LaserScan, self.scan_received)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.cam = rospy.Subscriber('camera/image_raw',Image, self.image_recieved)
        self.bridge = CvBridge()
        self.targets = []

    def do(self):
        fiducial_sub = rospy.Subscriber('ar_pose_marker', ARMarkers, self.getFiducials)
        goal_sub = rospy.Subscriber('move_base/status',GoalStatusArray,self.toFiducial)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            print self.state
            if self.state == self.STATE_KNOCK_TARGET or self.state == self.STATE_RETURN:
                if len(self.red_fiducial_names) > 0:
                    if self.state == self.STATE_KNOCK_TARGET:
                        self.name = self.IDtoName(self.red_fiducial_names[0])
                        self.sendFiducial()
                    elif self.state == self.STATE_RETURN:
                        for i in range(5): #back up
                            self.publish_twist_message(-2,0)
                        self.state = self.STATE_KNOCK_TARGET
                else:
                    self.state = self.STATE_DONE

            elif self.state == self.STATE_DONE:
                self.publish_twist_message(0,0)
                break
            
            r.sleep ()

    def sendFiducial(self):
        print "sending"
        sent = False
        while not sent:
            try:
                (g_trans,g_rot) = self.listener.lookupTransform('/map',
                                                                '/'+self.name+"_goal", 
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
                pass

    def toFiducial(self,data):
        if data.status_list:
            goal_status = data.status_list[-1].status
            if self.state == "going" and goal_status == 3:
                self.state = self.STATE_RETURN

    def getFiducials(self,data):
        ''' IN: an array of markers from ar_pose
            OUT: publishes the pose of the closest fiducial to pub
        '''
        # Find fiducials
        if self.state == self.STATE_LOCATE_TARGETS:
            markers = data.markers
            allFiducals = []
            print "blah"
            for fiducial in markers:
                print "finding"
                markderID = fiducial.id
                fpose = fiducial.pose.pose
                x = fpose.position.x
                y = fpose.position.y
                dist = (x**2 + y**2)**0.5
                allFiducals.append((x, fpose, markderID))
            # If found any:
            if allFiducals:
                allFiducals.sort(key=lambda tup: tup[0])
                print allFiducals
                print self.targets
                if len(allFiducals) == len(self.targets):
                    for i in range(len(allFiducals)):
                        if self.targets[i] in red_centers:
                            self.red_fiducial_names.append(allFiducals[i][2])

                        # self.fiducial = closest[2]
                        # self.name = self.IDtoName(closest[1])
                        # self.sendFiducial()
            if len(self.red_fiducial_names) > 0:
                self.state = self.STATE_KNOCK_TARGET

    def publish_twist_message(self, vel, ang_vel):
        msg = Twist (Vector3 (vel, 0, 0), Vector3 (0, 0, ang_vel))
        self.pub.publish (msg)

    def image_recieved(self,msg):
        print "im rec"
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow('test',img)
        cv2.waitKey(3)
        if self.state == self.STATE_LOCATE_TARGETS:
            red_thresh, blue_thresh = self.preprocess_img(img)
            red_centers = self.locate_color(red_thresh, img)
            blue_centers = self.locate_color(blue_thresh, img)          
            self.targets = red_centers + blue_centers
            self.targets.sort(key=lambda tup: tup[1])
            #self.state = STATE_KNOCK_TARGET

    def preprocess_img(self,cimg):
        img_HSV = cv2.cvtColor(cimg, cv.CV_BGR2HSV)
        
        red_gaussian = cv2.GaussianBlur(img_HSV, (9,9), 2, 2)

        blue_gaussian = cv2.GaussianBlur(img_HSV, (9,9), 2, 2)

        ## TODO: FIND REAL RED VALUES!
        red_Threshed = cv2.inRange(red_gaussian, np.array((0,50,50)), np.array((10,255,255)))

        blue_Threshed = cv2.inRange(blue_gaussian, np.array((50,0,0)), np.array((100,255,255)))

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
