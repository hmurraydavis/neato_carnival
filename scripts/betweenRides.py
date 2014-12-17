#!/usr/bin/env python

from geometry_msgs.msg import Twist, Vector3, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rospy
import tf
import roslib
roslib.load_manifest('ar_pose')
roslib.load_manifest('actionlib_msgs')
from ar_pose.msg import ARMarkers
from actionlib_msgs.msg import GoalStatusArray
import tf.transformations as transform


class RideFinder():
    def __init__(self):
        self.fiducial = Pose(orientation=Quaternion(w=1))
        self.name = 'z'
        self.state = "finding" # states = ["finding","going","arrived"]

        self.listener = tf.TransformListener()
        self.goal_pub = rospy.Publisher('/move_base_simple/goal',PoseStamped)
        self.robo_pub = rospy.Publisher('/cmd_vel', Twist)

    def do(self):
        fiducial_sub = rospy.Subscriber('ar_pose_marker', ARMarkers, self.getFiducials)
        goal_sub = rospy.Subscriber('move_base/status',GoalStatusArray,self.toFiducial)
        while not rospy.is_shutdown():
            if self.state == "finding":
                self.findFiducials()
            elif self.state == "going":
                pass # toFiducial handles this.
            elif self.state == "arrived":
                return self.nextMode()
            else:
                print "invalid state"
                return self.nextMode()

    def IDtoName(self,markerID):
        # for IDs: 0 -> a, 1 -> b, ...
        # In ascii: a -> 97, b -> 98, ...
        return chr(markerID+97)

    def getFiducials(self,data):
        ''' IN: an array of markers from ar_pose
            OUT: publishes the pose of the closest fiducial to pub
        '''
        # Find fiducials
        markers = data.markers
        allFiducals = []
        for fiducial in markers:
            markderID = fiducial.id
            fpose = fiducial.pose.pose
            x = fpose.position.x
            y = fpose.position.y
            dist = (x**2 + y**2)**0.5
            allFiducals.append((dist, markderID, fpose))
        # If found any:
        if allFiducals:
            closest = min(allFiducals)
            if self.state == "finding":
                self.fiducial = closest[2]
                self.name = self.IDtoName(closest[1])
                self.sendFiducial()
                print "sent"
                self.state = "going"

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
                self.state = 'arrived'

    def findFiducials(self):
        speed = Twist(angular=Vector3(z=0.2))
        self.robo_pub.publish(speed)
        # getFiducial deals with states

    def nextMode(self):
        # set up which fiducal coresponds to which challenge
        if self.name == 'a':
            return 'tunnel'
        elif self.name == 'b':
            return 'bridge'
        elif self.name == 'c':
            return 'dominos'
        else:
            return 'error'

if __name__ == "__main__":
    rospy.init_node('between', anonymous=True)
    go = RideFinder()
    print go.do()

