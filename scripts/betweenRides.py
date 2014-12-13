#!/usr/bin/env python


from geometry_msgs.msg import Twist, Vector3, PoseStamped
import rospy
import tf
from ar_pose import ARMarkers


class lookForRides():
    def __init__(self):
        self.fiducial = None # tuple of (dist,x,y,id)
        sub = rospy.Subscriber('ar_pose_marker', ARMarkers, setFiducial)


def getFiducials(data):
    markers = data.markers
    print 'found some'



if __name__ == "__main__":
    rospy.init_node('between', anonymous=True)
    pub = rospy.Publisher('/move_base_simple/goal',PoseStamped)
    listener = tf.TransformListener()

    sub = rospy.Subscriber('ar_pose_marker', ARMarkers, getFiducials)
