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

from tunnelRide import TunnelRide
from dominos import DominoRide
from betweenRides import RideFinder


if __name__ == "__main__":
    rospy.init_node('between', anonymous=True)
    robo_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    between = RideFinder(robo_pub)
    tunnel = TunnelRide(robo_pub)
    dominos = DominoRide(robo_pub)

    while True:
        print "----NEXT---"
        between.reset()
        mode = between.do()
        print mode
        if mode == "tunnel":
            print "DOING TUNNEL"
            tunnel.reset()
            tunnel.do()
            mode = 'between'
        elif mode == "dominos":
            print "DOING DOMINOS"
            dominos.reset()
            dominos.do()
            mode = "between"
        else:
            break

