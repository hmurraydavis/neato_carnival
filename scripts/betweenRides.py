#!/usr/bin/env python


from geometry_msgs.msg import Twist, Vector3, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rospy
import tf
import roslib
roslib.load_manifest('ar_pose')
from ar_pose.msg import ARMarkers
import tf.transformations as transform


class lookForRides():
    def __init__(self):
        self.fiducial = None # tuple of (dist,x,y,id)
        # sub = rospy.Subscriber('ar_pose_marker', ARMarkers, getFiducial)

def IDtoName(markerID):
    # for IDs: 0 -> a, 1 -> b, ...
    # In ascii: a -> 97, b -> 98, ...
    return chr(markerID+97)


def getFiducials(data):
    ''' IN: an array of markers from ar_pose
        OUT: publishes the pose of the closest fiducial to pub
    '''
    markers = data.markers
    allFiducals = []
    for fiducial in markers:
        # TODO: verify how to get values from ar_pose
        markderID = fiducial.id
        fpose = fiducial.pose.pose
        x = fpose.position.x
        y = fpose.position.y
        dist = (x**2 + y**2)**0.5
        allFiducals.append((dist,markderID, fpose))
    if allFiducals:
        closest = min(allFiducals)
        sendFiducials(closest[1])

def sendFiducials(fiducial_id):
    name = IDtoName(fiducial_id)
    try:
        (g_trans,g_rot) = listener.lookupTransform('/map','/'+name+"_goal", rospy.Time(0))
        euler = transform.euler_from_quaternion(g_rot)
        rot = transform.quaternion_from_euler(0,0,euler[2]+3.14)
        point = Point(g_trans[0],g_trans[1],0 )
        quat = Quaternion(rot[0],rot[1],rot[2],rot[3])
        goal = PoseStamped(header = Header(stamp=rospy.Time.now(),
                                           frame_id="/map"), 
                           pose = Pose(point,quat))
        pub.publish(goal)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print "well, that worked well"




if __name__ == "__main__":
    rospy.init_node('between', anonymous=True)
    pub = rospy.Publisher('/move_base_simple/goal',PoseStamped)
    listener = tf.TransformListener()
    fiducials = []

    
    while not rospy.is_shutdown():
        sub = rospy.Subscriber('ar_pose_marker', ARMarkers, getFiducials)
        rospy.spin()

