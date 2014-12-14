#!/usr/bin/env python


from geometry_msgs.msg import Twist, Vector3, PoseStamped
import rospy
import tf
# from ar_pose import ARMarkers


class lookForRides():
    def __init__(self):
        self.fiducial = None # tuple of (dist,x,y,id)
        sub = rospy.Subscriber('ar_pose_marker', ARMarkers, setFiducial)

def IDtoName(markerID):
    # for IDs: 1 -> a, 2 -> b, ...
    # In ascii: a -> 97, b -> 98, ...
    return chr(markerID+96)


def getFiducials(data):
    ''' IN: an array of markers from ar_pose
        OUT: publishes the pose of the closest fiducial to pub
    '''
    markers = data.markers
    print "found some!"
    allFiducals = []
    for fiducial in markers:
        # TODO: verify how to get values from ar_pose
        markderID = fiducial.id
        pose = fiducial.pose.pose
        x = fiducial.pose.pose.point.x
        y = fiducial.pose.pose.point.y
        dist = (x**2 + y**2)**0.5
        allFiducals.append((dist,markderID, pose))
    closest = min(allFiducals)
    pub.publish(closest[2])



if __name__ == "__main__":
    rospy.init_node('between', anonymous=True)
    pub = rospy.Publisher('/move_base_simple/goal',PoseStamped)
    listener = tf.TransformListener()

    sub = rospy.Subscriber('ar_pose_marker', ARMarkers, getFiducials)
