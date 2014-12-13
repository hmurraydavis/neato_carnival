from geometry_msgs.msg import PoseWithCovariance, Twist, Vector3
from ar_pose.msg import ARMarkers


class Fiducial():
    def __init__(self):
        self.x
        self.y
        self.dist
        self.id
        sub = rospy.Subscriber('ar_pose_marker', ARMarkers, self.setFiducial)


    def setFiducial(self,data):
        ''' Run through the list of fiducials identified by ar_pose set self to the closest fiducial
            for incoming message type:
            http://docs.ros.org/hydro/api/ar_pose/html/msg/ARMarker.html
        '''
        markers = data.markers
        allFiducals = []
        for fiducial in markers:
            # TODO: verify how to get values from ar_pose message
            fid = fiducial.id
            location = fiducial.pose.pose.point
            orientation = fiducial.pose.pose.orientation

            dist = (x**2+y**2)**0.5
            allFiducals.append((dist,x,y,fid))
        fiducial = min(allFiducals)
        self.dist = fiducial[0]
        self.x = fiducial[1]
        self.y = fiducial[2]
        self.id = fiducial[3]

    def toFiducial(self,end_dist, end_angle):
        '''Actions for going to an identified fiducial
        OUTPUTS: Twist msg'''
        # publish a geometry_msgs/PoseStamped messg to move_base_simple/goal

