#!/usr/bin/env python

import time
import math
import rospy
import random
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from ar_pose import ARMarkers

#import the games! 
import bridge
import tunnel
import dominos
import Fiducial

bridge = bridge.bridge()
tunnel = tunnel.tunnel()
dominos = dominos.dominos()
lookForRides = betweenRides.lookForRides()

def pickMode(mode):
    '''Selects which mode to use and executes it.
    
    INPUT: string saying which mode the robot should be using
    OUTPUT: none
    '''
    nextMode = 'tunnel' #TODO: make behavors return next modevand delete this line
    
    if mode == 'bridge':
        #bridge stuff
        bridge.printBridge()
        nextMode = bridge.do()
        return nextMode
    elif mode == 'dominos':
        #knock over dominos
        nextMode = dominos.do()
        return nextMode
    elif mode == 'tunnel':
        #go through tunnels!
        nextMode = tunnel.do()
        return nextMode
    elif mode == 'inbetweenRides':
        #look for next ride to go on
        nextMode = lookForRides.do()
        return nextMode
    else:
        #default case
#        nextMode = default.do()
        return nextMode


class ride():
    '''Generic carnival ride. 
    
    has all feducal things and anything else generally needed by the behavors
    '''
    
    def __init__(self):
        '''Initialize the generic ride'''
        return

    def getFiducal(self):
        ''''Returns closest fiducial
        
        INPUT: array of marker values (contain position information)
        OUTPUT: distance in meters of the neeto from the feducal radially'''

        fiducial = Fiducial()
        

    def distToFeducal(self):
        ''''Returns distance to the feducal
        
        INPUT: array of marker values (contain position information)
        OUTPUT: distance in meters of the neeto from the feducal radially'''
        return getFiducal[0]
        
    
    def whichFeducial(self):
        '''Determines the identity of the closest fiducial 
        
        INPUT: array of marker values (contain position information)
        OUTPUT: String saying which feducial the robot is looking at'''
        return getFiducal[-1]
        
        
if __name__ == '__main__':
    '''Initializes ROS processes and controls the state of the robot once 
    indivigual behaviors yield controls
    INPUT: none
    OUTPUT: none'''
    ride = ride()
#    bridge = bridge()
    pickMode('bridge')
    try:
#        rospy.init_node('carnival', anonymous=True)
#        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
#        sub = rospy.Subscriber('scan', LaserScan, read_in_laser)

        while not True: #rospy.is_shutdown(): #TODO uncomment ros stuff
            mode = pickMode(mode)
    except True: pass
#        rospy.ROSInterruptException: pass
