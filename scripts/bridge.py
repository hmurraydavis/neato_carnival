#!/usr/bin/env python

import numpy as np
import cv2
import math
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Bridge():
    '''Behaviors to let the robot traverse the bridge.'''

    
    
    def closeImages(self):
        """
        Close images opened by Open CV, addresses the different method of closing
        images based on command line or called usage.
        INPUT: none
        OUTPUT: none
        **Closes all open cv2 images either on a keystroke (comman line use) or 
            immediately (called from another python script)
        """
        if False: #__name__ == '__main__': #butchered because ORS calling name needs waitkey(3)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            cv2.waitKey(3)
    
    
    def __init__(self, publisher): #below closeImages so I can use close images in it!
        '''Initialize bridge object, subscribe to camera and publishers
        
        INPUT: ROS Publisher to which bridge topics should be published
        OUTPUT: none
        EFFECT: Subscribe to ROS publisher
        Subscribe to ROS camera to get images
        Set default travel speed as stoped and forward motion, when used, to 1'''
        image = self.colorImgPreProcess(image)
        self.pub = publisher
        self.cam = rospy.Subscriber('camera/image_raw',Image, self.image_recieved)
        self.bridge = CvBridge()
        self.xspeed = 1
        self.cmd_vel = Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0))
        return
        
    def image_received(self, image_message):
        """
        Process image from camera and set the desired cmd_vel, callback function
        """
        # Convert the image message to something usable by opencv
        # http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
        # Note that mono8 and bgr8 are the two image encodings expected by most OpenCV functions.
        cv_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        image_data = extract_data(cv_image)
        linear_velocity, angular_velocity = self.clf.predict(image_data)
        self.cmd_vel = Twist(linear=Vector3(x=linear_velocity), angular=Vector3(z=angular_velocity))
        rospy.loginfo(self.cmd_vel)
            
            
    def colorImgPreProcess(self, image):
        """
        Prepare images to be analyzed in binary form by appling generic filtering.
        This makes them easier to work with and the resulting image less noisy.
        
        INPUT: image for pre-processing. Should be in color, though b&w should work.
        OUTPUT: returns a RGB image which has been filtered and looks nicer.
        """
        #do processing on the image while it's still in color
        image = cv2.medianBlur(image, 7)  #kernal size must be odd
        #image = cv2.bilateralFilter(image, 9, 75, 75) #TODO: uncomment when it won't cause C++ errors with ROS
        #self.closeImages() #uncomment if showing output image
        return image
        
        
    def getContoursOfBridge(self, im):
        '''Thresholds the image with hue to detect the red lines on the bridge, 
        them place a mask of this on a white background
        
        INPUT: image color image
        OUTPUT: BGR colorspace image derived by putting the mask of bridge 
                    from color thresholding on a white background
        '''
        #Make a white image so the mask can be on a white background
        white_image = np.zeros((len(im), len(im[0]), 3), np.uint8)
        white_image[:] = (0,0,0)
        
        cv2.imshow('image', im)
        im = self.colorImgPreProcess(im)
        
        #upper and lower BGR color bounds declared:
        rLower = 40
        gLower = 40
        bLower = 40

        BUpper = 255
        GUpper = 230
        RUpper = 230
        
        lower_redColor = np.array([bLower, gLower, rLower])
        upper_redColor = np.array([BUpper, GUpper, RUpper])
        
        #Make masks for taking the bridge and putting it on a background
        RGBMask = cv2.inRange(im, lower_redColor, upper_redColor)
        mask_inv = cv2.bitwise_not(RGBMask)
        # Threshold the RGB image to get only red colors
        res = cv2.bitwise_and(im,im,mask = RGBMask) #Get the bridge parts of the image
        res = cv2.bitwise_not(white_image, res, mask = mask_inv) #put them on a white background

#####        for row in res:
#####            for collumn in res:
#####                pixel = res[row, collumn][0][0]
#####                if pixel.tolist() == [0,0,0]:
#####                    pixel = np.array([0,0,0])
######                    res[row,collumn] = [255,255,255]
        cv2.imshow('rturened (res) image', res)
        self.closeImages()
        return res
          
    def findEdgesOfBridge(self, im):
        '''Finds the edges of the bridge so the robot can avoid driving off the 
        bridge.'''
        ## Im should be ready to have edge detection run on it. 
        for i in range(2):
            im = b.colorImgPreProcess(im)
            #im = cv2.blur(im,(5,5))
            
        #cv2.Canny(image, threshold1, threshold2[, edges[, apertureSize[, L2gradient]]]) --> edges
        edges = cv2.Canny(im,0,1155,apertureSize = 5)  #threshold 1 down --> more edges, threshold 2 down --> longer edges
        #cv2.imshow("edges detected on input image", edges)
        
        #cv2.HoughLinesP(image, rho, theta, threshold[, lines[, minLineLength[, maxLineGap]]]) -->lines
        lines = cv2.HoughLinesP(edges, 1, .1, 10)
        
        if len(lines)>0: #only print if lines are found so it doesn't error
            #print 'type: ', type(lines), 'Length: ', len(lines), 'lies are: \n', lines
            print 'number of lines: ', len(lines[0])
            pass
        else: print 'no lines found'
        
        for line in lines[0]:

            x1,y1,x2,y2 = line
            #cv2.line(img, pt1, pt2, color[, thickness[, lineType[, shift]]]) --> none
            cv2.line(im, (x1,y1), (x2,y2), [150,0,100],3)
        #cv2.imshow('image with lines', im)
        self.closeImages()
        return
    
    def cropBridge(self, im):
        dimensions_image = im.shape
        height = dimensions_image[0]
        width = dimensions_image[1]
        #print width ,'x', height
        crop_img = im[int(height*1.3/3):height, 0:width]            
        cv2.imshow('Croped image', crop_img)
        self.closeImages()
        return crop_img
        
    def getLargestGap(self, im):
        dimensions_image = im.shape
        height = dimensions_image[0]
        width = dimensions_image[1]
        threshold_for_gap = 760
        largest_gap = {'startPT':0, 'endPT':0, 'lengthGap':0} #initialize a zero dictionary representing the largest gap
        length_current_gap = 0
        inGap = False
        
        for collumn_number in range(width):
            inGapPast = inGap
            collumn = im[:,collumn_number]
            #print collumn, 'length column: ', len(collumn), '\n'
            collumn_hue_sum = np.ndarray.sum(collumn)
            average_pixel_column = collumn_hue_sum/height
            if average_pixel_column < threshold_for_gap: #case where the side of the bridge is in collumn:
                inGap = False
            elif average_pixel_column >= threshold_for_gap: #in a gap collumn: 
                length_current_gap = length_current_gap + 1
                inGap = True
            if ((inGap == True) and (inGapPast == False)) or (collumn_number == 0): #transioning into gap
                curentStartPt = collumn_number
            if ((inGap == False) and (inGapPast == True)) or (collumn_number == width): #exiting gap
                currentEndPt = collumn_number
                if length_current_gap > largest_gap['lengthGap']:
                    largest_gap['lengthGap'] = length_current_gap
                    largest_gap['startPT'] = curentStartPt
                    largest_gap['endPT'] = currentEndPt
        #cv2.rectangle(img, pt1, pt2, color[, thickness[, lineType[, shift]]])
        cv2.rectangle(im, (largest_gap['startPT'],0), (largest_gap['endPT'],30), [0,200,0], 3)
        
        #draw the line the robt thins is the center or where it is on the bridge:
        robotCenterInFrame = int(width/2)
        #red line
        cv2.line(im, (robotCenterInFrame,0),(robotCenterInFrame,40), [0,0,200], 3)
        
        #draw the line that's the center of the largest gap
        centerOfGap = largest_gap['startPT'] + (largest_gap['lengthGap']/2)
        cv2.line(im, (centerOfGap,0), (centerOfGap, 40), [200,0,0], 3)
        
        cv2.imshow('largest gap', im)
        self.closeImages()
        return {'center_of_gap': centerOfGap, 'robot_center':robotCenterInFrame}
        
    def steerRobotOnBridge(self, centerOfGap, robotCenter):
        K = .06 #PID constant
        portion = math.abs(centerOfGap - robotCenter)
        turnAmount = K * portion
        
        speed=Vector3(float(xspeed),0.0,0.0) 
        
        if centerOfGap < robotCenter: #Gap is to the left of the robot's center
            #turn left
            angular=Vector3(0.0,0.0,math.fabs(float(turnAmount)))
        elif centerOfGap > robotCenter:
            angular=Vector3(0.0,0.0,math.fabs(float(-1*turnAmount)))
        else:
            angular=Vector3(0.0,0.0,0.0)
        pub.publish(speed, angular)
    #print 'lft turn ', lft_trn_amt
    #Construct publish data:
        return Twist(speed,angular)
    
        
    def printBridge(self):
        print 'Bridges are pretty!'
        
#    def run(self):
#        '''Get camera images and publish the current, set point velocity
#        '''
#        self.running = True
#        self.sub = rospy.Subscriber('camera/image_raw', Image, self.image_received)
#        rate = rospy.Rate(20)
#        while not rospy.is_shutdown() and self.running:
#            self.pub.publish(self.cmd_vel)
#            rate.sleep()
    
    def image_recieved(self,msg):
        print "image"
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        #cv2.imshow('test',img)
        self.do_stuff(img)
        cv2.waitKey(3)
        
        
    def do_stuff(self, image):
        image = b.colorImgPreProcess(image)
        image = b.getContoursOfBridge(image)
#        b.findEdgesOfBridge(image)
        image = b.cropBridge(image)
        gap_and_robot_locations = b.getLargestGap(image)
        
        b.steerRobotOnBridge(gap_and_robot_locations['centerOfGap'], gap_and_robot_locations['robot_center'])
        b.closeImages()
        
    def do(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep ()
        
if __name__ == '__main__':
    
    rospy.init_node('between', anonymous=True)
    robo_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    b = Bridge(robo_pub)
    b.do()

    
