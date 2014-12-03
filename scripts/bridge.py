#may want to use a SIFT descriptor

import numpy as np
import cv2


class bridge():
    '''Behaviors to let the robot traverse the bridge and execute the behavior.'''
    
    
    def closeImages(self):
        """
        Close images opened by Open CV, addresses the different method of closing
        images based on command line or called usage.
        INPUT: none
        OUTPUT: none
        **Closes all open cv2 images either on a keystroke (comman line use) or 
            immediately (called from another python script)
        """
        if __name__ == '__main__':
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            cv2.waitKey(3)
        
    def __init__(self):
        image = cv2.imread('gimpBridge.jpg', cv2.IMREAD_COLOR)
        print image
        cv2.imshow('gimp bridge', image)
        self.closeImages()
        return
        
    
    def colorImgPreProcess(self, image):
        """
        Prepare images to be analyzed in binary form by appling generic filtering.
        This makes them easier to work with and the resulting image less noisy.

        INPUT: image for pre-processing. Should be in color, though b&w should work.
        OUTPUT: returns a RGB image which has been filtered and looks nicer.
        """
        #do processing on the image while it's still in color
        image = cv2.medianBlur(image, 7)  #kernal size must be odd
        image = cv2.bilateralFilter(image, 9, 75, 75)
        closeImages()
        return image  
    
    
    def imageProcessRedCups(self, image):
        """
        Process an input image with Open CV methods to focus on red cup like shapes

        INPUT: image
        OUTPUT: matrix of singular values of the binary image for prcessing
            with the ridge regression
        """
        #    global colorImgInput #from mouse clicking stuff
        #pre-process image while it's in color
        for i in range(2): #run color filtering 2x for better results
            image = colorImgPreProcess(image)  

        # define range of red color for HSV
        lower_redColor = np.array([30, 30, 160])
        upper_redColor = np.array([70, 80, 230])

        # Threshold the RGB image to get only red colors
        RGBMask = cv2.inRange(image, lower_redColor, upper_redColor)

        #dialate image to filter it:

        kernel = np.ones((6, 6), np.uint8)  #make 5 by 5 kernal size filter
        rgbDilated = cv2.dilate(RGBMask, kernel, iterations=1)

        #    cv2.setMouseCallback("color mask",mouse_event,[])
        cv2.imshow("color mask", RGBMask)
        cv2.imshow('dilated image', rgbDilated)

        return returnImg(rgbDilated)

          
    def findEdgesOfBridge(self):
        '''Finds the edges of the bridge so the robot can avoid driving off the 
        bridge.'''
        return
        
    def printBridge(self):
        print 'Bridges are pretty!'
        
    def do(self):
        return
        
if __name__ == '__main__':
    bridge()
