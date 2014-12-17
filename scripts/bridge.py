#may want to use a SIFT descriptor

import numpy as np
import cv2


class bridge():
    '''Behaviors to let the robot traverse the bridge and execute the behavior.'''
    filepath = 'bridgePics'
    
    
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
    
    
    def __init__(self): #below closeImages so I can use close images in it!
        image = cv2.imread('gimpBridge.jpg', cv2.IMREAD_COLOR)
        image = self.colorImgPreProcess(image)
        #cv2.imshow('gimp bridge', image)
        #self.closeImages()
        return
            
            
    def colorImgPreProcess(self, image):
        """
        Prepare images to be analyzed in binary form by appling generic filtering.
        This makes them easier to work with and the resulting image less noisy.
        INPUT: image for pre-processing. Should be in color, though b&w ahould work.
        OUTPUT: returns a RGB image which has been filtered and looks nicer.
        """
        #do processing on the image while it's still in color
        image = cv2.medianBlur(image, 7)  #kernal size must be odd
        image = cv2.bilateralFilter(image, 9, 75, 75)
        #self.closeImages()
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
        #cv2.imshow("color mask", RGBMask)
        #cv2.imshow('dilated image', rgbDilated)

        return returnImg(rgbDilated)

    
    def getContoursOfBridge(self, im):
        white_image = np.zeros((len(im), len(im[0]), 3), np.uint8)
        white_image[:] = (0,0,0)
        

        cv2.imshow('image', im)
        im = b.colorImgPreProcess(im)
        
        lower_redColor = np.array([220, 155, 50])
        upper_redColor = np.array([255, 255, 255])
        # Threshold the RGB image to get only red colors
        RGBMask = cv2.inRange(image, lower_redColor, upper_redColor)
        mask_inv = cv2.bitwise_not(RGBMask)
        res = cv2.bitwise_and(im,im,mask = RGBMask)
        res = cv2.bitwise_not(white_image, res, mask = mask_inv)
        cv2.imshow('res before blizzard', res)
        #res = cv2.bitwise_and(im, im, mask = mask_inv)
        cv2.imshow('red mask', RGBMask)
        cv2.imshow('masked image', res)
        cv2.add(white_image, res, mask = mask_inv)
        
        imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(RGBMask,127,255,0)
        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

#####        for row in res:
#####            for collumn in res:
#####                pixel = res[row, collumn][0][0]
#####                if pixel.tolist() == [0,0,0]:
#####                    pixel = np.array([0,0,0])
######                    res[row,collumn] = [255,255,255]
        cv2.imshow('res after switching to white', res)

        #lines = cv2.HoughLines(edges,1,np.pi/180,200)

        #cv2.imwrite('houghlines3.jpg',im)
######        self.closeImages()

##        edges = cv2.Canny(gray,50,150,apertureSize = 3)

##        lines = cv2.HoughLines(edges,1,np.pi/180,200)
##        for rho,theta in lines[0]:
##            a = np.cos(theta)
##            b = np.sin(theta)
##            x0 = a*rho
##            y0 = b*rho
##            x1 = int(x0 + 1000*(-b))
##            y1 = int(y0 + 1000*(a))
##            x2 = int(x0 - 1000*(-b))
##            y2 = int(y0 - 1000*(a))

##            cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)

        #cv2.imshow('houghlines3',im)
        self.closeImages()
##        print "done with contoring things."
          
    def findEdgesOfBridge(self, im):
        '''Finds the edges of the bridge so the robot can avoid driving off the 
        bridge.'''
        ## Im should be ready to have edge detection run on it. 
        edges = cv2.Canny(im,0,255,apertureSize = 5)
        cv2.imshow("edges detected on input image", edges)
        
        #cv2.HoughLinesP(image, rho, theta, threshold[, lines[, minLineLength[, maxLineGap]]]) -->lines
        cv2.HoughLinesP(edges, 1, 1)
        return
        
    def printBridge(self):
        print 'Bridges are pretty!'
        
    def do(self):
        return
        
if __name__ == '__main__':
    b = bridge()
    image = cv2.imread('bridgePics/b0.jpg', cv2.IMREAD_COLOR)
    image = b.colorImgPreProcess(image)
    b.getContoursOfBridge(image)
