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
        RGBMask = cv2.inRange(im, lower_redColor, upper_redColor)
        mask_inv = cv2.bitwise_not(RGBMask)
        res = cv2.bitwise_and(im,im,mask = RGBMask)
        res = cv2.bitwise_not(white_image, res, mask = mask_inv)
        
        imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(RGBMask,127,255,0)
        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

#####        for row in res:
#####            for collumn in res:
#####                pixel = res[row, collumn][0][0]
#####                if pixel.tolist() == [0,0,0]:
#####                    pixel = np.array([0,0,0])
######                    res[row,collumn] = [255,255,255]
        #cv2.imshow('rturened (res) image', res)
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
        print 'number of lines: ', len(lines[0])
        if len(lines)>0: #only print if lines are found so it doesn't error
            #print 'type: ', type(lines), 'Length: ', len(lines), 'lies are: \n', lines
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
        height = len(im[0])
        width = len(im[1])
        print width ,'x', height
        crop_img = im[int(height*1.3/3):height, 0:width]
        cv2.imshow('cropped image', crop_img)
        height = len(crop_img[0][0])
        width = len(crop_img[1])
        print width ,'x', height
#        crop_img = im[int(height*1.5/3):height, 0:width]
#        cv2.imshow('cropped image 2', crop_img)
        self.closeImages()
        
    def getLargestGap(self, im):
        np.ndarray.sum([3])
    
    def printBridge(self):
        print 'Bridges are pretty!'
    
    def do(self):
        b = bridge()
        image = cv2.imread('bridgePics/b0.jpg', cv2.IMREAD_COLOR)
        cv2.imshow('orig im: ', image)
        image = b.colorImgPreProcess(image)
        image = b.getContoursOfBridge(image)
        b.findEdgesOfBridge(image)
        b.cropBridge(image)
        b.getLargestGap(image)
        b.closeImages()
        
if __name__ == '__main__':
    b = bridge()
    b.do()

    
