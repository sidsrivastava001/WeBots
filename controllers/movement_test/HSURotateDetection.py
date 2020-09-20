#https://docs.opencv.org/3.4/d4/d73/tutorial_py_contours_begin.html
# Finidng largest countour StackOverFlow: https://stackoverflow.com/questions/44588279/find-and-draw-the-largest-contour-in-opencv-on-a-specific-color-python/44591580
# Thresholding - https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_thresholding/py_thresholding.html
# Adaptive Thresholding - https://docs.opencv.org/master/d7/d4d/tutorial_py_thresholding.html
# Image Operations Docs - https://docs.opencv.org/3.2.0/d0/d86/tutorial_py_image_arithmetics.html
# Array Operations DOcs - https://docs.opencv.org/2.4/modules/core/doc/operations_on_arrays.html#void%20split(const%20Mat&%20src,%20Mat*%20mvbegin)
# Find contour video: https://www.youtube.com/watch?v=_aTC-Rc4Io0
# Contour tutorial: http://opencvpython.blogspot.com/2012/06/hi-this-article-is-tutorial-which-try.html
# Getting most recent camera frame: https://stackoverflow.com/questions/41412057/get-most-recent-frame-from-webcam
# Contour Features: https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html
# Filling contours: https://answers.opencv.org/question/129207/contour-filling-with-drawcontours-fills-area-inside-contours/
# Contour Filtering: https://answers.opencv.org/question/65005/in-python-how-can-i-reduce-a-list-of-contours-to-those-of-a-specified-size/

import cv2
import numpy as np
import time

lower_red = np.array([0,0,100])
upper_red = np.array([255,255,255])

'''cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS,9)'''

def testThreshold(img):
    #convert from BGR to HSV color space
    imgCopy = img.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #apply threshold
    thresh = cv2.threshold(gray, 140, 255, cv2.THRESH_BINARY) [1]

    # draw all contours in green and accepted ones in red
    contours, h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.imwrite("unthresholded.png", gray) 
    #cv2.drawContours(gray, contours, -1, (0, 255, 0), 1)
    cv2.drawContours(gray, contours, -1, (150, 155, 155), 1)
    cv2.imwrite("thresholded.png", gray) 
        

def RotateImage(i,angle, scale, border_mode=cv2.BORDER_CONSTANT):
    """
    Return the rotated and scaled image. 
    By default the border_mode arg is BORDER_CONSTANT
    """
    (h, w) = i.shape[:2]
    print ("h:{0}  w:{1}".format(h,w))
    center = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D(center, angle, scale)
    return cv2.warpAffine(i, M, (w,h) ,flags=cv2.INTER_CUBIC, borderMode=border_mode )

def areaFilter(img, contours, threshold_area):
    for i in range(0,len(contours)):        
        area = cv2.contourArea(contours[i])
        #print("Area considered: " + str(area))
        if area < threshold_area:  
            cv2.drawContours(img, contours, i, (0,0,0), cv2.FILLED);
    return img

def areaFilterPrint(img, contours, threshold_area):
    for i in range(0,len(contours)):        
        area = cv2.contourArea(contours[i])
        print("Area considered smid: " + str(area))
        if area < threshold_area:  
            cv2.drawContours(img, contours, i, (0,0,0), cv2.FILLED);
    return img

def HSUDetect(img):
    #Getting images + applying thresholding
    black = np.zeros((240,320,3), dtype="uint8") #Completely Black Image, (width, height
    
    #To get rid of buffer
    '''
    for i in range(9):
        cap.grab()
    '''
    orig = img
    frame = cv2.cvtColor(orig, cv2.COLOR_BGR2GRAY)
    img = cv2.GaussianBlur(frame, (9, 9), 6)
    dst = cv2.adaptiveThreshold( img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 17, 2) #cv2.THRESH_BINARY, 15, -1) #cv2.THRESH_BINARY_INV, 17, 2) 
    #dum, dst = cv2.threshold(img,20,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
    
    (cont, h) = cv2.findContours(dst.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Contour Filter (by area and dimension filtering)
    threshold_area = 500     #threshold area
    aCount = 0
    hCount = 0
    for i in range(0,len(cont)):        
        area = cv2.contourArea(cont[i])
        x,y,width,height = cv2.boundingRect(cont[i])
        if area < threshold_area:  
            cv2.drawContours(dst, cont, i, (0,0,0), cv2.FILLED)
            #aCount+=1
        if height > 200:
            cv2.drawContours(dst, cont, i, (0,0,0), cv2.FILLED)
        if width > 200:
            cv2.drawContours(dst, cont, i, (0,0,0), cv2.FILLED)
        if height * width > 22500:
            cv2.drawContours(dst, cont, i, (0,0,0), cv2.FILLED)
    #print("Area Filtered: " + str(aCount))
    
    
    #Finding Contours:
    (contours, h) = cv2.findContours(dst.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.imwrite("parsedimg.png", dst)
    #print(len(contours))
    if len(contours) == 0:
        print("No Letters!")
        return None
    dstcopy = dst.copy()
    
    #Countour manipulation to get region of interest
    c = max(contours, key = cv2.contourArea) # largest countour
    contourcolor = (255,255,255) #B, G, R
    x,y,w,h = cv2.boundingRect(c)
    if x>5:
        x -= 5
    if y>5:
        y -= 5
    w += 10
    h += 10
    #print("x: {}, y: {}, w:{}, h:{}".format(x,y,w,h))
    
    
    #Drawing Countours or Green Contours:
    cv2.rectangle(orig,(x,y),(x+w,y+h),(0,255,0),2) # draw the book contour (in green)
    #cv2.rectangle(black, (x,y), (x+w, y+h), (255,255,0), 2)
    #cv2.drawContours(black, c, -1, contourcolor, -1)#, cv2.FILLED)
    #cv2.drawContours(orig, contours, -1, contourcolor, 3) #-1 is index of countour to draw. -1 means all. 3 - thickness of the lines
          
    # Image rotation
    
    sliced = dst[y:y+h, x:x+w]
    angle = cv2.minAreaRect(c)[-1]
    #(h, w) = sliced.shape[:2]
    #print ("h:{0}  w:{1}".format(h,w))
    center = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    dst2 = cv2.warpAffine(sliced, M, (w,h) ,flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_CONSTANT)
    (h2, w2) = dst2.shape
    print ("h:{0}  w:{1}".format(h2,w2))

    # Grabbing contour of rotated image (determine if needs 90 deg rotate)
    (contours, __) = cv2.findContours(dst2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    c2 = max(contours, key = cv2.contourArea) # largest countour
    x2,y2,w2,h2 = cv2.boundingRect(c2)
    '''x2 += 10
    y2 += 10
    w2 -= 10
    h2 -= 10'''
    #print("x2: {}, y2: {}, w2:{}, h2:{}".format(x2,y2,w2,h2))
    #cv2.rectangle(dst2,(x2,y2),(x2+w2,y2+h2),(255,255,255),2)
    #print(angle)
    
    # Rotates 90 if needed
    if (w2 >= h2):  #width must be smaller than height
        dst2 = RotateImage(dst2,90,1.0)  

   
    #Slicing Image
   # top = dst[y:y+int(0.33*h), x:x+w]
   # mid = dst[y+int(0.38*h):y+int(0.63*h), x:x+w]
   # smid = dst[y+int(0.45*h):y+int(0.55*h), x:x+w]
   # bot = dst[y+int(0.66*h):y+h, x:x+w]
    
    #dum, dst2 = cv2.threshold(dst2,20,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    
    #Filtering due to random tiny contours from rotating the image (area filter)
    
    (cont, h) = cv2.findContours(dst2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    threshold_area = 100     #threshold area  
    for i in range(0,len(cont)):        
        area = cv2.contourArea(cont[i])
        if area < threshold_area:  
            cv2.drawContours(dst2, cont, i, (0,0,0), cv2.FILLED);
    
    
    #dst = cv2.adaptiveThreshold( img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 17, 2) 
    
    # Gathering slices
    (roiy,roix) = dst2.shape
    # print ("y:{0} x:{1}".format(y,x))
    top = dst2[0:(roiy//3),0:roix]
    mid = dst2[int(roiy*0.37):int(roiy*0.63),0:roix]
    smid = dst2[8+int(roiy*0.40):int(roiy*0.55),0:roix]
    bot = dst2[2*roiy//3:roiy,0:roix]
    
    
    # Area filter on each of the slices
    (contop, h) = cv2.findContours(top.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    (conmid, h) = cv2.findContours(mid.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    (consmid, h) = cv2.findContours(smid.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #For S
    (conbot, h) = cv2.findContours(bot.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    areaFilter(top, contop, 20)
    areaFilter(mid, conmid, 20)
    areaFilter(smid, consmid, 20)
    areaFilter(bot, conbot, 20)

    #Calculating HSU
    (contop, h) = cv2.findContours(top.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    (conmid, h) = cv2.findContours(mid.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    (consmid, h) = cv2.findContours(smid.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #For S
    (conbot, h) = cv2.findContours(bot.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    '''
    widthmin = 20
    widthmax = 300
    heightmin = 20
    heightmax = 300
    print("Top: {}, Mid: {}, Bot: {}".format(len(contop), len(conmid), len(conbot)))
    for cnt in contop:
        rect = cv2.minAreaRect(cnt)       #I have used min Area rect for better result
        width = rect[1][0]
        height = rect[1][1]
        if (width>widthmax) and (height >heightmax) and (width <= widthMin) and (height < heightMin):
            del cnt
    cv2.drawContours(black, contop, 0, contourcolor, 3)
    ''' 
    
    # Determing HSU
    print("Top: {}, Mid: {}, SMid: {}, Bot: {}".format(len(contop), len(conmid), len(consmid), len(conbot)))
    if len(contop) == 2 and len(conmid) == 1 and len(conbot) == 2:
        print("H DETECTED")
    elif len(contop) == 1 and len(consmid) == 1 and len(conbot) == 1:
        print("S DETECTED")
    elif (len(contop) == 2 and len(conmid) == 2 and len(conbot) == 1) or (len(contop) == 1 and len(conmid) == 2 and len(conbot) == 2):
        print("U DETECTED")
        
    #Showing Image
   # cv2.imshow('Camera1', frame)
    cv2.imshow('dst', dst)
    cv2.imshow("RETR_EXTERNAL", orig)
   # cv2.imshow('black', black)
   # cv2.imshow('slice', sliced)
    cv2.imshow('dst2', dst2)  
    
    
    (smidh, smidw) = smid.shape
    cv2.imshow('top', top)
    cv2.imshow('mid', mid)
    if smidh!=0 and smidw!=0:
        cv2.imshow('smid', smid)
    cv2.imshow('bot', bot)
    
    # Break key
    # if cv2.waitKey(1) & 0xFF == ord('q'): #press q to exit
    #     break
    
    
# cap.release()
cv2.destroyAllWindows()


