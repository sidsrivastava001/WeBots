# Put this at top of code to import HSUreturn(): from HSUfinal import *
# https://github.com/derekgeng15/ArduinoMaze/blob/master/BFS%20Navigation/FINALCOPY.py

import numpy as np
import cv2
import math
import time
np.set_printoptions(threshold=np.inf)

def HSUreturn(img):
	for i in range(1):
		time.sleep(0.5)
		frame = img
		ret = True
		if(ret == True):
		    framecopy = frame.copy()
		    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
		    blur = cv2.GaussianBlur(gray,(5,5),0)
		    ret,thresh = cv2.threshold(blur,40,255,cv2.THRESH_BINARY_INV)
		    threshcopy = thresh.copy()
		    #cv2.imshow("Yoh", threshcopy)
		    if(ret==False):
		        print("Error")
		    ret,contours=cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		    cv2.drawContours(framecopy,contours,-1, (255,0,255),5)
		    contourcopy = framecopy.copy()
		    if(len(contours) !=0):
		        areas = [cv2.contourArea(c) for c in contours]
		        max_index = np.argmax(areas)
		        cnt = contours[max_index]
		
		        rect = cv2.minAreaRect(cnt)
		        box= cv2.boxPoints(rect)
		        box = np.int0(box)
		        cv2.drawContours(framecopy,[box],0,(255,0,255),8)
		    
		        (x,y) = rect[0]
		        (w,h) = rect[1]
		        angle = rect[2]
		    
		        framew = frame.shape[0]
		        framel = frame.shape[1]
		    
		        diag = int(math.sqrt(((framew)**2) + ((framel)**2)))
		        if(h>w):
		            theta = cv2.getRotationMatrix2D((framel/2,framew/2),(angle),1)
		        
		        else:
		            theta = cv2.getRotationMatrix2D((framel/2,framew/2),(90+(angle)),1)
		    
		    
		        newframe = cv2.warpAffine(frame,theta,(diag,diag))
		        
		        newframecopy = newframe.copy()
		        gray = cv2.cvtColor(newframe,cv2.COLOR_BGR2GRAY)
		        blur = cv2.GaussianBlur(gray,(5,5),0)
		        ret,thresh = cv2.threshold(blur,40,255,cv2.THRESH_BINARY_INV)
		        if ret==False:
		            print("Error")
		            
		        ret,contours=cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		        
		        if(len(contours)>=2):
		        
		            areas = [cv2.contourArea(c) for c in contours]
		            
		                  
		            for i in range (1):
		                max_index = np.argmax(areas)
		                areas = np.delete(areas,max_index)
		            
		            max_index = np.argmax(areas)
		            cnt = contours[max_index]
		                
		            
		            x,y,w,h = cv2.boundingRect(cnt)
		            print("x",x)
		            print("y",y)
		            print("w",w)
		            print("h",h)
		            cv2.rectangle(newframecopy, (x,y),(x+w,y+h),(123,255,124),20)
		            
		            row_chop = int(h/3)
		            col_chop = int(w/3)
		            
		            mid_top = newframecopy[y:(y+row_chop),(x+col_chop): (x+2*(col_chop))]
		            mid_bottom = newframecopy[(y+2*row_chop):(y+3*row_chop),(x+col_chop):(x+2*(col_chop))]
		            top_left = newframecopy[y:(y+row_chop), x:(x+col_chop)]
		            top_right = newframecopy[y:(y+row_chop),(x+2*(col_chop)):(x+3*(col_chop))]
		            
		            #Right-Top
		            gray = cv2.cvtColor(top_right,cv2.COLOR_BGR2GRAY)
		            blur = cv2.GaussianBlur(gray,(5,5),0)
		            ret,thresh = cv2.threshold(blur,40,255,cv2.THRESH_BINARY_INV)
		            if ret==False:
		                print("Error")
		                
		            ret,contours_topright=cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		            cv2.drawContours(top_right,contours_topright,-1, (255,0,255),5)
		            
		            #Left-Top
		            gray = cv2.cvtColor(top_left,cv2.COLOR_BGR2GRAY)
		            blur = cv2.GaussianBlur(gray,(5,5),0)
		            ret,thresh = cv2.threshold(blur,40,255,cv2.THRESH_BINARY_INV)
		            if ret==False:
		                print("Error")
		                
		            ret,contours_topleft=cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		            cv2.drawContours(top_right,contours_topleft,-1, (255,0,255),5)
		            
		            #Mid-Top
		            gray = cv2.cvtColor(mid_top,cv2.COLOR_BGR2GRAY)
		            blur = cv2.GaussianBlur(gray,(5,5),0)
		            ret,thresh = cv2.threshold(blur,40,255,cv2.THRESH_BINARY_INV)
		            if ret==False:
		                print("Error")
		                
		            ret,contours_top=cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		            cv2.drawContours(mid_top,contours_top,-1, (255,0,255),5)
		            
		            #Mid-Bottom
		            gray = cv2.cvtColor(mid_bottom,cv2.COLOR_BGR2GRAY)
		            blur = cv2.GaussianBlur(gray,(5,5),0)
		            ret,thresh = cv2.threshold(blur,40,255,cv2.THRESH_BINARY_INV)
		            if ret==False:
		                print("Error")
		               
		            ret,contours_bottom=cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		            cv2.drawContours(mid_top,contours_bottom,-1, (255,0,255),5)
		            
		            if(len(contours_topright)>=1 and len(contours_topleft)>=1):
		            
			            i=0
			            
			            if(len(contours_top)>= 1):
			                i = i+1
			            if(len(contours_bottom)>=1):
			                i = i+1
			            
			                
			            if(i==0):
			                print("H")
			                return("H")
			            if(i==1):
			                print("U")
			                return("U")
			            if(i==2):
			                print("S")
			                return("S")
			                
			            print(i)
			            
			            print("row",row_chop)
			            print("col",col_chop)
			            
			            cv2.imshow("top",mid_top)
			            cv2.imshow("bottom",mid_bottom)
			            cv2.imshow("ajfdskl", newframecopy)
		            
		            else:
			            cv2.imshow("notfullimage", newframecopy)
		        else:
		            cv2.imshow("alsonocontours",newframecopy) 
		    else:
		        cv2.imshow("nocontours",framecopy)
		else:
			pass
			#print("Yeet")
	time.sleep(5)
	cv2.destroyAllWindows()
	vd.release()
