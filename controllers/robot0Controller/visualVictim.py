import cv2
import pytesseract

class visual:
    def __init__(self):
        pass

    def getLetter(cam="C"):
        if(cam == "L"):
            img = cv2.imread('visionLeft.png')
        elif(cam == "R"):
            img = cv2.imread('visionRight.png')
        else:
            img = cv2.imread('visionCenter.png')
        custom_config = r'--oem 3 --psm 10'
        imgCopy = img.copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #apply threshold
        thresh = cv2.threshold(gray, 140, 255, cv2.THRESH_BINARY) [1]
        # draw all contours in green and accepted ones in red
        contours, h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.imwrite("unthresholded.png", gray)
        cv2.drawContours(imgCopy, contours, -1, (0, 255, 0), 1)
        cv2.drawContours(gray, contours, -1, (150, 155, 155),1)
        cv2.imwrite("thresholded.png", gray)
        for i, c in enumerate(contours):
            rect = cv2.boundingRect(c)
            x,y,w,h = rect
            box = cv2.rectangle(img, (x,y), (x+w,y+h), (0,0,255), 2)
            cropped = img[y: y+h, x: x+w]
            text = pytesseract.image_to_string(cropped, config=custom_config)
            if("S" in text or "s" in text):
                return "S"
            elif("H" in text):
                return "H"
            elif("U" in text):
                return "U"
        return "None"