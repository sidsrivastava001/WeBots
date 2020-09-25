import cv2
import numpy as np

img = cv2.imread("0 left.png")

cv2.imshow('Original', img)
print(img.shape)
(height, width, depth) = img.shape
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imshow('Grayed', gray)
rMask = np.zeros((height, width, 1), np.uint8) # rMask IS FOR LEFT CAMERA IMAGE
rMask[0:width, 0:(int(height * (5/6)))] = 255
maskedImg = cv2.bitwise_and(gray, gray, mask=rMask)
cv2.imshow("TEST MASK", maskedImg)


cv2.waitKey(0)
cv2.destroyAllWindows()