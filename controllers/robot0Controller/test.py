import cv2

img = cv2.imread("vision.png")
cv2.imshow('Original', img)

cv2.waitKey(0)
cv2.destroyAllWindows()