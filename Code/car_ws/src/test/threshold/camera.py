import cv2

cap = cv2.VideoCapture(0)

while(cap.isOpened()):
    retval, frame = cap.read()
    cv2.imshow('Live', frame)
    if cv2.waitKey(5) >= 0:
        break
