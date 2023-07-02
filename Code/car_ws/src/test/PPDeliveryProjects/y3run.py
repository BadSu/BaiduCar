import Yolov3CarAPI
import time
import cv2



classes=["tower","hhl","yyl","twg","barge",
         "trade","konjac","citrus","swordfish","tornado",
         "spray","dam","vortex"]
yolov3api = Yolov3CarAPI.Yolov3Car(config_file="./inference0420/config.json", 
                                classes=classes)
yolov3api.parse_config()

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
cap.set(3, 640) # width
cap.set(4, 480) # height
yolov3api.parse_config()
while cap.isOpened():
    retval, frame = cap.read()
    frame, res = yolov3api.run(frame)
    print("res_list:", res)
    cv2.imshow('Live', frame)
    if cv2.waitKey(5) >= 0:
        break
