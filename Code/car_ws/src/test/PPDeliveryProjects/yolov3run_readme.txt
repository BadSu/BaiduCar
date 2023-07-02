import Yolov3CarAPI
import time
import cv2


classes = ["fire", "fire_s", "limit_20", "limit_20_s", "limit_40", "oldman", "oldman_s", "p", "p_s", "student"]



################################### image

#cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture("fire.mp4")
yolov3api = Yolov3CarAPI.Yolov3Car(config_file="./baidu_car/config.json", 
                                   classes=classes,
                                   draw=True)
yolov3api.parse_config()
while cap.isOpened():
    retval, frame = cap.read()
    frame, res = yolov3api.run(frame)
    print("res_list:", res)
    cv2.imshow('Live', frame)
    if cv2.waitKey(5) >= 0:
        break

################################# img_path (str)

yolov3api = Yolov3CarAPI.Yolov3Car(config_file="./baidu_car/config.json", 
                                   classes=classes,
                                   img_path="car_images/")
yolov3api.parse_config()
yolov3api.run()


################################## video_path (str)

yolov3api = Yolov3CarAPI.Yolov3Car(config_file="./baidu_car/config.json", 
                                   classes=classes,
                                   video_path="./baidu_car_images/fire.mp4",
                                   draw=True)
yolov3api.parse_config()
yolov3api.run()

