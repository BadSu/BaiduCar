import Yolov3CarAPI
import time
import cv2
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError


class Yolo:
    def __init__(self):
        rospy.init_node('Yolo', anonymous=False)
        rospy.Subscriber("/usb_cam/image_raw", Image, self.Image_callback)
        #rospy.Subscriber('/control_type', Int32, self.control_type_callback)
        self.type_pub = rospy.Publisher("/control_type",Int32,queue_size = 1000)
        classes = ["fire", "fire_s", "limit_20", "limit_20_s", "limit_40", "oldman", "oldman_s", "p", "p_s", "student", "apple", "watermelon", "green_bucket", "red_bucket", "classify_s"]
        self.yolov3api = Yolov3CarAPI.Yolov3Car(config_file="./baidu_car0311/config.json", 
                                        classes=classes,
                                        draw=False)
    def Image_callback(self, image):

        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = image.header
        self.boundingBoxes.image_header = image.header
        self.getImageStatus = True
        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)
        self.Frame_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)


    def main(self): 
        self.yolov3api.parse_config()
        num = 0
        while not rospy.is_shutdown():
            Frame = cv2.resize(self.Frame_image, (480, 360))
            frame, res = yolov3api.run(Frame)
            if res!= []:
                str=','.join('%s'%id for id in(res[0])) 
                res=str.split(',')
                print(res[0])
                num+=1
                if num >10:
                   self.type_pub.publish(0)

                if cv2.waitKey(5) >= 0:
                    break


if __name__=="__main__":
    yolo = Yolo()
    yolo.main()
    print("done")


