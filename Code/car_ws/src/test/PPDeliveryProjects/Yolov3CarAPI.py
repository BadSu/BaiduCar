"""Demo for ppnc runtime on board"""
import time
import json
import sys
import numpy as np
import cv2
import os

try:
    from ppnc import PPNCPredictor
except:
    print("no ppnc runtime detected in system path")
    parent_path = os.path.abspath(os.path.join(__file__, *(['..'] * 2)))
    sys.path.insert(0, parent_path + '/python')
    from ppnc import PPNCPredictor

class Yolov3Car:
    def __init__(self,
                 config_file,
                 classes,
                 image=0,
                 img_path="",
                 video_path = "",
                 threshold = 0.3,
                 printf=False,
                 ):
        self.config_file = config_file
        self.classes = classes
        self.image = image
        self.img_path = img_path
        self.video_path = video_path
        self.threshold = threshold
        self.printf = printf
        # self.colors = [(255,255,0), (0,255,0), (0,255,255), (255,0,0)]


    def parse_config(self):
        """parse config"""
        with open(self.config_file, "r") as f:
            config = json.load(f)
            self.predictor = PPNCPredictor(config)
            self.predictor.load()


    def preprocess(self, input_size=(640, 480), image_file=""):
        """parse image

        Args:
            image_file (str): path
            input_size (tuple, optional): h x w. Defaults to (320, 320).
        """
        input = {}
        if image_file !="":
            img = cv2.imread(image_file)
        else:
            img = self.image
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        scale_factor = [input_size[0] / img.shape[0], input_size[1] / img.shape[1]]
        factor = np.array(scale_factor, dtype=np.float32)
        input['scale_factor'] = factor.reshape((1, 2))
        input["im_shape"] = np.array(input_size)[np.newaxis, :]
        img = cv2.resize(img, input_size, interpolation=2)

        mean = np.array([0.485, 0.456, 0.406])[np.newaxis, np.newaxis, :]
        std = np.array([0.229, 0.224, 0.225])[np.newaxis, np.newaxis, :]
        img = img / 255
        img -= mean
        img /= std
        img = img.astype(np.float32, copy=False)

        img = img.transpose((2, 0, 1))
        img = img[np.newaxis, :]
        input['image'] = img
        return input

    def draw_box(self, image_file, res, threshold=0.2):
        """draw box

        Args:
            image_file (str): image path
            res (numpy): output
            threshold (float, optional): . Defaults to 0.2.
        """
        img = cv2.imread(image_file)
        for i in res:
            label = int(i[0])
            score = i[1]
            if score < threshold:
                continue
            xmin, ymin, xmax, ymax = i[2:]
            cv2.rectangle(img, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 0, 255), 4)
        cv2.imwrite('{}_render.jpg'.format(image_file[:-4]), img)

    def draw_box1(self, img, res):
        ress = []
        for i in res:
            cv2.rectangle(img, (i[1], i[2]), (i[3], i[4]), (0, 0, 255), 4)
            cv2.putText(img,i[0],(i[1],i[2]-10),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,0))
            ress.append(i)
        return img,ress

    def draw_box2(self, img, res):
        true_box = [img.shape[1]//4,img.shape[0]//4,img.shape[1]//4*3,img.shape[0]]
        cv2.rectangle(img, (true_box[0], true_box[1]), (true_box[2], true_box[3]), (0, 0, 255), 5)
        ress = []
        for i in res:

            if i[1] > true_box[0] and i[2] > true_box[1] and i[3] < true_box[2] and i[4] < true_box[3]:
                cv2.rectangle(img, (i[1], i[2]), (i[3], i[4]), (0, 0, 255), 4)
                cv2.putText(img,i[0],(i[1],i[2]-10),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,0))
                ress.append(i)
            continue
        return img,ress

    def run(self, image1=0, draw=False):
        if not isinstance(image1,int):
                self.image = image1
                t0 = time.perf_counter()
                feeds = self.preprocess(input_size=(416, 416))
                # Step 4: set inputs to ppnc predictor
                self.predictor.set_inputs(feeds)
                t1 = time.perf_counter()
                # Step 5: run ppnc predictor
                self.predictor.run()
                t2 = time.perf_counter()
                # Step 6: get infered result
                res = self.predictor.get_output(0)
                
                tmp = []
                #print(len(res))
                for i in res:
                    label = int(i[0])
                    score = i[1]
                    if score < self.threshold:
                        continue
                    cn = self.classes[label]
                    xmin, ymin, xmax, ymax = i[2:]
                    tmp.append([cn,int(xmin),int(ymin),int(xmax),int(ymax)])
                    #break
                res = tmp
                t3 = time.perf_counter()
                if draw == False:
                    frame,ress = self.draw_box1(image1,res)
                else:
                    frame,ress = self.draw_box2(image1,res)
                if self.printf == True:
                    print(f'TimeConsuming:\n'
                          f'Preprocess: {(t1 - t0) * 1000} ms\n'
                          f'Inference: {(t2 - t1) * 1000} ms\n'
                          f'Postprocess: {(t3 - t2) * 1000} ms\n'
                          f'End2END: {(t3 - t0) * 1000} ms')
                return image1, ress



        if self.img_path != "" and self.video_path == "":
            image_files = os.listdir(self.img_path)
            for image_file in image_files:
                image_file = os.path.join(self.img_path, image_file)
                # Step 3: prepare inputs
                t0 = time.perf_counter()
                feeds = self.preprocess(input_size=(640, 640), image_file=image_file)
                # Step 4: set inputs to ppnc predictor
                self.predictor.set_inputs(feeds)
                t1 = time.perf_counter()
                # Step 5: run ppnc predictor
                self.predictor.run()
                t2 = time.perf_counter()
                # Step 6: get infered result
                res = self.predictor.get_output(0)
                t3 = time.perf_counter()
                # Step 7: draw box
                self.draw_box(image_file, res, 0.5)
                t4 = time.perf_counter()
                print(f'TimeConsuming:\n'
                      f'Preprocess: {(t1 - t0) * 1000} ms\n'
                      f'Inference: {(t2 - t1) * 1000} ms\n'
                      f'Postprocess: {(t3 - t2) * 1000} ms\n'
                      f'Drawing: {(t4 - t3) * 1000} ms\n'
                      f'End2END: {(t4 - t0) * 1000} ms')

        if self.video_path != "" and self.img_path == "":
            cap = cv2.VideoCapture(self.video_path)
            while cap.isOpened():
                retval, frame = cap.read()
                t0 = time.perf_counter()
                self.image = frame
                feeds = self.preprocess(input_size=(416, 416))
                # Step 4: set inputs to ppnc predictor
                self.predictor.set_inputs(feeds)
                t1 = time.perf_counter()
                # Step 5: run ppnc predictor
                self.predictor.run()
                t2 = time.perf_counter()
                # Step 6: get infered result
                res = self.predictor.get_output(0)
                tmp = []
                for i in res:
                    label = int(i[0])
                    score = i[1]
                    if score < self.threshold:
                        continue
                    cn = self.classes[label]
                    xmin, ymin, xmax, ymax = i[2:]
                    tmp.append([cn,int(xmin),int(ymin),int(xmax),int(ymax)])
                    break
                res = tmp
                t3 = time.perf_counter()
                if self.draw == True:
                    frame = self.draw_box1(frame,res)
                print(f'TimeConsuming:\n'
                      f'Preprocess: {(t1 - t0) * 1000} ms\n'
                      f'Inference: {(t2 - t1) * 1000} ms\n'
                      f'Postprocess: {(t3 - t2) * 1000} ms\n'
                      f'End2END: {(t3 - t0) * 1000} ms')
                print("res_list:", res)
                cv2.imshow('Live', frame)
                if cv2.waitKey(5) >= 0:
                   break



