"""Demo for ppnc runtime on board"""
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

def parse_config(config_file):
    """parse config"""
    with open(config_file, "r") as f:
        config = json.load(f)
    return config

def preprocess(img_path, input_size=(416, 416)):
    """parse image

    Args:
        img_path (str): path
        input_size (tuple, optional): h x w. Defaults to (320, 320).
    """
    input = {}
    img = cv2.imread(img_path)
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

def draw_box(img_path, res, threshold=0.2):
    """draw box

    Args:
        img_path (str): image path
        res (numpy): output
        threshold (float, optional): . Defaults to 0.2.
    """
    img = cv2.imread(img_path)
    for i in res:
        label = i[0]
        score = i[1]
        if score < threshold:
            continue
        xmin, ymin, xmax, ymax = i[2:]
        cv2.rectangle(img, (int(xmin), int(ymin)), 
            (int(xmax), int(ymax)), (0, 0, 255), thickness=4)
    cv2.imwrite('{}_render.jpg'.format(img_path[:-4]), img)

if __name__ == "__main__":
    # Step 1: parse config file
    assert len(sys.argv) >= 3, "config and image file must be provided"
    config_file = sys.argv[1]
    # image_file = sys.argv[2]
    image_files = os.listdir(sys.argv[2])
    config = parse_config(config_file)
    # Step 2: initialize ppnc predictor
    predictor = PPNCPredictor(config)
    predictor.load()
    for image_file in image_files:
        image_file = os.path.join(sys.argv[2], image_file)
        # Step 3: prepare inputs
        feeds = preprocess(image_file, input_size=(416, 416))
        # Step 4: set inputs to ppnc predictor
        predictor.set_inputs(feeds)
        # Step 5: run ppnc predictor
        predictor.run()
        # Step 6: get infered result
        res = predictor.get_output(0)
        # Step 7: draw box
        draw_box(image_file, res, 0.5)
