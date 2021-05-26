from __future__ import division, print_function

# import the necessary packages
import numpy as np
import argparse
import cv2
import aux

import rospkg
import os

class Estacao:
    def __init__(self, objetivo):
        rospack = rospkg.RosPack()
        path = rospack.get_path('ros_projeto')
        scripts = os.path.join(path,  "scripts")

        proto = os.path.join(scripts,"MobileNetSSD_deploy.prototxt.txt")
        model = os.path.join(scripts, "MobileNetSSD_deploy.caffemodel")

        self.net = cv2.dnn.readNetFromCaffe(proto, model)
        self.CLASSES = aux.mobilenet_classes()
        self.confianca = 0.2
        self.COLORS = np.random.uniform(0, 255, size=(len(self.CLASSES), 3))

        self.objetivo = objetivo


    def estacao_objetivo(self, frame):
        image = frame.copy()
        result_frame, result_tuples = aux.detect(self.net, image, self.confianca, self.COLORS, self.CLASSES)
        
        
        
        
        cv2.imshow("Mobilenet", result_frame)