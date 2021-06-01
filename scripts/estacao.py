from __future__ import division, print_function

# import the necessary packages
import numpy as np
import argparse
import cv2
import auxiliar as aux

import rospkg
import os

class Estacao:
    def __init__(self, objetivo):
        rospack = rospkg.RosPack()
        path = rospack.get_path('ros_projeto')
        scripts = os.path.join(path,  "scripts")

        proto = os.path.join(scripts,"MobileNetSSD_deploy.prototxt.txt")
        model = os.path.join(scripts, "MobileNetSSD_deploy.caffemodel")

        self.NET = cv2.dnn.readNetFromCaffe(proto, model)
        self.CLASSES = aux.mobilenet_classes()
        self.CONFIDENCE = 0.5
        self.COLORS = np.random.uniform(0, 255, size=(len(self.CLASSES), 3))

        self.objetivo = objetivo

        self.encontrou_estacao = False
        self.centro_estacao = (10000,10000)


    def estacao_objetivo(self, frame):
        '''
        Funcao que trabalha com a mobilenet
        Retorna se encontrou a estacao e seu centro
        '''
        if frame is not None:
            image = frame.copy()
            result_frame, results = aux.detect(self.NET, image, self.CONFIDENCE, self.COLORS, self.CLASSES)
            for result in results:
                if result[0] == self.objetivo:
                    self.encontrou_estacao = True
                    self.centro_estacao = (result[2][0] + result[3][0])/2

            cv2.imshow("Mobilenet", result_frame)
            return self.encontrou_estacao, self.centro_estacao