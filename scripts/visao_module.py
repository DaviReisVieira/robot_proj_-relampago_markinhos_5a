#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import division, print_function


import rospy
import numpy as np
import aux
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
# import mobilenet_simples as mnet



# def processa(frame):
#     '''Use esta funcao para basear o processamento do seu robo'''

#     result_frame, result_tuples = mnet.detect(frame)

#     centro = (frame.shape[1]//2, frame.shape[0]//2)


#     def cross(img_rgb, point, color, width,length):
#         cv2.line(img_rgb, (point[0] - int(length/2), point[1]),  (point[0] + int(length/2), point[1]), color ,width, length)
#         cv2.line(img_rgb, (point[0], point[1] - int(length/2)), (point[0], point[1] + int(length/2)),color ,width, length)

#     cross(result_frame, centro, [255,0,0], 1, 17)


#     return centro, result_frame, result_tuples



def identifica_cor(frame, cor):

    if cor == "blue":
        cor_menor = np.array([75, 50, 50])
        cor_maior = np.array([95, 255, 255])
    elif cor == "green":
        cor_menor = np.array([45, 100, 100])
        cor_maior = np.array([75, 255, 255])
    elif cor == "orange":
        cor_menor = np.array([0, 200, 200])
        cor_maior = np.array([8, 255, 255])

    bgr = frame.copy()
    segmentado_cor = aux.make_mask(bgr, cor_menor, cor_maior, kernel=True)
    centro = (frame.shape[1]//2, frame.shape[0]//2)

    maior_contorno, maior_contorno_area = aux.encontrar_maior_contorno(segmentado_cor.copy())
    media = aux.find_center(frame, maior_contorno, centro)

    s1 = "{:d} {:d}".format(*media)
    s2 = "{:0.1f}".format(maior_contorno_area)
    aux.text(frame, s1, (20, 100))
    aux.text(frame, s2, (20, 50))

    return centro, maior_contorno_area, media
