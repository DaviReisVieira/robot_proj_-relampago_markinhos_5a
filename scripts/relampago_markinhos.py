#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import tf
import aux
import math
import cv2
import cv2.aruco as aruco
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped

from nav_msgs.msg import Odometry
from std_msgs.msg import Header

class relampagoMarkinhos:

    def __init__(self,missao,conceitoC = False):
        self.conceitoC = conceitoC
        self.bridge = CvBridge()
        rospy.init_node("projeto")
        
        self.topico_imagem = "/camera/image/compressed"
        self.recebedor = rospy.Subscriber(self.topico_imagem, CompressedImage, self.roda_todo_frame, queue_size=4, buff_size=2**24)
        self.velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        #self.recebe_scan = rospy.Subscriber("/scan", LaserScan, self.scaneou)
        #self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

        self.camera_bgr = None
        self.centro_imagem = (320,240)
        self.velocidade = Twist()
        self.ang_amarelo = 0
        self.centro_x_amarelo = 320
        self.distancia = 100
        self.frame = 0
        self.skip = 0

        self.iniciar_missao()

    def scaneou(self,dado):
        ranges = np.array(dado.ranges).round(decimals=2)
        self.distancia = ranges[0]

    def set_velocidade(self, v_lin=0.0, v_ang=0.0):
        self.velocidade.linear.x = v_lin
        self.velocidade.angular.z = v_ang

    def roda_todo_frame(self,imagem):
        try:
            imagem_original = self.bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
            self.camera_bgr = imagem_original

            self.regressao_linha()

            cv2.waitKey(1)
        except CvBridgeError as e:
            print('ex', e)

    def regressao_linha(self):
        # mask = aux.makeMask(self.camera_bgr,(30, 55, 42), (32, 255, 255))
        mask = aux.filter_color(self.camera_bgr,np.array([22, 50, 50],dtype=np.uint8), np.array([36, 255, 255],dtype=np.uint8))
        img, centro_yellow  =  aux.center_of_mass_region(mask, 0, 300, mask.shape[1], mask.shape[0])  
        saida_bgr, m, h = aux.ajuste_linear_grafico_x_fy(mask)
        # contornos = aux.encontrar_contornos(mascara_amarelo)
        # centro_contornos, xList, yList = aux.find_center(mascara_amarelo, contornos)
        # img_regressao, m, h = aux.regressao_por_centro(mascara_amarelo, xList, yList)
        
        ang = math.atan(m)
        ang_deg = math.degrees(ang)

        self.ang_amarelo = ang_deg
        self.centro_x_amarelo = centro_yellow[0]

        cv2.imshow("Original", img)
        cv2.imshow("Filtro", saida_bgr)

    def aruco_ids(self):
        img = self.camera_bgr
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict)
        aruco.drawDetectedMarkers(img, corners, ids)
        
        ids = np.array(ids).flatten()
        cv2.imshow("Filtro", img)

        return ids, img

    def seguir_linha(self):
        if self.centro_imagem[0] -10 < self.centro_x_amarelo < self.centro_imagem[0] + 10:
            self.set_velocidade(0.2,0.0)
            self.velocidade_saida.publish(self.velocidade)
            print('dentro')
            if -15 < self.ang_amarelo < 15:  # para angulos centrados na vertical, regressao de x = f(y) como está feito
                self.set_velocidade(0.4,0.0)
                self.velocidade_saida.publish(self.velocidade)
            print('dentro 1')
        else: 
            print('dentro 2')
            delta_x = self.centro_imagem[0] - self.centro_x_amarelo
            max_delta = 150.0
            w = (delta_x/max_delta)*0.20
            self.set_velocidade(0.2,w)
            self.velocidade_saida.publish(self.velocidade)
        
        #self.velocidade_saida.publish(self.velocidade)

    def missao_conceito_c(self):
        self.seguir_linha()
        #ids,img=self.aruco_ids()
        print("Leituras Ditancia:",self.distancia)
        print("Leituras Centro Amarelo:",self.centro_x_amarelo)
        print("Leituras Centro Amarelo:",self.ang_amarelo)

    def iniciar_missao(self):
        r = rospy.Rate(200)
        try: 
            while not rospy.is_shutdown():
                if self.conceitoC:
                    self.missao_conceito_c()
            r.sleep()

        except rospy.ROSInterruptException:
            print("Oh Deus quantos CTRL+C")