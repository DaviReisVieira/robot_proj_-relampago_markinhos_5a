#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import tf
import aux
import math
from math import pi
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
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from ros_functions import RosFunctions
from ros_actions import RosActions
from garra import Garra

from prints import encerrar_missao

class RelampagoMarkinhos:

    def __init__(self, objetivo, creeper, conceitoC = False):
        rospy.init_node("projeto")

        self.objetivo = objetivo
        self.conceitoC = conceitoC
        self.creeper = creeper
        self.dic = {}


        self.functions = RosFunctions()
        self.actions = RosActions(self.functions)
        self.garra = Garra()

        self.iniciar_missao()
    
    

    def missao_conceito_c(self):
        #img = self.camera_bgr
        #if img is not None:
        #    centro, maior_contorno_area, media = self.creeper.identifica_cor(self)
        #    #print(maior_contorno_area)
        #    if maior_contorno_area > 700 or self.FLAG == 'pegando_creeper':
        #        self.FLAG = 'pegando_creeper'
        #        self.pegar_creeper()
        #    
        #    if self.FLAG != 'pegando_creeper':
        #        self.segue_pista()
        self.actions.segue_pista()         
            
    def pegar_creeper(self):
        img = self.camera_bgr
        if img is not None:
            
            centro, maior_contorno_area, media = self.creeper.identifica_cor(self)
            #print(centro,maior_contorno_area,media)
            if self.distancia[1] < 0.21 and not self.creeper_atropelado:
                print('PARO')
                self.momento_garra = rospy.get_time()
                self.creeper_atropelado = True

            if maior_contorno_area > 1200 and not self.creeper_atropelado:

                if len(centro) != 0 and len(media) != 0:
                    if centro[0] -15 < media[0] < centro[0] + 15:
                        self.set_velocidade(0.1)
                    else: 
                        delta_x = centro[0] - media[0]
                        max_delta = 150
                        w = (delta_x/max_delta)*0.15
                        self.set_velocidade(0.1,w)
                        
            if self.creeper_atropelado:
                self.set_velocidade()
                self.garra.capturar_objeto(self.momento_garra)

            self.velocidade_saida.publish(self.velocidade)

        
    def iniciar_missao(self):
        # r = rospy.Rate(200)
        try: 
            while not rospy.is_shutdown():
                if self.conceitoC:
                    self.missao_conceito_c()
                else:
                    print('Eu sou a velocidade...')

            rospy.sleep(0.1)

        except rospy.ROSInterruptException:
            encerrar_missao()