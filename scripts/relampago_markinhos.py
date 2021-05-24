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
from estacao import Estacao

from prints import encerrar_missao

class RelampagoMarkinhos:

    def __init__(self, objetivo, creeper, conceitoC = False):
        rospy.init_node("projeto")

        self.objetivo = objetivo
        self.conceitoC = conceitoC
        self.creeper = creeper
        self.dic = {}


        self.functions = RosFunctions(objetivo)
        self.garra = Garra()
        self.actions = RosActions(self.functions)
        self.estacao = Estacao(objetivo)

        self.FLAG = 'segue_pista'        
        self.creeper_atropelado = False
        self.momento_garra = 0

        self.dic['mobilenet'] = False

        self.iniciar_missao() 


    ##======================== GETTERS =========================##
    def get_dic(self):
        # Getter do dicionário de variáveis desta classe
        return self.dic

    def pegar_creeper(self,centro, maior_contorno_area, media): 
        print(media,maior_contorno_area)  
        dic_functions = self.functions.get_dic()
        if dic_functions['distancia_frontal'] < 0.21 and not self.creeper_atropelado:
            print('PARO')
            self.momento_garra = rospy.get_time()
            self.creeper_atropelado = True

        if maior_contorno_area > 700 and not self.creeper_atropelado:
            if len(centro) != 0 and len(media) != 0:
                if centro[0] -15 < media[0] < centro[0] + 15:
                    print('oe3')
                    self.actions.set_velocidade(0.1)
                else: 
                    print('oe4')
                    delta_x = centro[0] - media[0]
                    max_delta = 150
                    w = (delta_x/max_delta)*0.15
                    self.actions.set_velocidade(0.1,w)
                    
        if self.creeper_atropelado:
            self.actions.set_velocidade()
            self.garra.capturar_objeto(self.momento_garra)

    def cacador_creeper(self):
        img = self.functions.get_camera_bgr()
        if img is not None:
           centro, maior_contorno_area, media = self.creeper.identifica_creepers(self.functions)
           if maior_contorno_area > 700 or self.FLAG == 'pegando_creeper':
               self.FLAG = 'pegando_creeper'
               self.pegar_creeper(centro, maior_contorno_area, media)
           
           if self.FLAG != 'pegando_creeper':
               print('oe2')
               self.actions.segue_pista()

    def encontrar_estacao(self):
        img = self.functions.get_camera_bgr()
        self.dic['mobilenet'] = True
        self.actions.segue_pista()
        self.estacao.estacao_objetivo(img)

    def missao_conceito_c(self):        
        # self.actions.segue_pista()  
        # self.cacador_creeper() 
        self.encontrar_estacao()      

        
    def iniciar_missao(self):
        try: 
            while not rospy.is_shutdown():
                if self.conceitoC:
                    self.missao_conceito_c()
                else:
                    print('Eu sou a velocidade...')

            rospy.sleep(0.01)

        except rospy.ROSInterruptException:
            encerrar_missao()