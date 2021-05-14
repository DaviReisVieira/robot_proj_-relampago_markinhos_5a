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

from ros_functions import rosFunctions
from ros_actions import rosActions
from prints import encerrar_missao

class relampagoMarkinhos:

    def __init__(self, objetivo, creeper, conceitoC = False):
        self.objetivo = objetivo
        self.conceitoC = conceitoC
        self.creeper = creeper
        self.dic = {}

        rospy.init_node("projeto")

        self.functions = rosFunctions()
        self.actions = rosActions()


        self.ang_odom = 0
        self.x_odom = 0
        self.y_odom = 0
        self.camera_bgr = None
        self.centro_imagem = (320,240)
        self.velocidade = Twist()
        self.ang_amarelo = 0
        self.centro_x_amarelo = 320
        self.distancia = [100,100,100]
        self.ids = None
        self.FLAG = 'segue_linha'
        self.sinalizacao = 'nenhuma'
        self.posi_salva = (0,0)
        self.passou_bifurcacao = False
        # self.dic_ids = {}
        self.creeper_atropelado = False
        self.comecou_garra = False
        self.momento_garra = 0.0

        

        self.momento = 0.0

        self.iniciar_missao()

    def get_camera_bgr(self):
        return self.camera_bgr



    def set_velocidade(self, v_lin=0.0, v_ang=0.0):
        self.velocidade.linear.x = v_lin
        self.velocidade.angular.z = v_ang

    

    def seguir_linha(self):
        if self.centro_imagem[0] -10 < self.centro_x_amarelo < self.centro_imagem[0] + 10:
            self.set_velocidade(0.4)
            self.velocidade_saida.publish(self.velocidade)
            if -16 < self.ang_amarelo < 16:
                self.set_velocidade(0.6)
                self.velocidade_saida.publish(self.velocidade)
        else: 
            delta_x = self.centro_imagem[0] - self.centro_x_amarelo
            max_delta = 150
            w = (delta_x/max_delta)*0.20
            self.set_velocidade(0.2,w)
            self.velocidade_saida.publish(self.velocidade)

    def rotacionar(self,delta,v_ang=-1*(pi/5)):
        self.set_velocidade()
        self.velocidade_saida.publish(self.velocidade)
        now = rospy.get_time()
        while now - self.momento < delta and self.ang_amarelo is not None:
            #print('DELTA CALCULADO', now - momento)
            self.set_velocidade(0.0, v_ang)
            self.velocidade_saida.publish(self.velocidade)
            #print('rotacionando')
            now = rospy.get_time()
        else:
            if self.sinalizacao == 'retorna':
                self.FLAG = 'retorna_odom_bifurcacao'
            elif self.sinalizacao=='rotatoria':
                self.FLAG = 'retorna_odom_rotatoria'
            else:
                self.FLAG = 'segue_linha'
    

    def segue_pista(self):
        if self.sinalizacao == 'bifurcacao' and (self.distancia[0] <= 1.15 or self.distancia[1] <= 1.15 or self.distancia[2] <= 1.15):
            self.posi_salva = (self.x_odom, self.y_odom)
            self.momento = rospy.get_time()
            #print('momento', momento)
            self.rotacionar(1)
            self.passou_bifurcacao = True
            self.sinalizacao == 'nenhuma'
            print('bifurcacao:',self.posi_salva)
        elif self.sinalizacao == 'retorna' and (self.distancia[0] <= 0.7 or self.distancia[1] <= 0.7 or self.distancia[2] <= 0.7):
            self.momento = rospy.get_time()
            self.rotacionar(5)
            self.sinalizacao == 'nenhuma'
        elif self.sinalizacao == 'rotatoria' and (self.distancia[0] <= 0.7 or self.distancia[1] <= 0.7 or self.distancia[2] <= 0.7):
            self.posi_salva = (self.x_odom, self.y_odom)
            self.momento = rospy.get_time()
            self.rotacionar(2)
            self.sinalizacao == 'nenhuma'
            self.passou_bifurcacao = False
            print('rotatoria:',self.posi_salva)
        elif self.FLAG == 'retorna_odom_bifurcacao':
            if self.posi_salva[1] - 0.3 < self.y_odom < self.posi_salva[1] + 0.6:
                self.momento = rospy.get_time()
                self.rotacionar(2)
                self.FLAG = 'segue_linha'
            else:
                self.seguir_linha()
        elif self.FLAG == 'retorna_odom_rotatoria':
            now = rospy.get_time()
            if self.posi_salva[0] - 0.7 < self.x_odom < self.posi_salva[0] + 0.7 and self.posi_salva[1] - 0.3 < self.y_odom < self.posi_salva[1] + 0.3 and now-self.momento > 10:
                self.momento = rospy.get_time()
                self.rotacionar(2)
                self.FLAG = 'segue_linha'
            else:
                self.seguir_linha()
        elif self.FLAG == 'segue_linha':
            self.seguir_linha()
        #print(self.distancia)

    def missao_conceito_c(self):
        img = self.camera_bgr
        if img is not None:
            centro, maior_contorno_area, media = self.creeper.identifica_cor(self)
            #print(maior_contorno_area)
            if maior_contorno_area > 700 or self.FLAG == 'pegando_creeper':
                self.FLAG = 'pegando_creeper'
                self.pegar_creeper()
            
            if self.FLAG != 'pegando_creeper':
                self.segue_pista()
        # self.segue_pista()

    def usar_garra(self,momento):
        now = rospy.get_time()
        if not self.comecou_garra:
            self.set_velocidade()
            self.velocidade_saida.publish(self.velocidade)
            self.comecou_garra = True
        elif now - momento < 1.5:
            self.garra.publish(-1.0) ## Aberto
            self.ombro.publish(0.0) ## esta para frente
        elif 1.5 <= now - momento < 3:
            print('Fecha garra')
            self.garra.publish(0.0)  ## Fechado   
        elif 3 <= now - momento < 5:
            print('levanta ombro')
            self.ombro.publish(1.5)  ## Levanta          
            
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
                self.usar_garra(self.momento_garra)

            self.velocidade_saida.publish(self.velocidade)

        
    def iniciar_missao(self):
        # r = rospy.Rate(200)
        try: 
            while not rospy.is_shutdown():
                if self.conceitoC:
                    self.missao_conceito_c()
                else:
                    print('Eu sou a velocidade...')

            rospy.sleep(0.01)

        except rospy.ROSInterruptException:
            encerrar_missao()