#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import tf
import auxiliar as aux
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
from estacao import Estacao

from prints import encerrar_missao
from termcolor import colored

class RelampagoMarkinhos:

    def __init__(self, objetivo, creeper, estacao, conceitoC = False, somente_pista = False):
        '''
        dasdfasd
        '''
        rospy.init_node("projeto")

        self.objetivo = objetivo
        self.conceitoC = conceitoC
        self.somente_pista = somente_pista
        self.creeper = creeper
        self.estacao = estacao
        self.dict = {}


        self.functions = RosFunctions(objetivo)
        self.actions = RosActions(self.functions, self.creeper, self.estacao)

        self.missao = 'creeper'
        self.FLAG = 'procurando_creeper'

        self.iniciar_missao() 
    

    def controle(self):
        '''
        adjcnkajbd
        '''
        self.Resultado = self.actions.get_resultado()

        # Enquanto nao encontra o creeper -> procurando
        if self.FLAG == 'procurando_creeper':
            self.actions.procurando_creeper()
            if self.Resultado == 'encontrou_creeper':
                self.FLAG = 'pegar_creeper'

        # Encontrou o creeper -> sair para buscar
        elif self.FLAG == 'pegar_creeper':
            self.actions.pegar_creeper()
            if self.Resultado == 'pegou_creeper':
                self.FLAG = 'retorna_pista'

        # Encontrar estacao -> procurando
        elif self.FLAG == 'procurando_estacao':
            self.actions.procurando_estacao()
            if self.Resultado == 'encontrou_estacao':
                self.FLAG = 'deixar_creeper'

        # Encontrou estacao -> sair para delivery
        elif self.FLAG == 'deixar_creeper':
            self.actions.deixar_creeper()
            if self.Resultado == 'largou_creeper':
                self.FLAG = 'retorna_pista'


        # Retorna pista
        elif self.FLAG == 'retorna_pista':
            self.actions.retorna_pista()
            if self.Resultado == 'retornou':
                if self.missao == 'creeper':
                    if self.conceitoC:
                        self.FLAG = 'seguir_pista'
                    else:
                        self.FLAG = 'procurando_estacao'
                        self.missao = 'estacao'
                elif self.missao == 'estacao':
                    self.FLAG = 'procurando_creeper'
                    self.missao = 'creeper'

        elif self.FLAG == 'seguir_pista':
            self.actions.segue_pista()

        ####### Repete ########

        
    def iniciar_missao(self):
        '''
        sadadsf
        '''
        try: 
            while not rospy.is_shutdown():
                if self.somente_pista:
                    self.actions.segue_pista()
                else:
                    self.controle()
                # self.actions.encontrar_estacao()
            rospy.sleep(0.01)

        except rospy.ROSInterruptException:
            encerrar_missao()