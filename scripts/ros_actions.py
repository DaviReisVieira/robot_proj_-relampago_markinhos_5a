from __future__ import print_function, division
import rospy
import numpy as np
import tf
import aux
import cv2
from math import atan2
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped, Point
from std_msgs.msg import Float64


class RosActions:

    ##========================== INIT ==========================##
    def __init__(self, RosFunctions):
        self.RosFunctions = RosFunctions
        self.dic = {}

        self.velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        #Start parado:
        self.velocidade = Twist()
        self.set_velocidade()

        self.FLAG = 'segue_linha'
        self.dic['momento'] = 0
        self.dic['posicao_bifurcacao'] = [0,0]
        self.dic['posicao_rotatoria'] = [0,0]
        self.dic['angulo_salvo'] = 0

    ##======================== GETTERS =========================##
    def get_dic(self):
        return self.dic_funcions

    ##======================== SETTERS =========================##
    def set_velocidade(self, v_lin=0.0, v_ang=0.0):
        self.velocidade.linear.x = v_lin
        self.velocidade.angular.z = v_ang
        self.velocidade_saida.publish(self.velocidade)

    
    ##======================= FUNCTIONS ========================##
    def segue_pista(self):
        dic_functions = self.RosFunctions.get_dic()
        self.controle_sinalizacao(dic_functions)
        self.organiza_movimentacao(dic_functions)
        self.retorna_odom_sinalizacao(dic_functions)
        if self.FLAG == 'segue_linha':
            self.seguir_linha()

            

    #-------------------------- Linha --------------------------
    '''
    Já possui controle proporcional
    '''
    # v = theta/k, w = theta*k, k eh natural positivo

    def seguir_linha(self):
        dic_functions = self.RosFunctions.get_dic()
        if dic_functions['centro_imagem'][0]-10 < dic_functions['centro_x_amarelo'] < dic_functions['centro_imagem'][0] + 10:
            self.set_velocidade(0.4)
            if -16 < dic_functions['ang_amarelo'] < 16:
                self.set_velocidade(0.6)
        else: 
            delta_x = dic_functions['centro_imagem'][0] - dic_functions['centro_x_amarelo'] 
            max_delta = 150
            w = (delta_x/max_delta)*0.20
            self.set_velocidade(0.2,w)

    #------------------------- Rotação -------------------------
    def rotacao_odom(self, dic_functions, angulo): 
        soma = self.dic['angulo_salvo'] + angulo
        if soma < 0:
            soma += 360
        elif soma > 360:
            soma -= 360
        if soma - 0.5 <= dic_functions['ang_odom'] <= soma + 0.5:
            if self.FLAG == 'retorna':
                self.FLAG = 'retorna_odom_bifurcacao'
            else:
                self.FLAG = 'segue_linha'
        else:
            self.set_velocidade(0.0, (angulo/abs(angulo))*0.10)
        

    #----------------------- Sinalização -----------------------
    def controle_sinalizacao(self, dic_functions):
        if dic_functions['sinalizacao'] == 'bifurcacao' and (dic_functions['distancia_frontal'] <= 1.15) and self.FLAG == 'segue_linha':
            self.FLAG = 'bifurcacao'
            print('bifurcacao')
            self.dic['momento'] = rospy.get_time()
            self.dic['posicao_bifurcacao'] = dic_functions['posicao']
            self.RosFunctions.set_dic('corte_direita',True)
            self.seguir_linha()
        elif dic_functions['sinalizacao'] == 'retorna' and (dic_functions['distancia_frontal'] <= 0.7) and self.FLAG == 'segue_linha':
            self.FLAG = 'retorna'
            print('retorna')
            self.dic['angulo_salvo'] = dic_functions['ang_odom']
        elif dic_functions['sinalizacao'] == 'rotatoria' and (dic_functions['distancia_frontal'] <= 0.7) and self.FLAG == 'segue_linha':
            self.FLAG = 'rotatoria'
            print('rotatoria')
            self.dic['momento'] = rospy.get_time()
            self.dic['posicao_rotatoria'] = dic_functions['posicao']
            self.RosFunctions.set_dic('corte_direita',True)
            self.seguir_linha()


    #---------------------- Movimentação -----------------------
    def organiza_movimentacao(self,dic_functions):
        now = rospy.get_time()
        if self.FLAG == 'bifurcacao':
            if now - self.dic['momento'] > 4:
                self.FLAG = 'segue_linha'
                self.RosFunctions.set_dic('corte_direita',False)
            else:
                self.seguir_linha()
        elif self.FLAG == 'rotatoria':
            if now - self.dic['momento'] > 5:
                if self.em_rotatoria:
                    self.FLAG = 'segue_linha'
                    self.em_rotatoria = False
                else:
                    self.FLAG = 'retorna_odom_rotatoria'
                    self.em_rotatoria = True
                self.RosFunctions.set_dic('corte_direita',False)
            else:
                self.seguir_linha()
        elif self.FLAG == 'retorna':
            self.rotacao_odom(dic_functions, 180)

    def retorna_odom_sinalizacao(self,dic_functions):
        now = rospy.get_time()
        if self.FLAG == 'retorna_odom_bifurcacao':
            if self.dic['posicao_bifurcacao'][1] - 0.3 < dic_functions['posicao'][1] < self.dic['posicao_bifurcacao'][1] + 0.6:
                self.dic['momento'] = rospy.get_time()
                self.RosFunctions.set_dic('corte_direita',True)
                self.FLAG = 'bifurcacao'
            else:
                self.seguir_linha()
        elif self.FLAG == 'retorna_odom_rotatoria':
            if self.dic['posicao_rotatoria'][0] - 0.7 < dic_functions['posicao'][0] < self.dic['posicao_rotatoria'][0] + 0.7 and self.dic['posicao_rotatoria'][1] - 0.3 < dic_functions['posicao'][1] < self.dic['posicao_rotatoria'][1] + 0.3 and now-self.dic['momento'] > 10:
                self.dic['momento'] = rospy.get_time()
                self.RosFunctions.set_dic('corte_direita',True)
                self.FLAG = 'rotatoria'
            else:
                self.seguir_linha()

            
    #------------------------ OFF-Road -------------------------
    def retorna_pista(self, posicao0, angulo0):
        self.dic_functions = self.RosFunctions.get_dic()

        objetivo = Point()
        objetivo.x = posicao0[0]
        objetivo.y = posicao0[1]

        x, y = self.dic_functions["posicao"]
        theta = angulo0

        inc_x = objetivo.x - x
        inc_y = objetivo.y - y
        angle_to_objetivo = atan2(inc_y, inc_x)
        
        if (objetivo.x - 5 < x < objetivo.x + 5 and objetivo.y - 5 < y < objetivo.y + 5):
            if abs(angle_to_objetivo - theta) < 0.1:
                self.set_velocidade(0.05, 0.1)
            else:
                self.set_velocidade(0.3, 0.0)
        else:
            self.FLAG = "segue_linha"
            
        

        

        
