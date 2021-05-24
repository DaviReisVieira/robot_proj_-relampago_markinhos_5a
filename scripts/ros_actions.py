from __future__ import print_function, division
import rospy
import numpy as np
import tf
import aux
import cv2
from math import atan2
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped, Point
from std_msgs.msg import Float64
from termcolor import colored


class RosActions:
    # Classe que cuidará da movimentação do markinhos, iniciada na classe RelampagoMarkinhos

    ##========================== INIT ==========================##
    def __init__(self, RosFunctions):
        '''
        Inicialização e recebe a classe RosFunctions para utilizar suas variáveis
        Indicação do publisher de velocidade.
        '''
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
        self.em_rotatoria = True

    ##======================== GETTERS =========================##
    def get_dic(self):
        # Getter do dicionário de variáveis desta classe
        return self.dic

    ##======================== SETTERS =========================##
    def set_velocidade(self, v_lin=0.0, v_ang=0.0):
        # Set da velocidade desejada no robô - publish imediato
        self.velocidade.linear.x = v_lin
        self.velocidade.angular.z = v_ang
        self.velocidade_saida.publish(self.velocidade)

    
    ##======================= FUNCTIONS ========================##
    def segue_pista(self):
        '''
        Função principal de seguir a pista do projeto
        Organiza as funções de modo a identificar a sinalização também
        '''
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
        '''
        Função responsável por seguir a linha amarela com controle proporcional de velocidade
        '''
        dic_functions = self.RosFunctions.get_dic()
        if dic_functions['centro_imagem'][0]-10 < dic_functions['centro_x_amarelo'] < dic_functions['centro_imagem'][0] + 10:
            self.set_velocidade(0.4)
            if -16 < dic_functions['ang_amarelo'] < 16:
                self.set_velocidade(0.6)
        else: 
            delta_x = dic_functions['centro_imagem'][0] - dic_functions['centro_x_amarelo'] 
            max_delta = 150
            w = (delta_x/max_delta)*0.25
            self.set_velocidade(0.2,w)

    #------------------------- Rotação -------------------------
    def rotacao_odom(self, dic_functions, angulo): 
        '''
        Recebe o ângulo com a amplitude de rotação
        O robô realiza essa rotação partinfo fo seu angulo de odometria
        positivo para a esquerda, negativo para a direita
        '''
        soma = self.dic['angulo_salvo'] + angulo
        if soma < 0:
            soma += 360
        elif soma > 360:
            soma -= 360
        if soma - 1 <= dic_functions['ang_odom'] <= soma + 1:
            if self.FLAG == 'retorna':
                self.FLAG = 'retorna_odom_bifurcacao'
            elif self.FLAG == 'entra_rotatoria':
                self.dic['momento'] = rospy.get_time()
                self.FLAG = 'retorna_odom_rotatoria'
            else:
                print(colored(" - 'Relâmpago Markinhos': Concentra, concentra, velocidade!","red"))
                self.FLAG = 'segue_linha'
        else:
            self.set_velocidade(0.0, (angulo/abs(angulo))*0.30)
        

    #----------------------- Sinalização -----------------------
    def controle_sinalizacao(self, dic_functions):
        '''
        Verifica se o id de sinalização está pronto para se tornar um ação, avaliando o tipo de sinalização disponível e a que distância está dela
        recebe a sinalização provinda da função que cria uma máscara dos id's vistos pelo aruco
        '''
        if dic_functions['sinalizacao'] == 'bifurcacao' and (dic_functions['distancia_frontal'] <= 1.15) and self.FLAG == 'segue_linha':
            print(colored('Bifurcação para a direita - VELOCIDADE!','yellow'))
            print(" - 'Doc Hudson': Antigamente os carros não dirigiam pra ganhar tempo, dirigiam pra aproveitar o tempo.")
            self.FLAG = 'bifurcacao'
            self.dic['momento'] = rospy.get_time()
            self.dic['posicao_bifurcacao'] = dic_functions['posicao']
            self.RosFunctions.set_dic('corte_direita',True)
            self.RosFunctions.set_dic('sinalizacao','nenhuma')
            self.seguir_linha()
        elif dic_functions['sinalizacao'] == 'retorna' and (dic_functions['distancia_frontal'] <= 0.7) and self.FLAG == 'segue_linha':
            print(colored('Retorno! - Marty avisou pelo rádio do perigo em frente!','yellow'))
            print(colored(" - 'Relâmpago Markinhos': Caramba Marty, essa foi por pouco!","red"))
            self.FLAG = 'retorna'
            print('retorna')
            self.dic['angulo_salvo'] = dic_functions['ang_odom']
        elif dic_functions['sinalizacao'] == 'rotatoria' and (dic_functions['distancia_frontal'] <= 0.7) and self.FLAG == 'segue_linha':
<<<<<<< HEAD
            self.FLAG = 'rotatoria'
            print('rotatoria')
            self.dic['momento'] = rospy.get_time()
=======
            print(colored('Entrando no circuito oval!','yellow'))
            print(colored(" - 'Relâmpago Markinhos': Estou no circuito oval! Me avisa para sair.","red"))
            print(" - 'Doc Hudson': Amigo, você é um corredor corajoso.")
            print(" - 'Marty': Pode deixar, amigo!")
            self.FLAG = 'entra_rotatoria'
            self.RosFunctions.set_dic('passou_bifurcacao',False)
            self.dic['angulo_salvo'] = dic_functions['ang_odom']
>>>>>>> 13f1c2e4fc96263812c7f54026f7a21d3c03ceb3
            self.dic['posicao_rotatoria'] = dic_functions['posicao']


    #---------------------- Movimentação -----------------------
    def organiza_movimentacao(self,dic_functions):
        '''
        dependendo da FLAG que está sendo executada, organiza a movimentação do robô e a máscara para a regressão da linha amarela
        aplica essas variações pelo delta do tempo passado na ação
        '''
        now = rospy.get_time()
        if self.FLAG == 'bifurcacao':
            if now - self.dic['momento'] > 4:
                self.FLAG = 'segue_linha'
                self.RosFunctions.set_dic('corte_direita',False)
                self.RosFunctions.set_dic('passou_bifurcacao',True)
            else:
                self.seguir_linha()
        elif self.FLAG == 'entra_rotatoria':
            self.rotacao_odom(dic_functions, -75)
        elif self.FLAG == 'rotatoria':
            if now - self.dic['momento'] > 5:
                self.FLAG = 'retorna_odom_rotatoria'
            else:
                self.seguir_linha()
        elif self.FLAG == 'sai_rotatoria':
            self.rotacao_odom(dic_functions, -75)
        elif self.FLAG == 'retorna':
            self.rotacao_odom(dic_functions, 180)

    def retorna_odom_sinalizacao(self,dic_functions):
        '''
        Função que identifica se o robô voltou à odometria maracada de cada sinalização para realizar a sequência de movimentos necessária para continuar na pista
        '''
        now = rospy.get_time()
        if self.FLAG == 'retorna_odom_bifurcacao':
            if self.dic['posicao_bifurcacao'][1] - 0.6 < dic_functions['posicao'][1] < self.dic['posicao_bifurcacao'][1] + 0.6:
                self.dic['momento'] = rospy.get_time()
                self.RosFunctions.set_dic('corte_direita',True)
                self.FLAG = 'bifurcacao'
                print(colored('Relâmpago Markinhos está na bifurcação!','yellow'))
                print(colored(" - 'Relâmpago Markinhos': Eu sei aonde estamos indo!","red"))
            else:
                self.seguir_linha()
        elif self.FLAG == 'retorna_odom_rotatoria':
            if self.dic['posicao_rotatoria'][0] - 0.7 < dic_functions['posicao'][0] < self.dic['posicao_rotatoria'][0] + 0.7 and self.dic['posicao_rotatoria'][1] - 0.3 < dic_functions['posicao'][1] < self.dic['posicao_rotatoria'][1] + 0.3 and now-self.dic['momento'] > 10:
                self.dic['angulo_salvo'] = dic_functions['ang_odom']
                self.FLAG = 'sai_rotatoria'
                print(colored('Saindo do circuito oval!','yellow'))
                print(" - 'Marty': Pode sair agora, amigo!")
                print(colored(" - 'Relâmpago Markinhos': Eu sou a velocidade!","red"))
            else:
                self.seguir_linha()

            
    #------------------------ OFF-Road -------------------------
    def retorna_pista(self, posicao0, angulo0):
        '''
        Função que orienta o robô depois que saiu da pist principal para ir buscar o creeper e precisa voltar ao ponto de saída
        Retorna ao ponto de saída por odometria e se indireita com o mesmo ângulo, para continuar a pista sem problemas.
        '''
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
            
        

        

        
