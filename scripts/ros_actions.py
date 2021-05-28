from __future__ import print_function, division
import rospy
import numpy as np
import tf
import cv2
from math import atan2, degrees
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped, Point
from std_msgs.msg import Float64
from termcolor import colored
from garra import Garra


class RosActions:
    # Classe que cuidará da movimentação do markinhos, iniciada na classe RelampagoMarkinhos

    ##========================== INIT ==========================##
    def __init__(self, RosFunctions, creeper, estacao):
        '''
        Inicialização e recebe a classe RosFunctions para utilizar suas variáveis
        Indicação do publisher de velocidade.
        '''
        self.RosFunctions = RosFunctions
        self.creeper = creeper
        self.estacao = estacao
        self.garra = Garra()
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
        self.dic['resultado'] = 'start'
        self.em_rotatoria = True
        self.posicao0 = None
        self.angulo0 = 0
        self.Procurando = True
        self.creeper_atropelado = False
        self.creeper_centralizado = False
        self.momento_garra = 0

        self.chegou = False
        self.ready = False

    ##======================== GETTERS =========================##
    def get_dic(self):
        # Getter do dicionário de variáveis desta classe
        return self.dic

    def get_resultado(self):
        # Getter do dicionário de variáveis desta classe
        return self.dic['resultado']

    ##======================== SETTERS =========================##
    def set_velocidade(self, v_lin=0.0, v_ang=0.0):
        # Set da velocidade desejada no robô - publish imediato
        self.velocidade.linear.x = v_lin
        self.velocidade.angular.z = v_ang
        self.velocidade_saida.publish(self.velocidade)

    
    ##======================= MOVEMENT =========================##
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

    def pegar_creeper(self): 
        v_lin = 0.1
        dic_functions = self.RosFunctions.get_dic()
        img = self.RosFunctions.get_camera_bgr()
        if img is not None:
            centro, maior_contorno_area, media = self.creeper.identifica_creepers(self.RosFunctions)
            print(dic_functions['distancia_frontal'])
            if not self.creeper_atropelado:
                # v_lin = 
                if 0.25 < dic_functions['distancia_frontal'] < 0.28:
                    v_lin = 0.07

                if dic_functions['distancia_frontal'] < 0.24:
                    self.garra.abrir_garra()
                    self.creeper_centralizado = True
                    v_lin = 0.03

                if dic_functions['distancia_frontal'] <= 0.16:
                    print('PAROU PARA PEGAR O CREEPER')
                    self.momento_garra = rospy.get_time()
                    self.creeper_atropelado = True

                if self.creeper_centralizado:
                    self.set_velocidade(v_lin)

                if maior_contorno_area > 700 and not self.creeper_centralizado:
                    if len(centro) != 0 and len(media) != 0:
                        if centro[0] -15 < media[0] < centro[0] + 15:
                            self.set_velocidade(v_lin)
                        else: 
                            delta_x = centro[0] - media[0]
                            max_delta = 150
                            w = (delta_x/max_delta)*0.15
                            self.set_velocidade(v_lin,w)
                        
            if self.creeper_atropelado:
                self.set_velocidade()
                self.dic['resultado'] = self.garra.capturar_objeto(self.momento_garra)      


    def procurando_creeper(self):
        dic_functions = self.RosFunctions.get_dic()
        img = self.RosFunctions.get_camera_bgr()
        if img is not None:
            centro, maior_contorno_area, media = self.creeper.identifica_creepers(self.RosFunctions)
            if (maior_contorno_area > 700) and self.Procurando:
                if self.posicao0 is None:
                    self.posicao0 = dic_functions["posicao"]
                    self.angulo0 = dic_functions["ang_odom"]
                    print(colored(" - 'Relâmpago Markinhos': Localizei o Alvo!","red"))
                    print('Posição salva: ',self.posicao0,self.angulo0)
                    self.dic['resultado'] = 'encontrou_creeper'
                    self.Procurando = False
            elif self.Procurando:
                self.segue_pista()

    def encontrar_estacao(self):
        img = self.RosFunctions.get_camera_bgr()
        self.segue_pista()
        self.estacao.estacao_objetivo(img)

    ##======================= FUNCTIONS ========================##
    #-------------------------- Linha --------------------------
    def seguir_linha(self):
        '''
        Função responsável por seguir a linha amarela com controle proporcional de velocidade FALTA FAZER CONTROLE DERIVATIVO (erro/dt)
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
            self.dic['angulo_salvo'] = dic_functions['ang_odom']
        elif dic_functions['sinalizacao'] == 'rotatoria' and (dic_functions['distancia_frontal'] <= 0.7) and self.FLAG == 'segue_linha':
            print(colored('Entrando no circuito oval!','yellow'))
            print(colored(" - 'Relâmpago Markinhos': Estou no circuito oval! Me avisa para sair.","red"))
            print(" - 'Doc Hudson': Amigo, você é um corredor corajoso.")
            print(" - 'Marty': Pode deixar, amigo!")
            self.FLAG = 'entra_rotatoria'
            self.RosFunctions.set_dic('passou_bifurcacao',False)
            self.dic['angulo_salvo'] = dic_functions['ang_odom']
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
    def retorna_pista(self):
        '''
        Função que orienta o robô depois que saiu da pist principal para ir buscar o creeper e precisa voltar ao ponto de saída
        Retorna ao ponto de saída por odometria e se indireita com o mesmo ângulo, para continuar a pista sem problemas.
        '''
        self.dic_functions = self.RosFunctions.get_dic()
        goal = Point()
        goal.x = self.posicao0[0]
        goal.y = self.posicao0[1]
        x, y = self.dic_functions["posicao"]

        dist = np.sqrt((goal.x - x)**2 + (goal.y - y)**2)

        inc_x = goal.x - x
        inc_y = goal.y - y

        theta = self.dic_functions["ang_odom"]
        angle_to_goal = degrees(atan2(inc_y, inc_x))
        if angle_to_goal > 360:
            angle_to_goal -= 360
        elif angle_to_goal < 0:
            angle_to_goal += 360
        
        dif_abs = abs(angle_to_goal - theta)

        print(f"angle to goal: {angle_to_goal}\nrobot angle:{theta}")

        if dist <= 0.2:
            self.set_velocidade() 
            self.chegou = True
        if not self.chegou:
            if abs(angle_to_goal - theta) > 5 and (theta - angle_to_goal) < 0 and dist > 0.2:
                self.set_velocidade(0, 0.17)
            elif abs(angle_to_goal - theta) > 5 and (theta - angle_to_goal) > 0 and dist > 0.2:
                self.set_velocidade(0, -0.17)
            else:
                self.set_velocidade(0.23, 0.0)
        if self.chegou:
            if (theta - self.angulo0 <= 0) and not self.ready:
                self.set_velocidade(0, 0.3)
                if self.angulo0 - 0.5 < theta < self.angulo0 + 0.5:
                    self.ready = True
            elif (theta - self.angulo0 > 0) and not self.ready:
                self.set_velocidade(0, -0.3)
                if self.angulo0 - 0.5 < theta < self.angulo0 + 0.5:
                    self.ready = True
            elif self.ready:
                self.set_velocidade()
                self.dic['resultado'] = 'retornou'


        
        

        

        
