from __future__ import print_function, division
import rospy
import numpy as np
import math
import tf
import aux
import cv2
import cv2.aruco as aruco
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from tf import transformations
from tf import TransformerROS
import tf2_ros


class RosFunctions:
    # Classe que cuidará dos sensores do markinhos, iniciada na classe RelampagoMarkinhos e utilizada na RosActions

    ##========================== INIT ==========================##
    def __init__(self):
        '''
        função de init da classe
        Start dos Subscribers (inicialização dos sensores e suas funções de controle)
        Criação do dicionário de variáveis e a inicialização delas
        '''
        self.bridge = CvBridge()
        self.dic = {}

        self.recebedor = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.roda_todo_frame, queue_size=4, buff_size=2**24)
        self.recebe_scan = rospy.Subscriber("/scan", LaserScan, self.scaneou)
        self.recebe_odom = rospy.Subscriber("/odom", Odometry , self.recebeu_leitura_odometria)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)


        #self.dic_relampago = relampago_markinhos.get_dic()
        #self.dic_actions = ros_actions.get_dic()

        self.camera_bgr = None

        self.dic['corte_direita'] = False

        self.dic['centro_imagem'] = (0,0)
        self.dic['distancia_frontal'] = 100
        self.dic['distancia_lateral_esquerda'] = 100
        self.dic['distancia_lateral_direita'] = 100
        self.dic['posicao'] = [0, 0]
        self.dic['ang_odom'] = 0
        self.dic['ang_amarelo'] = 0
        self.dic['centro_x_amarelo'] = 320
        self.dic['ids'] = []
        self.dic['sinalizacao'] = 'nenhuma'
        self.dic['passou_bifurcacao'] = False
        
        


    ##======================== GETTERS =========================##
    def get_dic(self):
        # Getter do dicionário da classe
        return self.dic

    def get_camera_bgr(self):
        # Getter da camera em codificação de cores bgr da função "roda_todo_frame"
        return self.camera_bgr

    ##======================== SETTERS =========================##
    def set_dic(self,chave,variavel):
        # Setter da variável do dicionrio para a classe RosActions
        self.dic[chave] = variavel
    
    ##====================== SUBSCRIBERS =======================##

    #------------------------- Camera ----------------------------
    def roda_todo_frame(self,imagem):
        '''
        função que roda todo o frame enviado pelo robô para ser analisado
        Responsável por aplicar a regressão da linha amarela, identificar o aruco e orientar a organização da sinalização identificados
        '''
        try:
            imagem_crua = imagem
            imagem_original = self.bridge.compressed_imgmsg_to_cv2(imagem_crua, "bgr8")
            self.camera_bgr = imagem_original

            self.dic['centro_imagem'] = (self.camera_bgr.shape[1]//2,self.camera_bgr.shape[0]//2)

            self.regressao_linha()
            self.aruco_ids(True)
            self.identifica_sinais()

            cv2.waitKey(1)
        except CvBridgeError as e:
            print('ex', e)


    #-------------------------- Lazer ----------------------------
    def scaneou(self, dado):
        '''
        Função que cuida do laser do robô para identificação da distância
        Salva a distância frontal, 40 graus para a direita e para a esquerda, identificando a menor distância nessas direções
        '''
        ranges = np.array(dado.ranges).round(decimals=2)
        distancia_frontal = [ranges[357],ranges[358],ranges[359],ranges[0],ranges[1],ranges[2],ranges[3]]
        distancia_lateral_esquerda = [ranges[319],ranges[320], ranges[321]]
        distancia_lateral_direita = [ranges[39],ranges[40],ranges[41]]
        self.dic['distancia_frontal'] = min(distancia_frontal)
        self.dic['distancia_lateral_esquerda'] = min(distancia_lateral_esquerda)
        self.dic['distancia_lateral_direita'] = min(distancia_lateral_direita)


    #-------------------------- Odom -----------------------------
    def recebeu_leitura_odometria(self, dado):
        '''
        Recebe e organiza a leitura da odometria
        Salva a posição x e y e o ângulo z em graus
        '''
        x_odom = dado.pose.pose.position.x
        y_odom = dado.pose.pose.position.y
        quat = dado.pose.pose.orientation

        lista = [quat.x, quat.y, quat.z, quat.w]
        angulos = np.degrees(transformations.euler_from_quaternion(lista)) 
        if angulos[2] < 0:
            ang_odom = 360 + angulos[2]
        else:
            ang_odom = angulos[2]

        #print('posicao x:', self.x_odom, ',  posixao y', self.y_odom)
        self.dic['posicao'] = [x_odom, y_odom]
        self.dic['ang_odom'] = ang_odom


    ##======================= FUNCTIONS ========================##
    def regressao_linha(self):
        '''
        Faz a regressão linear da linha tracejada amarela por meio do centro dos traços
        Utiliza funções salvas no arquivo auxiliar aux.py
        '''
        mask = aux.filtrar_cor(self.camera_bgr,np.array([22, 50, 50], dtype=np.uint8), np.array([36, 255, 255], dtype=np.uint8))
        
        if self.dic['corte_direita']:
            img, centro_amarelo = aux.regiao_centro_de_massa(mask, mask.shape[1]//2, 0, mask.shape[1], mask.shape[0])  
        else:
            img, centro_amarelo = aux.regiao_centro_de_massa(mask, 0, 300, mask.shape[1], mask.shape[0])  
        
        saida_bgr, m, h = aux.ajuste_linear_grafico_x_fy(mask)
        
        ang = math.atan(m)
        ang_deg = math.degrees(ang)

        self.dic['ang_amarelo'] = ang_deg
        self.dic['centro_x_amarelo'] = centro_amarelo[0]

        cv2.imshow("Filtro", img)
        # cv2.imshow("Regressão", saida_bgr)

    #-------------------------- Aruco ----------------------------
    def aruco_ids(self, draw_image = False):
        '''
        Identifica os ids salvos nos arucos vistos pela camera
        '''
        img = self.camera_bgr
        if img is not None:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict)
            
            self.dic['ids'] = np.array(ids).flatten()

            if draw_image:
                aruco.drawDetectedMarkers(img, corners, ids)
            cv2.imshow("Original", img)

    def identifica_sinais(self):
        # Função que dos ids identificados, organiza a sinalização da pista
        try:
            for i in self.dic['ids']:
                if i == 100:
                    self.dic['sinalizacao'] = 'bifurcacao'
                elif i == 200:
                    self.dic['sinalizacao'] = 'rotatoria'
                elif (i == 50 or i == 150) and (self.dic['sinalizacao'] != 'bifurcacao' or self.dic['passou_bifurcacao']):
                    self.dic['sinalizacao'] = 'retorna'
        except Exception:
	        #print("Ocorreu uma erro na leitura dos IDs. Markinhos não passa bem.")
            pass