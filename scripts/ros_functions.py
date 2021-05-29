from __future__ import print_function, division
#from relampago_markinhos import RelampagoMarkinhos
import rospy
import numpy as np
import math
import tf
import auxiliar as aux
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
    def __init__(self, objetivo):
        '''
        função de init da classe
        Start dos Subscribers (inicialização dos sensores e suas funções de controle)
        Criação do dicionário de variáveis e a inicialização delas
        '''
        self.bridge = CvBridge()
        self.dict = {}
        self.dict_ids = {}

        self.recebedor = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.roda_todo_frame, queue_size=4, buff_size=2**24)
        self.recebe_scan = rospy.Subscriber("/scan", LaserScan, self.scaneou)
        self.recebe_odom = rospy.Subscriber("/odom", Odometry , self.recebeu_leitura_odometria)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)


        #self.dict_relampago = relampago_markinhos.get_dic()
        #self.dict_actions = ros_actions.get_dic()

        self.camera_bgr = None

        self.dict['objetivo'] = objetivo

        self.dict['corte_direita'] = False

        self.dict['centro_imagem'] = (0,0)
        self.dict['distancia_frontal'] = 100
        self.dict['distancia_lateral_esquerda'] = 100
        self.dict['distancia_lateral_direita'] = 100
        self.dict['posicao'] = [0, 0]
        self.dict['ang_odom'] = 0
        self.dict['ang_amarelo'] = 0
        self.dict['centro_x_amarelo'] = 320
        self.dict['ids'] = []
        self.dict['sinalizacao'] = 'nenhuma'
        self.dict['passou_bifurcacao'] = False
        self.dict_ids["centro_id"] = (0, 0)
        self.dict["id"] = 12
        self.dict['distancia_aruco'] = 100000


        self.ids = []
        
        self.marker_size = 20

        self.calib_path  = "/home/borg/catkin_ws/src/robot202/ros/exemplos202/scripts/"
        self.camera_matrix   = np.loadtxt(self.calib_path+'cameraMatrix_raspi.txt', delimiter=',')
        self.camera_distortion   = np.loadtxt(self.calib_path+'cameraDistortion_raspi.txt', delimiter=',')


    ##======================== GETTERS =========================##
    def get_dic(self):
        # Getter do dicionário da classe
        return self.dict

    def get_dic_ids(self):
        # Getter do dicionário da classe
        return self.dict_ids

    def get_camera_bgr(self):
        # Getter da camera em codificação de cores bgr da função "roda_todo_frame"
        return self.camera_bgr

    ##======================== SETTERS =========================##
    def set_dic(self,chave,variavel):
        # Setter da variável do dicionrio para a classe RosActions
        self.dict[chave] = variavel
    
    ##====================== SUBSCRIBERS =======================##

    #------------------------- Camera ----------------------------
    def roda_todo_frame(self,imagem):
        '''
        função que roda todo o frame enviado pelo robô para ser analisado
        Responsável por aplicar a regressão da linha amarela, identificar o aruco e orientar a organização da sinalização identificados
        Identifica o mobilenet e calcula a área da estação desejada quando necessário
        '''
        #dic_relampago = RelampagoMarkinhos.get_dic()
        try:
            imagem_crua = imagem
            imagem_original = self.bridge.compressed_imgmsg_to_cv2(imagem_crua, "bgr8")
            self.camera_bgr = imagem_original

            self.dict['centro_imagem'] = (self.camera_bgr.shape[1]//2,self.camera_bgr.shape[0]//2)

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
        self.dict['distancia_frontal'] = min(distancia_frontal)
        self.dict['distancia_lateral_esquerda'] = min(distancia_lateral_esquerda)
        self.dict['distancia_lateral_direita'] = min(distancia_lateral_direita)


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
        self.dict['posicao'] = [x_odom, y_odom]
        self.dict['ang_odom'] = ang_odom


    ##======================= FUNCTIONS ========================##
    def regressao_linha(self):
        '''
        Faz a regressão linear da linha tracejada amarela por meio do centro dos traços
        Utiliza funções salvas no arquivo auxiliar aux.py
        '''
        mask = aux.filtrar_cor(self.camera_bgr,np.array([22, 50, 50], dtype=np.uint8), np.array([32, 255, 255], dtype=np.uint8), True)
        
        if self.dict['corte_direita']:
            img, centro_amarelo = aux.regiao_centro_de_massa(mask, mask.shape[1]//2, 0, mask.shape[1], mask.shape[0])  
        else:
            img, centro_amarelo = aux.regiao_centro_de_massa(mask, 0, 300, mask.shape[1], mask.shape[0])  
        
        saida_bgr, m, h = aux.ajuste_linear_grafico_x_fy(mask)
        
        ang = math.atan(m)
        ang_deg = math.degrees(ang)

        self.dict['ang_amarelo'] = ang_deg
        self.dict['centro_x_amarelo'] = centro_amarelo[0]

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
            self.corners, self.ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict)

            if draw_image:
                aruco.drawDetectedMarkers(img, self.corners, self.ids)
            cv2.imshow("Aruco", img)

            self.dict['ids'] = np.array(self.ids)
            
            # self.centro_aruco()
            # self.distancia_aruco()
                
            


    def identifica_sinais(self):
        # Função que dos ids identificados, organiza a sinalização da pista
        try:
            for i in self.dict['ids']:
                if i == 100:
                    self.dict['sinalizacao'] = 'bifurcacao'
                elif i == 200:
                    self.dict['sinalizacao'] = 'rotatoria'
                elif (i == 50 or i == 150) and (self.dict['sinalizacao'] != 'bifurcacao' or self.dict['passou_bifurcacao']):
                    self.dict['sinalizacao'] = 'retorna'
        except Exception:
	        #print("Ocorreu uma erro na leitura dos IDs. Markinhos não passa bem.")
            pass

    def centro_aruco(self):
        if self.ids is not None:
            if self.dict['id'] in self.ids:
                i = list(self.ids).index(self.dict['id'])
                p1, p2 = (np.array(self.corners[i][0][0], dtype=np.uint8), np.array(self.corners[i][0][2], dtype=np.uint8))
                centro = ((p2[0] + p1[0])//2, (p2[1] + p1[1])//2)
                print(centro)
                    
        

    def distancia_aruco(self):
        self.dict['distancia_aruco'] = 100000
        if self.ids is not None:
            if self.dict['id'] in self.ids:
                i = list(self.ids).index(self.dict['id'])
                ret = aruco.estimatePoseSingleMarkers(self.corners[i], self.marker_size, self.camera_matrix, self.camera_distortion)
                rvec, self.tvec = ret[0][0,0,:], ret[1][0,0,:]
                self.dict['distancia_aruco'] = np.sqrt(self.tvec[0]**2 + self.tvec[1]**2 + self.tvec[2]**2)


            #-- Print distance
            str_dist = "Dist aruco = {0:.4f}".format(self.dict['distancia_aruco'])
            print(str_dist)