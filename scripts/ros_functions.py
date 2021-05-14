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

import relampago_markinhos
import ros_actions


class rosFunctions:

    ##========================== INIT ==========================##
    def __init__(self):
        self.bridge = CvBridge()
        self.dic = {}

        self.recebedor = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.roda_todo_frame, queue_size=4, buff_size=2**24)
        self.recebe_scan = rospy.Subscriber("/scan", LaserScan, self.scaneou)
        self.recebe_odom = rospy.Subscriber("/odom", Odometry , self.recebeu_leitura_odometria)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)


        #self.dic_relampago = relampago_markinhos.get_dic()
        #self.dic_actions = ros_actions.get_dic()

        self.camera_bgr = None


    ##======================== GETTERS =========================##
    def get_dic(self):
        return self.dic_funcions

    ##======================== SETTERS =========================##

    
    ##====================== SUBSCRIBERS =======================##

    #------------------------- Camera ----------------------------
    def roda_todo_frame(self,imagem):
        try:
            self.imagem_crua = imagem
            imagem_original = self.bridge.compressed_imgmsg_to_cv2(self.imagem_crua, "bgr8")
            self.camera_bgr = imagem_original

            self.regressao_linha()
            self.aruco_ids()
            self.identifica_sinais()

            cv2.waitKey(1)
        except CvBridgeError as e:
            print('ex', e)


    #-------------------------- Lazer ----------------------------
    def scaneou(self, dado):
        ranges = np.array(dado.ranges).round(decimals=2)
        self.distancia = [ranges[358],ranges[0],ranges[2]]


    #-------------------------- Odom -----------------------------
    def recebeu_leitura_odometria(self, dado):

        self.x_odom = dado.pose.pose.position.x
        self.y_odom = dado.pose.pose.position.y
        quat = dado.pose.pose.orientation

        lista = [quat.x, quat.y, quat.z, quat.w]
        angulos = np.degrees(transformations.euler_from_quaternion(lista)) 
        if angulos[2] < 0:
            self.ang_odom = 360 + angulos[2]
        else:
            self.ang_odom = angulos[2]

        #print('posicao x:', self.x_odom, ',  posixao y', self.y_odom)



    ##======================= FUNCTIONS ========================##
    def regressao_linha(self):
        mask = aux.filtrar_cor(self.camera_bgr,np.array([22, 50, 50],dtype=np.uint8), np.array([36, 255, 255],dtype=np.uint8))
        img, centro_amarelo = aux.regiao_centro_de_massa(mask, 0, 300, mask.shape[1], mask.shape[0])  
        saida_bgr, m, h = aux.ajuste_linear_grafico_x_fy(mask)
        
        ang = math.atan(m)
        ang_deg = math.degrees(ang)

        self.dic['ang_amarelo'] = ang_deg
        self.dic['centro_x_amarelo'] = centro_amarelo[0]

        # cv2.imshow("Filtro", img)
        # cv2.imshow("Regressão", saida_bgr)

    #-------------------------- Aruco ----------------------------
    def aruco_ids(self, draw_image = False):
        img = self.camera_bgr
        if img is not None:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict)
            
            self.dic['ids'] = np.array(ids).flatten()

            if draw_image:
                aruco.drawDetectedMarkers(img, corners, ids)
            cv2.imshow("Original", img)
            # self.dic_ids = {}
            # try:
            #     for i in range(len(self.ids)):
            #         self.dic_ids[self.ids[i]]  =  np.array(corners[i])
            # except Exception:
            #     pass
            # print(self.dic_ids)

    def identifica_sinais(self):
        try:
            for i in self.dic['ids']:
                if i == 100:
                    self.dic['sinalizacao'] = 'bifurcacao'
                elif i == 200:
                    self.dic['sinalizacao'] = 'rotatoria'
                elif (i == 50 or i == 150) and (self.dic['sinalizacao'] != 'bifurcacao' or self.passou_bifurcacao):
                    self.dic['sinalizacao'] = 'retorna'
        except Exception:
            pass


