'''
@author: NicolasQueiroga
'''

#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
import math
import cv2
from sklearn.linear_model import LinearRegression
import os


def convert_to_tuple(html_color):
    colors = html_color.split("#")[1]
    r = int(colors[0:2],16)
    g = int(colors[2:4],16)
    b = int(colors[4:],16)
    return (r,g,b)

def to_1px(tpl):
    img = np.zeros((1,1,3), dtype=np.uint8)
    img[0,0,0] = tpl[0]
    img[0,0,1] = tpl[1]
    img[0,0,2] = tpl[2]
    return img

def to_hsv(html_color):
    tupla = convert_to_tuple(html_color)
    hsv = cv2.cvtColor(to_1px(tupla), cv2.COLOR_RGB2HSV)
    return hsv[0][0]

def ranges(value):
    hsv = to_hsv(value)
    hsv2 = np.copy(hsv)
    hsv[0] = max(0, hsv[0]-15)
    hsv2[0] = min(180, hsv[0]+ 15)
    hsv[1:] = 50
    hsv2[1:] = 255
    return hsv, hsv2 

def make_mask(bgr, low, high, kernel=False):
    """
        Can recieve the img in bgr, and color range or the color value in hex
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, low, high)

    if kernel:
        kernel_final = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(6,6))
        mask = cv2.morphologyEx( mask, cv2.MORPH_OPEN, kernel_final )
        mask = cv2.morphologyEx( mask, cv2.MORPH_CLOSE, kernel_final )

    return mask 
# -----------------------------------------------------------------------------------------------------------
# 3----------------------------------------------------------------------------------------------------------
def encontrar_contornos(mask):
    """
        Não mude ou renomeie esta função
        deve receber uma imagem preta e branca os contornos encontrados
    """
    contornos, arvore = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

    return contornos

def encontrar_maior_contorno(segmentado):
    """
        Não mude ou renomeie esta função
        deve receber uma imagem preta e 
        branca e retornar APENAS o maior contorno obtido
    """
    contornos, arvore = cv2.findContours(segmentado.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    maior = None
    maior_area = 0
    for c in contornos:
        area = cv2.contourArea(c)
        if area > maior_area:
            maior_area = area
            maior = c


    return maior, maior_area
# -----------------------------------------------------------------------------------------------------------
# 4----------------------------------------------------------------------------------------------------------
def find_center(frame, maior_contorno, centro):
    """
        Não mude ou renomeie esta função
        deve receber um contorno e retornar, 
        respectivamente, a imagem com uma cruz no centro de cada segmento 
        e o centro dele. formato: img, x, y
    """
    if not maior_contorno is None :
        cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 0], 5)
        maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
        media = maior_contorno.mean(axis=0)
        media = media.astype(np.int32)
        cv2.circle(frame, (media[0], media[1]), 5, [0, 255, 0])
        crosshair(frame, centro)
    else:
        media = (0, 0)

    return media

def crosshair(img, point, size=20, color=(128, 0, 0)):
    x, y = point
    cv2.line(img, (x - size, y), (x + size, y), color, 2)
    cv2.line(img, (x, y - size), (x, y + size), color, 2)
# -----------------------------------------------------------------------------------------------------------
# 5----------------------------------------------------------------------------------------------------------
def allCircles(bgr, mask, color=(0, 255, 0)):
    """
        retorna os círculsos encontrados na imagem
        No formato (x,y,raio)
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mascara_blur = cv2.blur(mask, (3,3))
    mask = mascara_blur
    retorno, mask_limiar = cv2.threshold(mask, 100 ,255, cv2.THRESH_BINARY)    
    
    bordas = auto_canny(mask_limiar)
    circles = cv2.HoughCircles(image=bordas, method=cv2.HOUGH_GRADIENT, dp=2.5, minDist=40, param1=50, param2=100, minRadius=5, maxRadius=150)
    mask_limiar_rgb = cv2.cvtColor(mask_limiar, cv2.COLOR_GRAY2RGB)
    bordas_rgb = cv2.cvtColor(bordas, cv2.COLOR_GRAY2RGB)

    output =  bordas_rgb

    if circles is not None:        
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            # draw the outer circle
            cv2.circle(output,(i[0],i[1]),i[2],color,2)
            # draw the center of the circle
            cv2.circle(output,(i[0],i[1]),2,(0,0,0),3)
    return circles 
    
def auto_canny(image, sigma=0.33):
    v = np.median(image)

    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    return edged
# -----------------------------------------------------------------------------------------------------------
# 6----------------------------------------------------------------------------------------------------------
def desenhar_linha_entre_pontos(img, X, Y, color=(255, 120, 0)):
    """
        Não mude ou renomeie esta função
        deve receber uma lista de coordenadas XY, e retornar uma imagem 
        com uma linha entre os centros EM SEQUENCIA do mais proximo.
    """
    img_line = img.copy()
    for i in range(len(X) - 2):
        cv2.line(img_line, (X[i], Y[i]), (X[i+1], Y[i+1]), color, thickness=3, lineType=8)

    return img_line

def regressao_por_centro(img, x, y):
    """
        Não mude ou renomeie esta função
        deve receber uma lista de coordenadas XY,
        e estimar a melhor reta, utilizando o metodo preferir, 
        que passa pelos centros. Retorne a imagem com a reta e os parametros da reta
        
        Dica: cv2.line(img,ponto1,ponto2,color,2) desenha uma linha que passe entre os pontos,
        mesmo que ponto1 e ponto2 não pertençam a imagem.
    """
    array_x = np.array(x).reshape(-1, 1)
    array_y = np.array(y).reshape(-1, 1)
    modelo = LinearRegression()
    modelo.fit(array_x, array_y)
    coef_ang, h = modelo.coef_, modelo.intercept_
    # x_total = np.arange(0,1000)
    # x_total_array = np.array(x_total).reshape((-1, 1))
    # y_pred = modelo.predict(x_total_array)

    x_min = int(min(array_x))
    x_max = int(max(array_x))

    y_min = int(coef_ang*x_min + h)
    y_max = int(coef_ang*x_max + h)

    cv2.line(img, (x_min, y_min), (x_max, y_max), (255, 255, 0), 3)

    return img, coef_ang, h

def angle_with_vertical(lm):
    """
        Não mude ou renomeie esta função
        deve receber uma lista de coordenadas XY, 
        e estimar a melhor reta, utilizando o metodo preferir, que passa 
        pelos centros. Retorne a imagem com a reta.
        
        Dica: cv2.line(img,ponto1,ponto2,color,2) desenha uma linha que passe entre os pontos, 
        mesmo que ponto1 e ponto2 não pertençam a imagem.
    """
    angulo = 90 + math.degrees(math.atan(lm))

    return angulo    
# -----------------------------------------------------------------------------------------------------------
# 7----------------------------------------------------------------------------------------------------------
def estimar_linha_nas_faixas(img, mask):
    """
        Não mude ou renomeie esta função
        deve receber uma imagem preta e branca e retorna 
        dois pontos que formen APENAS uma linha em cada faixa. 
        Desenhe cada uma dessas linhas na iamgem.
        formato: [[(x1,y1),(x2,y2)], [(x1,y1),(x2,y2)]]
    """
    out = []
    isLeft = False
    lines = cv2.HoughLinesP(mask, 1, np.pi/180, 50, None, 50, 10)
    if lines is not None:
        for i in range(0, len(lines)):
            l = lines[i][0]
            x1, y1, x2, y2 = (l[0], l[1], l[2], l[3]) 
            grad = (y2 - y1)/(x2 - x1)
            if grad < 0 and not isLeft:
                p = []
                p.append((x1, y1))
                p.append((x2, y2))
                out.append(p)
                isLeft = True
                cv2.line(img, (x1, y1), (x2, y2), (255, 100, 0), 3)
            elif grad >= 0:
                p = []
                p.append((x1, y1))
                p.append((x2, y2))
                out.append(p)
                cv2.line(img, (x1, y1), (x2, y2), (255, 100, 0), 3)
                return out

    return None

def calcular_equacao_das_retas(linhas):
    """
        Não mude ou renomeie esta função
        deve receber dois pontos que estejam 
        em cada uma das faixas e retornar a equacao das 
        duas retas. Onde y = h + m * x. Formato: [(m1,h1), (m2,h2)]
    """
    out = []
    for line in linhas:
        x1, y1 = line[0]
        x2, y2 = line[1]

        m = (y2 - y1)/(x2 - x1)
        h = y2 - m*x2

        out.append((m, h))

    return out

def calcular_ponto_de_fuga(img, equacoes):
    """ 
        Não mude ou renomeie esta função
        deve receber duas equacoes de retas e retornar o 
        ponto de encontro entre elas. Desenhe esse ponto na imagem.
    """
    m1, h1 = equacoes[0]
    m2, h2 = equacoes[1]
    pX = int((h2 - h1)/(m1 - m2))
    pY = int(m1*pX + h1)
    img = cv2.circle(img, (pX,pY), radius=12, color=(0, 0, 255), thickness=-1)

    return img, (pX, pY)   
# -----------------------------------------------------------------------------------------------------------
# 8----------------------------------------------------------------------------------------------------------
def mobilenet_classes():
    CLASSES = [ "background", "aeroplane", "bicycle", "bird", "boat",
                "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
                "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
                "sofa", "train", "tvmonitor" ]
    return CLASSES

def load_mobilenet():
    """
        Não mude ou renomeie esta função
        Carrega o modelo e os parametros da MobileNet. Retorna a classe da rede.
    """
    proto = "/mobilenet_detection/MobileNetSSD_deploy.prototxt.txt" 
    model = "/mobilenet_detection/MobileNetSSD_deploy.caffemodel"

    rede = cv2.dnn.readNetFromCaffe(proto, model)

    return rede


def detect(net, frame, CONFIDENCE, COLORS, CLASSES):
    """
        Recebe - uma imagem colorida BGR
        Devolve: objeto encontrado
    """
    image = frame.copy()
    (h, w) = image.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)

    #print("[INFO] computing object detections...")
    net.setInput(blob)
    detections = net.forward()

    results = []

    for i in np.arange(0, detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > CONFIDENCE:
            idx = int(detections[0, 0, i, 1])
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            startX, startY, endX, endY = box.astype("int")

            label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
            #print("[INFO] {}".format(label))
            cv2.rectangle(image, (startX, startY), (endX, endY), COLORS[idx], 2)
            y = startY - 15 if startY - 15 > 15 else startY + 15
            cv2.putText(image, label, (startX, y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

            results.append((CLASSES[idx], confidence*100, (startX, startY),(endX, endY) ))

    return image, results


def calcula_iou(boxA, boxB):
    """
        Não mude ou renomeie esta função
        Calcula o valor do "Intersection over Union" para saber se as caixa se encontram
    """

    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])

    # compute the area of intersection rectangle
    interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)

    # compute the area of both the prediction and ground-truth rectangles
    boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
    boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)

    # compute the intersection over union by taking the intersection
    # area and dividing it by the sum of prediction + ground-truth
    # areas - the interesection area
    iou = interArea / float(boxAArea + boxBArea - interArea)

    # return the intersection over union value
    return iou
# -----------------------------------------------------------------------------------------------------------
# 9----------------------------------------------------------------------------------------------------------


# -----------------------------------------------------------------------------------------------------------
# misc-------------------------------------------------------------------------------------------------------
def center_of_mass(mask):
    """ Retorna uma tupla (cx, cy) que desenha o centro do contorno"""
    M = cv2.moments(mask)
    # Usando a expressão do centróide definida em: https://en.wikipedia.org/wiki/Image_moment
    m00 = max(M["m00"],1) # para evitar dar erro quando não há contornos
    cX = int(M["m10"] / m00)
    cY = int(M["m01"] / m00)
    return [int(cX), int(cY)]

def text(img, a, p, color=(255, 255, 255), font=cv2.FONT_HERSHEY_SIMPLEX, width=2, size=4):
    cv2.putText(img, str(a), p, font,size,color,width,cv2.LINE_AA)
    return
# -----------------------------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------------------------
def filtrar_cor(bgr, low, high):
    """ Retorna a máscara com o range"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    x_center, y_center = hsv.shape[1]//2, hsv.shape[0]//2
    y_max, x_max, _ = hsv.shape
    mask = cv2.inRange(hsv, low, high)

    return mask   
    
import statsmodels.api as sm

def ajuste_linear_x_fy(mask):
    """Recebe uma imagem já limiarizada e faz um ajuste linear
        retorna coeficientes linear e angular da reta
        e equação é da forma
        x = coef_angular*y + coef_linear
    """ 
    pontos = np.where(mask==255)
    ximg = pontos[1]
    yimg = pontos[0]

    ## Caso adicionado para evitar resultados invalidos
    if len(ximg) < 10: 
        return 0,0, [[0],[0]]

    yimg_c = sm.add_constant(yimg)
    model = sm.OLS(ximg,yimg_c)
    results = model.fit()
    coef_angular = results.params[1] # Pegamos o beta 1
    coef_linear =  results.params[0] # Pegamso o beta 0
    return coef_angular, coef_linear, pontos # Pontos foi adicionado para performance, como mencionado no notebook

def ajuste_linear_grafico_x_fy(mask_in, print_eq = False): 
    """
       Faz um ajuste linear e devolve uma imagem rgb com aquele ajuste desenhado sobre uma imagem
       Trabalhando com x em funcão de y
    """
    y_centro, x_centro = mask_in.shape[0]//2, mask_in.shape[1]//2
    y_max, x_max = mask_in.shape
    mask = mask_in[y_centro:y_max, 0:x_max]

    coef_angular, coef_linear, pontos  = ajuste_linear_x_fy(mask)
    if print_eq: 
        print("x = {:3f}*y + {:3f}".format(coef_angular, coef_linear))
    ximg = pontos[1]
    yimg = pontos[0]
    y_bounds = np.array([min(yimg), max(yimg)])
    x_bounds = coef_angular*y_bounds + coef_linear
    x_int = x_bounds.astype(dtype=np.int64)
    y_int = y_bounds.astype(dtype=np.int64)
    mask_bgr =  cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)    
    cv2.line(mask_bgr, (x_int[0], y_int[0]), (x_int[1], y_int[1]), color=(0,0,255), thickness=5);    

    return mask_bgr, coef_angular, coef_linear

def regiao_centro_de_massa(mask, x1, y1, x2, y2):
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    clipped = mask[y1:y2, x1:x2]
    c = center_of_mass(clipped)
    c[0]+=x1
    c[1]+=y1
    crosshair(mask_bgr, c, 10, (0,0,255))
    cv2.rectangle(mask_bgr, (x1, y1), (x2, y2), (255,0,0),2,cv2.LINE_AA)
    centro = (int(c[0]), int(c[1]))
    return mask_bgr, centro


def identifica_cor(frame, cor):

    if cor == "blue":
        cor_menor = np.array([75, 50, 50])
        cor_maior = np.array([95, 255, 255])
    elif cor == "green":
        cor_menor = np.array([45, 100, 100])
        cor_maior = np.array([75, 255, 255])
    elif cor == "orange":
        cor_menor = np.array([0, 200, 200])
        cor_maior = np.array([8, 255, 255])

    bgr = frame.copy()
    segmentado_cor = make_mask(bgr, cor_menor, cor_maior)
    centro = (frame.shape[1]//2, frame.shape[0]//2)

    maior_contorno, maior_contorno_area = encontrar_maior_contorno(segmentado_cor.copy())
    media = find_center(frame, maior_contorno, centro)

    s1 = "{:d} {:d}".format(*media)
    s2 = "{:0.1f}".format(maior_contorno_area)
    # text(frame, s1, (20, 100))
    # text(frame, s2, (20, 50))


    return centro, maior_contorno_area, media

from nav_msgs.msg import Odometry
from std_msgs.msg import Header

class relampagoMarkinhos:

    def __init__(self,missao,conceitoC = False):
        self.conceitoC = conceitoC
        self.bridge = CvBridge()
        rospy.init_node("projeto")
        
        self.topico_imagem = "/camera/image/compressed"
        self.recebedor = rospy.Subscriber(self.topico_imagem, CompressedImage, self.roda_todo_frame, queue_size=4, buff_size=2**24)
        self.velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.recebe_scan = rospy.Subscriber("/scan", LaserScan, self.scaneou)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

        self.camera_bgr = None
        self.centro_imagem = (320,240)
        self.velocidade = Twist()
        self.ang_amarelo = 0
        self.centro_x_amarelo = 320
        self.distancia = 100
        self.ids = None
        self.FLAG = 'segue_linha'
        self.sinalizacao = 'nenhuma'

        # self.parameters  = aruco.DetectorParameters_create()
        # self.parameters.minDistanceToBorder = 0
        # self.parameters.adaptiveThreshWinSizeMax = 2000

        self.iniciar_missao()

    def scaneou(self, dado):
        ranges = np.array(dado.ranges).round(decimals=2)
        self.distancia = ranges[0]

    def set_velocidade(self, v_lin=0.0, v_ang=0.0):
        self.velocidade.linear.x = v_lin
        self.velocidade.angular.z = v_ang

    def roda_todo_frame(self,imagem):
        try:
            imagem_original = self.bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
            self.camera_bgr = imagem_original

            self.regressao_linha()
            self.aruco_ids()
            self.identifica_sinais()

            cv2.waitKey(1)
        except CvBridgeError as e:
            print('ex', e)

    def regressao_linha(self):
        # mask = aux.makeMask(self.camera_bgr,(30, 55, 42), (32, 255, 255))
        mask = aux.filter_color(self.camera_bgr,np.array([22, 50, 50],dtype=np.uint8), np.array([36, 255, 255],dtype=np.uint8))
        img, centro_amarelo = aux.center_of_mass_region(mask, 0, 300, mask.shape[1], mask.shape[0])  
        saida_bgr, m, h = aux.ajuste_linear_grafico_x_fy(mask)
        # contornos = aux.encontrar_contornos(mascara_amarelo)
        # centro_contornos, xList, yList = aux.find_center(mascara_amarelo, contornos)
        # img_regressao, m, h = aux.regressao_por_centro(mascara_amarelo, xList, yList)
        
        ang = math.atan(m)
        ang_deg = math.degrees(ang)

        self.ang_amarelo = ang_deg
        self.centro_x_amarelo = centro_amarelo[0]

        #cv2.imshow("Filtro", img)
        #cv2.imshow("Regressão", saida_bgr)

    def aruco_ids(self):
        img = self.camera_bgr
        if img is not None:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict)
            aruco.drawDetectedMarkers(img, corners, ids)
            self.ids = np.array(ids).flatten()
            cv2.imshow("Original", img)


    def seguir_linha(self):
        if self.centro_imagem[0] -10 < self.centro_x_amarelo < self.centro_imagem[0] + 10:
            self.set_velocidade(0.4,0.0)
            self.velocidade_saida.publish(self.velocidade)
            if -15 < self.ang_amarelo < 15:
                self.set_velocidade(0.5,0.0)
                self.velocidade_saida.publish(self.velocidade)
        else: 
            delta_x = self.centro_imagem[0] - self.centro_x_amarelo
            max_delta = 150.0
            w = (delta_x/max_delta)*0.20
            self.set_velocidade(0.2,w)
            self.velocidade_saida.publish(self.velocidade)

    def rotacionar(self,v_ang,momento,delta):
        self.set_velocidade(0.0,0.0)
        self.velocidade_saida.publish(self.velocidade)
        now = rospy.get_time()
        while now - momento < delta:
            #print('DELTA CALCULADO', now - momento)
            self.set_velocidade(0.0,v_ang)
            self.velocidade_saida.publish(self.velocidade)
            #print('rotacionando')
            now = rospy.get_time()
        else:
            self.FLAG = 'segue_linha'
    
    def missao_conceito_c(self):
        #print("Leituras Distancia:",self.distancia)
        self.segue_pista()
        #1.21 - 50
        #1.21 - 200

    def identifica_sinais(self):
        try:
            for i in self.ids:
                if i == 100:
                    self.sinalizacao = 'bifurcacao'
                elif i == 200:
                    self.sinalizacao = 'rotatoria'   
                elif i == 50:
                    self.sinalizacao = 'retorna'
        except Exception:
            pass

    def segue_pista(self):
        if self.sinalizacao == 'bifurcacao' and self.distancia <= 1.25:
            momento = rospy.get_time()
            #print('momento', momento)
            self.rotacionar(-1*(pi/5),momento,1)
            self.sinalizacao == 'nenhuma'
        elif self.sinalizacao == 'retorna' and self.distancia <= 0.7:
            momento = rospy.get_time()
            self.rotacionar(-1*(pi/5),momento,5)
            self.sinalizacao == 'nenhuma'
        elif self.sinalizacao == 'rotatoria' and self.distancia <= 0.9:
            momento = rospy.get_time()
            self.rotacionar(-1*(pi/5),momento,2.5)
            self.sinalizacao == 'nenhuma'
        elif self.FLAG == 'segue_linha':
            self.seguir_linha()
        #print(self.distancia)
        
    def iniciar_missao(self):
        # r = rospy.Rate(200)
        try: 
            while not rospy.is_shutdown():
                if self.conceitoC:
                    self.missao_conceito_c()
            rospy.sleep(0.01)

        except rospy.ROSInterruptException:
            print("Oh Deus quantos CTRL+C")