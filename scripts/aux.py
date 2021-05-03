'''
@author: NicolasQueiroga
'''

#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
from sklearn.linear_model import LinearRegression
import os

print("Trabalhando em", os.getcwd())
# -----------------------------------------------------------------------------------------------------------
# 1----------------------------------------------------------------------------------------------------------
def hsv_hists(img, plt):
    """
        Plota o histograma de cada um dos canais HSV
        img - imagem HSV
        plt - objeto matplotlib
    """
    plt.figure(figsize=(20,10)); 
    img_h = img[:,:,0]
    img_s = img[:,:,1]
    img_v = img[:,:,2]
    histo_plot(img_h, "r","H", plt);
    histo_plot(img_s, "g","S", plt);
    histo_plot(img_v, "b","V", plt);

def make_hist(img_255, c, label, plt):
    """
        img_255 - uma imagem com 3 canais de 0 até 255
        c a cor do plot
        label - o label do gráfico
        plt - matplotlib.pyplot
    """
    hist,bins = np.histogram(img_255.flatten(),256,[0,256])
    cdf = hist.cumsum()
    cdf_normalized = cdf * hist.max()/ cdf.max()

    # plt.plot(cdf_normalized, color = c)
    plt.hist(img_255.flatten(),256,[0,256], color = c)
    plt.xlim([0,256])
    plt.legend(label, loc = 'upper left')
    plt.plot()

def histo_plot(img, cor, label, plt):
    """
        img - imagem
        cor - cor
        plt - matplotlib.pyplot object

    """
    plt.figure(figsize=(10,5))
    make_hist(img, cor, label, plt)
    plt.show()
    plt.figure(figsize=(10,5))
    plt.imshow(img, cmap="Greys_r")#, vmin=0, vmax=255)    
    plt.title(label)
# -----------------------------------------------------------------------------------------------------------
# 2----------------------------------------------------------------------------------------------------------
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

def makeMask(hsv, low=None, high=None, hexx=None):
    """
        Can recieve the img in hsv, and color range or the color value in hex
    """
    if hexx is not None and low is None and high is None:
        low, high = ranges(hexx)

    mask = cv2.inRange(hsv, low, high)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(6,6))
    mask = cv2.morphologyEx( mask, cv2.MORPH_OPEN, kernel )
    mask = cv2.morphologyEx( mask, cv2.MORPH_CLOSE, kernel )
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
    return maior
# -----------------------------------------------------------------------------------------------------------
# 4----------------------------------------------------------------------------------------------------------
def findCenter(img, contornos):
    """
        Não mude ou renomeie esta função
        deve receber um contorno e retornar, 
        respectivamente, a imagem com uma cruz no centro de cada segmento 
        e o centro dele. formato: img, x, y
    """
    X = []
    Y = []
    areaL = []

    for c in contornos:
        areaL.append(cv2.contourArea(c))
    mean = np.mean(areaL)

    for c in contornos:
        area = cv2.contourArea(c)
        M = cv2.moments(c)
        if (M["m00"] != 0) and area >= mean/2:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            X.append(cX)
            Y.append(cY)
            p = (int(cX), int(cY))
            crosshair(img, p, 20, (128, 0, 0))

    return img, X, Y

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
            cv2.circle(output,(i[0],i[1]),2,(0,0,255),3)
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
    arrayX = np.array(x).reshape(-1, 1)
    arrayY = np.array(y).reshape(-1, 1)
    modelo = LinearRegression()
    modelo.fit(arrayX, arrayY)
    lm, h = modelo.coef_, modelo.intercept_

    xMin = int(min(arrayX))
    xMax = int(max(arrayX))

    yMin = int(lm*xMin + h)
    yMax = int(lm*xMax + h)

    cv2.line(img, (xMin, yMin), (xMax, yMax), (255, 255, 0), 3)

    return img, lm

def angleWithVertical(img, lm):
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
    proto = "./mobilenet_detection/MobileNetSSD_deploy.prototxt.txt" 
    model = "./mobilenet_detection/MobileNetSSD_deploy.caffemodel"

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

    print("[INFO] computing object detections...")
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
            print("[INFO] {}".format(label))
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
def calcular_h(centro1, centro2):
    """ 
        Não mude ou renomeie esta função
        deve receber dois pontos e retornar a distancia absoluta entre eles
    """
    x1, y1 = centro1
    x2, y2 = centro2
    d = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    return d

def encontrar_foco(D,H,h):
    """
        Não mude ou renomeie esta função
        deve receber respectivamente a distancia real,
        o a distancia real entre os circulos e a distancia
        na image entre os circulos e deve retornar o foco
    """
    f = D*h/H
    return f

def encontrar_distancia(f,H,h):
    """
        Não mude ou renomeie esta função
        deve receber respectivamente o foco a 
        distancia real entre os circulos e a distancia na 
        image entre os circulos e retornar a distancia real
    """
    D = f*H/h
    return D
# -----------------------------------------------------------------------------------------------------------
# misc-------------------------------------------------------------------------------------------------------
def center_of_mass(data):
    M = cv2.moments(data)
    if (M["m00"] != 0):
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return (int(cX), int(cY))

def texto(img, a, p, color=(0, 255, 255), font=cv2.FONT_HERSHEY_SIMPLEX, width=2, size=1):
    cv2.putText(img, str(a), p, font,size,color,width,cv2.LINE_AA)
    return
# -----------------------------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------------------------