#!/usr/bin/python3
# -*- coding: utf-8 -*-ss
'''
@author: NicolasQueiroga, fran-janela, DaviReisVieira
'''


import numpy as np
import cv2
import os
import statsmodels.api as sm


def filtrar_cor(bgr, low, high, kernel=False):
    '''
    Can recieve the img in bgr, and color range or the color value in hex
    '''
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, low, high)

    if kernel:
        kernel_final = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(6,6))
        mask = cv2.morphologyEx( mask, cv2.MORPH_OPEN, kernel_final )

    return mask 


def encontrar_contornos(mask):
    '''
        Não mude ou renomeie esta função
        deve receber uma imagem preta e branca os contornos encontrados
    '''
    contornos, arvore = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

    return contornos

def encontrar_maior_contorno(segmentado):
    '''
    Não mude ou renomeie esta função
    deve receber uma imagem preta e 
    branca e retornar APENAS o maior contorno obtido
    '''

    contornos, arvore = cv2.findContours(segmentado.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    maior = None
    maior_area = 0
    for c in contornos:
        area = cv2.contourArea(c)
        if area > maior_area:
            maior_area = area
            maior = c


    return maior, maior_area

    
def find_center(frame, maior_contorno, centro):
    '''
    Não mude ou renomeie esta função
    deve receber um contorno e retornar, 
    respectivamente, a imagem com uma cruz no centro de cada segmento 
    e o centro dele. formato: img, x, y
    '''
    if not maior_contorno is None:
        cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 0], 1)
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


def desenhar_linha_entre_pontos(img, X, Y, color=(255, 120, 0)):
    '''
        Não mude ou renomeie esta função
        deve receber uma lista de coordenadas XY, e retornar uma imagem 
        com uma linha entre os centros EM SEQUENCIA do mais proximo.
    '''

    img_line = img.copy()
    for i in range(len(X) - 2):
        cv2.line(img_line, (X[i], Y[i]), (X[i+1], Y[i+1]), color, thickness=3, lineType=8)

    return img_line


def mobilenet_classes():
    CLASSES = [ "background", "aeroplane", "bicycle", "bird", "boat",
                "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
                "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
                "sofa", "train", "tvmonitor" ]
    return CLASSES


def detect(net, frame, CONFIDENCE, COLORS, CLASSES):
    '''
    Recebe - uma imagem colorida BGR
    Devolve: objeto encontrado
    '''
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


def center_of_mass(mask):
    '''
    Retorna uma tupla (cx, cy) que desenha o centro do contorno
    '''
    M = cv2.moments(mask)
    m00 = max(M["m00"],1) # para evitar dar erro quando não há contornos
    cX = int(M["m10"] / m00)
    cY = int(M["m01"] / m00)
    return [int(cX), int(cY)]

def text(img, a, p, color=(255, 255, 255), font=cv2.FONT_HERSHEY_SIMPLEX, width=2, size=4):
    cv2.putText(img, str(a), p, font,size,color,width,cv2.LINE_AA)
    return


def ajuste_linear_x_fy(mask):
    '''
    Recebe uma imagem já limiarizada e faz um ajuste linear
    retorna coeficientes linear e angular da reta
    e equação é da forma
    x = coef_angular*y + coef_linear
    '''
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
    '''
    Faz um ajuste linear e devolve uma imagem rgb com aquele ajuste desenhado sobre uma imagem
    Trabalhando com x em funcão de y
    '''
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
    '''
    Recebe frame e coordenadas da tela para corte
    Retorna centro da imagem e máscara cortada
    '''
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
    '''
    Recebe frame e cor do Creeper
    Retorna centro da image, maior contorno da imagem e media do centro de massa do contorno
    '''
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
    segmentado_cor = filtrar_cor(bgr, cor_menor, cor_maior)
    centro = (frame.shape[1]//2, frame.shape[0]//2)

    maior_contorno, maior_contorno_area = encontrar_maior_contorno(segmentado_cor.copy())
    media = find_center(frame, maior_contorno, centro)

    return centro, maior_contorno_area, media
