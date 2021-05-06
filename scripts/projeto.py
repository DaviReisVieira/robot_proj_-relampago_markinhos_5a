#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import tf
import aux
import cv2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped

from nav_msgs.msg import Odometry
from std_msgs.msg import Header

print("EXECUTE ANTES da 1.a vez: ")
print("wget https://github.com/Insper/robot21.1/raw/main/projeto/ros_projeto/scripts/MobileNetSSD_deploy.caffemodel")
print("PARA TER OS PESOS DA REDE NEURAL")

import visao_module


bridge = CvBridge()

cv_image = None
media = []
centro = []
atraso = 1.5E9


area = 0.0
check_delay = False 

resultados = []

x = 0
y = 0
z = 0 
id = 0

frame = "camera_link"
tfl = 0
tf_buffer = tf2_ros.Buffer()



def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global media
    global centro
    global resultados

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime
    delay = lag.nsecs
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        centro, saida_net, resultados =  visao_module.processa(temp_image)        
        for r in resultados:
            pass

        cv_image = saida_net.copy()
        bgr = temp_image.copy()
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        yellow1, yellow2 = (25, 50, 50), (35, 255, 255)
        mask = aux.makeMask(hsv, yellow1, yellow2)

        # contours = bgr.copy()
        # maskContours = aux.encontrar_contornos(mask)
        # cv2.drawContours(bgr, maskContours, -2, (255, 0, 0), 6)

        # imgWithCenter, xList, yList = aux.findCenter(bgr, contours)
        # imgWithLines = aux.desenhar_linha_entre_pontos(bgr, xList, yList)

        # if xList:
	    #     imgWithRegression, slope = aux.regressao_por_centro(bgr, xList, yList)

        # angle = aux.angleWithVertical(bgr, slope)

        cv2.imshow("img", mask)
        cv2.waitKey(1)

    except CvBridgeError as e:
        print('ex', e)




if __name__=="__main__":

    rospy.init_node("cor")
    topico_imagem = "/camera/image/compressed"
    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    tfl = tf2_ros.TransformListener(tf_buffer)
    tolerancia = 25

    try:
        vel = Twist(Vector3(0,0,0), Vector3(0,0,np.pi/10.0))
        
        while not rospy.is_shutdown():
            for r in resultados:
                print(r)
            
            velocidade_saida.publish(vel)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")


