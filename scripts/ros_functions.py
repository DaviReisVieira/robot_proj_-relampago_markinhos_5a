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

import visao_module

bridge = CvBridge()
atraso = 1.5E9
cv_image = None
media = []
centro = []


area = 0.0
check_delay = False 

resultados = []

x = 0
y = 0
z = 0 
id = 0

frame = "camera_link"


def init():
    rospy.init_node("cor")
    topico_imagem = "/camera/image/compressed"
    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)


def roda_todo_frame(imagem):
    print("frame")
    global cv_image, media, centro, resultados, angle


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

        contours = bgr.copy()
        maskContours = aux.encontrar_contornos(mask)
        cv2.drawContours(contours, maskContours, -2, (255, 0, 0), 6)

        imgWithCenter = bgr.copy()
        imgWithCenter, xList, yList = aux.find_center(imgWithCenter, maskContours)

        imgWithLines = bgr.copy()
        imgWithLines = aux.desenhar_linha_entre_pontos(imgWithLines, xList, yList)

        imgWithRegression = bgr.copy()
        if xList:
            imgWithRegression, slope = aux.regressao_por_centro(imgWithRegression, xList, yList)
            out = imgWithRegression.copy()
            angle = aux.angle_with_vertical(out, slope)
            aux.texto(out, f"angle", (50, 50))

        cv2.imshow("img", imgWithRegression)
        cv2.waitKey(1)

    except CvBridgeError as e:
        print('ex', e)



def get_angle():
    return angle