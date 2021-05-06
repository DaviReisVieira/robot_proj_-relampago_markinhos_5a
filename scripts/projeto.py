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

import visao_module
import ros_functions
import ros_actions


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


if __name__ == "__main__":
    
    ros_functions.init()
    velocidade_saida = ros_actions.init()

    try:
        vel = Twist(Vector3(0,0,0), Vector3(0,0,0.3))
        
        while not rospy.is_shutdown():
            for r in resultados:
                print(r)
            
            velocidade_saida.publish(vel)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")

