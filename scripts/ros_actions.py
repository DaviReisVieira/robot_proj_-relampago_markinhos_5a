from __future__ import print_function, division
import rospy
import numpy as np
import tf
import aux
import cv2
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from std_msgs.msg import Float64

import ros_functions as rosFunctions

class rosActions:

    ##========================== INIT ==========================##
    def __init__(self):
        self.dic = {}

        self.velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.ombro = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
        self.garra = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)


    ##======================== GETTERS =========================##
    def get_dic(self):
        return self.dic_funcions

    ##======================== SETTERS =========================##
    def set_velocidade(self, v_lin=0.0, v_ang=0.0):
        self.velocidade.linear.x = v_lin
        self.velocidade.angular.z = v_ang