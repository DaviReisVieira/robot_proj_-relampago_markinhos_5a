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

import ros_functions




def init():
    tfl = 0
    tf_buffer = tf2_ros.Buffer()

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    tfl = tf2_ros.TransformListener(tf_buffer)
    tolerancia = 25

    return velocidade_saida


def follow_road():
    angle = ros_functions.get_angle()


def celeracuzao():
    Twist(Vector3(0, 0, vel), Vector3(0, 0, 0))
