import rospy
from std_msgs.msg import Float64

import aux

class Garra:
    def __init__(self):
        self.comecou_garra = False
        self.ombro = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
        self.garra = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)

    def capturar_objeto(self, momento):
        now = rospy.get_time()
        if not self.comecou_garra:
            self.comecou_garra = True
        elif now - momento < 1.5:
            self.garra.publish(-1.0) ## Aberto
            self.ombro.publish(0.0) ## esta para frente
        elif 1.5 <= now - momento < 3:
            print('Fecha garra')
            self.garra.publish(0.0)  ## Fechado   
        elif 3 <= now - momento < 5:
            print('levanta ombro')
            self.ombro.publish(1.5)  ## Levanta   
        
    def soltar_objeto(self, momento):
        pass
        