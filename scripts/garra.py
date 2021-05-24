import rospy
from std_msgs.msg import Float64

import aux

class Garra:
    def __init__(self,actions):
        self.actions=actions
        self.comecou_garra = False
        self.ombro = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
        self.garra = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)

        self.ombro.publish(-1.0)  ## Levanta  
        self.garra.publish(0.0)  ## Fechado   

    def capturar_objeto(self, momento,posicao0,angulo0):
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

        self.actions.retorna_pista(posicao0,angulo0)

            
        
    def soltar_objeto(self, momento):
        pass
        