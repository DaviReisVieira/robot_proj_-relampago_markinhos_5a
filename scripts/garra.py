import rospy
from std_msgs.msg import Float64

import aux

class Garra:
    def __init__(self):
        self.comecou_garra = False
        self.ombro = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
        self.garra = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)

        self.ombro.publish(-1.0)  ## Levanta  
        self.garra.publish(0.0)  ## Fechado  

    def abrir_garra(self):
        self.garra.publish(-1.0) ## Aberto
        self.ombro.publish(0.0) ## esta para frente


    def capturar_objeto(self, momento):
        now = rospy.get_time()
        if not self.comecou_garra:
            self.comecou_garra = True
        elif now - momento < 2.0:
            self.garra.publish(0.0)  ## Fechado   
            print('Fecha garra')
        elif 2.0 <= now - momento < 3.0:
            self.ombro.publish(1.5)  ## Levanta
            print('levanta ombro')
        else:
            return 'pegou_creeper'  
        return 'pegando_creeper' 
        
    def soltar_objeto(self, momento):
        pass
        