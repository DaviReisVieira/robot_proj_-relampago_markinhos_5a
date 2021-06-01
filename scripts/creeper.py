import auxiliar as aux

class Creeper:
    def __init__(self, cor, codigo):
        self.cor = cor
        self.codigo = codigo

    def get_cor(self):
        return self.cor

    def get_codigo(self):
        return self.codigo

    def identifica_creepers(self, ros_functions):
        '''
        Mascara da cor do objetivo do creeper,
        retorna o centro da imagem, o centro dos contornos e a area do maior contorno
        '''
        centro, maior_contorno_area, media = aux.identifica_cor(ros_functions.camera_bgr, self.get_cor())
        return centro, maior_contorno_area, media
    
        