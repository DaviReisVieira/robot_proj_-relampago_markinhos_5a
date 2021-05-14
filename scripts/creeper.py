import aux

class Creeper:
    def __init__(self, cor, codigo):
        self.cor = cor
        self.codigo = codigo

    def get_cor(self):
        return self.cor

    def get_codigo(self):
        return self.codigo

    def identifica_cor(self, relampago):
        centro, maior_contorno_area, media = aux.identifica_cor(relampago.get_camera_bgr(), self.get_cor())
        return centro, maior_contorno_area, media
    
        