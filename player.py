import pygame
from motor import Motor
import math
import numpy as np
from numpy import linalg as la
#classe jogador
class Player(pygame.sprite.Sprite):

    def __init__(self):
        super().__init__()
        #parametros
        self.tau = 0.005 #cste de tempo
        self.m = 0.250 #kg
        self.Iz = 2*10**(-4) #kg.m2
        self.l = 0.1 #m
        self.wmax = 15000 #rpm
        self.kf = 1.744*10**(-8) #constante de força
        #parametros do jogo
        self.image = pygame.image.load('assets/drone/drone_1.png')
        self.rect = self.image.get_rect()
        self.rect.x = 335
        self.rect.y = 760
        self.rot = pygame.transform.rotate(self.image, 0)
        #atitude ?
        self.angle = 0
        #velocidade linear
        self.velocity = np.transpose(np.array([0.0, 0.0]))
        #posicao
        self.pos = np.transpose(np.array([self.rect.x-self.rot.get_rect().width/2, self.rect.y-self.rot.get_rect().height/2]))
        #velocidade angular
        self.omega = 0.0
        #velocidade de rotaçao dos motores
        self.w = np.transpose(np.array([0.0, 0.0]))
        #forca de controle
        self.Fc = np.transpose(np.array([ 0.0, np.sum(np.multiply(self.w,self.w)*self.kf)]))
        #torque de controle
        self.Tc = self.l*self.kf*(self.w[0]**2-self.w[1]**2)
        #matriz de rotacao
        self.Drb = np.array([[math.cos(self.angle*0.0174533), -math.sin(self.angle*0.0174533)],[math.sin(self.angle*0.0174533), math.cos(self.angle*0.0174533)]]) 
        #Forca peso
        self.P = -self.m*9.8 #N

    def atualizar_dina_cine(self):
        
        #velocidade angular
        self.omega += self.tau*self.Tc/self.Iz
        #angulo
        self.angle += self.tau*self.omega
        #forca de controle
        self.Fc = np.transpose(np.array([ 0, np.sum(np.multiply(self.w,self.w)*self.kf)]))
        #torque de controle
        self.Tc = self.l*self.kf*(self.w[0]**2-self.w[1]**2)
        #matriz de rotacao
        self.Drb = np.array([[math.cos(self.angle*0.0174533), -math.sin(self.angle*0.0174533)],[math.sin(self.angle*0.0174533), math.cos(self.angle*0.0174533)]])
        #velocidade linear
        self.velocity += self.tau*(1/self.m)*(np.dot(self.Drb,self.Fc))
        #self.velocity[1] -=  self.tau*(1/self.m)*self.P
        # #posicao
        print("moteurs: ", self.w)
        print("Fc: ", self.Fc)
        print("Tc: ", self.Tc )
        print("omega: ",self.omega)
        print("angle: ", self.angle)
        print("vitesse: ",self.velocity)
        print("pos: ",self.pos)

        self.pos[0] += self.tau*self.velocity[0]
        self.pos[1] += self.tau*self.velocity[1]
        self.rect.y = self.pos[1]+self.rot.get_rect().height/2
        self.rect.x = self.pos[0]+self.rot.get_rect().width/2 
        self.rot = pygame.transform.rotate(self.image, self.angle)
        self.pos[0] = self.rect.x-self.rot.get_rect().width/2
        self.pos[1] = self.rect.y-self.rot.get_rect().height/2
        

    def move_right(self):
        self.pos[0] += self.velocity[0]
        self.rect.x = self.pos[0]+self.rot.get_rect().width/2
    
    def move_left(self):
        self.pos[0] -= self.velocity[0]
        self.rect.x = self.pos[0]+self.rot.get_rect().width/2

    def move_up(self):
        self.pos[1] -= self.velocity[0]
        self.rect.y = self.pos[1]+self.rot.get_rect().height/2
    
    def move_down(self):
        self.pos[1] += 0.3
        self.rect.y = self.pos[1]+self.rot.get_rect().height/2

    def rotate_right(self):
        self.angle -= self.omega
        self.rot = pygame.transform.rotate(self.image, self.angle)
        self.pos[0] = self.rect.x-self.rot.get_rect().width/2
        self.pos[1] = self.rect.y-self.rot.get_rect().height/2

#funcoes de ligacao e desligacao dos motores para testes dinamicos
    def motor_right_on(self):
        self.w[1] = 20000.0
    
    def motor_right_off(self):
        self.w[1] = 0

    def motor_left_on(self):
        self.w[0] = 20000.0

    def motor_left_off(self):
        self.w[0] = 0

    


#funções de teste para controle estatico
    def rotate_left(self):
        self.angle += self.omega
        self.rot = pygame.transform.rotate(self.image, self.angle)
        self.pos[0] = self.rect.x-self.rot.get_rect().width/2
        self.pos[1] = self.rect.y-self.rot.get_rect().height/2
    
    def move_forward(self):
        self.pos[0] -= math.cos((self.angle-90)*0.0174533)*la.norm(self.velocity)
        self.pos[1] += math.sin((self.angle-90)*0.0174533)*la.norm(self.velocity)
        self.rect.x = self.pos[0]+self.rot.get_rect().width/2
        self.rect.y = self.pos[1]+self.rot.get_rect().height/2

    def move_backward(self):
        pass
    