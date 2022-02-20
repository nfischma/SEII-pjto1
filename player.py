import pygame
from motor import Motor
import math
import numpy as np
from numpy import linalg as la
import matplotlib.pyplot as plt
from control.matlab import *

from scipy.integrate import odeint

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
        self.pos_rot = np.transpose(np.array([self.rect.x-self.rot.get_rect().width/2, self.rect.y-self.rot.get_rect().height/2]))
        self.pos_dis = np.transpose(np.array([0,0]))

        

        #tempo
        self.t = [0.0,self.tau]

        #Forca peso
        self.P = -self.m*9.8 #N

        #r
        #posicao
        self.pos = np.transpose(np.array([self.rect.x-self.rot.get_rect().width/2, self.rect.y-self.rot.get_rect().height/2]))
        #listas posiçoes
        self.r = np.copy(self.pos) + np.copy(self.pos)
        #comando de posicao
        self.rbarra = 0.0

        #v
        #velocidade linear
        self.velocity = np.transpose(np.array([0.0, 0.0]))
        #lista velocidades
        self.dr = [0.0, (self.r[1]-self.r[0])/self.tau]

        #phi
        #atitude
        self.angle = 0.0
        #lista atitude
        self.phi = [self.angle, self.angle]
        #comando de angulo
        self.phibarra = 0.0

        #omega
        #velocidade angular
        self.omega = 0.0
        #lista velocidade angular
        self.dphi = [0.0, (self.phi[1]-self.phi[0])/self.tau]

        #Drb
        #matriz de rotacao
        #self.Drb = np.array([[math.cos((self.angle)*0.0174533), -math.sin((self.angle)*0.0174533)],[math.sin((self.angle)*0.0174533), math.cos((self.angle)*0.0174533)]]) 

        #w
        #velocidade de rotaçao dos motores
        self.w = np.transpose(np.array([0.0, 0.0]))
        #velocidade de rotação de comando
        self.wbarra = np.transpose(np.array([0.0, 0.0]))


        #Fc
        #forca de controle
        #self.Fc = np.transpose(np.array([ 0.0, 0.0]))
        #self.Fc[1] = self.kf*(self.w[1]**2+self.w[0]**2)
        #lista Fc
        self.Fcs = [np.transpose(np.array([ 0.0, 0.0]))]
        #Fc de comando
        self.Fcbarra = np.transpose(np.array([ 0.0, 0.0]))

        #Tc
        #torque de controle
        #self.Tc = self.l*self.kf*(self.w[0]**2-self.w[1]**2)
        #liste Tc
        #self.Tc = [self.l*self.kf*(self.w[0]**2-self.w[1]**2)]
        #Tc de comando
        self.Tcbarra = 0.0


    def atualizar_posição_tela(self):
        #atualiza a posição e a rotação do robo na tela
        # #posicao
        self.rot = pygame.transform.rotate(self.image, self.angle)
        self.pos_rot[0] = self.rect.x-self.rot.get_rect().width/2 
        ##self.pos_dis[0] +=  self.tau*self.velocity[0]*10
        ##self.pos[0] = self.pos_rot[0] + self.pos_dis[0]
        self.pos[0] = self.x[-1]
        self.pos[1] = self.y[-1]

        
        ##print(self.rect.y-self.rot.get_rect().height/2 - self.pos_dis[1] )
        ##if self.pos[1]<880 or self.velocity[1] > 0 :
        ##    self.pos_rot[1] = self.rect.y-self.rot.get_rect().height/2 
        ##    self.pos_dis[1] -= self.tau*self.velocity[1]*10
        ##    self.pos[1] = self.pos_rot[1] + self.pos_dis[1]
        ##else:
        ##    self.velocity[1] = 5.0;
        
        self.rect.y = self.pos[1]+self.rot.get_rect().height/2
        self.rect.x = self.pos[0]+self.rot.get_rect().width/2 

    def Cp(self):
        #Controlador da posicção: 
        #entrada r,v e saida Fcbarra, phibarra
        pass

    def Ca(self):
        #controlador do angulo
        #entrada phi,omega e saida Tcbarra
        pass
        

    def Cw(self):
        #Controlador dos motores:
        #entrada Fc,Tc e saida a velocidade de rotação dos motores
        pass

    def din_robo(y, t, wbarra):
        # Parametros da planta
         #parametros
        tau = 0.005         #cste de tempo
        m = 0,250           #kg
        Iz = 2*10**(-4)     #kg.m2
        l = 0.1             #m
        wmax = 15000        #rpm
        kf = 1.744*10**(-8) #constante de força
        P = m*9.8           #N

        w, r, v, phi, omega = y

        #forca de controle
        Fc = np.transpose(np.array([0,kf*(w[1]**2+w[0]**2)]))
        #torque de controle
        Tc = l*kf*(w[0]**2-w[1]**2)
        #Matriz de rotação
        Drb = np.array([[math.cos((phi)*0.0174533), -math.sin((phi)*0.0174533)],[math.sin((phi)*0.0174533), math.cos((phi)*0.0174533)]])

        # Dinamica do robo
        wp = (-w+wbarra)/tau
        rp = v
        vp = (np.dot(Drb,Fc) + P)/m
        phip = omega
        omegap = Tc/Iz


        return [wp, rp, vp, phip, omegap]


    def atualizar_dinamica(self):
        #FC e phibarra
        self.Cp()
        #Tc
        self.Ca()
        #wbarras
        self.Cw()

        #dinamica robo
        # Evoluindo a din. da planta
        w0 = self.w
        r0 = self.r[1]
        v0 = self.dr[1]
        phi0 = self.phi[1]
        omega0 = self.dphi[1]
        x0 = [w0, r0, v0, phi0, omega0]   # condicao inicial
        sol = odeint(self.din_robo, x0, [0.0, self.tau], args=(self.wbarra,))
        
        self.w = sol[:,0][-1]
        self.r += sol[:,1][-1]
        self.dr += sol[:,2][-1]
        self.phi += sol[:,3][-1]
        self.dphi += sol[:,4][-1]
        self.t += self.tau

        #atualizar posicao
        self.pos = self.r[-1]
        self.velocity = self.dr[-1]
        self.angle = self.phi[-1]
        self.omega = self.dphi[-1]
        
        



















#funcoes de teste
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
        self.w[1] = 10000.0

    def motor_left_on(self):
        self.w[0] = 10000.0

    def motors_off(self):
        self.w[0] = 0.0
        self.w[1] = 0.0

    def motor_lr_on(self):
        self.w[0] = 10000.0
        self.w[1] = 10000.0


    


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
    