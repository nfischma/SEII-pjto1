import pygame
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
        self.tau = 0.05 #cste de tempo
        self.m = 0.250 #kg
        self.Iz = 2*10**(-4) #kg.m2
        self.l = 0.1 #m
        self.wmax = 15000#*2*math.pi/60 #rad/s   
        self.kf = 1.744*10**(-8) #constante de força
        #parametros do jogo
        self.image = pygame.image.load('assets/drone/drone_1.png')
        self.rect = self.image.get_rect()
        self.pos_init = [335,760]
        self.rect.x = self.pos_init[0]
        self.rect.y = self.pos_init[1]
        self.rot = pygame.transform.rotate(self.image, 0)
        self.pos_rot = np.transpose(np.array([self.rect.x-self.rot.get_rect().width/2, self.rect.y-self.rot.get_rect().height/2]))
        self.colisaoh = "none"
        self.colisaov = "none"


        #tempo
        self.t = [0.0,self.tau]

        #Forca peso
        self.P = self.m*9.8 #N

        #r
        #posicao
        self.pos = [self.rect.x-self.rot.get_rect().width/2, self.rect.y-self.rot.get_rect().height/2]
        #listas posiçoes
        self.r = np.array([[0.0,0.0],[0.0,0.0]])
        #comando de posicao
        self.rbarra = [0.0,100.0]

        #v
        #velocidade linear
        self.velocity = [0.0, 0.0]
        #lista velocidades
        self.dr = np.array([[0.0,0.0],[0.0, 0.0]])

        #phi
        #atitude
        self.angle = 0.0
        #lista atitude
        self.phi = [self.angle, self.angle]
        #comando de angulo
        self.phibarra = 0.0
        #angule da forca total
        self.alpha = 0.0

        #omega
        #velocidade angular
        self.omega = 0.0
        #lista velocidade angular
        self.dphi = [0.0, 0.0]

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
        #Forca total
        self.Ft = np.transpose(np.array([ 0.0, 0.0]))

        #Tc
        #torque de controle
        #self.Tc = self.l*self.kf*(self.w[0]**2-self.w[1]**2)
        #liste Tc
        #self.Tc = [self.l*self.kf*(self.w[0]**2-self.w[1]**2)]
        #Tc de comando
        self.Tcbarra = 0.0

        #lista de erros dos controladores
        self.erroCp = [0.0, 0.0]
        self.erroCa = [0.0, 0.0]
        self.erroCw = [0.0, 0.0]


    def atualizar_posicao_tela(self):
        #atualiza a posição e a rotação do robo na tela
        #atualizar angulo
        self.angle = self.phi[-1]
        self.omega = self.dphi[-1]
        self.rot = pygame.transform.rotate(self.image, self.angle)
        self.pos_rot[0] = self.rect.x-self.rot.get_rect().width/2 
        self.pos_rot[1] = self.rect.y-self.rot.get_rect().height/2

        #atualizar posicao
        self.pos[0] = self.pos_rot[0] + self.r[:,-1][0]/10
        self.pos[1] = self.pos_rot[1] + self.r[:,-1][1]/10
        #atualizar rect
        self.rect.y = self.pos[1]+self.rot.get_rect().height/2
        self.rect.x = self.pos[0]+self.rot.get_rect().width/2 


    def Cp(self):
        #Controlador da posicção: 
        #entrada r,v e saida Fcbarra, phibarra
        #parametros pid
        Kp = 2.0;
        Ki = 2.0;
        Kd = 10.0;

        #calculo do erro para Fcbarra
        erro = self.rbarra - self.r[:,-1]
        if self.rbarra[1]-self.r[:,-1][1] >= 0.0:
            self.erroCp += [math.sqrt(erro[0]**2 + erro[1]**2)]
        else:
            self.erroCp += [-math.sqrt(erro[0]**2 + erro[1]**2)]

        #correção do erro
        Cpp = Kp*self.erroCp[-1]
        Cpi = Ki*(self.erroCp[-1]+self.erroCp[-2])*self.tau/2
        Cpd = Kd*math.sqrt(self.dr[:,-1][0]**2+self.dr[:,-1][1]**2)
        
        self.Ft = Cpp + Cpi + Cpd
        #alpha
        self.alpha = math.atan2(self.rbarra[1]-self.pos[1], self.rbarra[0]-self.pos[0])

        if self.Ft*math.sin(self.alpha)<-self.P:
            self.Ft = -self.P

        print("Cp")
        print("r :", self.r[:,-1], "      v :", self.dr[:,-1])


        
        self.phibarra = math.atan2(self.Ft*math.sin(self.alpha),(self.Ft*math.cos(self.alpha)+self.P))
        if self.phibarra%math.pi*2 > math.pi /2 : 
            self.phibarra = math.pi - self.phibarra%math.pi
        elif self.phibarra%math.pi*2 < -math.pi/2:
            self.phibarra = -math.pi -self.phibarra%math.pi

        self.Fcbarra = self.Ft*math.sin(self.alpha)/math.sin(self.phibarra)
        print("fcbarra :", self.Fcbarra, "      phibarra :", self.phibarra)
        

       


    def Ca(self):
        #controlador do angulo
        #entrada phibarra, phi,omega e saida Tcbarra
        #parametros pid
        Kp = 0.5;
        Ki = 0.7;
        Kd = 10.0;

        #calculo do erro para Tcbarra
        self.erroCa += [self.phibarra-self.phi[-1]]
        print("Ca")
        print("phi :", self.phi[-1],"      self.dphi :", self.dphi[-1])
        #correção do erro
        Cpp = Kp*self.erroCa[-1]
        Cpi = Ki*(self.erroCa[-1]+self.erroCa[-2])*self.tau/2
        Cpd = Kd*self.dphi[-1]
        print("Tcbarra :", self.Tcbarra)
        self.Tcbarra = Cpp + Cpi + Cpd


    def Cw(self):
        #Controlador dos motores:
        #entrada Fcbarra,Tcbarra e saida wbarra
        F1 = (self.Fcbarra+self.Tcbarra/self.l)/2
        F2 = (self.Fcbarra-self.Tcbarra/self.l)/2
        self.wbarra[1] = math.sqrt(abs(F1))/self.kf
        self.wbarra[0] = math.sqrt(abs(F2))/self.kf
        if abs(self.wbarra[0])> self.wmax and abs(self.wbarra[1])>self.wmax:
            maxi = 0 
            for i in range(len(self.wbarra)):
                if(self.wbarra[i]>maxi):
                    maxi = self.wbarra[i]
            self.wbarra[0] = self.wbarra[0]*self.wmax/maxi
            self.wbarra[1] = self.wbarra[1]*self.wmax/maxi
        else:
            for wrotor in self.wbarra:
                if wrotor > self.wmax:
                    wrtor = self.wmax
                elif wrotor < -self.wmax:
                    wrotor = -self.wmax
        print("Cw")
        print("Fcbarra :", self.Fcbarra, "      Tcbarra :", self.Tcbarra)
        print("wbarra :", self.wbarra)
        

    def din_robo(self, y, t, wbarra, tempo):
        # Parametros da planta
         #parametros
        tau = tempo         #cste de tempo
        m = 0.250           #kg
        Iz = 2*10**(-4)     #kg.m2
        l = 0.1             #m
        wmax = 15000#*2*math.pi/60        #rpm
        kf = 1.744*10**(-8) #constante de força
        P = m*9.8           #N

        w0,w1, r0,r1, v0,v1, phi, omega = y
        w = [w0,w1]
        r = [r0,r1]
        v = [v0,v1]
        #forca de controle
        Fc = np.transpose(np.array([0,kf*(w[1]**2+w[0]**2)]))
        #torque de controle
        Tc = l*kf*(w[0]**2-w[1]**2)
        #Matriz de rotação
        Drb = np.array([[math.cos((phi)*0.0174533), -math.sin((phi)*0.0174533)],[math.sin((phi)*0.0174533), math.cos((phi)*0.0174533)]])

        # Dinamica do robo
        wp = [0.0, 0.0]
        wp[0] = (-w[0]+wbarra[0])/tau
        wp[1] = (-w[1]+wbarra[1])/tau
        
        for wrotor in wp:
            if wrotor > wmax:
                wrtor = wmax
            elif wrotor < -wmax:
                wrotor = -wmax
        
        rp = [0.0, 0.0]
        rp[0] = v[0]
        rp[1] = v[1]
        vp = [0.0, 0.0]
        vp = np.dot(Drb,Fc)/m
        vp[1] = vp[1] - P/m
        vp = vp.tolist()
        phip = omega
        dphip = Tc/Iz


        return [wp[0],wp[1], rp[0],rp[1], vp[0],vp[1], phip, dphip]


    def atualizar_dinamica(self, tempo):
        #posicao e angulo na tela
        self.atualizar_posicao_tela()

        #FC e phibarra
        self.Cp()
        #Tc
        self.Ca()
        #wbarras
        self.Cw()

        #dinamica robo
        # Evoluindo a din. da planta
        #condições iniciais
        w0 = self.w
        w0 = w0.tolist()
        r0 = self.r[:,-1]
        r0[1] = r0[1]
        r0 = r0.tolist()
        v0 = self.dr[:,-1]
        v0[1] = v0[1]
        v0 = v0.tolist()
        phi0 = self.phi[-1]
        dphi0 = self.dphi[-1]
        x0 = [w0[0], w0[1], r0[0], r0[1], v0[0], v0[1], phi0, dphi0]   # condicao inicial

        #TENTAR COM RK4
        #calculo eq. dif.
        sol = odeint(self.din_robo, x0, [0.0, self.tau], args=(self.wbarra,tempo,))
        
        #solucao eq. dif.
        self.w[0] = sol[:,0][-1]
        self.w[1] = sol[:,1][-1]
        r1 = np.transpose(np.array([[sol[:,2][-1],sol[:,3][-1]]]))
        self.r = np.concatenate((self.r,r1), axis=1)
        dr1 = np.transpose(np.array([[sol[:,4][-1],sol[:,5][-1]]]))
        self.dr = np.concatenate((self.dr,dr1), axis=1)
        self.phi += [sol[:,6][-1]]
        self.dphi += [sol[:,7][-1]]
        self.t += [self.t[-1] + tempo]

        self.rebote()
        
    def rebote(self):
        if self.colisaoh == "right" and self.dr[:,-1][0] > 0:
            self.dr[:,-1][0] = -self.dr[:,-1][0]*80/100
            self.dr[:,-2][0] = -self.dr[:,-2][0]*80/100
            self.velocity[0] = -self.velocity[0]
            self.colisaoh = "none"
        elif self.colisaoh == "left" and self.dr[:,-1][0] < 0:
            self.dr[:,-1][0] = -self.dr[:,-1][0]*80/100
            self.dr[:,-2][0] = -self.dr[:,-2][0]*80/100
            self.velocity[0] = -self.velocity[0]
            self.colisaoh = "none"
        if self.colisaov == "up" and self.dr[:,-1][1] < 0:
            self.dr[:,-1][1] = -self.dr[:,-1][1]*80/100
            self.dr[:,-2][1] = -self.dr[:,-2][1]*80/100
            self.velocity[1] = -self.velocity[1]
            self.colisaov = "none"
        elif self.colisaov == "down" and self.dr[:,-1][1] > 0:
            self.dr[:,-1][1] = -self.dr[:,-1][1]
            self.dr[:,-2][1] = -self.dr[:,-2][1]*80/100
            self.velocity[1] = 20
            self.colisaov = "none"
        



















if __name__ == "__main__":
    player = Player()
    player.atualizar_dinamica()
