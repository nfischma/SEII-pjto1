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
        self.rbarra = [10.0,10.0]
        self.lrbarra = np.transpose(np.array([self.rbarra]))
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
        self.phibarra = [0.0,0.0]
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
        self.pos[0] = self.pos_init[0] + self.r[:,-1][0]
        self.pos[1] = self.pos_init[1] - self.r[:,-1][1]
        #atualizar rect
        self.rect.y = self.pos[1]+self.rot.get_rect().height/2
        self.rect.x = self.pos[0]+self.rot.get_rect().width/2 


    # def Cp(self):
    #     #Controlador da posicção: 
    #     #entrada r,v e saida Fcbarra, phibarra
    #     #parametros pid
    #     Kp = 1.0;
    #     Ki = 0.5;
    #     Kd = 1.0;

    #     #calculo do erro para Fcbarra
    #     erro = self.rbarra - self.r[:,-1]
    #     if self.rbarra[1]-self.r[:,-1][1] >= 0.0:
    #         self.erroCp += [math.sqrt(erro[0]**2 + erro[1]**2)]
    #     else:
    #         self.erroCp += [-math.sqrt(erro[0]**2 + erro[1]**2)]

    #     #correção do erro
    #     Cpp = Kp*self.erroCp[-1]
    #     Cpi = Ki*(self.erroCp[-1]+self.erroCp[-2])*self.tau/2
    #     Cpd = Kd*math.sqrt(self.dr[:,-1][0]**2+self.dr[:,-1][1]**2)
        
    #     self.Ft = Cpp + Cpi + Cpd
    #     #alpha
    #     self.alpha = math.atan2(self.rbarra[1]-self.pos[1], self.rbarra[0]-self.pos[0])

    #     if self.Ft*math.sin(self.alpha)<-self.P:
    #         self.Ft = -self.P

    #     print("Cp")
    #     print("r :", self.r[:,-1], "      v :", self.dr[:,-1])


        
    #     self.phibarra += [math.atan2(self.Ft*math.sin(self.alpha),(self.Ft*math.cos(self.alpha)+self.P))]
    #     if self.phibarra[-1]%math.pi*2 > math.pi /2 : 
    #         self.phibarra[-1] = math.pi - self.phibarra[-1]%math.pi
    #     elif self.phibarra[-1]%math.pi*2 < -math.pi/2:
    #         self.phibarra[-1] = -math.pi -self.phibarra[-1]%math.pi

    #     self.Fcbarra = self.Ft*math.sin(self.alpha)/math.sin(self.phibarra[-1])
    #     print("fcbarra :", self.Fcbarra, "      phibarra :", self.phibarra)
        

       


    # def Ca(self):
    #     #controlador do angulo
    #     #entrada phibarra, phi,omega e saida Tcbarra
    #     #parametros pid
    #     Kp = 1.0;
    #     Ki = 0.0;
    #     Kd = 0.0

    #     #calculo do erro para Tcbarra
    #     self.erroCa += [self.phibarra[-1]-self.phi[-1]]
    #     print("Ca")
    #     print("phi :", self.phi[-1],"      self.dphi :", self.dphi[-1])
        
    #     #correção do erro
    #     Cpp = Kp*self.erroCa[-1]
    #     Cpi = Ki*(self.erroCa[-1]+self.erroCa[-2])*self.tau/2
    #     Cpd = Kd*self.dphi[-1]
    #     print("Tcbarra :", self.Tcbarra)
    #     self.Tcbarra = Cpp + Cpi + Cpd


    # def Cw(self):
    #     #Controlador dos motores:
    #     #entrada Fcbarra,Tcbarra e saida wbarra
    #     F1 = (self.Fcbarra+self.Tcbarra/self.l)/2
    #     F2 = (self.Fcbarra-self.Tcbarra/self.l)/2
    #     self.wbarra[1] = math.sqrt(abs(F1))/self.kf
    #     self.wbarra[0] = math.sqrt(abs(F2))/self.kf
    #     if abs(self.wbarra[0])> self.wmax and abs(self.wbarra[1])>self.wmax:
    #         maxi = 0 
    #         for i in range(len(self.wbarra)):
    #             if(self.wbarra[i]>maxi):
    #                 maxi = self.wbarra[i]
    #         self.wbarra[0] = self.wbarra[0]*self.wmax/maxi
    #         self.wbarra[1] = self.wbarra[1]*self.wmax/maxi
    #     else:
    #         for wrotor in self.wbarra:
    #             if wrotor > self.wmax:
    #                 wrtor = self.wmax
    #             elif wrotor < -self.wmax:
    #                 wrotor = -self.wmax
    #     print("Cw")
    #     print("Fcbarra :", self.Fcbarra, "      Tcbarra :", self.Tcbarra)
    #     print("wbarra :", self.wbarra)
        

    # def din_robo(self, y, t, wbarra, tempo):
    #     # Parametros da planta
    #      #parametros
    #     tau = tempo         #cste de tempo
    #     m = 0.250           #kg
    #     Iz = 2*10**(-4)     #kg.m2
    #     l = 0.1             #m
    #     wmax = 15000#*2*math.pi/60        #rpm
    #     kf = 1.744*10**(-8) #constante de força
    #     P = m*9.8           #N

    #     w0,w1, r0,r1, v0,v1, phi, omega = y
    #     w = [w0,w1]
    #     r = [r0,r1]
    #     v = [v0,v1]
    #     #forca de controle
    #     Fc = np.transpose(np.array([0,kf*(w[1]**2+w[0]**2)]))
    #     #torque de controle
    #     Tc = l*kf*(w[0]**2-w[1]**2)
    #     #Matriz de rotação
    #     Drb = np.array([[math.cos((phi)*0.0174533), -math.sin((phi)*0.0174533)],[math.sin((phi)*0.0174533), math.cos((phi)*0.0174533)]])

    #     # Dinamica do robo
    #     wp = [0.0, 0.0]
    #     wp[0] = (-w[0]+wbarra[0])/tau
    #     wp[1] = (-w[1]+wbarra[1])/tau
        
    #     for wrotor in wp:
    #         if wrotor > wmax:
    #             wrtor = wmax
    #         elif wrotor < -wmax:
    #             wrotor = -wmax
        
    #     rp = [0.0, 0.0]
    #     rp[0] = v[0]
    #     rp[1] = v[1]
    #     vp = [0.0, 0.0]
    #     vp = np.dot(Drb,Fc)/m
    #     vp[1] = vp[1] - P/m
    #     vp = vp.tolist()
    #     phip = omega
    #     dphip = Tc/Iz
    #     print("test :", [wp[0],wp[1], rp[0],rp[1], vp[0],vp[1], phip, dphip])
    #     return [wp[0],wp[1], rp[0],rp[1], vp[0],vp[1], phip, dphip]

    def rk4(self, tk, h, xk, uk):
        k1 = np.array(self.x_dot(tk, xk, uk))
        k2 = np.array(self.x_dot(tk+h/2.0, xk+h*k1/2.0, uk))
        k3 = np.array(self.x_dot(tk+h/2.0, xk+h*k2/2.0, uk))
        k4 = np.array(self.x_dot(tk+h, xk+h*k3, uk))
        xkp1 = xk + (h / 6.) * (k1 + 2*k2 + 2*k3 + k4)
        return xkp1


    def x_dot(self, t, x, w_):
        # State vector
        # x = [ w r_xy v_xy phi omega]' \in R^8
        #print('x: ', x)
        #print('w_: ', w_)
        ## Parâmetros
        w_max = 15000. # velocidade máxima do motor
        m = 0.25 # massa
        g = 9.81 # aceleração da gravidade
        l = 0.1 # tamanho
        kf = 1.744e-08 # constante de força
        Iz = 2e-4 # momento de inércia
        tal = 0.005
        Fg = np.array([[0],\
            [-m*g]])
        ## Estados atuais
        w = x[0:2]
        r = x[2:4]
        v = x[4:6]
        phi = x[6]
        ome = x[7]
        ## Variáveis auxiliares
        # forças
        f1 = kf * w[0]**2
        1
        f2 = kf * w[1]**2
        # Torque
        Tc = l * (f1 - f2)
        # Força de controle
        Fc_B = np.array( [[0], \
            [(f1 + f2)]])
        # Matriz de atitude
        D_RB = np.array([ [ np.cos(phi), -np.sin(phi)], \
            [ np.sin(phi), np.cos(phi)]])
        ## Derivadas
        w_dot = (-w + w_)/tal
        r_dot = v
        v_dot = (1/m)*(D_RB @ Fc_B + Fg)
        v_dot = v_dot.reshape(2,)
        phi_dot = np.array([ome])
        ome_dot = np.array([Tc/Iz])
        xkp1 = np.concatenate([ w_dot, \
            r_dot, \
            v_dot, \
            phi_dot,\
            ome_dot ])
        return xkp1
    

    def atualizar_dinamica(self, tempo):
        self.atualizar_posicao_tela()
        # PARÂMETROS DE SIMULAÇÃO
        h = 2.5e-3 # passo da simulação de tempo continuo
        Ts = 10e-3 # intervalo de atuação do controlador
        fTh = Ts/h
        maxT = tempo
        tc = np.arange(0,maxT,h) # k
        td = np.arange(0,maxT,Ts) # j
        tam = len(tc)
        j = 0
        # Vetor de estados
        # State vector
        # x = [ w r_xy v_xy phi omega]' \in R^8
        x = np.zeros([8,tam])
        #x[:,0] = np.array([0.,1.,2,3,4,5,6,7,])
        w0 = self.w
        w0 = w0.tolist()
        r0 = self.r[:,-1]
        r0[1] = r0[1]
        r0 = r0.tolist()
        v0 = self.dr[:,-1]
        v0[1] = v0[1]
        v0 = v0.tolist()
        phi0 = self.phi[-1]*math.pi/180
        dphi0 = self.dphi[-1]*math.pi/180
        x0 = [w0[0], w0[1], r0[0], r0[1], v0[0], v0[1], phi0, dphi0]   # condicao inicial

        x[:,0] = np.array(x0)
        # Parâmetros do sistema de controle
        # Vetor de controle relativo à rotação
        w_ = np.zeros([2,len(td)]) # comando de controle
        # Vetor dos erros de posição
        eP_ = np.zeros([2,len(td)])
        ePm1 = 0 # erro posição k-1 (passo anterior)
        eVm1 = 0 # erro atitude k-1 (passo anterior)
        # Constanstes do modelo
        m = 0.25 # massa
        g = 9.81 # aceleração da gravidade
        l = 0.1 # tamanho
        kf = 1.744e-08 # constante de força
        Iz = 2e-4 # momento de inércia
        tal = 0.05
        Fe = np.array([-m*g])
        # Restrições do controle
        phi_max = 15*np.pi/180. # ângulo máximo
        w_max = 15000
        Fc_max = kf*w_max**2 # Força de controle máximo
        Tc_max = l*kf*w_max**2
        #Fc_min = 0.1 * Fc_max
        #Fc_max = 0.9 * Fc_max
        # Waypoints
        r_ = np.array([self.rbarra])
        r_ID = 0
        r_IDN = 0

        ### Execução da simulação
        for k in range(tam-1):
            # Sistema de controle
            if (k % fTh) == 0:
                # Extrai os dados do vetor
                r_k = x[2:4,k]
                v_k = x[4:6,k]
                phi_k = x[6,k]
                ome_k = x[7,k]
                # Comando de posição
                v_ = np.array([0,0])
                #####################
                # Controle de Posição
                kpP = np.array([0.075])
                kdP = np.array([0.25])
                eP = r_[:,r_ID] - r_k
                eV = v_ - v_k
                eP_[:,j] = eP
                #print(eP, eV)
                # Definição do próximo waypoint
                if np.linalg.norm(eP) < .1 and r_ID < (r_IDN):
                     r_ID += 1
                #     print("Bucar Waypoint: ", r_ID)
                Fx = kpP * eP[0] + kdP * eV[0]
                Fy = kpP * eP[1] + kdP * eV[1] - Fe
                Fy = np.maximum(0.2*Fc_max, np.minimum(Fy, 0.8*Fc_max))
                #####################
                # Controle de Atitude
                phi_ = np.arctan2(-Fx, Fy)
                if np.abs(phi_) > phi_max:
                    #print(phi_*180/np.pi)
                    signal = phi_/np.absolute(phi_)
                    phi_ = signal * phi_max
                    # Limitando o ângulo
                    Fx = Fy * np.tan(phi_)
                Fxy = np.array([Fx, Fy])
                Fc = np.linalg.norm(Fxy)
                f12 = np.array([Fc/2.0, Fc/2.0])
                # Constantes Kp e Kd
                kpA = np.array([.75])
                kdA = np.array([0.02])
                ePhi = phi_ - phi_k
                eOme = 0 - ome_k
                Tc = kpA * ePhi + kdA * eOme
                Tc = np.maximum(-0.4*Tc_max, np.minimum(Tc, 0.4*Tc_max))
                # Delta de forças
                df12 = np.absolute(Tc)/2.0
                # Forças f1 e f2 final f12' = f12 + deltf12
                if (Tc >= 0.0):
                    f12[0] = f12[0] + df12
                    f12[1] = f12[1] - df12
                else:
                    f12[0] = f12[0] - df12
                    f12[1] = f12[1] + df12
                # Comando de rpm dos motores
                w1_ = np.sqrt(f12[0]/(kf))
                w2_ = np.sqrt(f12[1]/(kf))
                #
                # Limitando o comando do motor entre 0 - 15000 rpm
                w1 = np.maximum(0., np.minimum(w1_, w_max))
                w2 = np.maximum(0., np.minimum(w2_, w_max))
                # Determinação do comando de entrada
                w_[:,j] = np.array([w1, w2])
                j = j+1
            else:
                phi_ = self.phibarra[-1]
            # Simulação um passo a frente
            #print("xk",x[:,k])
            x[:,k+1] = self.rk4(tc[k], h, x[:,k], w_[:,j-1])
            x[0,k+1] = np.maximum(0., np.minimum(x[0,k+1], w_max))
            x[1,k+1] = np.maximum(0., np.minimum(x[1,k+1], w_max))
            #print("xkp1", x[:,k+1])
            # Processaento de variáveis intermediárias
            # obtem a força aplicada por cada rotor
            f = np.zeros([3, tam])
            for k in range(tam):
                w = x[0:2,k]
                f[0:2,k] = np.array([kf*w[0]**2, kf*w[1]**2])
                f[2,k] = f[0,k] + f[1,k] # Fc total em B
            #print(np.max(x[:,0]), np.min(x[:,1]))
            #print(np.max(f[:,0]), np.min(f[:,1]), np.min(f[:,2]))
        
        
        if tam <=2:
            phi_ = self.phibarra[-1]
        for i in range(1,len(tc)):
            self.phibarra += [phi_] 
        self.w[0] = x[0,:][-1]
        self.w[1] = x[1,:][-1]
        self.r = np.concatenate((self.r, np.array([x[2,:][1::],x[3,:][1::]])),axis=1)
        self.dr = np.concatenate((self.dr, np.array([x[4,:][1::],x[5,:][1::]])),axis=1)
        self.phi = np.concatenate((self.phi, np.array(x[6,:][1::])*180/math.pi))
        self.dphi = np.concatenate((self.dphi, np.array(x[7,:][1::])*180/math.pi))
        for i in range(1,len(tc)):
            tc[i] = self.t[-1]+tc[i]
            self.lrbarra = np.concatenate((self.lrbarra,np.transpose(np.array([self.rbarra]))),axis=1)
        self.t = np.concatenate((self.t,tc[1::]))

        self.rebote()

    def rebote(self):
        if self.colisaoh == "right" and self.dr[:,-1][0] > 0:
            self.dr[:,-1][0] = -self.dr[:,-1][0]
            self.dr[:,-2][0] = -self.dr[:,-2][0]
            self.velocity[0] = -self.velocity[0]
            self.colisaoh = "none"
        elif self.colisaoh == "left" and self.dr[:,-1][0] < 0:
            self.dr[:,-1][0] = -self.dr[:,-1][0]
            self.dr[:,-2][0] = -self.dr[:,-2][0]
            self.velocity[0] = -self.velocity[0]
            self.colisaoh = "none"
        if self.colisaov == "up" and self.dr[:,-1][1] < 0:
            self.dr[:,-1][1] = -self.dr[:,-1][1]
            self.dr[:,-2][1] = -self.dr[:,-2][1]
            self.velocity[1] = -self.velocity[1]
            self.colisaov = "none"
        elif self.colisaov == "down" and self.dr[:,-1][1] > 0:
            self.dr[:,-1][1] = -self.dr[:,-1][1]*80/100
            self.dr[:,-2][1] = -self.dr[:,-2][1]*80/100
            self.velocity[1] = -self.velocity[1]*80/100
            self.colisaov = "none"
        




    # def atualizar_dinamica_2(self, tempo):
    #         #posicao e angulo na tela
    #         self.atualizar_posicao_tela()

    #         #FC e phibarra
    #         self.Cp()
    #         #Tc
    #         self.Ca()
    #         #wbarras
    #         self.Cw()
            

    #         #dinamica robo
    #         # Evoluindo a din. da planta
    #         #condições iniciais
    #         w0 = self.w
    #         w0 = w0.tolist()
    #         r0 = self.r[:,-1]
    #         r0[1] = r0[1]
    #         r0 = r0.tolist()
    #         v0 = self.dr[:,-1]
    #         v0[1] = v0[1]
    #         v0 = v0.tolist()
    #         phi0 = self.phi[-1]
    #         dphi0 = self.dphi[-1]
    #         x0 = [w0[0], w0[1], r0[0], r0[1], v0[0], v0[1], phi0, dphi0]   # condicao inicial

    #         #TENTAR COM RK4
    #         #calculo eq. dif.
    #         sol = self.rk4(self.din_robo, x0, [self.t[-1], self.t[-1]+tempo], args = (self.wbarra, tempo))
    #         #sol = odeint(self.din_robo, x0, [0.0, tempo], args=(self.wbarra,tempo,))
    #         print("[w0[0], w0[1], r0[0], r0[1], v0[0], v0[1], phi0, dphi0] ")
    #         print(x0)
    #         print(sol[:,0][-1], sol[:,1][-1], sol[:,2][-1], sol[:,3][-1], sol[:,4][-1], sol[:,5][-1], sol[:,6][-1], sol[:,7][-1])
    #         #solucao eq. dif.
    #         self.w[0] = sol[:,0][-1]
    #         self.w[1] = sol[:,1][-1]
    #         r1 = np.transpose(np.array([[sol[:,2][-1],sol[:,3][-1]]]))
    #         self.r = np.concatenate((self.r,r1), axis=1)
    #         dr1 = np.transpose(np.array([[sol[:,4][-1],sol[:,5][-1]]]))
    #         self.dr = np.concatenate((self.dr,dr1), axis=1)
    #         self.phi += [sol[:,6][-1]]
    #         self.dphi += [sol[:,7][-1]]
    #         self.t += [self.t[-1] + tempo]

    #         self.rebote()














if __name__ == "__main__":
    player = Player()
    player.atualizar_dinamica(0.05)
