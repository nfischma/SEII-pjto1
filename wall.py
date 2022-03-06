import pygame
import math
import numpy as np
from numpy import linalg as la
import matplotlib.pyplot as plt
from control.matlab import *

from scipy.integrate import odeint

#classe muros
class Wall(pygame.sprite.Sprite):
    def __init__(self, lado, point1, point2):
        super().__init__()

        #tipo de muro
        self.lado = lado

        if self.lado == "up" or self.lado=="down":
            self.tipo = "vertical"
        elif self.lado == "right" or self.lado=="left":
            self.tipo = "horizontal"

        #limites: p1 em cima na esquerda e p2 embaixo da direita
        self.p1 = point1
        self.p2 = point2
        self.xmin = self.p1[0]
        self.xmax = self.p2[0]
        self.ymin = self.p1[1]
        self.ymax = self.p2[1]

    def detecao_colisaoh(self,player_pos):
        if player_pos[1] > self.ymin and player_pos[1] < self.ymax and player_pos[0] >  self.xmin and player_pos[0] < self.xmax :
            return self.lado
        else:
            return "none"
        
    def detecao_colisaov(self,player_pos):
        print("x : ",player_pos[1], self.ymin, self.ymax)
        print("y : ",player_pos[0], self.xmin, self.xmax)
        if  player_pos[1] > self.ymin and player_pos[1] < self.ymax and player_pos[0] >  self.xmin and player_pos[0] < self.xmax:
            return self.lado
            print("self.lado, self.lado, self.lado")
        else:
            return "none"
    
    