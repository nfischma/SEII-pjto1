import pygame
from player import Player
from numpy import linalg as la
import numpy as np
import matplotlib.pyplot as plt
from control.matlab import *
from wall import Wall
class Game:

    def __init__(self):
        #generar o drone
        self.player = Player()
        self.pressed = {
        }
        self.position = [335, 760]
        self.player.rbarra[0] = self.position[0]
        self.player.rbarra[1] = self.position[1]

        self.wall1 = Wall("down",[0,800],[1620,960])

        self.walls = [self.wall1]

