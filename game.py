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

        self.wall_down = Wall("down",[0,800],[1620,960])
        self.wall_left = Wall("left",[0,0],[200,960])
        self.wall_right = Wall("right",[1400,0],[1620,960])
        self.wall_up = Wall("up",[0,0], [1620,200])

        self.walls = [self.wall_up, self.wall_left, self.wall_right, self.wall_down]

