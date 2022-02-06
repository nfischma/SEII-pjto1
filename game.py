import pygame
from player import Player
from motor import Motor
from numpy import linalg as la
import numpy as np

class Game:

    def __init__(self):
        #generar o drone
        self.player = Player()
        self.pressed = {
        }
        self.position = [335, 760]
        self.angle = 0

