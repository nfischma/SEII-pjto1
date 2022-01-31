import pygame
from player import Player
from motor import Motor
class Game:

    def __init__(self):
        #generar o drone
        self.player = Player()
        self.pressed = {
        }

