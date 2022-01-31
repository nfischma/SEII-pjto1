import pygame
from motor import Motor
#classe jogador
class Player(pygame.sprite.Sprite):

    def __init__(self):
        super().__init__()
        self.weight = 1 #kg
        self.image = pygame.image.load('assets/drone/drone_1.png')
        self.rect = self.image.get_rect()
        self.rect.x = 750
        self.rect.y = 500
        self.motor1 = Motor()
        self.motor2 = Motor()
        self.velocity = 5
    
    def move_right(self):
        self.rect.x += self.velocity
    
    def move_left(self):
        self.rect.x -= self.velocity

    def move_up(self):
        self.rect.y -= self.velocity
    
    def move_down(self):
        self.rect.y += self.velocity

    