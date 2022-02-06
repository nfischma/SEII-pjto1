import pygame
from motor import Motor
import math
#classe jogador
class Player(pygame.sprite.Sprite):

    def __init__(self):
        super().__init__()
        self.weight = 1 #kg
        self.image = pygame.image.load('assets/drone/drone_1.png')
        self.rect = self.image.get_rect()
        self.rect.x = 335
        self.rect.y = 760
        self.motor1 = Motor()
        self.motor2 = Motor()
        self.velocity = 1
        self.rot = pygame.transform.rotate(self.image, 0)
        self.angle = 0
        self.pos = [ self.rect.x-self.rot.get_rect().width/2, self.rect.y-self.rot.get_rect().height/2]
    
    def move_right(self):
        self.pos[0] += self.velocity
        self.rect.x = self.pos[0]+self.rot.get_rect().width/2
    
    def move_left(self):
        self.pos[0] -= self.velocity
        self.rect.x = self.pos[0]+self.rot.get_rect().width/2

    def move_up(self):
        self.pos[1] -= self.velocity
        self.rect.y = self.pos[1]+self.rot.get_rect().height/2
    
    def move_down(self):
        self.pos[1] += 0.3
        self.rect.y = self.pos[1]+self.rot.get_rect().height/2

    def rotate_right(self):
        self.angle -=1
        self.rot = pygame.transform.rotate(self.image, self.angle)
        self.pos[0] = self.rect.x-self.rot.get_rect().width/2
        self.pos[1] = self.rect.y-self.rot.get_rect().height/2
        

    def rotate_left(self):
        self.angle += 1
        self.rot = pygame.transform.rotate(self.image, self.angle)
        self.pos[0] = self.rect.x-self.rot.get_rect().width/2
        self.pos[1] = self.rect.y-self.rot.get_rect().height/2
    
    def move_forward(self):
        self.pos[0] -= math.cos((self.angle-90)*0.0174533)*self.velocity
        self.pos[1] += math.sin((self.angle-90)*0.0174533)*self.velocity
        self.rect.x = self.pos[0]+self.rot.get_rect().width/2
        self.rect.y = self.pos[1]+self.rot.get_rect().height/2

    def move_backward(self):
        pass
    