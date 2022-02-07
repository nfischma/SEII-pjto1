#python 3.8.10 
import pygame
from game import Game
from player import Player
from motor import Motor
import math
from numpy import linalg as la
import time
import numpy as np

pygame.init()


#criação janela
pygame.display.set_caption("Simulação drone")
screen = pygame.display.set_mode((1620,960))

#criação background
background = pygame.image.load('assets/background.png')

#caregar o jogo
game = Game()

running = True

#clock
clock = time.time()

while running:
    #mostrar o background
    screen.blit(background, (0, 0))

    #atualizar posição jogador
    #screen.blit(game.player.image, game.player.rect)
    screen.blit(game.player.rot,game.player.pos)
    


    for event in pygame.event.get():
        #fecha o jogo se o jogador fecha a janela
        if event.type == pygame.QUIT:
            running = False
            pygame.quit()
        elif event.type == pygame.KEYDOWN:
            game.pressed[event.key]=True
        elif event.type == pygame.KEYUP:
            game.pressed[event.key]=False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            game.position = [a-b for a, b in zip(pygame.mouse.get_pos(),(65,40))]
            game.angle = math.atan2(game.position[1]-game.player.rect.y, game.position[0]-game.player.rect.x)
            #playerrot = pygame.transform.rotate(game.player.image, 360-angle*57.29)

    if game.pressed.get(pygame.K_RIGHT):
        game.player.motor_right_on()
    elif game.pressed.get(pygame.K_LEFT):
        game.player.motor_left_on()
    elif game.pressed.get(pygame.K_UP):
        game.player.motor_lr_on()
    else:
        game.player.motors_off()

    if time.time()-clock > game.player.tau:
        game.player.atualizar_dina_cine() #movimento estatico, precisa ser dinamico
        clock = time.time()
        
    #atualizar a tela
    pygame.display.flip()