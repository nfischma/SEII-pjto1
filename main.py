#python 3.8.10 
import pygame
from game import Game
from player import Player
import math
from numpy import linalg as la
import time
import numpy as np
import matplotlib.pyplot as plt
from control.matlab import *
from wall import Wall

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
begin = time.time()
fim_calculo = time.time()
tempo_calculo_din = 0.05



while running:
    


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
            game.player.rbarra[0] = game.position[0]-game.player.pos_init[0]
            game.player.rbarra[1] = -game.position[1]+game.player.pos_init[1]


    if time.time()-clock >= game.player.tau:
        
        #Detetar colisao
        for wall in game.walls:
            print("tipo de muro :", wall.tipo)
            if wall.tipo == "horizontal":
                game.player.colisaoh = wall.detecao_colisaoh(game.player.pos)
            elif wall.tipo == "vertical":
                game.player.colisaov = wall.detecao_colisaov(game.player.pos)
        
        print("")
        print("")
        #mostrar o background
        screen.blit(background, (0, 0))

        #atualizar posição jogador
        screen.blit(game.player.rot,game.player.pos)
    
        
        clock = time.time()
        comeco_calculo = time.time()
        if(fim_calculo==0):
            fim_calculo = comeco_calculo+0.05
        game.player.atualizar_dinamica(tempo_calculo_din) #movimento estatico, precisa ser dinamico
        fim_calculo = time.time()
        tempo_calculo_din = fim_calculo - comeco_calculo




        
        
    #atualizar a tela
    pygame.display.flip()