#python 3.8.10 
import pygame
from game import Game
from player import Player
from motor import Motor
pygame.init()


#criação janela
pygame.display.set_caption("Simulação drone")
screen = pygame.display.set_mode((1620,960))

#criação background
background = pygame.image.load('assets/background.png')

#caregar o jogo
game = Game()

running = True


while running:
    #mostrar o background
    screen.blit(background, (0, 0))

    #atualizar posição jogador
    screen.blit(game.player.image, game.player.rect)

    #atualizar a tela
    pygame.display.flip()


    for event in pygame.event.get():
        #fecha o jogo se o jogador fecha a janela
        if event.type == pygame.QUIT:
            running = False
            pygame.quit()
        elif event.type == pygame.KEYDOWN:
            game.pressed[event.key]=True
        elif event.type == pygame.KEYUP:
            game.pressed[event.key]=False

    if game.pressed.get(pygame.K_RIGHT):
        game.player.move_right()
    if game.pressed.get(pygame.K_LEFT):
        game.player.move_left()
    if game.pressed.get(pygame.K_UP):
        game.player.move_up()
    if game.pressed.get(pygame.K_DOWN):
        game.player.move_down()