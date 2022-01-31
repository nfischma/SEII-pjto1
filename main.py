#python 3.8.10 
import pygame
pygame.init()

#criação janela
pygame.display.set_caption("Simulação drone")
pygame.display.set_mode((1620,960))

running = True


while running:

    for event in pygame.event.get():
        #fecha o jogo se o jogador fecha a janela
        if event.type == pygame.QUIT:
            running = False
            pygame.quit()
        