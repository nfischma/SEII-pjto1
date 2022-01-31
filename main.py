#python 3.8.10 
import pygame
pygame.init()

#criação janela
pygame.display.set_caption("Simulação drone")
screen = pygame.display.set_mode((1620,960))

#criação background
background = pygame.image.load('assets/background.png')

running = True


while running:
    #mostrar o background
    screen.blit(background, (0, 0))

    #atualizar a tela
    pygame.display.flip()


    for event in pygame.event.get():
        #fecha o jogo se o jogador fecha a janela
        if event.type == pygame.QUIT:
            running = False
            pygame.quit()
        