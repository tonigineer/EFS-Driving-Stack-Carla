img_bg = 'bg.jpg'                                               
img_cursor = 'cursor.png'
WHITE = (255, 255, 255)
(width, height) = (400, 300)

import pygame                                                   
from pygame.locals import *
from sys import exit

pygame.init()                                                   

FPS = 60                                                        
fpsClock = pygame.time.Clock()                                 
window = pygame.display.set_mode((width, height), 0, 32)        
pygame.display.set_caption("Hello, World!")                    
pygame.display.flip()

# background = pygame.image.load(img_bg).convert()                
# mouse_cursor = pygame.image.load(img_cursor).convert_alpha()  

running = True
while running:                                                  
    for event in pygame.event.get():                            
        if event.type == pygame.QUIT:
            running = False

    window.fill(WHITE)                                         
    # window.blit(background, (0,0))                              

    # x, y = pygame.mouse.get_pos()                               
    # x-= mouse_cursor.get_width() / 2                            
    # y-= mouse_cursor.get_height() / 2
    # window.blit(mouse_cursor, (int(x), int(y)))                 
            
    print('asd')

    pygame.display.update()                                     
    fpsClock.tick(FPS)