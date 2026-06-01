import pygame
from pygame.locals import *
import random
import math
import numpy as np
import particle_physics

from particle_physics import particle
from particle_physics import step

pygame.init()

width = 1280
height = 480

object_width = 4
object_height = 4

radius = 2

#direction 
forward = 1
backward = -1

x_position = random.randint(0,1280)
y_position = random.randint(0,480)

#colours
black = (0,0,0)
cyan = (0,255,255)
bg = (204, 102, 0)
red = (255, 0, 0)
white = (255, 255, 255)

screen = pygame.display.set_mode((width, height))

running = True

clock = pygame.time.Clock()
fps = 60

font = pygame.font.SysFont('Constantia', 15)

clicked = False

class button():

    width = 50
    height = 30

    #colours
    button_col = (255, 0, 0)
    hover_col = (75, 225, 255)
    click_col = (50, 150, 255)
    text_col = black

    def __init__(self, x, y, text):
        self.x = x
        self.y = y
        self.text = text
    
    def draw_button(self):
        global clicked
        action = False

        pos = pygame.mouse.get_pos()

        button_rect = pygame.Rect(self.x, self.y, self.width, self.height)

        #check mouse hover
        if button_rect.collidepoint(pos): #automatically compares mouse pos with rect pos
            if pygame.mouse.get_pressed()[0] == 1:
                clicked = True
                pygame.draw.rect(screen, self.click_col, button_rect)
            elif pygame.mouse.get_pressed()[0] == 0 and clicked == True:
                clicked = False
                action = True
            else:
                pygame.draw.rect(screen, self.hover_col, button_rect)
        else:
            pygame.draw.rect(screen, self.button_col, button_rect)
        
        text_img = font.render(self.text, True, self.text_col)
        text_len = text_img.get_width()
        screen.blit(text_img, 
                    (
                        self.x + int(self.width / 2) - int(text_len / 2),
                          self.y + 5
                    )
                )

        return action

def SpawnParticle():
    mass = 1 # keeping constant for now

    limit = 10 # a limit to number of attempts to spawn particle before which it is concluded that no space is left

    for _ in range(limit):
        x = random.uniform(0, width)
        y = random.uniform(0, height)
        for p in particle_physics.particles:
            if math.sqrt(math.pow(x - p.x, 2) + math.pow(y - p.y, 2)) <= 2*radius:
                break
        else:
            angle = random.randint(0,360)
            speed = random.randint(1, 3) #Hypotenuse

            #Trigonometry helps us get vx and vy components
            vx = speed * (math.cos(math.radians(angle)))
            vy = speed * (math.sin(math.radians(angle)))
            new_particle = particle(mass,x, y, vx, vy, radius)
            particle_physics.particles.append(new_particle)
            return
    print("no more space")

spawn = button(1220, 10, 'spawn')

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    clock.tick(fps)

    screen.fill(black)#clearing screen
    if spawn.draw_button():          # spawn if button clicked
        SpawnParticle()

    num_of_particles = len(particle_physics.particles)

    step(width,height)
    
    for i in range(num_of_particles):
        pygame.draw.circle(screen, cyan, (int(particle_physics.particles[i].x), int(particle_physics.particles[i].y)), particle_physics.particles[i].radius)
    
    pygame.display.flip()#updating whole screen each frame/iteration
        
pygame.quit()
