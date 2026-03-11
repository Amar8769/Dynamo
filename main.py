import pygame
from pygame.locals import *

import physics
from physics import circle

import math

class Camera:
    def __init__(self, screen_height):
        self.screen_height = screen_height

    def world_to_screen(self, x, y):
        return x, self.screen_height - y

    def screen_to_world(self, x, y):
        return x, self.screen_height - y

width = 1280
height = 480

cam = Camera(height)

cyan = (0,255,255)
white = (255, 255, 255)

pygame.init()

running = True
clock = pygame.time.Clock()
fps = 60
dt = 1/fps

font = pygame.font.SysFont('Constantia', 15)

screen = pygame.display.set_mode((width,height))

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.MOUSEBUTTONDOWN:
              if event.button == 1:
                    click_pos = event.pos
                    wx, wy = cam.screen_to_world(event.pos[0], event.pos[1])

                    new_circle = circle(1, wx, wy, 0, 1, 20)
                    physics.physics_objects.append(new_circle)
                
    screen.fill(cyan)

    physics.step(dt, width,height)

    for p in physics.physics_objects:
        pygame.draw.circle(screen, white, cam.world_to_screen(p.x, p.y), p.radius)

    pygame.display.flip()
    clock.tick(60)
pygame.quit()