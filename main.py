import pygame
import numpy as np
import math

from Pathplanning import *
from Pathfollowing import *
from frame import *

screen = None

#============================================================#
start_points = [(200,180), (120,700), (500,700), (1000,700), (1100,400)] #시작 위치 정의
start_ = 1 #시작 위치 선택  
start_theta = 0 #시작 각도 정의
#============================================================#

parking_lines = create_parking_lines(np.pi/4, ((P_ENTRY[0]+P_END[0])//2, (P_ENTRY[1]+P_END[1])//2), 70, 100)

#init pygame -----------------------------------------------------------
try:
    pygame.init() #pygame 시작
    screen = pygame.display.set_mode((1200,800))
    pygame.display.set_caption("Game")
    
    rx, ry = planning(start_points[start_][0], start_points[start_][1], start_theta, 0, 0)
    
    player_car = PlayerCar(30, 100, 0)
    player_car.x, player_car.y = start_points[start_][0]-player_car.ch, start_points[start_][1]-player_car.cw
    player_car.angle = start_theta * 180 / np.pi
    
    vel = 0
    
    FPS = 60
    
    loop = True
    clock = pygame.time.Clock()
    
    car_move_allowed = False
    
    while loop:
        clock.tick(FPS)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                loop = False

        screen.fill((255,255,255))
        
        #시작 공간 그리기
        for s in start_points: 
            pygame.draw.circle(screen, (204,255,255), s, 80)
        
        #주차 공간 그리기
        for p0, p1 in parking_lines:
            pygame.draw.line(screen, (0,255,0), p0, p1, 2)
            
        #목표 경로 그리기    
        for x, y in zip(rx, ry):
            pygame.draw.circle(screen, (0,0,255), (x,y), 2)
        
        if car_move_allowed:
            ang, vel = tracking(screen, player_car.x, player_car.y, player_car.angle, vel, 0, 0)
            player_car.angle += ang
            
        draw(screen, player_car)
        
        
        keys = pygame.key.get_pressed()
        moved = False
        if keys[pygame.K_q]:
            car_move_allowed = True
        if keys[pygame.K_r]:
            car_move_allowed = False
            player_car.x, player_car.y = start_points[start_][0]-player_car.ch, start_points[start_][1]-player_car.cw
            player_car.angle = start_theta * 180 / np.pi
        '''
        if keys[pygame.K_a]:
            player_car.rotate(left=True)
            print(player_car.angle)
        if keys[pygame.K_d]:
            player_car.rotate(right=True)
            print(player_car.angle)
        if keys[pygame.K_w]:
            moved = True
            player_car.move_forward()
        if keys[pygame.K_s]:
            moved = True
            player_car.move_backward()

        if not moved:
            player_car.reduce_speed()
        '''
        
            
        pygame.display.update()
    #----------------


#except Exception as e:
#    print(e)
#    pygame.quit()
finally:
    pygame.quit()            
#-----------------------------------------------------------------------
#==================================================================================================