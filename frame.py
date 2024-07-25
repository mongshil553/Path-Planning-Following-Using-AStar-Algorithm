import pygame
import numpy as np
import math

from main import screen

# ----- CAR -----
def scale_image(img, factor):
    size = round(img.get_width() * factor), round(img.get_height() * factor)
    return pygame.transform.scale(img, size)


def blit_rotate_center(win, image, top_left, angle):
    rotated_image = pygame.transform.rotate(image, angle)
    new_rect = rotated_image.get_rect(
        center=image.get_rect(topleft=top_left).center)
    win.blit(rotated_image, new_rect.topleft)
    
CAR = scale_image(pygame.image.load("C:/Users/kijun/Documents/GitHub/Autonomous-Driving-Taksk1/car.png"), 0.15)

class AbstractCar:
    def __init__(self, max_vel, rotation_vel, rd):
        self.img = self.IMG
        self.max_vel = max_vel
        self.vel = 0
        self.rotation_vel = rotation_vel
        self.angle = 0
        self.x, self.y = self.START_POS
        self.acceleration = 0.3
        self.rd = rd
        
        self.ch = self.img.get_height()//2
        self.cw = self.img.get_width()//2
        self.circle = pygame.draw.circle(screen, (150, 150, 150), (self.x + self.cw, self.y + self.ch), self.rd, 2)
        
    def rotate(self, left=False, right=False):
        if left:
            self.angle += self.rotation_vel
        elif right:
            self.angle -= self.rotation_vel

    def draw(self, win):
        blit_rotate_center(win, self.img, (self.x, self.y), self.angle)
        self.circle = pygame.draw.circle(screen, (150, 150, 150), (self.x + self.cw, self.y + self.ch), self.rd, 2)

    def move_forward(self):
        self.vel = min(self.vel + self.acceleration, self.max_vel)
        self.move()
    
    def move_backward(self):
        self.vel = max(self.vel - self.acceleration, -self.max_vel/2)
        self.move()
    
    def move(self):
        radians = math.radians(self.angle)
        vertical = math.cos(radians) * self.vel
        horizontal = math.sin(radians) * self.vel

        self.y -= vertical
        self.x -= horizontal
        
        #print(self.x, self.y)
        #self.x += vertical
        #self.y -= horizontal

    def reduce_speed(self):
        self.vel = max(self.vel - self.acceleration / 1.5, 0)
        self.move()

class PlayerCar(AbstractCar):
    IMG = CAR
    START_POS = (180, 200)
    
    
    def __init__(self, max_vel, rotation_vel, search_radius):
        super().__init__(max_vel, rotation_vel, search_radius)
        #self.path = path
        self.current_point = 0
        self.vel = max_vel
    
    def calculate_angle(self, target):
        
        angle_in_radian = self.angle * np.pi/180
        T10 = np.linalg.inv(np.array([[math.cos(-(angle_in_radian + np.pi)), -math.sin(-(angle_in_radian + np.pi)), 0, self.x + self.cw],
                [math.sin(-(angle_in_radian + np.pi)), math.cos(-(angle_in_radian + np.pi)), 0, self.y + self.ch],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])) #{기준 좌표계}에서 {차 좌표계}로 변환하는 Transformation Matrix 생성
        p1 = np.dot(T10, np.transpose([target[0], target[1], 0, 1]))
        p = [p1[0], p1[1]]
        #print(p)
        
        if p[0] > 0:
            #print(1)
            difference_in_angle = 90 - math.atan2(p[1],p[0])*180/np.pi
            self.angle += min(self.rotation_vel, abs(difference_in_angle))
        elif p[0] < 0:
            #print(0)
            difference_in_angle = math.atan2(p[1],p[0])*180/np.pi
            if difference_in_angle < 0:
                difference_in_angle = 270 + difference_in_angle
            else:
                difference_in_angle = 180 - difference_in_angle
                
            self.angle -= min(self.rotation_vel, abs(difference_in_angle))
            #difference_in_angle = math.atan2(p[1],p[0])*180/np.pi - 90
            
        #print(self.angle)
        

        #if difference_in_angle > 0:
        #    self.angle -= min(self.rotation_vel, abs(difference_in_angle))
        #else:
        #    self.angle += min(self.rotation_vel, abs(difference_in_angle))
        
        
    
    def move(self, target=None):
        #if self.current_point >= len(self.path):
        #    return
        
        if target is not None:
            self.calculate_angle(target)
            #self.update_path_point()
        super().move()

def draw(win, player_car):

    player_car.draw(win)
    pygame.display.update()
