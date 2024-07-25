import cv2
import numpy as np
import matplotlib.pyplot as plt
import heapq
import math
import pygame
from copy import deepcopy
from scipy.optimize import curve_fit

# ------ Todo List ------
    # Path Following
    
    # 동적 가중치 적용 <- Collision 가중치가 높으면 삥 돌아서 가는 경우가 있음. 목표와 가까워질 때까지는 Collision 가중치를 낮게,
    # 특정 threshold보다 목표가 가까워지면 Collision 가중치를 높게 설정하면, 멀리서 접근할 때는 Collision 신경 안 쓰고 빠르게 접근
    # 가까워지면 Collision을 크게 신경 씀
    
    #전진 / 후진 구분

# -----------------------

class Queue(object):
    def __init__(self):
        self.elements = []

    def length(self):
        return len(self.elements)

    def push(self, x, priority):
        heapq.heappush(self.elements, (priority, x))

    def pop(self):
        return heapq.heappop(self.elements)[1]


class AStar():
    def __init__(self, grid, barrier_list, collision_gain, theta_gain):
        self.grid = grid
        self.queue = None
        self.came_from = None
        self.cost_so_far = None
        self.barrier_list = barrier_list
        self.theta = None
        self.collision_gain = collision_gain
        self.theta_gain = theta_gain

    def run(self, start, goal, theta):
        self.queue = Queue()
        self.queue.push(start, 0)
        self.came_from = {}
        self.cost_so_far = {}
        self.cost_so_far[start] = 0
        self.theta = theta

        while self.queue.length() > 0:
            current = self.queue.pop()

            if current == goal:
                break

            (x, y) = current
            candidates = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]

            for next in [(h, v) for h, v in candidates if self.grid[v][h] != 0]:
                new_cost = self.calc_cost(current, next)
                if next not in self.came_from or new_cost < self.cost_so_far[next]:
                    self.queue.push(next, new_cost + self.heuristic(goal, next))
                    self.cost_so_far[next] = new_cost
                    self.came_from[next] = current

        current = goal
        path = []
        while current is not start:
            path.append(current)
            current = self.came_from[current]
        path.reverse()
        return path

    def calc_cost(self, current, next):
        (x, y) = next
        return self.cost_so_far[current] + self.grid[y][x]

    def heuristic(self, goal, next):
        
        #Shortest Distance + Collision + Degree Difference를 휴리스틱값
        return self.check_dist(goal, next) + self.check_collision(goal, next) + self.check_degree(goal, next)
    
    def check_dist(self, goal, next):
        (x1, y1) = goal
        (x2, y2) = next

        dx = x1 - x2
        dy = y1 - y2
        
        return math.sqrt(dx*dx+dy*dy) #다음 노드부터 목표 노드까지의 직선 거리 반환
    
    def check_collision(self, goal, next):
        
        if next == goal:
            return 0 #다음 노드와 목표 노드가 동일하므로 0 반환
        
        line = pygame.draw.line(screen, (0,255,0), goal, next, 2) #다음 노드부터 목표 노드까지의 직선 생성
        for bi in self.barrier_list: #주차 공간 주변 가상의 장애물 중
            if line.colliderect(bi): #장애물과 직선이 부딪힌다면
                return self.collision_gain #collision_gain 반환
        
        return 0 #다음 노드부터 목표 노드까지 아무런 장애물이 없으므로 0 반환
    
    def check_degree(self, goal, next):
        
        u = np.array(goal) - np.array(next) #다음 노드부터 목표 노드까지의 벡터
        u_theta = math.atan2(u[1], u[0])    #벡터의 각도 계산
        
        dt = abs(abs(u_theta) - abs(self.theta))*self.theta_gain #목표 각도와, u 벡터의 각도 차 계산, 그 후 theta_gain 곱하기
        
        return int(dt)
    
    def objective(self, x, a, b, c, d):
         return a * sin(b - x) + c * x**2 + d
    
    def smooth(self, path, weight_data=0.5, weight_smooth=0.1, tolerance=0.000001):
        
       # x = [p[0] for p in path]
       # y = [p[1] for p in path]
        
       # popt, _ = curve_fit(objective, x, y)
       # a, b, c, d = popt
        
       # x_line = arange(min(x), min(y), 1)
        
        
        new = deepcopy(path)
        dims = len(path[0])
        change = tolerance

        while change >= tolerance:
            change = 0.0
            for i in range(1, len(new) - 1):
                for j in range(dims):

                    x_i = path[i][j]
                    y_i, y_prev, y_next = new[i][j], new[i - 1][j], new[i + 1][j]

                    y_i_saved = y_i
                    y_i += weight_data * (x_i - y_i) + weight_smooth * (y_next + y_prev - (2 * y_i))
                    new[i][j] = y_i

                    change += abs(y_i - y_i_saved)

        return np.array(new, dtype='int').tolist()
        
    
def create_imaginary_boundary(parking_theta, parking_xy, width, height):
    T01 = np.array([[math.cos(-parking_theta), -math.sin(-parking_theta), 0, parking_xy[0]],
                [math.sin(-parking_theta), math.cos(-parking_theta), 0, parking_xy[1]],
                [0, 0, 1, 0],
                [0, 0, 0, 1]]) #{기준 좌표계}에서 {주차 공간 좌표계}로 변환하는 Transformation Matrix 생성

    x_idxs = range(-height//2, height//2)
    y_idxs = range(-width//2, width//2)

    #{주차 공간 좌표계}에서 주차 공간 근처에 가상 장애물 정의
    parking_barrier1 = np.concatenate(([[x,y, 0, 1] for (x,y) in zip(list(x_idxs), [width//2 for _ in x_idxs])],
                                      [[x,y, 0, 1] for (x,y) in zip([car_height//2 for _ in y_idxs], list(y_idxs))],
                                      [[x,y, 0, 1] for (x,y) in zip(list(x_idxs), [-width//2 for _ in x_idxs])]))
    parking_barrier1 = np.transpose(parking_barrier1)
    parking_barrier0 = np.dot(T01, parking_barrier1) #{주차 공간 좌표계} -> {기준 좌표계}로 좌표 변환
    barriers = [(p[0],p[1]) for p in np.transpose(parking_barrier0)]
    barriers = np.array(barriers, dtype='int') #{기준 좌표계}에서 index를 위해 interpolation이 아닌 단순 truncate
    
    return barriers

def create_parking_lines(parking_theta, parking_xy, width, height):
    T01 = np.array([[math.cos(-parking_theta), -math.sin(-parking_theta), 0, parking_xy[0]],
                [math.sin(-parking_theta), math.cos(-parking_theta), 0, parking_xy[1]],
                [0, 0, 1, 0],
                [0, 0, 0, 1]]) #{기준 좌표계}에서 {주차 공간 좌표계}로 변환하는 Transformation Matrix 생성
    
    #주차 공간 꼭지점
    parking_v1 = np.array([[-height//2, width//2, 0, 1],[height//2, width//2, 0, 1],
                           [height//2, -width//2, 0, 1],[-height//2, -width//2, 0, 1]])
    
    parking_v1 = np.transpose(parking_v1)
    parking_v0 = np.dot(T01, parking_v1) #{주차 공간 좌표계} -> {기준 좌표계}로 좌표 변환
    parking_v = [(p[0],p[1]) for p in np.transpose(parking_v0)]
    pv = np.array(parking_v, dtype='int') #{기준 좌표계}에서 index를 위해 interpolation이 아닌 단순 truncate
    
    parking_v = ((pv[0], pv[1]),(pv[1], pv[2]),(pv[2], pv[3]),(pv[3], pv[0]))
    
    return parking_v


# ----- 맵 및 주차 공간 생성 -----
mygrid_x, mygrid_y = 800, 500 #맵 크기 정의
mygrid = [[1 for x_ in range(mygrid_x)] for y_ in range(mygrid_y)] #0: 장애물, 1:빈 공간

for i in range(mygrid_x): #가장자리는 장애물로 표시
    mygrid[0][i] = 0
    mygrid[mygrid_y-1][i] = 0
for i in range(mygrid_y):
    mygrid[i][0] = 0
    mygrid[i][mygrid_x-1]=0

goal = (650, 100) #주차 공간 위치 정의
parking_theta = np.pi/4 #주차 각도 정의                      <====================
start_points = [(100,80), (120,380), (380,380), (650,380)] #시작 위치 정의
start_ = 3 #시작 위치 선택                                   <====================

car_width, car_height = 50, 80 #자동차 크기 정의

parking_lines = create_parking_lines(parking_theta, goal, car_width, car_height)
barriers = create_imaginary_boundary(parking_theta, goal, car_width, car_height) #주차 공간 근처 가상의 장애물 생성
for x, y in barriers: #장애물 추가에 따른 맵 업데이트
    mygrid[y][x] = 0


#init pygame -----------------------------------------------------------
try:
    pygame.init() #pygame 시작
    screen = pygame.display.set_mode((mygrid_x,mygrid_y))
    pygame.display.set_caption("Game")

    TILESIZE = 5
    barrier_list = []
    
    for x, y in barriers: #주차 공간에 가상의 장애물 그리기
        obj = pygame.draw.rect(screen, (255,255,255), (x, y, TILESIZE, TILESIZE))
        barrier_list.append(obj)


    astar = AStar(mygrid, barrier_list, 2000, 500)  #A* 알고리즘 객체 생성
    res1 = astar.run(start_points[start_], goal, parking_theta) #A* 알고리즘 실행
    
    res1 = [[x,y] for x,y in res1][::len(res1)//50] #Smoothing
    res1 = astar.smooth(res1, 0.1, 0.2)

    loop = True
    clock = pygame.time.Clock()
    while loop:
        clock.tick(600)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                loop = False

        screen.fill((255,255,255))
        
        #시작 공간 그리기
        for s in start_points: 
            pygame.draw.circle(screen, (204,255,255), s, 30)
            
        for x, y in barriers: #주차 공간에 가상의 장애물 그리기
            obj = pygame.draw.rect(screen, (0,0,0), (x, y, TILESIZE, TILESIZE))
        
        #주차 공간 그리기
        for p0, p1 in parking_lines:
            pygame.draw.line(screen, (0,255,0), p0, p1, 2)
            
        #A*에 의한 경로 그리기
        for i in range(len(res1)-1):
            pygame.draw.line(screen, (255,0,0), (res1[i][0], res1[i][1]), (res1[i+1][0], res1[i+1][1]), 2)
            
        pygame.display.update()
    #----------------


except Exception as e:
    print(e)
    pygame.quit()
finally:
    pygame.quit()            
#-----------------------------------------------------------------------