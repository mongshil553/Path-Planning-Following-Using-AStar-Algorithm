import pygame
import numpy as np
import math

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================
def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    global rx, ry
    global reverse_end_index    # 전진경로와 후진경로를 나누는 경계 인덱스 변수수
    angle = 50 # -50 ~ 50
    speed = 50 # -50 ~ 50


    # print(reverse_end_index, 'index;')

    goal_angle = math.atan2(P_END[1]-P_ENTRY[1], P_END[0]-P_ENTRY[0]) #주차 각도
    finish_guide0, finish_guide1 = get_finish_guide(AR, goal_angle, (110, 30))

    #주차 가이드 생성환
    finish_guide = []
    finish_guide.extend(zip(np.linspace(finish_guide0[0], finish_guide1[0], 3+1),np.linspace(finish_guide0[1], finish_guide1[1], 3+1)))
    finish_guide = [((finish_guide[i][0], finish_guide[i][1]), (finish_guide[i+1][0], finish_guide[i+1][1])) for i in range(len(finish_guide)-1)] #경로를 직선을 잇기
    

    search_radius = 100 #검색 반경
    Stop_Tolerance = 80 #주차장 도착 감지 거리

    rxy = list(zip(rx, ry))
    car_path = [((rxy[i][0], rxy[i][1]), (rxy[i+1][0], rxy[i+1][1])) for i in range(len(rxy)-1)] #경로를 직선을 잇기

    pygame.draw.circle(screen, (100,100,100), (x,y), search_radius, 2) #차를 중심으로 검색 반경 그리기

    
    if (math.sqrt((AR[0]-x)**2 + (AR[1]-y)**2)) <= Stop_Tolerance: #주차장에 도착했다면
        drive(0, 0) #정지하기

    elif any(pure_pursuit_detect(x, y, search_radius, p)[0] for p in finish_guide):
        for p in finish_guide: #주차장 가이드 중에 검색
            detected, pos = pure_pursuit_detect(x, y, search_radius, p) #주차장에 접근을 했는지
            if detected: #주차장 근처라면
                pygame.draw.circle(screen, (100,100,100), pos, 5)
                ref = calculate_reference(x, y, yaw, pos) #현재 위치, 현재 각도, 목표 위치를 통해 목표 각도 계산
                err, gained_angle = PID(ref, yaw) #PID 제어로 회전 각도 계산, 현재는 P제어만 있는데, 충분히 좋음
                
                target_velocity = calc_velocity(abs(err)) #속도 계산하기
                print(target_velocity)

                drive(-gained_angle, target_velocity) #이동하기
    
    else:
            for p in car_path[::-1]:
                detected, pos = pure_pursuit_detect(x, y, search_radius, p)
                if detected:
                    pygame.draw.circle(screen, (100,100,100), pos, 5)
                    ref = calculate_reference(x, y, yaw, pos)
                    err, gained_angle = PID(ref, yaw)
                    target_velocity = calc_velocity(abs(err))
                    print(target_velocity)
                    drive(-gained_angle, target_velocity)
                    break
        

#------- 경로 추종 중 다음 노드 찾는 함수 --------
'''
자동차를 중심으로 하는 원과, 경로 노드와 노드를 잇는 직선의 교점이 존재하는 판단하는 함수이다.
이 원은 자동차가 경로를 탐색할 때 사용되는 원이다.
교점이 존재한다면 해당 교점이 자동차가 목표로 해야하는 노드이다.
'''
def pure_pursuit_detect(x, y, search_radius, line): #직선과 원 사이 교점 찾기
    Q = np.array([x, y])        # Center of circle
    r = search_radius           # Radius of circle
    P1 = np.array(line[0])      # Start of line segment
    V = np.array([line[1][0]-P1[0], line[1][1]-P1[1]]) #Vector along line segment

    a = np.dot(V, V)
    b = 2 * np.dot(V, P1-Q)
    c = np.dot(P1, P1) + np.dot(Q, Q) - 2*np.dot(P1, Q) - r**2

    disc = b**2 - 4 * a * c
    if disc < 0:
        return False, None

    sqrt_disc = math.sqrt(disc)
    t1 = (-b + sqrt_disc) / (2 * a)
    t2 = (-b - sqrt_disc) / (2 * a)

    if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
        return False, None

    t = max(0, min(1, - b / (2 * a)))
    return True, P1 + t * V
#------------------------------------------------
        
#------ PID 입력을 위한 회전 각도 Reference 계산 함수 -----------
'''
target: 목표 노드
angle: 현재 각도

현재 각도에서 target을 바라보기 위해 회전해야 하는 각도를 계산해서 반환한다.
'''
def calculate_reference(x, y, angle, target): #PID 입력을 위한 회전 각도 Reference 계산

    angle_in_radian = angle * np.pi/180
    T10 = np.linalg.inv(np.array([[math.cos(-(angle_in_radian)), -math.sin(-(angle_in_radian)), 0, x],
            [math.sin(-(angle_in_radian)), math.cos(-(angle_in_radian)), 0, y],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])) #{기준 좌표계}에서 {차 좌표계}로 변환하는 Transformation Matrix 생성
    p1 = np.dot(T10, np.transpose([target[0], target[1], 0, 1]))
    p = [p1[0], p1[1]]
    
    if p[1] > 0: #target이 오른쪽에 있음
        difference_in_angle = math.atan2(p[1],p[0])*180/np.pi

        return angle - abs(difference_in_angle)
    else: #target이 왼쪽에 있음
        difference_in_angle = math.atan2(p[1],p[0])*180/np.pi
            
        return angle + abs(difference_in_angle)
#--------------------------------------------------------------

#----- 회전 각도를 제어하는 PID 함수 ------
'''
PID 제어인데, P제어만 해도 충분했다.
'''
def PID(ref, y): #PID 제어, 현재 P제어만 있음. ref: 목표 각도, y: 현재 각도
    err = ref - y #에러

    k_p = 10 #Proportional Gain
    
    return err, k_p * err
#----------------------------------------

#----- 속도 계산 함수 -----
'''
    각도 에러에 따른 속도를 계산한다.

    최대 오류 = threshold = 90
    최소 속력 = min_speed
    최대 속력 = max_speed
'''
def calc_velocity(err): #각도 에러에 따른 속도 계산
    threshold = 90
    min_speed = 20
    max_speed = 50

    result = (max_speed-min_speed) / (0-threshold) * min(threshold, err) + max_speed

    return result
#-------------------------

#----- 자동차 도착 판단하는 가이드 생성 함수 ------
'''
주차장에 도착했을 때 경로가 주차장 중앙에서 끊긴다.
현재 알고리즘으로는 차는 다음 목표 노드가 이미 지나온 경로 중 하나로 판단하기 때문에 그에 따른 제어가 발생한다.
이를 무시하기 위해, 주차장에 도착했을 때 따라갈 수 있는 경로를 임의 생성한다.

경로 생성 알고리즘 혹은 경로 추종 알고리즘을 발전시키면 가이드가 필요 없을 수 있다.
'''
def get_finish_guide(pos, ang, line_length): #주차장 도착했을 때 자동차를 이끌 수 있는 가이드 생성
    T01 = Transformation_Matrix(-ang, pos)

    pos0_a = np.transpose(np.dot(T01, np.transpose(np.array([0, line_length[0], 0, 1]))))
    pos0_b = np.transpose(np.dot(T01, np.transpose(np.array([0, -line_length[1], 0, 1]))))
    
    return [pos0_a[0], pos0_a[1]], [pos0_b[0], pos0_b[1]]
# -------------------
