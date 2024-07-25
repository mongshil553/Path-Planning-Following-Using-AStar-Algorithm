#!/usr/bin/env python
#-- coding:utf-8 --
####################################################################
# 프로그램이름 : parking.py
# 코드작성팀명 : 슈퍼카
####################################################################

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import pygame
import numpy as np
import math
import rospy
from PathPlanning import *
from xycar_msgs.msg import xycar_motor

#=============================================
# 모터 토픽을 발행할 것임을 선언
#============================================= 
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
xycar_msg = xycar_motor()

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#============================================= 
rx, ry = [300, 350, 400, 450], [300, 350, 400, 450]

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
AR = (1142, 62) # AR 태그의 위치
P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
P_END = (1129, 69) # 주차라인 끝의 좌표

#=============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):
    xycar_msg.angle = int(angle)
    xycar_msg.speed = int(speed)
    motor_pub.publish(xycar_msg)

#=============================================
# 경로를 생성하는 함수
# 차량의 시작위치 sx, sy, 시작각도 syaw
# 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
# 경로를 리스트를 생성하여 반환한다.
#=============================================
def planning(sx, sy, syaw, max_acceleration, dt):
    global rx, ry
    print("Start Planning")

    '''
    1. 주차 각도 계산
    2. 경로 최종 위치 계산
    3. 주차장 꼭지점을 계산
    4. 맵 축소(계산량 줄이기, 하지만 정확도가 떨어질 수 있다는 문제 발생)
    5. A* 알고리즘 실행 (시작이 전진, 후진 두 경우)
    6. 두 경로 중 더 짧은 것을 선택
    7. 경로를 원래 맵 크기로 복원
    8. 샘플링을 통해 Path Smoothing
    9. 샘플링에 의해 노드가 서로 멀리 떨어져있으므로 중간 채우기
    10. 경로 반환
    '''
    
    reduce_rate = 4 #MAP 축소 비율 -> Path Planning 시간 단축

    #화면은 1200*800이다.
    border = (((0,0), (1200//reduce_rate,0)), ((1200//reduce_rate,0), (1200//reduce_rate, 800//reduce_rate)),
                ((1200//reduce_rate, 800//reduce_rate), (0, 800//reduce_rate)), ((0, 800//reduce_rate), (0, 0)))
    
    goal_angle = math.atan2(P_END[1]-P_ENTRY[1], P_END[0]-P_ENTRY[0]) #주차 각도
    goal_center = (P_ENTRY[0]*0.3+P_END[0]*0.7, P_ENTRY[1]*0.3+P_END[1]+0.7) #주차 중심
    goal_center_reduced = (goal_center[0]//reduce_rate, goal_center[1]//reduce_rate) #축소된 MAP에서의 주차 중심

    barriers_point = create_parking_lines(-goal_angle, goal_center_reduced, 100//reduce_rate, 170//reduce_rate) #주차장 주위 장애물 계산
    

    astar = AStar(barriers_point, border, 50, 50) #A* 알고리즘 객체 생성
    path_0 = astar.run((int(sx//reduce_rate), int(sy//reduce_rate)), goal_center_reduced, syaw, goal_angle, 'drive') #알고리즘 실행(전진)
    path_1 = astar.run((int(sx//reduce_rate), int(sy//reduce_rate)), goal_center_reduced, syaw, goal_angle, 'reverse') #알고리즘 실행(전진)
    
    #더 짧은 경로 선택
    if len(path_0) == 0:
        path_ = path_1
    elif len(path_1) == 0:
        path_ = path_0
    elif len(path_0) <= len(path_1):
        path_ = path_0
    else:
        path_ = path_1
    #path_ = path_1

    path_ = list(zip([(p[0]+0)*reduce_rate for p in path_], [(p[1]+0)*reduce_rate for p in path_])) #원래 맵 크기로 복원

    path_ = [[x,y] for x,y in path_][::max(len(path_)//30, 1)] #Sampling
    path_ = astar.smooth(path_, 0.1, 0.2) #Smoothing


    path_final = [] #fill in path
    for i in range(len(path_)-1):
        path_final.extend(zip(np.linspace(path_[i][0], path_[i+1][0], 10+1),
                np.linspace(path_[i][1], path_[i+1][1], 10+1)))
    path_final.append(path_[-1])

    rx = [x[0] for x in path_final]
    ry = [x[1] for x in path_final]

    print("Finished Planning")
    return rx, ry

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================
def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    global rx, ry
    angle = 50 # -50 ~ 50
    speed = 50 # -50 ~ 50

    goal_angle = math.atan2(P_END[1]-P_ENTRY[1], P_END[0]-P_ENTRY[0]) #주차 각도
    finish_guide0, finish_guide1 = get_finish_guide(AR, goal_angle, (110, 30))

    #주차 가이드 생성
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
    else:
        for p in finish_guide: #주차장 가이드 중에 검색
            detected, pos = pure_pursuit_detect(x, y, search_radius, p) #주차장에 접근을 했는지
            if detected: #주차장 근처라면
                ref = calculate_reference(x, y, yaw, pos) #현재 위치, 현재 각도, 목표 위치를 통해 목표 각도 계산
                err, y = PID(ref, yaw) #PID 제어로 회전 각도 계산, 현재는 P제어만 있는데, 충분히 좋음
                
                target_velocity = calc_velocity(abs(err)) #속도 계산하기
                print(target_velocity)

                drive(-y, target_velocity) #이동하기
        else:
            for p in car_path[::-1]: #목표 경로 중에 검색
                detected, pos = pure_pursuit_detect(x, y, search_radius, p) #검색 반경 중 Intersect하는 경로 찾기
                if detected: #경로가 있다면
                    pygame.draw.circle(screen, (100,100,100), pos, 5) #해당 위치에 점 찍기

                    ref = calculate_reference(x, y, yaw, pos) #현재 위치, 현재 각도, 목표 위치를 통해 목표 각도 계산
                    err, y = PID(ref, yaw) #PID 제어로 회전 각도 계산, 현재는 P제어만 있는데, 충분히 좋음
                    
                    target_velocity = calc_velocity(abs(err)) #속도 계산하기
                    print(target_velocity)


                    drive(-y, target_velocity) #이동하기

                    break

    

    #drive(angle, speed)

