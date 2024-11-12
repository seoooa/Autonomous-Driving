import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from scipy.interpolate import splprep, splev
from scipy.optimize import minimize
import time


class LateralController:
    '''
    Lateral control using the Stanley controller

    functions:
        stanley 

    init:
        gain_constant (default=5)
        damping_constant (default=0.5)
    '''


    def __init__(self, gain_constant=5, damping_constant=0.6):

        self.gain_constant = gain_constant
        self.damping_constant = damping_constant
        self.previous_steering_angle = 0


    def stanley(self, waypoints, speed):
        '''
        ##### TODO #####
        one step of the stanley controller with damping
        args:
            waypoints (np.array) [2, num_waypoints]
            speed (float)
        '''

        # derive orientation error as the angle of the first path segment to the car orientation
        # derive stanley control law
        # derive cross track error as distance between desired waypoint at spline parameter equal zero ot the car position
        # 바퀴가 얼마나 꺾여야하는지 계산 (first term)
        dx = waypoints[0,1] - waypoints[0,0]
        dy = waypoints[1,1] - waypoints[1,0]

        heading_angle = dx/dy
        heading_error = np.arctan(heading_angle)

        first_term = heading_error

        e = waypoints[0,0] - 48

        # prevent division by zero by adding as small epsilon
        # crosstrack error가 너무 크면 일단 바퀴를 수직으로 해서 경로에 가까워지도록 구현 (second term)
        epsilon = 1E-10
        k = self.gain_constant

        second_term = np.arctan(k * e / (speed + epsilon))

        # derive damping
        # clip to the maximum stering angle (0.4) and rescale the steering action space
        # damping: 이전 단계의 steering과 현재 steering의 차이를 이용하여 감쇄운동하도록 구현
        delta = np.clip(first_term + second_term, -0.4, 0.4)
        damping_delta = delta - self.damping_constant * (delta - self.previous_steering_angle)

        self.previous_steering_angle = damping_delta

        return damping_delta