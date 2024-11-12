import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from scipy.interpolate import splprep, splev
from scipy.optimize import minimize
import time
import sys


def normalize(v):
    norm = np.linalg.norm(v,axis=0) + 0.00001
    return v / norm.reshape(1, v.shape[1])

def curvature(waypoints):
    '''
    ##### TODO #####
    Curvature as  the sum of the normalized dot product between the way elements
    Implement second term of the smoothin objective.

    args: 
        waypoints [2, num_waypoints] !!!!!
    '''
    way_element = normalize(np.diff(waypoints, axis = 1))
    curvature = np.sum(way_element[:,:-1] * way_element[:,1:])

    return curvature


def smoothing_objective(waypoints, waypoints_center, weight_curvature=40):
    '''
    Objective for path smoothing

    args:
        waypoints [2 * num_waypoints] !!!!!
        waypoints_center [2 * num_waypoints] !!!!!
        weight_curvature (default=40)
    '''
    # mean least square error between waypoint and way point center
    ls_tocenter = np.mean((waypoints_center - waypoints)**2)

    # derive curvature
    curv = curvature(waypoints.reshape(2,-1))

    return -1 * weight_curvature * curv + ls_tocenter


def waypoint_prediction(roadside1_spline, roadside2_spline, num_waypoints=6, way_type = "smooth"):
    '''
    ##### TODO #####
    Predict waypoint via two different methods:
    - center
    - smooth 

    args:
        roadside1_spline
        roadside2_spline
        num_waypoints (default=6)
        parameter_bound_waypoints (default=1)
        waytype (default="smoothed")
    '''
    if way_type == "center":
        ##### TODO #####
     
        # create spline arguments
        t = np.linspace(0, 1, num_waypoints)

        # derive roadside points from spline
        roadside1_points = np.array(splev(t, roadside1_spline))
        roadside2_points = np.array(splev(t, roadside2_spline))

        # derive center between corresponding roadside points
        way_points = (roadside1_points + roadside2_points)/2

        # output way_points with shape(2 x Num_waypoints)
        way_points = way_points.reshape(2 * num_waypoints)

        return way_points
    
    elif way_type == "smooth":
        ##### TODO #####

        # create spline points
        t = np.linspace(0, 1, num_waypoints)

        # roadside points from spline
        roadside1_points = np.array(splev(t, roadside1_spline))
        roadside2_points = np.array(splev(t, roadside2_spline))

        # center between corresponding roadside points
        way_points_center = (roadside1_points + roadside2_points)/2
        way_points_center = way_points_center.reshape(2 * num_waypoints)

        # init optimized waypoints
        way_points_init = np.random.uniform(low=0, high=65, size=way_points_center.shape)
        
        # optimization
        way_points = minimize(smoothing_objective, way_points_init, args = way_points_center).x

        return way_points.reshape(2,-1)


def target_speed_prediction(waypoints, num_waypoints_used=5,
                            max_speed=60, exp_constant=4.5, offset_speed=30):
    '''
    ##### TODO #####
    Predict target speed given waypoints
    Implement the function using curvature()

    args:
        waypoints [2,num_waypoints]
        num_waypoints_used (default=5)
        max_speed (default=60)
        exp_constant (default=4.5)
        offset_speed (default=30)
    
    output:
        target_speed (float)
    '''

    curv_center = curvature(waypoints)
    target_speed = (max_speed - offset_speed) * np.exp(-exp_constant * np.abs(num_waypoints_used - 2 - curv_center)) + offset_speed

    return target_speed