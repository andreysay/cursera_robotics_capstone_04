#!/usr/bin/python

import numpy as np

class DiffDriveController():
    """
    Class used for controlling the robot linear and angular velocity
    """
    def __init__(self, max_speed, max_omega):
        # TODO for Student: Specify these parameters
        self.kp=0.12
        self.ka=0.15
        self.kb=-0.01
        self.MAX_SPEED = max_speed
        self.MAX_OMEGA = max_omega
        
    def compute_vel(self, state, goal):
        """
        Function that computes the desired outputs given the state and goal
        Inputs:
        state - a numpy vector of size 3 by 1 with components (x,y,theta)
        goal - a numpy vector of size 2 by 1 specifying the location of the goal
        Outputs: a tuple with 3 elements
        v - a number specifying the forward speed (in m/s) of the robot (should 
            be no more than max_speed)
        omega - a number specifying the angular velocity (in rad/s) of the robot
            (should be no more than max_omega)
        done - a boolean value specifying if the robot has reached its goal (or
            is close enough
        """
        # YOUR CODE HERE
        k_array = np.array([[self.kp, 0, 0],
                            [0, self.ka, self.kb]])        
        epsilon = 0.18
        delta = np.array([[state[0],state[1]]])
        theta=state[2]
        p = np.sqrt(delta[0,0]*delta[0,0] + delta[0,1]*delta[0,1])
        alpha=-theta+np.arctan2(delta[0,1],delta[0,0])
        beta=-theta-alpha
        path_array = np.array([[p],[alpha],[beta]])
        control_array = np.dot(k_array,path_array)
        
        return (control_array[0,0],control_array[1,0], p < epsilon)
