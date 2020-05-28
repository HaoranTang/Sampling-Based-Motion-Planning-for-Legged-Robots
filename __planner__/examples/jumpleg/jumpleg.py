# -*- coding: utf-8 -*-
"""
Created on Fri May  1 16:39:14 2020

@author: zmc

CVXOPT installation
conda install -c conda-forge cvxopt
https://cvxopt.org/install/index.html
"""

# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import numpy as np
from src.rrt.rrt_base import RRTBase
from src.search_space.search_space import SearchSpace
from src.utilities.plotting import Plot
from scipy import linalg
import math
import matplotlib.pyplot as plt
from cvxopt import matrix, solvers
solvers.options['show_progress'] = False
import time

start_time = time.time()

# Change List to Linked List. Will be used for the post-processing of the path.
class ListNode:
    def __init__(self, x):
        self.val = x
        self.next = None

def listtolinked(list):
    cur = copy = ListNode(0)
    for i in list:
        cur.next = ListNode(i)
        cur = cur.next
    return copy.next
        
# dimensions of Search Space
X_dimensions = np.array([(0, 200), (0, 100)])  #[(x_lower,x_upper),(z_lower,z_upper)]
# obstacles
Obstacles = np.array([(20, 0, 60, 10),(70, 0, 100, 30)]) # ((lower_left),(upper_right))

CoM_height = 10
# starting location
x_init = (65, 0)
x_init = (x_init[0], list(x_init)[1] + CoM_height)  
# goal location
x_goal = (199, 0)
x_goal = (x_goal[0], list(x_goal)[1] + CoM_height)    

# creating sample space 
x_sample = [0]
z_sample = []
for i in range(len(Obstacles)):
    x_sample.append(Obstacles[i][0])
    x_sample.append(Obstacles[i][2])
    z_sample.append(0)
    z_sample.append(Obstacles[i][3])
x_sample.append(200)
z_sample.append(0)

# Huristic used for choosing jump direction
obstacle_last = Obstacles[-1][0]

# create search space
X = SearchSpace(X_dimensions, Obstacles)
path = [x_init]
x = x_init
Coef = []

# Used for reachability guided sample
lower_range = 10
upper_range = 30

# Used for collision checking
segments = 500
max_fail_time = 50

# Used for goal detection
goal_margin = 5

# Used for feasibility checking
v_max = 30

while linalg.norm([np.abs(x[0] - x_goal[0]), np.abs(x[1] - x_goal[1])],2) > goal_margin:    
    fail = 0
    while fail < max_fail_time:
        # Reachability guided sample
        x_range_1 = [min(x[0]+lower_range,X_dimensions[0][1]-0.01),min(x[0]+upper_range,X_dimensions[0][1]-0.01)]
        x_range_2 = [max(x[0]-upper_range,X_dimensions[0][0]+0.01),max(x[0]-lower_range,X_dimensions[0][0]+0.01)]
        x_rand_1 = (x_range_1[1] - x_range_1[0]) * np.random.random_sample() + x_range_1[0]
        x_rand_2 = (x_range_2[1] - x_range_2[0]) * np.random.random_sample() + x_range_2[0]  
        launch_angle = (6 - 3) * np.random.random_sample() + 3
        
        # If is still obstacle between the goal and the current position, then 50% left and 50% right
        if x[0] < obstacle_last:
            if np.random.random_sample() > 2:
                x_rand = x_goal[0]
                if x[0] < x_rand:
                    flag = 1
                else:
                    flag = 2           
            elif np.random.random_sample() > 0.5 or flag == 2:
                x_rand = x_rand_1
                flag = 1
            else:
                x_rand = x_rand_2
                flag = 2
        
        # If is no obstacle between the goal and the current position, only right
        else:
            x_rand = x_rand_1
            flag = 1
        
        # Find the corresponding Z value in the terrian for a sampled X
        for i in range(len(x_sample)):
            if x_rand < x_sample[i+1]:
                z_rand = z_sample[i]
                z_rand += 10
                break
                
        x_new = (x_rand,z_rand)
        
        # Huristic: don't explore the place around the position of the previous step
        if linalg.norm([np.abs(x_new[0] - path[-1][0]), np.abs(x[1] - x_goal[1])],2) < 10:
            break
        
        collide = False
        # Collision checking    
        if flag == 1:
            try:
                A = matrix([ [x[0]**2, x_new[0]**2], [x[0], x_new[0]], [1.0, 1.0] ],tc='d')
                b = matrix([ float(x[1]), float(x_new[1])], tc='d')
                G = matrix([ [-2*x[0], 2*x_new[0], 1], [-1.0, 1.0, 0.0], [0.0, 0.0, 0.0] ],tc='d')
                h = matrix([ -1, -1, 0], tc='d')
                c = matrix([ 0.0, 0.0, 0.0 ],tc= 'd')
                P = np.zeros((3,3))
                P[0][0]= -1
                P = matrix(P,tc='d')
                q = matrix([ 0.0, 0.0, 0.0 ], tc= 'd')
                sol = solvers.lp(c,G,h,A,b)
                coef = list(sol['x'])
            except ValueError:
                fail += 1
                collide = True
        elif flag == 2:
            try:
                A = matrix([ [x[0]**2, x_new[0]**2], [x[0], x_new[0]], [1.0, 1.0] ],tc='d')
                b = matrix([ float(x[1]), float(x_new[1])],tc='d')
                G = matrix([ [2*x[0], -2*x_new[0], 1], [1.0, -1.0, 0.0], [0.0, 0.0, 0.0] ],tc='d')
                h = matrix([ -1, -1, 0], tc='d')
                c = matrix([ 0.0, 0.0, 0.0 ],tc= 'd')
                P = np.zeros((3,3))
                P[0][0]= -1
                P = matrix(P,tc='d')
                q = matrix([ 0.0, 0.0, 0.0 ],tc= 'd')
                sol = solvers.lp(c,G,h,A,b)
                coef = list(sol['x'])
            except ValueError:
                fail += 1
                collide = True
 
        # Feasibility checking: if with the max initial speed, the robot cannot reach the highest point 
        #                       on the parabola, then re-sample        
        if collide == False:
        
            ea = coef[0]
            eb = coef[1]
            ec = coef[2]
            if ec - eb**2/(4*ea) > (100*v_max**2/20) *((2*ea*x[0]+eb)**2/(1+(2*ea*x[0]+eb)**2)) + x[1]:
                collide = True
        
        # Collision checking
        if collide == False:
            dx = (x_new[0] - x[0])/segments
            for i in range(segments):
                x_x = x[0] + i*dx
                x_y = coef[0] * x_x**2 + coef[1] * x_x + coef[2]
                if X.obstacle_free((x_x,x_y)):
                    pass
                else:
                    collide = True
        
        # Passed all the test, add a new point to the path
        if collide == False:
            path.append(x_new)
            x = x_new
            Coef.append(coef)
            break
        else:
            fail += 1
    
    # If cannot find next step, then go back to the previous step
    # This can help avoid the robot to be trapped at some undesirable position        
    if fail == max_fail_time:
        path.pop()
        Coef.pop()
        x = path[-1]

print("--- %s seconds ---" % (time.time() - start_time))
# TODO : Post-processing
# Get rid of the small steps

# Generate trajectory from way-point            
traj = []
X_plot = []
Z_plot = []

for i in range(len(path)-1):
    dx = (path[i+1][0]-path[i][0])/100
    for j in range(100):
        x_x = path[i][0] + j*dx
        x_z = Coef[i][0] * x_x**2 + Coef[i][1] * x_x + Coef[i][2]
        traj.append((x_x,x_z))
        X_plot.append(x_x)
        Z_plot.append(x_z)
traj.append(path[-1])

plot = Plot("jumpleg")
plot.plot_path(X, traj)
plot.plot_obstacles(X, Obstacles)
plot.plot_start(X, x_init)
plot.plot_goal(X, x_goal)
plot.draw(auto_open=True)