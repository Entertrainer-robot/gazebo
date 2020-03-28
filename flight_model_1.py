# -*- coding: utf-8 -*-
"""
Created on Tue Mar 10 22:09:18 2020

@author: psuba
"""

import numpy as np
import matplotlib.pyplot as plt

def deg_2_rad(deg):
    return (deg*np.pi)/180

def time_of_flight(v_0,theta,g):
    '''
    Returns the time of flight for the whole journey (neglecting air resistance)
    '''
    return ((2*v_0*np.sin(theta))/(-1*g))
    
def get_init_v_0ocity(x,y):
    '''
    Calculates the initial v_0ocity needs to hit a target at X-Y location
    '''
    return np.sqrt(((x**2)*g*-1)/(((2*x*np.sin(theta))-(2*y*np.cos(theta)**2))))

def map_xrojectile_hieght(v_0,theta, g):
    '''
    Returns the max projectile hieght it will reach
    '''
    return ((v_0**2*np.sin(theta)**2)/(-2*g))

def map_xrojectile_dist(v_0,g,theta):
    '''
    Returns the max projectile dist it will reach
    '''
    return ((v_0**2)/(-1*g))*np.sin(2*theta)

def cal_angle_reach(g,d,v_0):
    '''
    Returns the angle of reach required to trav_0 to trav_0 a distance in the X
    
    there are two solutions to this problem:
      theta = 0.5*srcsin(g*d/v^2)
          and
      theta = 90 - 0.5*srcsin(g*d/v^2)
        
    '''
    return [(0.5*np.arcsin((-1*g*d)/v_0**2)),90-(0.5*np.arcsin((-1*g*d)/v_0**2))]
    

#Get distance from wall
x_max = 10 #m
x_min = 0 #m
y_max = 10 #m
y_min = 0 #m


#seeded values
theta = deg_2_rad(30)

######################################
#   STOP FORGETTING THIS IS NEGATIVE
######################################
g= -9.81        #m/s^2
######################################
v_0 = 6.7056    #m/s

#initial parameter
p_x = 0
p_z = 0
v_x = v_0*np.cos(theta) #horzontal speed
v_z = v_0*np.sin(theta) #vertical speed 
max_range = -(2*v_x*v_z)/g
a_x = 0
a_y = g

timesteps = 0.01

data = np.zeros((5,1+len(np.arange(0.01,time_of_flight(v_0,theta,g),
                                       timesteps)))) 
i = 0
for t in np.arange(0.01,time_of_flight(v_0,theta,g),timesteps):
    i += 1 
    data[0][i] = t 
    data[1][i] = v_0*np.cos(theta)+a_x*t
    data[2][i] = v_0*np.sin(theta)+a_y*t
    data[3][i] = p_x + v_0*t*np.cos(theta) + a_x*t**2
    data[4][i] = p_z + v_0*t*np.sin(theta) + 0.5*a_y*t**2

    
fig, ax = plt.subplots()
ax.plot(data[0],data[4],)
ax.grid()
ax.set(xlabel='time (s)', ylabel='destance (m)',
       title='Range Sim y dist')
plt.show()

fig, ax = plt.subplots()
ax.plot(data[0],data[3],)
ax.grid()
ax.set(xlabel='time (s)', ylabel='destance (m)',
       title='Range Sim x dist')
plt.show()

t = time_of_flight(v_0,theta,g)
print('Max time of flight',t)
print('Max vert disp',map_xrojectile_hieght(v_0,theta,g))
print('Max horz disp',map_xrojectile_dist(v_0,g,theta))
print('',get_init_v_0ocity(0.9,0))
print(cal_angle_reach(g,0.9,v_0))