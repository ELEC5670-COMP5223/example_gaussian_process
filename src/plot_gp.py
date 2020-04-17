#!/usr/bin/env python

import roslib

import rospy
from ex_gaussian_process.srv import *
import matplotlib.pyplot as plt
from math import sqrt
import numpy as np

gp_pred = rospy.ServiceProxy('/prediction', GPPred)

# get data csv 
import rospkg
rospack = rospkg.RosPack()
package_base_path = rospack.get_path('ex_gaussian_process')
import os
datafile = os.path.join(package_base_path, 'data', 'data.csv')

# read ground truth data 
t_d, x_d = zip(*[line.strip().split(',') for line in open(datafile)])
t_d = [float(t) for t in t_d]
x_d = [float(x) for x in x_d]

xs = [-5+i*0.05 for i in range(261)]

# call the prediction services for inference
means,std_devs = zip(*[(pred.mean, pred.std_dev) for pred in (gp_pred([x]) for x in xs)])
means_pos, means_neg = zip(*[(m+2*s,m-2*s) for m,s in zip(means, std_devs)])

plt.plot(xs, means, 'r-', xs, means_pos, 'g-', xs, means_neg, 'g-', x_d, t_d, 'kD')
plt.fill_between(xs, means_neg, means_pos, color='0.7')
plt.show()

