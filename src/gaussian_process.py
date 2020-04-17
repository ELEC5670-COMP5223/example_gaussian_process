#!/usr/bin/env python
"""
	This script provides a plain implementation of gaussian process.
"""
import roslib;

import rospy

from ex_gaussian_process.msg import GPObs
from ex_gaussian_process.srv import *

from numpy import matrix, empty, zeros_like, diag
from numpy.linalg import norm, cholesky
import numpy.random
from math import exp, sqrt

# Gaussian Process object
class GaussianProcess(object):
	# initialize the Gaussian Process
	def __init__(self, mean_fn, kernel_fn, obs_noise):
		self.mean_fn = mean_fn
		self.kernel_fn = kernel_fn
		self.obs_noise = obs_noise
		self.C_N = matrix([[]])
		self.t_vector = []
		self.x_vector = []
		self.dim = 1	# number of components of x
		self.N = 0	# number of points
		self.prediction_service = rospy.Service('/prediction', GPPred,\
				self.handle_pred)
		self.sample_service = rospy.Service('/sample', GPSample,\
				self.handle_sample)
		self.observation_sub = rospy.Subscriber('/observation', GPObs,\
				self.obs_cb)
		rospy.loginfo("Gaussian Process initialized.")


	# observation callback
	def obs_cb(self, msg):
		# update the x and t vectors, the C matrix, and the dimensions
		rospy.loginfo("Got point (%s, %f)."%(str(msg.x), msg.t))
		new_x = matrix(msg.x).T
		if self.N == 0:
			self.dim = len(new_x)
		k_N = [self.kernel_fn(new_x, x) for x in self.x_vector]
		# extending vectors
		self.x_vector.append(new_x)
		self.t_vector.append(msg.t)
		# extending C_N matrix
		new_C_N = empty((self.N+1, self.N+1))
		#if self.N != 0:
		new_C_N[:self.N,:self.N] = self.C_N
		new_C_N[self.N,:self.N] = k_N
		new_C_N[:self.N,self.N] = k_N
		new_C_N[self.N, self.N] = self.kernel_fn(new_x, new_x)+self.obs_noise
		self.C_N = matrix(new_C_N)
		self.N = self.N+1


	# prediction handler
	def handle_pred(self, req):
		# computes the mean and covariance for a new point
		new_x = matrix(req.x).T
		if self.N == 0:
			pred_mean = self.mean_fn(new_x)
			pred_std_dev = self.kernel_fn(new_x, new_x)+self.obs_noise
		else:
			k_N = matrix([self.kernel_fn(new_x, x) for x in self.x_vector]).T
			c = self.kernel_fn(new_x, new_x)+self.obs_noise
			t_N = matrix(self.t_vector).T
			pred_mean = k_N.T * self.C_N.I * t_N
			pred_std_dev = sqrt(c - k_N.T * self.C_N.I * k_N)
		return GPPredResponse(pred_mean, pred_std_dev) # related to service response


	# sample handler
	def handle_sample(self, req):
		# compute a set of samples according to the current observations
		nv = len(req.x)/self.dim
		new_x_vector = matrix(req.x).reshape((nv, self.dim))
		k_N_v = matrix([[self.kernel_fn(x1, x2) for x1 in new_x_vector]\
				for x2 in self.x_vector])
		c_v = matrix([[self.kernel_fn(x1, x2) for x1 in new_x_vector]\
				for x2 in new_x_vector])\
				+ diag([self.obs_noise]*nv)
		t_N = matrix(self.t_vector).T
		mean_new_x = matrix([self.mean_fn(x) for x in new_x_vector]).T
		# compute mean and covariance
		if self.N == 0:
			pred_mean = mean_new_x
			pred_covariance = c_v
		else:
			pred_mean = mean_new_x + k_N_v.T * self.C_N.I * t_N
			pred_covariance = c_v - k_N_v.T * self.C_N.I * k_N_v
		# compute y vector
		A = cholesky(pred_covariance)
		u_v = matrix(numpy.random.normal(size=(nv,))).T
		y_v = pred_mean + A*u_v
		return GPSampleResponse(y_v) # related to service response



# kernel function of the Gaussian process
def gen_kernel(theta1, theta2=0., theta3=0., theta4=0.):
	def kernel_fn(x1, x2):
		return float(theta1*exp(-0.5*theta2*norm(x1-x2)**2) + theta3\
				+theta4*x1.T*x2)
	return kernel_fn


# mean function of the Gaussian process
def mean_fn(x):
	return 0.



# main function
def main():
	rospy.init_node('gaussian_process')
	#gp = GaussianProcess(mean_fn, gen_kernel(0.5, 1.46, 1.0, 1.0), 0.1)
	gp = GaussianProcess(mean_fn, gen_kernel(0.0, 0.0, 0.0, 0.0), 0.1) ## Tune your kernel function here
	rospy.spin()


if __name__ == "__main__":
	main()


