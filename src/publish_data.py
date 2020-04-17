#!/usr/bin/env python

import roslib

import rospy
from ex_gaussian_process.msg import GPObs
import rospkg

import sys
import time

def main():
	rospy.init_node('data_publisher')
	data_pub = rospy.Publisher('/observation', GPObs, queue_size=20)
	
	# one could use python argument to provide a data path. Or by default serch inside the ros package
	if len(sys.argv) > 1:
		datafile = open(sys.argv[1])
	else:
		rospack = rospkg.RosPack()
		package_base_path = rospack.get_path('ex_gaussian_process')
		import os
		datafile = open(os.path.join(package_base_path, 'data', 'data.csv'))
	for line in datafile:
		values = [float(token.strip()) for token in line.split(',')]
		t = values[0]
		x = values[1:]
		time.sleep(0.1)
		rospy.loginfo("Publishing (%s, %f)."%(str(x), t))
		data_pub.publish(GPObs(x, t))


if __name__ == "__main__":
	main()


