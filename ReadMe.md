# Example - Gaussian Process

This is the example code for Gaussian Process learning and inferencing.

## Setup

This is a complete stand-alone ROS package. We have tested this package in Ubuntu 18.04, ROS-Melodic and it should work on other recent version of ROS too.

1. [Install ROS](http://wiki.ros.org/ROS/Installation) in your machine. 
2. Set up a [workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
3. Unzip the files so that "ex_gaussian_process" is under the "src" folder of your workspace. [Example structure](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
4. [Build the package](http://wiki.ros.org/ROS/Tutorials/BuildingPackages) under the workspace folder. Simply "catkin_make" should work.
5. Update the python path.
```bash
source devel/setup.bash
```

## Usage

After being sure that the python path is correct for each terminal and **get roscore running**.

Start the gaussian process node with:
```
rosrun ex_gaussian_process gaussian_process.py
```

Publish data with:
```
rosrun ex_gaussian_process publish_data.py
```

Test the inference and visualize results with:
```
rosrun ex_gaussian_process plot_gp.py
```

Try to tune the kernel function in "gaussian_process.py" to get a nice fit to the data.

## What you could learn

1. The learning and inference process are presented in plain Python in "gaussian_process.py".
2. Get familiar with ROS concepts of nodes, topics, services, messages and so on.
