# ACTIVE SLAM AND PATH PLANNING FOR ROBOT NAVIGATION
reference paper link: https://ieeexplore.ieee.org/document/7125079

This is a simplified implementation and improvement of the active slam application from the reference paper.

Code Description:

The environment used is ROS interface along with gazebo simualator. This is an implementation of an active slam algorithm which autonomously navigates to new goal locations and and explores the area to form a map of the environment.

### Prerequisites - Tools and packages:
python2.7
Ros-kinetic
packages for ROS: tuertlebot3_simulations, turtlebot3_msgs, turtlebot3, gazebo_ros and slam_gmapping. Please refer to the ros wiki for more installation instructions on setting the ros for using gazebo turtlebot3 simulator for navigation.

### Steps to Run the Code:
1. Setup a workspace (create folder catkin_ws or with any name) and create the folder 'src' in ```~/catkin_ws ``` workspace.
2. initialize the workspace using the command ```catkin_init_workspace``` from command line in the folder ```~/catkin_ws```.
3. copy the ```active_slam``` folder from the submission and paste it in ```~/catkin_ws/src``` folder.
4. Run the command ```catkin_make``` from the ```~/catkin_ws``` location. The files should now be compiled and messages should be generated.
5. Source the workspace by running the command ```source devel/setup.bash``` from ```~catkin_ws``` location.
6. Run the following commands each in a different terminal/console from the location ```~/catkin_ws```
	
	* Command 1 in terminal 1:
		```roslaunch turtlebot3_gazebo turtlebot3_world.launch```

	* Command 2 in terminal 2:
		```roslaunch active_slam active_slam.launch```
	
	**NOTE** : BEFORE EXECUTING COMMAND 3, we need to setup the initial approximate grid position of the bot by trail and error method.
	
	* Command 3 in terminal 3:
		```python ./src/active_slam/scripts/global_planning.py```

Now you should see the code running with some intermediate outputs printed on console. Visit roswiki to change the parameters mentioned in the report.

