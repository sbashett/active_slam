import numpy as np
from map_navigation import *
import math
import random
LOW = 0
HIGH = 1
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

import time

class GoalPlanner():
 	
 	def __init__(self, nav_obj):
	  	self.nav_obj = nav_obj
 		self.map_width = nav_obj.map_meta.width
 		self.map_height = nav_obj.map_meta.height
		self.hidden = np.zeros((self.map_width*self.map_height), dtype=bool)
		self.frontier = np.zeros((self.map_width*self.map_height), dtype=bool)
		self.on_stack = np.zeros((self.map_width*self.map_height), dtype=bool)
		self.visited = np.zeros((self.map_width*self.map_height), dtype=bool)
		self.laser_range = rospy.get_param("/turtlebot3_slam_gmapping/maxUrange")
		self.goali = 0
		self.goalj = 0

		# world
		# self.prev_goali = 200
		# self.prev_goalj = 170
		# house
		# self.prev_goali = 210
		# self.prev_goalj = 140

		# map resolution 0.075
		# world
		self.prev_goali = 122
		self.prev_goalj = 100
		# house
		# self.prev_goali = 135
		# self.prev_goalj = 80

		# map res 0.06
		# world
		# self.prev_goali = 160
		# self.prev_goalj = 135


	def q2e_angle(self,w, x, y, z):

		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		
		X = math.atan2(t0, t1)
		t2 = +2.0 * (w * y - z * x)
		if t2 > +1.0 : 
			t2 = +1.0
		elif t2 < -1.0: 
			t2 = -1.0

		Y = math.asin(t2)
		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)

		Z = math.atan2(t3, t4)
		return X, Y, Z

	def distToState(self,i,j):

		x1,y1 = self.getMapCoordinates(i,j)

		x2,y2 = self.getMapCoordinates(self.prev_goali, self.prev_goalj)

		dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

		return dist


	def getMapCoordinates(self,gridi,gridj):
	# Since we set the origin as 0,0 with respect to the map co-ordinates 

		orign_x = int(self.map_height/2)
		orign_y = int(self.map_width/2)

		map_x = (orign_x - gridi)*self.nav_obj.map_meta.resolution
		map_y = (orign_y - gridj)*self.nav_obj.map_meta.resolution

		return map_x,map_y

	def get_angle(self,slope1,slope2):

		cal = (slope1-slope2)/(1+(slope1*slope2))
		rad = math.atan(cal)
		deg = math.degrees(rad)

		return deg

	def ishidden(self,pos_x,pos_y):

		x1 = self.nav_obj.robo_position.x
		y1 = self.nav_obj.robo_position.y

		x2,y2 = self.getMapCoordinates(pos_x,pos_y)

		# Getting slope of the line from the position to the gap 
		if y1 != y2:
			s1 = (x2 - x1)/(y2 - y1)
		else:
			s1 = math.tan(3.14/2)

		dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

		w = self.nav_obj.robo_orient.w
		x = self.nav_obj.robo_orient.x
		y = self.nav_obj.robo_orient.y
		z = self.nav_obj.robo_orient.z
		# Orientation of the robot
		roll,pitch,yaw = self.q2e_angle(w,x,y,z)

		s2 = math.tan(yaw)

		deg = self.get_angle(s1,s2)

		# check if the grid is within sensor range
		if (deg <= 90 or deg >=-90) and (dist < self.laser_range/2):
			return True
		else:
			return False

	def setFrontierGoal(self):

		nearest_fro_dist = float('Inf')
		nearest_fro_i = 0
		nearest_fro_j = 0
		for i in range(self.map_height):
			for j in range(self.map_width):
				index = i*self.map_width + j

				if self.frontier[index] == True:
					dist = self.distToState(i,j)

					if dist < nearest_fro_dist:

						nearest_fro_dist = dist
						nearest_fro_i = i
						nearest_fro_j = j

		self.goali = nearest_fro_i
		self.goalj = nearest_fro_j				 

	def fill_stack(self):

		rospy.loginfo("entered fill stack")
		total_grid_count = 0

		# loop over all the grid locations
		for i in range(self.map_height):
			for j in range(self.map_width):
				
				index = i*self.map_width + j
				total_grid_count += 1

				if self.nav_obj.map_grid_vals[index] != 0 or self.nav_obj.map_grid_vals[index] != 100:
					# the grid is a gap
					# if self.nav_obj.map_grid_vals[index] == -1:
					# 	unknown += 1

					# if gap not on stack
					if self.on_stack[index] == False:
						
						# check if gap is hidden or frontier
						val = self.ishidden(i,j) 
						if val == True:
							# set it to hidden state
							self.hidden[index] = True
						else:
							# set it to frontier state
							self.frontier[index] = True

						# add it to the stack
						self.on_stack[index] = True

		rospy.loginfo("total_grid_count: %d" %(total_grid_count))

	def check_path(self, i,j):
		srv = GetPlan()

		start = PoseStamped()
		goal = PoseStamped()

		prev_gx,prev_gy = self.getMapCoordinates(self.prev_goali, self.prev_goalj)

		start.header.seq = 0
		start.header.stamp = rospy.Time.now()
		start.header.frame_id = "map"
		start.pose.position.x = prev_gx
		start.pose.position.y = prev_gy
		start.pose.position.z = 0.0
		start.pose.orientation.x = 0
		start.pose.orientation.y = 0
		start.pose.orientation.z = 0
		start.pose.orientation.w = 1

		gx,gy = self.getMapCoordinates(i, j)

		goal.header.seq = 0
		goal.header.stamp = rospy.Time.now()
		goal.header.frame_id = "map"
		goal.pose.position.x = gx
		goal.pose.position.y = gy
		goal.pose.position.z = 0.0
		goal.pose.orientation.x = 0.0
		goal.pose.orientation.y = 0.0
		goal.pose.orientation.z = 0.0
		goal.pose.orientation.w = 1.0

		tolerance = 0.1

		plan = self.nav_obj.make_plan_client(start, goal, tolerance)

		if len(plan.plan.poses) == 0:
			rospy.loginfo("path to location:(%d,%d) not feasible" % (i, j))
			self.remove_from_stack(i*self.map_width + j)
			self.remove_from_stack((i+1)*self.map_width + j)
			self.remove_from_stack(i*self.map_width + j+1)
			self.remove_from_stack((i+1)*self.map_width + j+1)
			self.remove_from_stack((i-1)*self.map_width + j)
			self.remove_from_stack((i-1)*self.map_width + j-1)
			return False, None
		else:
			return True, len(plan.plan.poses)
		
	def set_current_goal(self):

		lower_poses_len_thresh = 10
		upper_poses_len_thresh = 30
		
		# checking if there are any goals on stack
		if np.sum(self.on_stack)  != 0:

			# check for hidden gaps
			if np.sum(self.hidden) > 0:

				rospy.loginfo("number of hidden states left: %d" %(np.sum(self.hidden)))
				backup_i = 0
				backup_j = 0

				for i in range(self.map_height):
					for j in range(self.map_width):
						index = i*self.map_width + j

						if self.hidden[index] == True and self.on_stack[index] == True:

							# check if global path to the goal is available	
							is_path_avail, poses_len = self.check_path(i,j)

							if is_path_avail == False:
								continue

							if (poses_len > lower_poses_len_thresh) and (poses_len < upper_poses_len_thresh):
								self.goali = i
								self.goalj = j

								rospy.loginfo("hidden goal states: %d,%d" % (self.goali, self.goalj))
								return True
							else:
								backup_i = i
								backup_j = j

				self.goali = backup_i
				self.goalj = backup_j

				rospy.loginfo("hidden goal states: %d,%d" % (self.goali, self.goalj))
				return True

			# check for frontier gaps
			else:
				''' select next goal form frontier points '''
				self.setFrontierGoal() 
				rospy.loginfo("frontier goal states: %d,%d" % (self.goali, self.goalj))
				return True

		# no goals on stack
		else:
			return False


	def mapCoverage(self,pos_y,pos_x):

		sum_coverage = 0
		
		for y in max(-int(self.map_height/2),(pos_y - 2)),min((pos_y+2),int(self.map_height/2)) : 
			for x in max(-int(self.map_width/2),(pos_x - 2)),min((pos_x+2),int(self.map_width/2)):
				index = y*self.map_width + x

				if self.nav_obj.map_grid_vals[index] == -1:
					sum_coverage += 1

		if sum_coverage > 3: 
			return LOW
		else:
			return HIGH


	def update_on_stack(self):
		# loop over all the grid locations
		for i in range(self.map_height):
			for j in range(self.map_width):
				
				index = i*self.map_width + j

				if (self.nav_obj.map_grid_vals[index] == 0 or self.nav_obj.map_grid_vals[index] == 100) and self.on_stack[index] == True :
					# print("removing from stack:", i, j)
					self.remove_from_stack(index)
				elif self.on_stack[index] == True:
					# if the unknown grid is on stack change between hidden and frontier if necessary					
					if self.ishidden(i,j) == True:
						self.hidden[index] = True
						self.frontier[index] = False
					else:
						self.hidden[index] = False
						self.frontier[index] = True


	def remove_from_stack(self, goalIndex):
		# print("removing goal index from stack", goalIndex)
		self.visited[goalIndex] = False
		self.hidden[goalIndex] = False
		self.frontier[goalIndex] = False
		self.on_stack[goalIndex] = False

	def move_to_prev_goal(self):
		(xGoal,yGoal) = self.getMapCoordinates(self.prev_goali,self.prev_goalj)		
		# result, pose_len = self.check_path()
		rospy.loginfo("backtracking to previous goal: %f ,%f" % (self.prev_goali, self.prev_goalj))

		result = self.nav_obj.moveToGoal(xGoal,yGoal)

	def rotate_nsec(self, n):
		twist = Twist()
		twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = BURGER_MAX_ANG_VEL
		self.nav_obj.vel_pub.publish(twist)
		rospy.sleep(n)
		twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0
		self.nav_obj.vel_pub.publish(twist)

	def move(self):
		''' send goal position '''

		goalIndex = self.goali * self.map_width + self.goalj
		(xGoal,yGoal) = self.getMapCoordinates(self.goali,self.goalj)

		rospy.loginfo("values of goalIndex in visited arraybefore move:%s" % (self.visited[goalIndex]))

		result = self.nav_obj.moveToGoal(xGoal,yGoal)

		self.rotate_nsec(5)

		# update the on_stack list after movement
		self.update_on_stack()

		# goal position reached
		if result == True:

			self.prev_goali = self.goali
			self.prev_goalj = self.goalj

			coverage = self.mapCoverage(self.goali,self.goalj) 
			if coverage == LOW and self.visited[goalIndex] == False:
				# keep the goals to be explored in the end
				rospy.loginfo("marking visited since low coverage: (%d,%d)" %(self.goali, self.goalj))
				self.visited[goalIndex] = True
				self.hidden[goalIndex] = False
				self.frontier[goalIndex] = True
				self.on_stack[goalIndex] = True
				return
			# goal need not be explored again						
			else:
				rospy.loginfo("reached goal and removing from stack: (%d,%d)" %(self.goali, self.goalj))
				self.remove_from_stack(goalIndex)
				return

		elif self.visited[goalIndex] == False:
			# try for second time
			rospy.loginfo("couldnt reach goal and marking for second time: (%d,%d)" %(self.goali, self.goalj))
			self.visited[goalIndex] = True
			self.hidden[goalIndex] = True
			self.frontier[goalIndex] = False
			self.on_stack[goalIndex] = True

			rospy.loginfo("goalIndex: %d, %d" % (goalIndex, (self.goali*self.map_width + self.goalj)))
			rospy.loginfo(self.visited[self.goali*self.map_width + self.goalj])
			self.move_to_prev_goal()
			return
		
		else:
			# tried for two times but couldnt reach the goal remove goal from stack
			rospy.loginfo("couldnt reach goal for second time; removing from stack: (%d,%d)" %(self.goali, self.goalj))
			self.remove_from_stack(goalIndex)
			self.remove_from_stack(goalIndex+1)
			self.remove_from_stack(goalIndex-1)
			self.remove_from_stack(goalIndex+self.map_width)
			self.remove_from_stack(goalIndex-self.map_width)
			self.move_to_prev_goal()			
			return


if __name__ == '__main__':
	try:
		nav_obj = MapNavigation()
		planner_obj = GoalPlanner(nav_obj)
		planner_obj.fill_stack()

		while(1):
			next_goal = planner_obj.set_current_goal()
			if next_goal == False:
				print("no more goal states")
				break
			planner_obj.move()
			print("value in visited array after move: %s" %(planner_obj.visited[planner_obj.goali*planner_obj.map_width + planner_obj.goalj]))

		print("exploration complete press ctrl^c to exit nodes")
		rospy.spin()

	except rospy.ROSInterruptException:
		rospy.loginfo("active_slam_params node terminated.")
