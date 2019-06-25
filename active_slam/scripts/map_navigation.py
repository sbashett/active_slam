import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from move_base_srv.srv import *
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from nav_msgs.srv import *
from std_msgs.msg import *
from tf2_msgs.msg import TFMessage
import numpy as np
  
class MapNavigation():

  def __init__(self):

    self.prev_init_local_pose = Pose()

    # initiliaze
    rospy.init_node('active_slam_params', anonymous=False)

    rospy.Subscriber("map", OccupancyGrid, self.map_callback)
    rospy.Subscriber("move_base/global_costmap/costmap", OccupancyGrid, self.global_costmap_callback)
    rospy.Subscriber("tf", TFMessage, self.pose_callback)
    rospy.Subscriber("/move_base/DWAPlannerROS/local_plan", Path, self.track_local_planner)
    rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.track_global_planner)
    self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    # define a client for to send goal requests to the move_base server through a SimpleActionClient
    self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    # wait for the action server to come up
    while(not self.ac.wait_for_server(rospy.Duration.from_sec(5.0))):
      rospy.loginfo("Waiting for the move_base action server to come up")

    # define a client handle to move_base/make_plan service to check if plan is feasible
    rospy.wait_for_service('/move_base/make_plan')
    try:
        self.make_plan_client = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    self.goal = MoveBaseGoal()

  def map_callback(self,data):
    self.map_meta = MapMetaData()
    self.map_header = Header() 
    self.map_meta = data.info
    self.map_header = data.header
    self.map_grid_vals = np.array(data.data)
    
  def global_costmap_callback(self,data):
    self.global_costmap_meta = MapMetaData()
    self.global_costmap_header = Header()
    self.global_costmap_meta = data.info
    self.global_costmap_header = data.header
    self.global_costmap_probs = np.array(data.data)

  def pose_callback(self,data):
    if data.transforms[-1].header.frame_id == "map":
        if data.transforms[-1].child_frame_id == "odom":
            self.robo_position = Point()
            self.robo_position.x = data.transforms[-1].transform.translation.x
            self.robo_position.y = data.transforms[-1].transform.translation.y

            self.robo_orient = Quaternion()
            self.robo_orient.x = data.transforms[-1].transform.rotation.x
            self.robo_orient.y = data.transforms[-1].transform.rotation.y
            self.robo_orient.z = data.transforms[-1].transform.rotation.z
            self.robo_orient.w = data.transforms[-1].transform.rotation.w


  def track_local_planner(self,path):
    self.prev_init_local_pose = path.poses[0].pose
  
  def track_global_planner(self,path):
    # print("made new global plan")
    pass

  def moveToGoal(self,xGoal,yGoal):

    #set up the frame parameters
    self.goal.target_pose.header.frame_id = "map"
    self.goal.target_pose.header.stamp = rospy.Time.now()

    # moving towards the goal*
    self.goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
    self.goal.target_pose.pose.orientation.x = 0.0
    self.goal.target_pose.pose.orientation.y = 0.0
    self.goal.target_pose.pose.orientation.z = 0.0
    self.goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo("Sending goal location ...")
    self.ac.send_goal(self.goal)

    self.ac.wait_for_result(rospy.Duration(40))

    while( (self.ac.get_state() != GoalStatus.ABORTED) and (self.ac.get_state() != GoalStatus.REJECTED) and (self.ac.get_state() != GoalStatus.PREEMPTING) and (self.ac.get_state() != GoalStatus.SUCCEEDED)):
        # print("2 robot current position:", self.robo_position)
        continue

    if self.ac.get_state() == GoalStatus.SUCCEEDED:
        rospy.loginfo("You have reached the destination")
        return True

    if self.ac.get_state() == GoalStatus.ABORTED:
        rospy.loginfo("The robot failed to reach the destination")

    return False

