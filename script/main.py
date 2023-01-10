#!/usr/bin/env python
import math
import rospy
import roslib
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import dynamic_window_approach as dwa

class Config:
  def __init__(self):
    self.max_speed = 3.0  # [m/s]
    self.min_speed = -0.5  # [m/s]
    self.max_yaw_rate = 90.0 * math.pi / 180.0  # [rad/s]
    self.max_accel = 5.0  # [m/ss]
    self.max_delta_yaw_rate = 90.0 * math.pi / 180.0  # [rad/ss]
    self.v_resolution = 0.3  # [m/s]
    self.yaw_rate_resolution = 0.5 * math.pi / 180.0  # [rad/s]
    self.dt = 0.1  # [s] Time tick for motion prediction
    self.predict_time = 1.0  # [s]
    self.to_goal_cost_gain = 0.15
    self.speed_cost_gain = 1.0
    self.obstacle_cost_gain = 1.0
    self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
    
    self.robot_radius = 1.0  # [m] for collision check
    self.ob = np.empty((1,2))
    # waypoints
    self.waypoints = np.array([[8.58, 2.5],
                              [2.5, 8.58],
                              [2.5, 2.5],
                              [8.58, 8.58]
                              ])
        
class Turtle:
  def __init__(self):
    rospy.init_node('main', anonymous=True)
    self.ob_sub = rospy.Subscriber("turtle1/pose", Pose, self.ob_callback)
    self.ob_pub = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=1)
    self.pose_sub = rospy.Subscriber("turtle2/pose", Pose, self.pose_callback)
    self.vel_pub = rospy.Publisher("turtle2/cmd_vel", Twist, queue_size=1)
    self.rate = rospy.Rate(10)
    
    self.config = Config()
    self.x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
    self.turtle2_pose = Pose()
    self.goalIndex = 0
  
    
  def ob_callback(self, msg):
    self.config.ob = np.array([[msg.x, msg.y]])
    
  def pose_callback(self, msg):
    self.turtle2_pose = msg
    self.turtle2_x = self.turtle2_pose.x
    self.turtle2_y = self.turtle2_pose.y
    self.turtle2_theta = self.turtle2_pose.theta
    self.turtle2_v = self.turtle2_pose.linear_velocity
    self.turtle2_omega = self.turtle2_pose.angular_velocity
  
    self.main()
  
  def dyn_obstacle(self, forward_speed=0.5, yaw_rate=0.5):
    # dynamic obstacle setting for circular motion
    ob_vel = Twist()
    ob_vel.linear.x = forward_speed
    ob_vel.angular.z = yaw_rate
    self.ob_pub.publish(ob_vel)
    
  
  def main(self):
    
    # obstacles [x(m) y(m), ....]
    self.dyn_obstacle(forward_speed=1.0, yaw_rate=0.8)
    ob = self.config.ob

    # goal position [x(m), y(m)]
    if self.goalIndex == len(self.config.waypoints):
      self.goalIndex = 0
    goal = self.config.waypoints[self.goalIndex]
      
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    self.x = np.array([self.turtle2_x, self.turtle2_y, self.turtle2_theta, self.turtle2_v, self.turtle2_omega])

    # input [forward speed, yaw_rate]
    u, _ = dwa.dwa_control(self.x, self.config, goal, ob)
    
    msg = Twist()
    msg.linear.x = u[0]
    msg.angular.z = u[1]
    
    # publish cmd_vel to turtle2
    self.vel_pub.publish(msg)
    
    rospy.loginfo("\nCurrent pose x: %.2f, y: %.2f", self.x[0], self.x[1])
    print("Goal    pose x: {}, y: {}".format(goal[0], goal[1]))
    print("\nlinear\n   x: {}\nangular\n   z: {}\n".format(msg.linear.x, msg.angular.z))

    # check reaching goal
    dist_to_goal = math.hypot(self.x[0] - goal[0], self.x[1] - goal[1])
    if dist_to_goal <= self.config.robot_radius:
      print("----------[ Goal ]----------")
      self.goalIndex += 1

if __name__ == '__main__':
  try:
    t = Turtle()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass