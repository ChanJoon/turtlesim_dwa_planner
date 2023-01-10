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
    self.max_speed = 1.5  # [m/s]
    self.min_speed = -0.5  # [m/s]
    self.max_yaw_rate = 40.0 * math.pi / 180.0  # [rad/s]
    self.max_accel = 5.0  # [m/ss]
    self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # [rad/ss]
    self.v_resolution = 0.2  # [m/s]
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
    self.ob_sub = rospy.Subscriber("turtle1/Pose", Pose, self.ob_callback)
    self.pose_sub = rospy.Subscriber("turtle2/pose", Pose, self.pose_callback)
    self.vel_pub = rospy.Publisher("turtle2/cmd_vel", Twist, queue_size=10)
    self.rate = rospy.Rate(10)
    
    self.config = Config()
    self.x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
    self.trajectory = np.array(self.x)
    self.turtle2_pose = Pose()
    self.isDone = False
    self.i = 0
  
    
  def ob_callback(self, msg):
    self.config.ob = np.array([msg.x, msg.y])
    
  def pose_callback(self, msg):
    self.turtle2_pose = msg
    self.init_x = self.turtle2_pose.x
    self.init_y = self.turtle2_pose.y
    self.init_theta = self.turtle2_pose.theta
    self.init_v = self.turtle2_pose.linear_velocity
    self.init_omega = self.turtle2_pose.angular_velocity
  
    self.main()
  
  def main(self):
    if self.i == len(self.config.waypoints):
      self.isDone = True
    if self.isDone is True:
      rospy.signal_shutdown("done")
      return
      
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    self.x = np.array([self.init_x, self.init_y, self.init_theta, self.init_v, self.init_omega])

    # input [forward speed, yaw_rate]
    msg = Twist()

    ob = self.config.ob

    # goal position [x(m), y(m)]
    goal = self.config.waypoints[self.i]
    rospy.loginfo("\nCurrent pose x: %.2f, y: %.2f", self.x[0], self.x[1])
    print("Goal    pose x: {}, y: {}".format(goal[0], goal[1]))
    u, predicted_trajectory = dwa.dwa_control(self.x, self.config, goal, ob)
    # x = dwa.motion(x, u, self.config.dt)  # simulate robot
    self.trajectory = np.vstack((self.trajectory, self.x))  # store state history
  
    msg.linear.x = u[0]
    msg.angular.z = u[1]
    print("\nlinear\n   x: {}\nangular\n   z: {}\n".format(msg.linear.x, msg.linear.z))
    self.vel_pub.publish(msg)

    # check reaching goal
    dist_to_goal = math.hypot(self.x[0] - goal[0], self.x[1] - goal[1])
    if dist_to_goal <= self.config.robot_radius:
      print("----------[ Goal ]----------")
      self.i += 1

if __name__ == '__main__':
  try:
    t = Turtle()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass