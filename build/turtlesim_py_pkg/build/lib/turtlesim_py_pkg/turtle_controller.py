#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from example_interfaces.msg import Bool

class TurtleControllerNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("turtle_controller") # MODIFY NAME
        
        self.declare_parameter("k_p_l", 0.25)
        self.declare_parameter("k_p_a", 1)
        self.declare_parameter("control_loop_time", 0.01)
        self.declare_parameter("check_loop_time", 1.00)
        
      
        
        self.turtle_subscriber_ = self.create_subscription(
            Pose, "turtle1/pose", self.callback_player_pose, 10)
        
        self.player_pose_ = None
        
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        
        self.dest_turtle_subscriber_ = self.create_subscription(
            Pose, "target_turtle_pose", self.callback_target_pose,10)
        
        self.target_pose_ = None
        
        self.target_hit_publisher_ = self.create_publisher(Bool, "killTurtle", 10)
        
        #Proportional speed control
        self.error_x = 0.00
        self.error_y = 0.00
        self.error_angular = 0.00
        self.k_p_linear = self.get_parameter("k_p_l").value
        self.k_p_angular = self.get_parameter("k_p_a").value
        
        
        self.control_loop_timer_ = self.create_timer(self.get_parameter("control_loop_time").value,self.control_loop)
        self.check_loop_timer_ = self.create_timer(self.get_parameter("check_loop_time").value, self.check_loop)
        
        self.get_logger().info("turtle controller has been started.")
        
    def callback_player_pose(self, msg):
        self.player_pose_= msg
        
    def callback_target_pose(self, msg):
        self.target_pose_= msg
    
    def control_loop(self):
        
        if self.player_pose_ == None or self.target_pose_ == None :
            return
        
        self.error_x = self.target_pose_.x-self.player_pose_.x
        self.error_y = self.target_pose_.y-self.player_pose_.y
        
        error_linear = math.sqrt(self.error_x*self.error_x + self.error_y*self.error_y)
        
        target_theta = math.atan2(self.error_y,self.error_x)
        
        self.error_angular = target_theta - self.player_pose_.theta
        
        if self.error_angular >= math.pi:
            self.error_angular -= 2*math.pi
        elif self.error_angular < -math.pi:
            self.error_angular += 2*math.pi
        
        
        
        linear_velocity_x_ = self.k_p_linear*error_linear
        
        
        
        angular_velocity_ = self.k_p_angular * self.error_angular
        
        vel_msg = Twist()
        vel_msg.linear.x = linear_velocity_x_
        vel_msg.linear.y = 0.00
        vel_msg.linear.z = 0.00
        vel_msg.angular.x = 0.00
        vel_msg.angular.y = 0.00
        vel_msg.angular.z = angular_velocity_
        self.cmd_vel_publisher_.publish(vel_msg)
        
    def check_loop(self):
        if self.player_pose_ == None or self.target_pose_ == None :
            return
        if abs(self.target_pose_.x-self.player_pose_.x) <= 0.1 and abs(self.target_pose_.y-self.player_pose_.y) <= 0.1:
            kill_msg = Bool()
            kill_msg.data = True
            self.target_hit_publisher_.publish(kill_msg)
            self.target_pose_ = None
        
        

            
        

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
