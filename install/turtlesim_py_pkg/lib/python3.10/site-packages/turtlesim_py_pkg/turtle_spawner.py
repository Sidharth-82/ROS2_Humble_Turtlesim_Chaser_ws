#!/usr/bin/env python3
import random
import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from example_interfaces.msg import Bool
from example_interfaces.msg import String

from turtlesim.srv import Kill
from turtlesim.srv import Spawn
from functools import partial


class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        
        self.declare_parameter("max_turtle_count", 1)
        
        self.max_turtle_count_ = self.get_parameter("max_turtle_count").value
        
        self.target_turtle_number_ = 2
        self.total_turtle_counter_ = 0
        self.target_pose_ = Pose()
        self.target_name_ = "turtle" + str(self.target_turtle_number_)
        self.temp_pose_ = Pose()
        self.temp_name_ = ""
        self.target_turtle_pose_subscriber_ = None
        
        
        self.target_turtle_pose_publisher_ = self.create_publisher(
            Pose, "target_turtle_pose", 10)
        self.kill_subscriber_ = self.create_subscription(
            Bool, "killTurtle", self.callback_remove_turtle, 10)
        
        
        
        self.turtle_pose_timer_ = self.create_timer(0.01, self.pose_loop)
        
        self.spawn_turtles()
        
        self.get_logger().info("turtle spawner has been started.")

    def pose_loop(self):
                   
        if self.target_turtle_pose_subscriber_ is not None:
            self.destroy_subscription(self.target_turtle_pose_subscriber_)
            self.target_turtle_pose_subscriber_ = None
        else:
            self.target_turtle_pose_subscriber_ = self.create_subscription(
            Pose, str(self.target_name_)+"/pose", self.callback_target_pose, 10)
            
            
        #self.target_turtle_pose_subscriber_.callback
        
        new_msg = self.target_pose_
        self.target_turtle_pose_publisher_.publish(new_msg)
        
    def callback_target_pose(self, msg):
        self.target_pose_ = msg  
    
    def spawn_turtles(self):
        #while self.total_turtle_counter_ < self.max_turtle_count_:
            self.temp_pose_.x = round(random.uniform(1, 9),4)
            self.temp_pose_.y = round(random.uniform(1, 9),4)
            self.temp_pose_.theta = round(random.uniform(0, 360),4)
            self.temp_pose_.linear_velocity = 0.00
            self.temp_pose_.angular_velocity = 0.00
            self.temp_name_ = "turtle" + str(self.target_turtle_number_+ self.total_turtle_counter_)
            
            client_ = self.create_client(Spawn, "/spawn")
            while not client_.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server spawn_turtle..")
                
                
            request = Spawn.Request()
            request.x = self.temp_pose_.x
            request.y = self.temp_pose_.y
            request.theta = self.temp_pose_.theta
            request.name = self.temp_name_
                
            
            future = client_.call_async(request)
            
            if future.done:
                try: 
                    response = future.result()
                    self.get_logger().info("Spawned new turtle")
                    self.total_turtle_counter_ = self.total_turtle_counter_ + 1
                except Exception as e:
                    self.get_logger().error("Service call failed %r" % (e,))
        #return

        
      
        
    def callback_remove_turtle(self, msg):
        if msg.data:
            
            client_ = self.create_client(Kill, "/kill")
            while not client_.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server kill_turtle...")
                
                
            request = Kill.Request()
            request.name = self.target_name_
            
                
            
            future = client_.call_async(request)
            if future.done:
                try: 
                    response = future.result()
                    self.get_logger().info("killed turtle")
                    self.total_turtle_counter_ = self.total_turtle_counter_ - 1
                    self.target_turtle_number_ = self.target_turtle_number_ + 1
                    self.target_name_ = "turtle" + str(self.target_turtle_number_)
                    self.turtle_pose_timer_.callback
                    self.spawn_turtles()
                    
                    
                except Exception as e:
                    self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()