from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    turtle_sim_node = Node(
        package= "turtlesim",
        executable= "turtlesim_node"
    )
    
    turtle_spawner_node = Node(
        package= "turtlesim_py_pkg",
        executable= "turtle_spawner",
        parameters= [
            {"max_turtle_count": 1}            
        ]
    )
    
    

    turtle_controller_node = Node(
        package= "turtlesim_py_pkg",
        executable= "turtle_controller",
        parameters= [
            {"k_p_l": 2.00},
            {"k_p_a": 6},
            {"control_loop_time" : 0.01},
            {"check_loop_time": 0.75}
        ]
    )
    
    ld.add_action(turtle_sim_node)
    ld.add_action(turtle_controller_node)
    
    ld.add_action(turtle_spawner_node)
    
    
    
    return ld
    
    

