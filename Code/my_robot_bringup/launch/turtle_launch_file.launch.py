from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_spawner_node = Node(
        package="turtlesim",  
        executable="turtlesim_node"   
    )
    enemy_turtle_spawner_node = Node(
            package="turtlesim_catch_them_all",
            executable="turtle_spawner",
            name="turtle_spawner",
            parameters=[
                {"spawn_frequency":1.0},
                {"turtle_name_prefix":"Enemy_turtle_"}
            ]
        )
    turtle_controller_node = Node(
            package="turtlesim_catch_them_all",
            executable="turtle_controller",
            name="turtle_controller",
            parameters=[
                {"catch_closest_turtle_first":True}
            ]
        )

    

    
    ld.add_action(turtlesim_spawner_node)
    ld.add_action(enemy_turtle_spawner_node)
    ld.add_action(turtle_controller_node)


    return ld