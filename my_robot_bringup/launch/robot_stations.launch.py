from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    remap_topics = {
    "giskard": ("robot_news_station", "robot_news_station_giskard"),
    "bb8": ("robot_news_station", "robot_news_station_bb8"),
    "daniel": ("robot_news_station", "robot_news_station_daniel"),
    "jander": ("robot_news_station", "robot_news_station_jander"),
    "c3po": ("robot_news_station", "robot_news_station_c3po")
    }
    
    robots_name = ["giskard", "bb8", "daniel", "jander", "c3po"]
    robot_news_station_nodes = []

    for name in  robots_name:
        robot_news_station_nodes.append(Node(
            package="my_python_package",
            executable="robot_news_station",
            name="robot_news_station_" + name,  # Changed curly braces to concatenate strings
            remappings=[
                remap_topics[name] # Corrected the remappings syntax
            ],
            parameters=[
                {"robot_name": name}  # Using the current robot name variable
            ]
        ))

    smartphone_node = Node(
        package="my_python_package",  
        executable="smartphone"   
    )

    for node in robot_news_station_nodes:
        ld.add_action(node)
    ld.add_action(smartphone_node)
    return ld
