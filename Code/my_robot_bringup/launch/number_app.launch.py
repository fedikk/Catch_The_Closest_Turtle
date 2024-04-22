from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    remap_number_topic = ("number","my_number")
    
    number_publisher_node = Node(
        package="my_python_package" ,     # here we put the package name
        executable="number_publisher",    # here we put the executable name
        name="my_number_publisher"  ,     # here we put the name of the node 
        remappings= [
            remap_number_topic            # here we are going to change the topic name from "number" to "my_number"
        ],
        parameters= [                     # here we give the parameteres   
            {"number_to_publish":1},
            {"publish_time":5}
        ]
    )   
    counter_node = Node(
        package="my_cpp_pkg" ,  
        executable="number_counter",   
        name="my_number_counter",
        remappings= [
            remap_number_topic,
            ("number_count","my_number_count")      
        ]
    )

    ld.add_action(number_publisher_node)
    ld.add_action(counter_node)
    return ld