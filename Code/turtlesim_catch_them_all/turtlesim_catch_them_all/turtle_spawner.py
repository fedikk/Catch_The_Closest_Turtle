#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
import random 

from example_interfaces.srv import AddTwoInts

from turtlesim.srv import Spawn
from turtlesim.srv import Kill


from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray 
from my_robot_interfaces.srv import CatchTurtle

from functools import partial

class TurtleSpawnerClientNode(Node): 
    
    def __init__(self):
        super().__init__("turtle_spawner_client") 

        self.declare_parameter("spawn_frequency",2.0)
        self.declare_parameter("turtle_name_prefix","Enemy_turtle_")


        self.turtle_counter_ = 0
        self.alive_turtles_ = []


        self.turtle_name_prefix = self.get_parameter("turtle_name_prefix").value
        self.turtle_spawn_frequency_ = self.get_parameter("spawn_frequency").value
        
        
        
        
        self.spawn_turtle_timer = self.create_timer(
            self.turtle_spawn_frequency_, self.spawn_new_turtle)

        #Publishing an array off alive turtles (name + coordinates) on the /alive_turtles topic
        self.alive_turtles_publisher_ = self.create_publisher (
            TurtleArray,"alive_turtles",10)    

        #kill the cought turtle
        self_kill_turtle_cought_ = self.create_service(CatchTurtle,"catch_turtle",self.callback_catch_turtle)


        # self.publish_alive_turtle_timer_ = self.create_timer(self.time_,self.publish_alive_turtles)
        self.get_logger().info("Turtle spawner client has been started !!!")
    
    

    def callback_catch_turtle(self,request,response):
        self.call_kill_server(request.name)
        response.success = True
        return response
    

    def call_kill_server(self,name):
        client = self.create_client(Kill,"kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for add two ints server !!!")
        
        request = Kill.Request()
        request.name = name 
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_kill_turtle,name=name))



      # # the callback is made to hundle the response from the server 
    def callback_kill_turtle(self, future,name):
        try:
            future.result()
            for (i, turtle ) in enumerate(self.alive_turtles_):
                if(turtle.name == name ) : 
                    del self.alive_turtles_[i]           # self.alive_turtles_.remove(self.alive_turtles_[i])
                    self.publish_alive_turtles()
                    break                                      
            self.get_logger().info( name + " is Dead !!")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))



    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtles_publisher_.publish(msg)




    def spawn_new_turtle(self):
        self.turtle_counter_ +=1
        name = self.turtle_name_prefix + str(self.turtle_counter_)
        spawn_x = random.uniform(0.0,11.0)
        spawn_y = random.uniform(0.0,11.0)
        theta= random.uniform(0.0, 2 * math.pi)
        self.call_spawn_turtle(spawn_x,spawn_y,theta,name)
 
   
    def call_spawn_turtle(self,x,y,theta,name):
        client = self.create_client(Spawn,"spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for add two ints server !!!")
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name 
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_spawn_turtle,x=x,y=y,theta=theta,name=name))
    
    # # the callback is made to hundle the response from the server 
    def callback_spawn_turtle(self, future, x, y,theta,name):
        try:
            response = future.result()
            if response.name != "":
                response.name = name
                self.get_logger().info("turtle " + str(response.name) + " is been alive !!!!!!" )
                new_turtle = Turtle()
                new_turtle.x = x
                new_turtle.y = y
                new_turtle.theta = theta 
                new_turtle.name = name 
                self.alive_turtles_.append(new_turtle)
                self.publish_alive_turtles()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))




def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerClientNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()