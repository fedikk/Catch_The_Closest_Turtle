#!/usr/bin/env python3
import math 

import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose

from geometry_msgs.msg import Twist

from my_robot_interfaces.msg import Turtle,TurtleArray 
from my_robot_interfaces.srv import CatchTurtle

from functools import partial

class TurtleControllerNode(Node): 
    
    def __init__(self):
        super().__init__("turtle_controller") 
        self.declare_parameter("catch_closest_turtle_first",True)
        

        self.turtle_to_catch = None
        self.catch_the_closest_turtle_first_ = self.get_parameter("catch_closest_turtle_first").value

        #the turtle position
        self.pose_ = None
        
        # publish for velocity to the turtle1
        self.cmd_vel_publisher_ = self.create_publisher(Twist,"turtle1/cmd_vel",10)


        # subscribe to get the the turtle1 pose 
        self.subscriber_ = self.create_subscription(
            Pose,"turtle1/pose",self.callback_turtle_pose,10)
        

        # create a subscription to alive turtles 
        self.alive_turtles_subscribe_ = self.create_subscription(
            TurtleArray,"alive_turtles",self.callback_alive_turtles,10)

        self.timer_ = self.create_timer(
            0.01,self.control_loop)
        self.get_logger().info("Turtle controller  has Been started")   
    


    def callback_turtle_pose(self, msg):
        self.pose_ = msg

    def callback_alive_turtles(self,msg):
        if len(msg.turtles) > 0 :
            if (self.catch_the_closest_turtle_first_):
                closest_turtle= None
                closest_turtle_distance = None

                for turtle in msg.turtles: 
                    dist_x =  turtle.x - self.pose_.x 
                    dist_y =  turtle.y - self.pose_.y 
                    distance = math.sqrt ( dist_x*dist_x + dist_y*dist_y  )
                    if closest_turtle == None or distance < closest_turtle_distance: 
                        closest_turtle = turtle
                        closest_turtle_distance = distance
                self.turtle_to_catch = closest_turtle
            else : 
                self.turtle_to_catch = msg.turtles[0]
        

    # the control loop could be a sensor read value function or controll loop in real projects 
    def control_loop(self): 
        if self.pose_ == None  or self.turtle_to_catch == None : 
            return 
        dist_x =  self.turtle_to_catch.x - self.pose_.x 
        dist_y =  self.turtle_to_catch.y - self.pose_.y 
        distance = math.sqrt ( dist_x*dist_x + dist_y*dist_y  )
        
        msg = Twist()

        if  distance > 0.5 : 
            #position 
            msg.linear.x = 2*distance

            #oriantation 
            goal_theta = math.atan2( dist_y , dist_x )
            diff = - self.pose_.theta + goal_theta
            if diff > math.pi :
                diff -= 2*math.pi
            elif diff < - math.pi :
                diff += 2*math.pi
            msg.angular.z =  6*diff
        else: 
            self.call_catch_turtle_server(self.turtle_to_catch.name)
            msg.linear.x = 0.0
            msg.angular.z = 0.0 
            self.get_logger().info(" GOAL reached Successfully!!!!")
            self.turtle_to_catch = None 
        self.cmd_vel_publisher_.publish(msg)
    
    def call_catch_turtle_server(self,name):
        client = self.create_client(CatchTurtle,"catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for add two ints server !!!")
        
        request = CatchTurtle.Request()
        request.name = name 
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_catch_turtle,name=name))


      # # the callback is made to hundle the response from the server 
    def callback_call_catch_turtle(self, future,name):
        try:
            response = future.result() 
            if not response.success :
                self.get_logger().error("error while catching " + name )                    
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))





def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()