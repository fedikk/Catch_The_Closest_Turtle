# TurtleSim Catch Them All

Description 
In this project, we will work with turtlesim
the used nodes are 3 :
  
  
  ● The __turtlesim_node__ from the turtlesim package
  
  ● A custom node to control the turtle . we will call the node  __turtle_controller__.
  
  ● A custom node to spawn turtles on the window, and to manage which turtle is still “alive”.
    This node is called __turtle_spawner__.
    
  
the package of this project will be __turtlesim_catch_them_all__ to put our new nodes.

## Node Communication 

![Capture d'écran 2024-04-22 105752](https://github.com/fedikk/Catch_The_Closest_Turtle/assets/98516504/3473832c-93ec-4ea5-be29-970863427e81)


## Demo



https://github.com/fedikk/Catch_The_Closest_Turtle/assets/98516504/fdf12f46-bd5c-411e-a142-4bee1541482c



# Project Conclusion

Well you can always improve a project.

If you want to go further you can, for example:

  *  Make the new turtles move randomly. You will then need to be able to keep an eye on each turtle, and thus dynamically create a subscription for each alive turtle’s pose! For that you can      create a “middle” node whose goal is to monitor the playground, and tell the turtle controller where to go at any given time.

  * Change the pen color each time the “master turtle” has caught another turtle.

  * Add another “master turtle” node to catch turtles even faster!

  * ...

With this project under your belt you should now be able to apply all the knowledge you got here on any of your robot projects using ROS2.

Some of the functionalities you saw in this section are in fact pretty similar to what you can find on existing robots.
