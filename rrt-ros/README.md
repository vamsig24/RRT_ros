# rrt-ros
RRT implementation on a given map

## Prerequisites
You need to have installed [ROS - Robot Operating System](http://www.ros.org/) in your system.

## Running and visualization
Clone the rrt-ros repository in to your preferable location. Then compile it using catkin as follows.



`cd rrt-ros`

`catkin_make`

Source the setup file of the rrt-ros to make the rrt-planning package visible to ros.

`source devel/setup.bash` 

Run roscore in another terminal.

`roscore`

Then run rrt node as follows.

`rosrun rrt-planning rrt`

You will be now promped to run rviz for visualization. Then run Rviz in another terminal.

`rosrun rviz rviz`

