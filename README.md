# rrt-ros
RRT implementation on a given map

## Prerequisites
You need to have installed [ROS - Robot Operating System](http://www.ros.org/) in your system.

## Running and visualization
Run roscore in another terminal.

`roscore`

Starting /map

`cd RRT_ros/maps`

`rosrun map_server map_server test.yaml`

Open another terminal and clone the rrt-ros repository in to your preferable location. Then compile it using catkin as follows.

`git clone https://github.com/vamsig24/RRT_ros.git`

`cd RRT_ros/rrt-ros`

`catkin_make`

Source the setup file of the rrt-ros to make the rrt-planning package visible to ros.

`source devel/setup.bash` 

Then run rrt node as follows.

`rosrun rrt-planning rrt`

You will be now promped to run rviz for visualization. Then run Rviz in another terminal.

`rosrun rviz rviz`
