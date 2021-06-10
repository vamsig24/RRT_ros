# rrt-ros
RRT implementation on a given map

## Prerequisites
You need to have installed [ROS - Robot Operating System](http://www.ros.org/) in your system.

## Running and visualization
Run roscore in another terminal.

`roscore`

Open another terminal and clone the rrt-ros repository in to your preferable location. Then compile it using catkin as follows.

`git clone https://github.com/vamsig24/RRT_ros.git`

`cd RRT_ros/rrt-ros`

`catkin_make`

Source the setup file of the rrt-ros to make the rrt-planning package visible to ros.

`source devel/setup.bash` 

Then launch map,rrt and rviz using the following command

`roslaunch rrt-planning rrt_start.launch`

As rviz has already been launched with the map and marker, you need to publish a goal from another terminal. According to the current resolution of the map, the maximum x and y for the goal is around 31.

`rostopic pub /goal geometry_msgs/Point '{x: 15, y: 16}'`

---
**NOTE**

### Not reaching goal
If the algorithm doesn't look like reaching the goal even after a few min(probably 3-4min). Then try to tweak the goal bias and sigma and relaunch code and publish the goal.
Goal bias of around 0.5 to 0.7 and sigma of around 0.6 to 0.9 worked in most of the cases for me.

### Rviz not planning/showing the goal
It might be because your goal is too close to an obstacle, check the terminal in which you launched the 3 nodes for any feedback, you should be able to see the distance printed if your goal is too close. Planning will start once a safe goal is published.

---
