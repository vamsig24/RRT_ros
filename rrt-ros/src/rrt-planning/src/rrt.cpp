//
// Created by malintha on 9/30/17.
// Edited by Vamsi on June 3rd 2021

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include "rrtImpl.h"
#include "nav_msgs/OccupancyGrid.h"

RRT initRRT();

Node runRRT(ros::Publisher, int);

void addEdge(geometry_msgs::Point p1, geometry_msgs::Point p2, ros::Publisher, bool);

void populateRviz();

void populateObstacles(const nav_msgs::OccupancyGridConstPtr &grid);

void setgoal(const geometry_msgs::PointConstPtr &goal_point);

bool check_goal(const geometry_msgs::PointConstPtr &goal_point);

void drawFinalPath(geometry_msgs::Point p1, geometry_msgs::Point p2, ros::Publisher marker_pub);

bool moveRobot(ros::Publisher marker_pub, geometry_msgs::Point);

Node init, goal;
static RRT rrt = initRRT();
double goal_bias = 0.5;
double sigma = 0.8;
static bool success = false;

static int init_x = 0;
static int init_y = 0;

float goal_x =10;
float goal_y =10;

static std::vector<visualization_msgs::Marker> obsVec;
static int path_node_index;
static bool pn_index_initialized = false;

static ros::Publisher marker_pub;

int main(int argc, char **argv) {
    ros::init(argc, argv, "rrt_planner");
    ros::NodeHandle n("/");
    ros::NodeHandle param_nh("~");
    param_nh.getParam("goal_bias",goal_bias);
    param_nh.getParam("sigma",sigma);
    ROS_INFO("goal_bias: %f sigma: %f ",goal_bias,sigma);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("some_chatter", 10);
    // ros::Subscriber sub = n.subscribe("map", 1000, populateObstacles);
     nav_msgs::OccupancyGridConstPtr msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map");
     populateObstacles(msg);
     ros::Duration(2).sleep();
     geometry_msgs::PointConstPtr msg_point = ros::topic::waitForMessage<geometry_msgs::Point>("goal");
     while(check_goal(msg_point))
     {
         ROS_INFO("Give a proper goal");
         msg_point = ros::topic::waitForMessage<geometry_msgs::Point>("goal");
         ROS_INFO("goal_x: %f, goal_y: %f",msg_point->x,msg_point->y);
         sleep(2);
     }
     setgoal(msg_point);
     ROS_INFO("goal_bias: %f sigma: %f ",goal_bias,sigma);
     ros::Duration(2).sleep();
    ros::Rate loop_rate(20);
    int frame_count = 0;

    while (ros::ok()) {
        ROS_INFO("Frame: %d", frame_count);
        populateRviz();
        
        if (!success) {
            Node next_node = runRRT(marker_pub, frame_count);
            geometry_msgs::Point next_point = next_node.point;

            if ((rrt.getEuclideanDistance(next_point, goal.point) <= 1) && frame_count > 2) {
                addEdge(next_point, goal.point, marker_pub, false);
                next_node.children.push_back(goal);
                goal.parentId = next_node.id;
                success = true;
            }
            // ROS_INFO("goal_x: %f, goal_y: %f",goal.point.x,goal.point.y);
        }

        if (success) {
            std::vector<Node> pathNodes;
            std::vector<Node> allNodes = rrt.getNodesList();

            pathNodes.push_back(goal);
            int tempParentId = goal.parentId;
            while (tempParentId != init.parentId) {
                for (int i = allNodes.size() - 1; i >= 0; i--) {
                    Node tempNode = allNodes[i];
                    if ((tempNode.id) == tempParentId) {
                        pathNodes.push_back(tempNode);
                        tempParentId = tempNode.parentId;
                    }
                }
            }

            std::cout << "\n\nPath retrieved!! \n\n";

            Node next;
            Node curr;
            for (int i = pathNodes.size() - 2; i >= 0; i--) {
                curr = pathNodes[i];
                next = pathNodes[i + 1];
                drawFinalPath(curr.point, next.point, marker_pub);
            }

            if (!pn_index_initialized) {
                path_node_index = pathNodes.size();
                pn_index_initialized = true;
            }
            bool isMoved = false;
            if (frame_count % 3 == 0) {
                geometry_msgs::Point next_pose = pathNodes[--path_node_index].point;

                isMoved = moveRobot(marker_pub, next_pose);
            }
            if (isMoved) {
                return 0;
            }
        }

        while (marker_pub.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                return 0;
            }
            ROS_WARN_ONCE("Please run Rviz in another terminal. %d",marker_pub.getNumSubscribers());
            sleep(1);
        }

        //iterate ROS
        ros::spinOnce();
        loop_rate.sleep();
        ++frame_count;
    }

    return 0;
}

bool moveRobot(ros::Publisher marker_pub, geometry_msgs::Point next_pose) {

    static visualization_msgs::Marker rob;
    rob.type = visualization_msgs::Marker::CUBE;


    rob.header.frame_id = "map";
    rob.header.stamp = ros::Time::now();
    rob.ns = "rob";
    rob.id = 0;
    rob.action = visualization_msgs::Marker::ADD;
    rob.lifetime = ros::Duration();

    rob.scale.x = 0.5;
    rob.scale.y = 0.5;
    rob.scale.z = 0.5;
    rob.pose.orientation.w = 1;
    rob.pose.orientation.x = rob.pose.orientation.y = rob.pose.orientation.z = 0;
    rob.color.r = 1.0f;
    rob.color.g = 0.5f;
    rob.color.b = 0.5f;
    rob.color.a = 1.0;

    //calculate m to change the orientation of the robot
    float m = (next_pose.y - rob.pose.position.y) / (next_pose.x - rob.pose.position.x);

    rob.pose.orientation.z = atan(m) + M_PI / 2;
    rob.pose.position = next_pose;

    marker_pub.publish(rob);

    if ((rob.pose.position.x == goal.point.x) && (rob.pose.position.y == goal.point.y)) {
        marker_pub.publish(rob);
        return true;
    }

    return false;
}

RRT initRRT() {
    init.point.x = init_x;
    init.point.y = init_y;
    init.id = -1;
    init.parentId = -2;
    goal.point.x = goal_x;
    goal.point.y = goal_y;
    goal.id = 10000;
    RRT rrt(init, goal, sigma, 32.1, 0, 32.1, 0);
    return rrt;
}


Node runRRT(ros::Publisher marker_pub, int frameid) {
    geometry_msgs::Point rand_point = rrt.getRandomConfig();
    geometry_msgs::Point tempP;
    tempP.x = 0;
    tempP.y = 0;
    Node rand_node(tempP);
    Node next_node(tempP);
    Node nearest_node = rrt.getNearestNode(rand_point);
    // ROS_INFO("Point X:%f Y: %f",nearest_node.point.x,nearest_node.point.y);

    //decide whether to extend toward the goal or a random point
    double r = rand() / (double) RAND_MAX;
    if (r < goal_bias) {
        next_node = rrt.expand(nearest_node, goal, obsVec, frameid);
    } else {
        rand_node.point = rand_point;
        next_node = rrt.expand(nearest_node, rand_node, obsVec, frameid);
    }

    if ((next_node.point.x != nearest_node.point.x) && (next_node.point.y != nearest_node.point.y)) {
        std::cout << "Rand_config: \n" << rand_point << "nearest_node: \n" << nearest_node.point << "next_node: \n"
                  << (next_node).point << "\n\n";
        addEdge(nearest_node.point, (next_node).point, marker_pub, false);
    }
    return next_node;
}

void drawFinalPath(geometry_msgs::Point p1, geometry_msgs::Point p2, ros::Publisher marker_pub) {
    static visualization_msgs::Marker edge;
    edge.type = visualization_msgs::Marker::LINE_LIST;
    edge.header.frame_id = "map";
    edge.header.stamp = ros::Time::now();
    edge.ns = "finalPath";
    edge.id = 4;
    edge.action = visualization_msgs::Marker::ADD;
    edge.pose.orientation.w = 1;

    edge.scale.x = 0.08;

    edge.color.g = edge.color.r = 1;
    edge.color.a = 1.0;

    edge.points.push_back(p1);
    edge.points.push_back(p2);

    marker_pub.publish(edge);
}

void addEdge(geometry_msgs::Point p1, geometry_msgs::Point p2, ros::Publisher marker_pub, bool isFinal) {
    static visualization_msgs::Marker edge, vertex;
    vertex.type = visualization_msgs::Marker::POINTS;
    edge.type = visualization_msgs::Marker::LINE_LIST;
    edge.header.frame_id = "map";
    edge.header.stamp = ros::Time::now();
    edge.ns = "edges";
    edge.id = 3;
    edge.action = visualization_msgs::Marker::ADD;
    edge.pose.orientation.w = 1;

    edge.scale.x = 0.04;
    if (!isFinal) {
        edge.color.r = 1.0;
    } else {
        edge.color.g = edge.color.r = 1;
    }
    edge.color.a = 1.0;

    edge.points.push_back(p1);
    edge.points.push_back(p2);

    marker_pub.publish(edge);
}

void setgoal(const geometry_msgs::PointConstPtr &goal_point){
    goal_x = goal_point->x;
    goal_y = goal_point->y;
    goal.point.x = goal_x;
    goal.point.y = goal_y;

}

bool check_goal(const geometry_msgs::PointConstPtr &goal_point){
    float x = goal_point->x;
    float y = goal_point->y;

 for (int i = 0; i < obsVec.size(); i++) {
        visualization_msgs::Marker obs = obsVec[i];

        float obs_x = obs.pose.position.x;
        float obs_y = obs.pose.position.y;

        float dist = std::sqrt(std::pow((obs_x- x), 2) + std::pow((obs_y - y), 2));
        if(dist<0.5)
        {
            ROS_INFO("dist from goal to obstacle is %f",dist);
            return true;
        }
        
    }
    return false;
}


void populateRviz() {
    visualization_msgs::Marker v_start, v_end;
    v_start.type = v_end.type = visualization_msgs::Marker::POINTS;
    v_start.header.frame_id = v_end.header.frame_id = "map";
    v_start.header.stamp = v_end.header.stamp = ros::Time::now();
    v_start.ns = v_end.ns = "start/end vertices";
    v_start.id = 0;
    v_end.id = 1;
    v_start.action = v_end.action = visualization_msgs::Marker::ADD;

    v_start.color.a = 1.0f;
    v_start.color.g = 1.0f;
    v_start.scale.x = v_start.scale.y = 0.5;
    v_end.scale.x = v_end.scale.y = 0.5;

    v_end.color.a = 1.0f;
    v_end.color.r = 1.0f;

    geometry_msgs::Point ps, pe;
    ps.x = init_x;
    ps.y = init_y;
    pe.x = goal_x;
    pe.y = goal_y;
    v_start.points.push_back(ps);
    v_end.points.push_back(pe);

    //publish edge and vertices
    marker_pub.publish(v_start);
    marker_pub.publish(v_end);

    // populateObstacles(grid);

};

void populateObstacles(const nav_msgs::OccupancyGridConstPtr &grid) {

    int count_obj=0;
    for (int width=0; width < grid->info.width; ++width)
    {
        for (int height=0; height < grid->info.height; ++height)
        {
            if(grid->data[height*grid->info.width + width] == 0)
            {
               double x = width * grid->info.resolution + grid->info.resolution / 2;
               double y = height * grid->info.resolution + grid->info.resolution / 2;
            //    ROS_INFO("X: %f Y: %f \n\n",x,y);

                visualization_msgs::Marker obs1;
                obs1.type = visualization_msgs::Marker::CUBE;
                obs1.header.frame_id = "map";
                obs1.header.stamp = ros::Time::now();
                obs1.ns = "obstacles";
                obs1.lifetime =ros::Duration();
                obs1.action = visualization_msgs::Marker::ADD;
                obs1.id = count_obj;
                count_obj++;
                obs1.scale.x = 0.1;
                obs1.scale.y = 0.075;
                obs1.scale.z = 0.1;
                obs1.pose.position.x = x;
                obs1.pose.position.y = y;
                obs1.pose.position.z = 0.25;
                obs1.pose.orientation.x = 0.0;
                obs1.pose.orientation.y = 0.0;
                obs1.pose.orientation.z = 0.0;
                obs1.pose.orientation.w = 1;
                obs1.color.a = 1;

                obs1.color.r =6.6f;
                obs1.color.g = obs1.color.b = 6.6f;
                marker_pub.publish(obs1);
                obsVec.push_back(obs1);



            }
        }
    }
    
    // visualization_msgs::Marker obs1, obs2, obs3, obs4, obs5, obs6;

    // obs1.type = obs2.type = obs3.type = obs4.type = obs5.type = obs6.type = visualization_msgs::Marker::CUBE;
    // obs1.header.frame_id = obs2.header.frame_id = obs3.header.frame_id = obs4.header.frame_id = obs5.header.frame_id = obs6.header.frame_id = "map";
    // obs1.header.stamp = obs2.header.stamp = obs3.header.stamp = obs4.header.stamp = obs5.header.stamp = obs6.header.stamp = ros::Time::now();
    // obs1.ns = obs2.ns = obs3.ns = obs4.ns = obs5.ns = obs6.ns = "obstacles";
    // obs1.lifetime = obs2.lifetime = obs3.lifetime = obs4.lifetime = obs5.lifetime = obs6.lifetime = ros::Duration();
    // obs1.action = obs2.action = obs3.action = obs4.action = obs5.action = obs6.action = visualization_msgs::Marker::ADD;

    // obs1.id = 0;
    // obs2.id = 1;
    // obs3.id = 2;
    // obs4.id = 3;
    // obs5.id = 4;
    // obs6.id = 5;

    // obs1.scale.x = obs2.scale.x = 2;
    // obs1.scale.y = obs2.scale.y = 12;
    // obs3.scale.y = 4;
    // obs3.scale.x = 7;
    // obs4.scale.x = obs4.scale.y = 2;
    // obs5.scale.x = 5;
    // obs5.scale.y = 3;
    // obs6.scale.x = 3;
    // obs6.scale.y = 7;

    // obs1.scale.z = obs2.scale.z = obs3.scale.z = obs4.scale.z = obs5.scale.z = obs6.scale.z = 0.25;


    // obs1.pose.position.x = 10.5;
    // obs1.pose.position.y = 10.5;
    // obs1.pose.position.z = 0.25;
    // obs1.pose.orientation.x = 0.0;
    // obs1.pose.orientation.y = 0.0;
    // obs1.pose.orientation.z = 0.0;
    // obs1.pose.orientation.w = 1;
    // obs1.color.a = 1;
    // // obs1.color.r = obs1.color.g = obs1.color.b = 6.6f;
    // obs1.color.r =1;
    // obs1.color.g = obs1.color.b = 0;

    // obs2.pose.position.x = 14;
    // obs2.pose.position.y = 14;
    // obs2.pose.position.z = 0.25;
    // obs2.pose.orientation.x = 0.0;
    // obs2.pose.orientation.y = 0.0;
    // obs2.pose.orientation.z = 0.0;
    // obs2.pose.orientation.w = 1;
    // // obs2.color.a = 1;
    // // obs2.color.r = obs2.color.g = obs2.color.b = 6.6f;
    // obs2.color.g =1;
    // obs2.color.r = obs2.color.b = 0;

    // obs3.pose.position.x = 16.5;
    // obs3.pose.position.y = 2;
    // obs3.pose.position.z = 0.25;
    // obs3.pose.orientation.x = 0.0;
    // obs3.pose.orientation.y = 0.0;
    // obs3.pose.orientation.z = 0.0;
    // obs3.pose.orientation.w = 1;
    // // obs3.color.a = 1;
    // // obs3.color.r = obs3.color.g = obs3.color.b = 6.6f;
    // obs3.color.b =1;
    // obs3.color.g = obs3.color.r = 0;

    // obs4.pose.position.x = 16;
    // obs4.pose.position.y = 9;
    // obs4.pose.position.z = 0.25;
    // obs4.pose.orientation.x = 0.0;
    // obs4.pose.orientation.y = 0.0;
    // obs4.pose.orientation.z = 0.0;
    // obs4.pose.orientation.w = 1;
    // obs4.color.a = 1;
    // obs4.color.r = obs4.color.g = obs4.color.b = 6.6f;

    // obs5.pose.position.x = 2.5;
    // obs5.pose.position.y = 18.5;
    // obs5.pose.position.z = 0.25;
    // obs5.pose.orientation.x = 0.0;
    // obs5.pose.orientation.y = 0.0;
    // obs5.pose.orientation.z = 0.0;
    // obs5.pose.orientation.w = 1;
    // obs5.color.a = 1;
    // obs5.color.r = obs5.color.g = obs5.color.b = 6.6f;

    // obs6.pose.position.x = 1.5;
    // obs6.pose.position.y = 13.5;
    // obs6.pose.position.z = 0.25;
    // obs6.pose.orientation.x = 0.0;
    // obs6.pose.orientation.y = 0.0;
    // obs6.pose.orientation.z = 0.0;
    // obs6.pose.orientation.w = 1;
    // obs6.color.a = 1;
    // obs6.color.r = obs6.color.g = obs6.color.b = 6.6f;

    // marker_pub.publish(obs1);
    // marker_pub.publish(obs2);
    // marker_pub.publish(obs3);
    // marker_pub.publish(obs4);
    // marker_pub.publish(obs5);
    // marker_pub.publish(obs6);

    // obsVec.push_back(obs1);
    // obsVec.push_back(obs2);
    // obsVec.push_back(obs3);
    // obsVec.push_back(obs4);
    // obsVec.push_back(obs5);
    // obsVec.push_back(obs6);
}