#include "ros/ros.h"
#include <iostream>
#include <string>
#include "nav_msgs/Odometry.h"
#include "dynamic_global_planner/mesh_maker.h"
#include "dynamic_global_planner/Graph.h"
#include "dynamic_global_planner/Node.h"
#include "dynamic_global_planner/Neighbour.h"

int main(int argv, char **argc) {
  ROS_INFO("Dynamic Global Planner Initiated");
  ros::init(argv, argc, "smart_planner");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  std::string ns = ros::this_node::getNamespace();
  cv::Mat input_image = cv::imread("/home/krishna/new_global_planner_ws/src/dynamic_global_planner/config/AddverbMap.png");
  
  Mesh mesh_object(input_image);
  mesh_object.preprocessImage(2);
  mesh_object.displayMap();
  mesh_object.probabilisticMeshMake(20);
  mesh_object.genNeighbours();
  mesh_object.drawGraphonImage();


  ros::Publisher graph_pub = nh.advertise<dynamic_global_planner::Graph>("graph_topic", 1);
  dynamic_global_planner::Graph graph_msg;
  while (ros::ok())
  {
    if(mesh_object.robots.size() > 0) mesh_object.robots.clear();
    
    for(int r = 0; r < 10; r++)
    {
      std::string s = "/robot_";
      std::string robot = std::to_string(r);
      std::string topic = "/base_pose_ground_truth";
      s.append(robot);
      s.append(topic);
      //ROS_INFO_STREAM(s);
      boost::shared_ptr<nav_msgs::Odometry const> sharedloc;
      nav_msgs::Odometry loc;
      sharedloc = ros::topic::waitForMessage<nav_msgs::Odometry>(s,nh);
      if(sharedloc != NULL)
      {
        loc = *sharedloc;
        std::tuple<float, float> temp_t(loc.pose.pose.position.x, loc.pose.pose.position.y);
        mesh_object.robots.push_back(temp_t);
        //if(r < 11) ROS_INFO_STREAM(loc.pose.pose.position.x << ", " << loc.pose.pose.position.y);
      }
    }
    
    // Clear both object arrays
    graph_msg.mesh.clear();
    graph_msg.mesh_neighbour.clear();
    for(auto node_ptr:mesh_object.graph)
    {
      dynamic_global_planner::Node node;
      dynamic_global_planner::Neighbour neigh;
      node.x = (*node_ptr).getX();
      node.y = (*node_ptr).getY();
      node.weight = (*node_ptr).weight;
      float weight_count = 1;
      for(auto a: mesh_object.robots)
      {
        float dist = mesh_object.getEucledianDistance(node_ptr->getX(), node_ptr->getY(), std::get<0>(a), std::get<1>(a));
        if(dist < 3.0)
        {
          weight_count += 1.0;
        }
      }
      node_ptr->weight = weight_count;
      for(auto ngh_ptr:(*node_ptr).neighbours)
      {
        dynamic_global_planner::Node n_node;
        n_node.x = (*ngh_ptr).getX();
        n_node.y = (*ngh_ptr).getY();
        n_node.weight = (*ngh_ptr).weight;
        neigh.neighbours.push_back(n_node);
      }
      graph_msg.mesh.push_back(node);
      graph_msg.mesh_neighbour.push_back(neigh);
    }

    //mesh_object.drawGraphonImage();
    //mesh_object.displayMapwithCrowds();

    graph_pub.publish(graph_msg); // Publishes the mesh
    ROS_INFO_STREAM("Published, launch death star!");
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

 
