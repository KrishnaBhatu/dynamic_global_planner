#include "ros/ros.h"
#include <iostream>
#include "dynamic_global_planner/mesh_maker.h"
#include "dynamic_global_planner/Graph.h"
#include "dynamic_global_planner/Node.h"
#include "dynamic_global_planner/Neighbour.h"

int main(int argv, char **argc) {
  ros::init(argv, argc, "smart_planner");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  std::string ns = ros::this_node::getNamespace();
  cv::Mat input_image = cv::imread("/home/siddhesh/Downloads/AddverbMap.png");
  
  Mesh mesh_object(input_image);
  mesh_object.preprocessImage(5);
  mesh_object.displayMap();
  mesh_object.probabilisticMeshMake(20);
  mesh_object.genNeighbours();
  mesh_object.drawGraphonImage();

  ros::Publisher graph_pub = nh.advertise<dynamic_global_planner::Graph>("graph_topic", 1);
  dynamic_global_planner::Graph graph_msg;
  while (ros::ok())
  {
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


    graph_pub.publish(graph_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

 