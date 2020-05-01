#include "ros/ros.h"
#include <iostream>
#include "dynamic_global_planner/mesh_maker.h"

int main(int argv, char **argc) {
  ros::init(argv, argc, "smart_planner");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  std::string ns = ros::this_node::getNamespace();
  cv::Mat input_image = cv::imread("/home/krishna/new_global_planner_ws/src/dynamic_global_planner/config/AddverbMap.png");
  
  Mesh mesh_object(input_image);
  mesh_object.preprocessImage(5);
  mesh_object.displayMap();
  mesh_object.probabilisticMeshMake(20);
  mesh_object.genNeighbours();
  mesh_object.drawGraphonImage();
  
  return 0;
}

 