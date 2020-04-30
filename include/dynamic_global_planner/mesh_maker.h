#ifndef MESH_MAKER_H
#define MESH_MAKER_H

#include <ros/ros.h>
#include <set>
#include "dynamic_global_planner/graph_node.h"

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

class Mesh
{
    public:
        std::vector<Node*> graph;
        Mesh(cv::Mat img)
        {
            input_image_ = img;
        }

        void preprocessImage(int erosion_iteration);

        void displayMap();

        void probabilisticMeshMake(int spacing_factor);

        bool checkObsInArea(int i, int j, int size);

        void drawGraphonImage();

        ~Mesh(){};

    private:
        cv::Mat input_image_;
};


#endif