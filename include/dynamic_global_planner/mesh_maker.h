#ifndef MESH_MAKER_H
#define MESH_MAKER_H

#include <ros/ros.h>
#include <algorithm>
#include <set>
#include <queue>
#include <limits>
#include "dynamic_global_planner/graph_node.h"

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

class Mesh
{
    public:
        std::vector<Node*> graph;
        std::set<std::vector<float>> obstacles;
        Mesh(cv::Mat img)
        {
            input_image_ = img;
        }

        void preprocessImage(int erosion_iteration);

        void displayMap();

        void probabilisticMeshMake(int spacing_factor);

        bool checkObsInArea(int i, int j, int size);

        void drawGraphonImage();

        void genNeighbours();

        float getEucledianDistance(float x1, float y1, float x2, float y2);

        bool checkPathIntersection(float x1, float y1, float x2, float y2, float xi, float yi);

        std::vector<Node*> findShortestPath(float x_start, float y_start, float x_goal, float y_goal);

        Node* findNearestNode(float x, float y);

        ~Mesh(){};

    private:
        cv::Mat input_image_;
};


#endif