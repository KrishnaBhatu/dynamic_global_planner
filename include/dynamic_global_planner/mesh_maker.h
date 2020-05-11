#ifndef MESH_MAKER_H
#define MESH_MAKER_H

#include <ros/ros.h>
#include <algorithm>
#include <set>
#include <queue>
#include <limits>
#include <tuple>
#include "dynamic_global_planner/graph_node.h"

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

/**
 * @brief Mesh used by the global planner
 * 
 */
class Mesh
{
    public:
        /**
         * @brief Construct a new Mesh object
         * 
         */
        Mesh(){};
        std::vector<Node*> graph;
        std::vector<std::tuple<float, float>> robots;
        std::set<std::vector<float> > obstacles;
        Mesh(cv::Mat img)
        {
            input_image_ = img;
        }

        /**
         * @brief Erode the static obstacles in the warehouse image which works a minkowski distance
         * 
         * @param erosion_iteration Factor to increase or decrease the erosion operation
         */
        void preprocessImage(int erosion_iteration);

        /**
         * @brief Debugging method to see if the mesh is generated properly or not
         * 
         */
        void displayMap();

        /**
         * @brief PRM method to generate mesh
         * 
         * @param spacing_factor Grid length used for generating PRM
         */
        void probabilisticMeshMake(int spacing_factor);

        /**
         * @brief Method to check if there is obstacle in the mesh grid
         * 
         * @param i x co-ordinate of the top-left corner of grid
         * @param j y co-ordinate of the top-left corner of grid
         * @param size Size of grid (height/width) 
         * @return true If there is an obstacle
         * @return false If there is not an obstacle
         */
        bool checkObsInArea(int i, int j, int size);

        /**
         * @brief Debugging method to see if the mesh is generated properly or not and also check if path is generated with a temp start
         *        goal position
         * 
         */
        void drawGraphonImage();

        /**
         * @brief Method to add neighbours to the node 
         * 
         */
        void genNeighbours();

        /**
         * @brief Get the Eucledian Distance 
         * 
         * @param x1 x co-ordinate of point1
         * @param y1 y co-ordinate of point1
         * @param x2 x co-ordinate of point2
         * @param y2 y co-ordinate of point2
         * @return float 
         */
        float getEucledianDistance(float x1, float y1, float x2, float y2);

        /**
         * @brief Checks if there is obstacle in the path of two points
         * 
         * @param x1 x co-ordinate of point1
         * @param y1 y co-ordinate of point1
         * @param x2 x co-ordinate of point2
         * @param y2 y co-ordinate of point2
         * @param xi x co-ordinate of point between point1 and point2
         * @param yi y co-ordinate of point between point1 and point2
         * @return true If there is an obstacle
         * @return false If there is not an obstacle
         */
        bool checkPathIntersection(float x1, float y1, float x2, float y2, float xi, float yi);

        /**
         * @brief Temp method to find path which is re-defined in death-star
         */
        std::vector<Node*> findShortestPath(float x_start, float y_start, float x_goal, float y_goal);

        /**
         * @brief Temp method to find path which is re-defined in death-star
         */
        Node* findNearestNode(float x, float y);

        /**
         * @brief Temp method to display crowding on an image, used for debugging
         * 
         */
        void displayMapwithCrowds();
        
        /**
         * @brief Destroy the Mesh object
         * 
         */
        ~Mesh(){};

    private:
        cv::Mat input_image_;
};


#endif