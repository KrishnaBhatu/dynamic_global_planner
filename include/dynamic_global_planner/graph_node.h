#ifndef GRAPH_NODE_H
#define GRAPH_NODE_H

#include <vector>

/**
 * @brief Node class is used to define the datatype for nodes in the mesh
 * 
 */
class Node
{
    public:
        std::vector<Node*> neighbours;
        float weight;

        /**
         * @brief Construct a new Node object
         * 
         */
        Node(){weight = 1;};

        /**
         * @brief Getter for x co-ordinate
         * 
         * @return float 
         */
        float getX()
        {
            return x_;
        }

        /**
         * @brief Getter for y co-ordinate
         * 
         * @return float 
         */
        float getY()
        {
            return y_;
        }

        /**
         * @brief Setter for x co-ordinate
         * 
         * @param x 
         */
        void setX(float x)
        {
            x_ = x;
        }

        /**
         * @brief Setter for y co-ordinate
         * 
         * @param y 
         */
        void setY(float y)
        {
            y_ = y;
        }

        /**
         * @brief Destroy the Node object
         * 
         */
        ~Node(){};
    private:
        float x_; //In meters
        float y_; //In meters
};

#endif