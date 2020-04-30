#ifndef GRAPH_NODE_H
#define GRAPH_NODE_H

#include <vector>

class Node
{
    public:
        std::vector<Node*> neighbours;
        float weight;

        Node(){};

        float getX()
        {
            return x_;
        }

        float getY()
        {
            return y_;
        }

        void setX(float x)
        {
            x_ = x;
        }

        void setY(float y)
        {
            y_ = y;
        }
        
        ~Node(){};
    private:
        float x_; //In meters
        float y_; //In meters
};

#endif