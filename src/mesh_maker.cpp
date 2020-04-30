#include "dynamic_global_planner/mesh_maker.h"

void Mesh::preprocessImage(int erosion_iterations)
{
    
    cv::threshold(input_image_, input_image_, 127, 255, 0);
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                       cv::Size( 2*5 + 1, 2*5+1 ));

    cv::erode(input_image_, input_image_, element), cv::Point(-1, -1), erosion_iterations;
}

void Mesh::displayMap()
{
    
    cv::imshow("Image", input_image_);
    cv::waitKey(0);
    cv::destroyAllWindows();
}


bool Mesh::checkObsInArea(int i, int j, int size)
{
    bool val = false; 
    for(int x = i; x < (i+size); x++)
    {
        for(int y = j; y < (j+size); y++)
        {
            //ROS_INFO_STREAM(x << ", " << y);
            if(x < input_image_.cols && y < input_image_.rows)
            {
                cv::Vec3b value = input_image_.at<cv::Vec3b>(y, x);
                //ROS_INFO_STREAM(value);
                if(value == cv::Vec3b(0,0,0))
                {
                    val = true;
                    float qx = (float)x/10;
                    float qy = (float)(input_image_.rows - y)/10;
                    std::vector<float> temp_v{qx, qy};
                    obstacles.insert(temp_v);
                }
            }
        }
    }
    return val;
}


void Mesh::probabilisticMeshMake(int spacing_factor)
{
    for(int i = 0; i < input_image_.rows; i+= spacing_factor)
    {
        for(int j = 0; j < input_image_.cols; j+= spacing_factor)
        {
            if(!checkObsInArea(j, i, spacing_factor))
            {
                
                int x_select = rand() % spacing_factor;
                int y_select = rand() % spacing_factor;
                Node* graph_node = new Node();
                int x_new = j + x_select;
                if(x_new >  input_image_.cols) x_new = input_image_.cols;
                int y_new = i + y_select;
                if(y_new >  input_image_.rows) y_new = input_image_.rows;
                float x = (float)x_new/10;
                float y = (float)(input_image_.rows - y_new)/10;
                graph_node->setX(x);
                graph_node->setY(y);
                graph.push_back(graph_node); 
            }
        }
    }
}

void Mesh::drawGraphonImage()
{
    if(graph.size() > 0)
    {
        cv::Mat img(input_image_.rows, input_image_.cols, CV_8UC3, cv::Scalar(0,0, 100));

        for(auto a: graph)
        {
            int x = (int)(a->getX()*10);
            int y = (int)(a->getY()*10);
            y = input_image_.rows - y;
            cv::Vec3b& color = img.at<cv::Vec3b>(y,x);
            color[0] = 255;
            for(auto N: a->neighbours)
            {
                int x1 = (int)(N->getX()*10);
                int y1 = (int)(N->getY()*10);
                y1 = input_image_.rows - y1;
                cv::line(img, cv::Point(x,y), cv::Point(x1,y1), cv::Scalar(0,255, 0), 1);
            }
        }
        cv::imshow("Graph", img);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }
    return;
}

float Mesh::getEucledianDistance(float x1, float y1, float x2, float y2)
{
    return sqrt(pow((x1-x2), 2) + pow((y1-y2), 2));
}

bool Mesh::checkPathIntersection(float x1, float y1, float x2, float y2, float xi, float yi)
{
    float path_threshold = 0.1;
    if(xi < std::max(x1, x2) && xi > std::min(x1, x2) && yi < std::max(y1, y2) && yi > std::min(y1,y2)) // Check inside bounding box
    {
        // Check for line intersection
        float point_loc = (x2-x1)*(yi-y1) - (y2-y1)*(xi-x1);
        if(point_loc < path_threshold && point_loc > ((-1)*path_threshold))
        {
            return true; // Collision
        }
    }
    return false;
}

void Mesh::genNeighbours()
{
    ROS_INFO_STREAM("-----Generating Neighbours--------");
    ROS_INFO_STREAM("No of nodes-> " << graph.size());
    ROS_INFO_STREAM("No of obs-> " << obstacles.size());
    float neighbour_threshold = 5.0;
    
    if(graph.size() > 0)
    {
        for(auto curr: graph)
        {
            for(auto neighbour: graph)
            {
                if(neighbour != curr)
                {
                    // Condition for distance
                    if(getEucledianDistance(curr->getX(), curr->getY(), neighbour->getX(), neighbour->getY()) < neighbour_threshold)
                    {
                        // Condition for obstacle intersection
                        bool no_intersection = true;
                        
                        for(int i = std::min(curr->getX(), neighbour->getX())*10; i < std::max(curr->getX(), neighbour->getX())*10; i++)
                        {
                            for(int j = std::min(curr->getY(), neighbour->getY())*10; j < std::max(curr->getY(), neighbour->getY())*10; j++)
                            {
                                float nx = (float)i / 10;
                                float ny = (float)j / 10;
                                std::vector<float> temp{nx,ny};
                                if(obstacles.find(temp) != obstacles.end())
                                {
                                    //ROS_INFO_STREAM(x << ", " << y);
                                    if(checkPathIntersection(curr->getX(), curr->getY(), neighbour->getX(), neighbour->getY(), nx, ny))
                                    {
                                        no_intersection = false;
                                        break;
                                    }        
                                }
                            }
                        }
                        
                        /**
                        for(auto obs: obstacles)
                        {
                            float x = obs[0];
                            float y = obs[1];
                            ROS_INFO_STREAM(x << ", " << y);
                            if(checkPathIntersection(curr->getX(), curr->getY(), neighbour->getX(), neighbour->getY(), x, y))
                            {
                                no_intersection = false;
                                break;
                            }
                        }
                        **/
                        if(no_intersection)
                        {
                            curr->neighbours.push_back(neighbour);
                        }

                    }
                }
            }
        }
    }
    ROS_INFO_STREAM("-----Generating Done--------");
}

