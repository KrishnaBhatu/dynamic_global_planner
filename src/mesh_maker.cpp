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
                
                int x_select = rand() % 5;
                int y_select = rand() % 5;
                Node* graph_node = new Node();
                int x_new = j + x_select;
                if(x_new >  input_image_.cols) x_new = input_image_.cols;
                int y_new = i + y_select;
                if(y_new >  input_image_.rows) y_new = input_image_.rows;
                float x = (float)x_new/10;
                //float y = (float)(input_image_.rows - y_new)/10;
                float y = (float)y_new/10;
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
            //y = input_image_.rows - y;
            cv::Vec3b& color = img.at<cv::Vec3b>(y,x);
            color[0] = 255;
        }
        cv::imshow("Graph", img);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }
    return;
}

