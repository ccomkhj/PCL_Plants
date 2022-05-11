
#pragma once

#include <iostream>
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <iterator>



template <class T, class P>
class Passthrough{
    public:
        double thres;
        P cloud;

        Passthrough (P& point_cloud){
            cloud = point_cloud;
        }

        void process(P& filtered_cloud, float center, double thres, std::string direction){
            pass.setInputCloud(cloud);
            std::string dir_list[] = {"x", "X", "y", "Y", "z", "Z"};

            if (std::find(std::begin(dir_list), std::end(dir_list), direction) == std::end(dir_list) ) {
                std::cerr << "Wrong direction! it should be among x,y,z." << std::endl;
            } 
            transform(direction.begin(), direction.end(), direction.begin(), ::tolower);
            pass.setFilterFieldName( direction );

            pass.setFilterLimits(center - thres, center + thres); // need to get offset value using statistics.
            pass.filter(*filtered_cloud);
        }
        
    private: 
        pcl::PassThrough<T> pass;
        

};

