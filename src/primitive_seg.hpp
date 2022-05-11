
#pragma once

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>



template <class T, class P>
class Primitive_seg{
    public:
        double thres;
        P cloud;

        Primitive_seg (double threshold, P& point_cloud){
            thres = threshold;
            cloud = point_cloud;
        }

        void process(pcl::PointIndices::Ptr &inliers, pcl::ModelCoefficients::Ptr &coefficients){
            seg.setOptimizeCoefficients (true);  // Optional
            // Mandatory
            seg.setModelType (pcl::SACMODEL_PLANE); //PLANE model
            seg.setMethodType (pcl::SAC_RANSAC);  //RANSAC method 
            seg.setDistanceThreshold (thres); //determines how close a point must be to the model in order to be considered an inlier

            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);      

              // model parameter: estimated plane parameters in ax + by + cz + d = 0
            std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
        }
        
    private: 
        pcl::SACSegmentation<T> seg; 

};

