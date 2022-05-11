
#pragma once
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>



template <class T, class P>
class Outremove{
    public:
        int K;
        double thres;
        P cloud;

        Outremove (int k, double threshold, P& point_cloud){
            K = k;
            thres = threshold;
            cloud = point_cloud;
        }

        void process(P& cloud_filtered){
            std::cout << "In process!!";
            std::cerr << "Cloud before filtering: " << std::endl;
            std::cerr << *cloud << std::endl;
            sor.setInputCloud (cloud);
            sor.setMeanK (K); // The number of neighbors to analyze for each point is set to K,
            sor.setStddevMulThresh (thres); // all points who have a distance larger than 1 standard deviation of the mean distance to the query point will be marked as outliers and removed
            sor.filter (*cloud_filtered);

            std::cerr << "Cloud after filtering: " << std::endl;
            std::cerr << *cloud_filtered << std::endl;
            cloud->clear();
        }
    private: 
        pcl::StatisticalOutlierRemoval<T> sor;

};

