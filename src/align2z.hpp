
#pragma once

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <math.h>



template <class P>
class Align2Z{
    public:
        P cloud;

        

        Align2Z (P& point_cloud){
            cloud = point_cloud;
        }

        void compute_mtx(pcl::ModelCoefficients::Ptr &coefficients, pcl::ModelCoefficients &plane_coeff_xy){
            plane_coeff_xy.values.resize (4);    // We need 4 values
            plane_coeff_xy.values[0] = 0.0;
            plane_coeff_xy.values[1] = 0.0;
            plane_coeff_xy.values[2] = 1.0;
            plane_coeff_xy.values[3] = 0.0;

            // get X and Y theta to align all point clouds to the global axis.
            float theta_x = atan2( coefficients->values[1], sqrt(pow(coefficients->values[0],2) + pow(coefficients->values[2], 2) ));
            float theta_y = atan2( coefficients->values[0], coefficients->values[2] );
            
            trans.rotate (Eigen::AngleAxisf (theta_x, Eigen::Vector3f::UnitX()));
            trans.rotate (Eigen::AngleAxisf (theta_y, Eigen::Vector3f::UnitY()));
       
            // Print the transformation
            std::cout << trans.matrix() << std::endl;
        }

        void compute_mtx(Eigen::Vector3f vec_1, Eigen::Vector3f vec_2, Eigen::Vector3f mass_center){
            
            float value = vec_1.dot(vec_2) / vec_1.norm() / vec_2.norm();
            float theta_z = acos (value ) ;
            trans.rotate (Eigen::AngleAxisf (theta_z, Eigen::Vector3f::UnitZ()));
        }

        void transform(P& transformed_cloud){
            trans.translation() << 0,0,0;
             pcl::transformPointCloud (*cloud, *transformed_cloud, trans);
             cloud->clear();
        }
        
        void transform(P& transformed_cloud, Eigen::Vector4f centroid){
            trans.translation() << -centroid (0), -centroid (1), -centroid (2);
             pcl::transformPointCloud (*cloud, *transformed_cloud, trans);
             cloud->clear();
        }
        
    private: 
        Eigen::Affine3f trans = Eigen::Affine3f::Identity();
        
        
        

};

