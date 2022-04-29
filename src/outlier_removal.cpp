
#include <iostream>
#include <thread>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <math.h>

using namespace std::chrono_literals;
int
 main (int argc, char** argv)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> ("/home/hexaburbach/codes/point_cloud/burbach_farm.pcd", *cloud) == -1)
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }


//   // Fill in the cloud data
//   cloud->width  = 15;
//   cloud->height = 1;
//   cloud->points.resize (cloud->width * cloud->height);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (200); // The number of neighbors to analyze for each point is set to 50,
  sor.setStddevMulThresh (1.5); // all points who have a distance larger than 1 standard deviation of the mean distance to the query point will be marked as outliers and removed
  sor.filter (*cloud_filtered);


  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;
  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;


  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg; 
  seg.setOptimizeCoefficients (true);  // Optional
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE); //PLANE model
  seg.setMethodType (pcl::SAC_RANSAC);  //RANSAC method 
  seg.setDistanceThreshold (0.01); //determines how close a point must be to the model in order to be considered an inlier

  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);


  // model parameter: estimated plane parameters in ax + by + cz + d = 0
  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

    // pcl::visualization::CloudViewer viewer ("Cluster viewer");
    
    // viewer.showCloud(cloud);
    // while (!viewer.wasStopped ())
    // {
    // }

  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  // Define a translation of 2.5 meters on the x axis.
  transform_2.translation() << 0.0, 0.0, 0.0;

  pcl::ModelCoefficients plane_coeff_xy;
  plane_coeff_xy.values.resize (4);    // We need 4 values
  plane_coeff_xy.values[0] = 0.0;
  plane_coeff_xy.values[1] = 0.0;
  plane_coeff_xy.values[2] = 1.0;
  plane_coeff_xy.values[3] = 0.0;

  // get X and Y theta to align all point clouds to the global axis.
  float theta_x = atan2( coefficients->values[1], sqrt(pow(coefficients->values[0],2) + pow(coefficients->values[2], 2) ));
  float theta_y = atan2( coefficients->values[0], coefficients->values[2] );
  transform_2.rotate (Eigen::AngleAxisf (theta_x, Eigen::Vector3f::UnitX()));
  transform_2.rotate (Eigen::AngleAxisf (theta_y, Eigen::Vector3f::UnitY()));

  // Print the transformation
  printf ("\nMethod #2: using an Affine3f\n");
  std::cout << transform_2.matrix() << std::endl;

  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*cloud_filtered, *transformed_cloud, transform_2);

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor(0,0,0);
  // viewer->addPointCloud<pcl::PointXYZRGB> (cloud_filtered, "input cloud");
  viewer->addPointCloud<pcl::PointXYZRGB> (transformed_cloud, "transformed cloud");
  // viewer->addPlane(*coefficients, "RANSAC");
  viewer->addPlane(plane_coeff_xy, "XYplane"); // This is xy plane.
  viewer->addCoordinateSystem (1.0); // x=red axis, y=green axis, z=blue axis z direction
    
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }
  return (0);
}