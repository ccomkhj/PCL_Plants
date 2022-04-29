
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
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/pca.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <math.h>

using namespace std::chrono_literals;
int
 main (int argc, char** argv)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> ("/home/hexaburbach/codes/point_cloud/data/burbach_farm.pcd", *cloud) == -1)
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }

  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (200); // The number of neighbors to analyze for each point is set to 50,
  sor.setStddevMulThresh (1.5); // all points who have a distance larger than 1 standard deviation of the mean distance to the query point will be marked as outliers and removed
  sor.filter (*cloud_filtered);
  cloud->clear();


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
  cloud_filtered->clear();

  ///////////////////////////////Get the plane after axis alignment//////////////////////////////////////////////

  pcl::ModelCoefficients::Ptr coefficients2 (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg2; 
  seg2.setOptimizeCoefficients (true);  // Optional
  // Mandatory
  seg2.setModelType (pcl::SACMODEL_PLANE); //PLANE model
  seg2.setMethodType (pcl::SAC_RANSAC);  //RANSAC method 
  seg2.setDistanceThreshold (0.01); //determines how close a point must be to the model in order to be considered an inlier

  seg2.setInputCloud (transformed_cloud);
  seg2.segment (*inliers2, *coefficients2);


  // model parameter: estimated plane parameters in ax + by + cz + d = 0
  std::cerr << "Model coefficients: " << coefficients2->values[0] << " " 
                                      << coefficients2->values[1] << " "
                                      << coefficients2->values[2] << " " 
                                      << coefficients2->values[3] << std::endl;

  float z2center = -coefficients2->values[3]/coefficients2->values[2];

  pcl::PointCloud<pcl::PointXYZ>::Ptr center(new pcl::PointCloud<pcl::PointXYZ>());


  ////////////////////////////////////////////////////////////////////////////////////////////////////////

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr z_filtered_transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

  /// Filter using z-axis
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (transformed_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (z2center-0.2, z2center+0.2); // need to get offset value using statistics.
  pass.filter (*z_filtered_transformed_cloud);
  transformed_cloud->clear();

 
	/////////////////////////////
	

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr x_filtered_transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

  /// Filter using x-axis
  pcl::PassThrough<pcl::PointXYZRGB> pass_x;
  pass_x.setInputCloud (z_filtered_transformed_cloud);
  pass_x.setFilterFieldName ("x");
  pass_x.setFilterLimits (-0.7, +0.7); // need to get offset value using statistics.
  pass_x.filter (*x_filtered_transformed_cloud);

  z_filtered_transformed_cloud->clear();

  ///////// Concave /////////

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ConcaveHull<pcl::PointXYZRGB> chull;
  chull.setInputCloud (x_filtered_transformed_cloud);
  chull.setAlpha (0.25);
  chull.reconstruct (*cloud_hull);
  x_filtered_transformed_cloud->clear();

  //////////////////////////////

  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
  // pcl::ConvexHull<pcl::PointXYZRGB> chull;
  // chull.setInputCloud (x_filtered_transformed_cloud);
  // chull.reconstruct (*cloud_hull);
  // x_filtered_transformed_cloud->clear();

  //////////////////

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor(1,1,1);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud_hull, "concave cloud");
  viewer->addPlane(plane_coeff_xy, "XYplane"); // This is xy plane.
  viewer->addCoordinateSystem (1.0); // x=red axis, y=green axis, z=blue axis z direction

    ///////////////////////
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }
  return (0);
}