
#include <iostream>
#include <thread>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>


using namespace std::chrono_literals;
int
 main (int argc, char** argv)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> ("/home/hexaburbach/codes/point_cloud/burbach_farm.pcd", *cloud) == -1)
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }


//   // Fill in the cloud data
//   cloud->width  = 15;
//   cloud->height = 1;
//   cloud->points.resize (cloud->width * cloud->height);


  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg; 
  seg.setOptimizeCoefficients (true);  // Optional
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE); //PLANE 모델 사용
  seg.setMethodType (pcl::SAC_RANSAC);  //RANSAC 방법 사용 
  seg.setDistanceThreshold (0.1); //determines how close a point must be to the model in order to be considered an inlier

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);


  //모델 파라미터 출력 estimated plane parameters (in ax + by + cz + d = 0
  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;  

    // pcl::visualization::CloudViewer viewer ("Cluster viewer");
    
    // viewer.showCloud(cloud);
    // while (!viewer.wasStopped ())
    // {
    // }






    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "input cloud");
    
    viewer->addPlane(*coefficients);
    viewer->addCoordinateSystem (1.0);
    
    while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }
  return (0);
}