
#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/auto_io.h>

#include "volume_compute.hpp"
#include <math.h>

using namespace std::chrono_literals;


int main(int argc, char **argv)
{


  pcl::PolygonMesh mesh;
  if (pcl::io::load("/home/hexaburbach/codes/point_cloud/data/drone_burbach_one_lettuce.ply", mesh) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  Volume_compute vc(mesh);
  std::cout << "Volume (in m^2): " << vc.volumeOfMesh() << std::endl;

  /////////////////////////////   if you want to print every point, then uncomment it.   ///////////////////////////////

  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  // if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/hexaburbach/codes/point_cloud/data/drone_burbach_one_lettuce.pcd", *cloud) == -1)
  // {
  //   std::cout << "Cloud reading failed." << std::endl;
  //   return (-1);
  // }

  // std::cerr << *cloud << std::endl;
  // for (const auto& point: *cloud)
  //   std::cout << "    " << point.x
  //             << " "    << point.y
  //             << " "    << point.z << std::endl;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPolygonMesh(mesh,"meshes",0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }
  return (0);
  
}