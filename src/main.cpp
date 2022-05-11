
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
#include <pcl/octree/octree_search.h>
#include <vector>
#include <pcl/features/moment_of_inertia_estimation.h>

#include "outremove.hpp"
#include "primitive_seg.hpp"
#include "align2z.hpp"
#include "passthrough.hpp"
#include "moment_inertia.hpp"


#include <math.h>

using namespace std::chrono_literals;
int main(int argc, char **argv)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/hexaburbach/codes/point_cloud/data/burbach_farm.pcd", *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  //////////////// Remove outliers using RANSAC //////////////

  Outremove<pcl::PointXYZRGB, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> outremove(200, 1.5, cloud);
  outremove.process(cloud_filtered);

  //////////////// Create the segmentation object //////////////

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  Primitive_seg<pcl::PointXYZRGB, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> prim_seg(0.01, cloud_filtered);
  prim_seg.process(inliers, coefficients);

  //////////////// Transform all point cloud to align it along z-axis //////////////

  pcl::ModelCoefficients plane_coeff_xy;
  Align2Z<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> align2z(cloud_filtered);
  align2z.compute_mtx(coefficients, plane_coeff_xy);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  align2z.transform(transformed_cloud);

  ///////////////////////////////Get the plane after axis alignment//////////////////////////////////////////////

  pcl::ModelCoefficients::Ptr coefficients2(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers2(new pcl::PointIndices);

  Primitive_seg<pcl::PointXYZRGB, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> prim_seg2(0.01, transformed_cloud);
  prim_seg2.process(inliers2, coefficients2);

  float z2center = -coefficients2->values[3] / coefficients2->values[2];

  /////////////////////////////////////// Passthrough Z //////////////////////////////////////////
  Passthrough<pcl::PointXYZRGB, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pass_through(transformed_cloud);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pass_through.process(filtered_transformed_cloud, z2center, 0.2, "z");

  ////////////////// moment of inertia (compute eigen vector and center of point cloud) //////////////////

  Moment<pcl::PointXYZRGB, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> moment(filtered_transformed_cloud);

	float major_value, middle_value, minor_value;
  std::vector <float> moment_of_inertia, eccentricity;
	pcl::PointXYZRGB min_point_AABB, max_point_AABB, min_point_OBB, max_point_OBB, position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
	Eigen::Vector3f major_vector, middle_vector, minor_vector, mass_center;

  moment.process(moment_of_inertia, eccentricity, min_point_AABB, max_point_AABB, min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB, major_value, middle_value, minor_value, major_vector, middle_vector, minor_vector, mass_center);

  ////////////////////////////////// align to principal axis //////////////////////////////////////
  Align2Z<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> align_major (filtered_transformed_cloud);
  Eigen::Vector3f x_ax;
  x_ax << 1, 0, 0;
  align_major.compute_mtx(major_vector, x_ax, mass_center);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr principal_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  align_major.transform(principal_cloud);

  ////////////////////////////////////// Move to Center //////////////////////////////////////////

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid (*principal_cloud, centroid);

  Align2Z<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> move2center (principal_cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr centered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  move2center.transform(centered_cloud, centroid);


  ////////////////////////////////////// Passthrough Y //////////////////////////////////////////

  Passthrough<pcl::PointXYZRGB, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pass_through_y(centered_cloud);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pass_through_y.process(all_filtered_cloud, 0, 0.6, "y");

  ////////////////////////////////////////////////////////////////////////////////////////////////

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor(0,0,0);
  viewer->addPointCloud<pcl::PointXYZRGB> (all_filtered_cloud, "transformed cloud");
  viewer->addCoordinateSystem (1.0); // x=red axis, y=green axis, z=blue axis z direction

  // TODO: Scaling the point cloud, Include Hull, Meshing, Octree, RGS

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }
  return (0);
}