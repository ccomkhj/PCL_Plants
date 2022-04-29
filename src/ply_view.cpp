#include <iostream>
//#include <unistd.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>


// Main function
int main(int argc, char **argv)
{
    // Fetch point cloud filename in arguments | Works with PLY files
    std::string filename = "/home/hexaburbach/codes/point_cloud/burbach_farm.ply";

    // Load file | Works with PLY files
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::io::loadPLYFile(filename, *source_cloud);


    // Visualization 
    printf("\n Point cloud colors :\n"
        " \t white \t = \t original point cloud \n");

    pcl::visualization::PCLVisualizer viewer(" Point Cloud Datsets Visualizer");
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Set background to a dark grey                            

                                                    // Define R,G,B colors for the point cloud 
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(source_cloud);

    // We add the point cloud to the viewer and pass the color handler 
    viewer.addPointCloud(source_cloud, rgb, "original_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");

    //******************************
    std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> points = source_cloud->points;

    //******************************
    while (!viewer.wasStopped()) {   // Display the visualizer until the 'q' key is pressed
        viewer.spinOnce();

    }

    return 0;
} // End main()