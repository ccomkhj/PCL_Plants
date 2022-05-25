
#pragma once


#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/auto_io.h>

#include <math.h>


struct Volume_compute{

    pcl::PolygonMesh mesh;

    Volume_compute (pcl::PolygonMesh& Mesh){
        mesh = Mesh;
    }

    float signedVolumeOfTriangle(pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3) 
    {
        float v321 = p3.x*p2.y*p1.z;
        float v231 = p2.x*p3.y*p1.z;
        float v312 = p3.x*p1.y*p2.z;
        float v132 = p1.x*p3.y*p2.z;
        float v213 = p2.x*p1.y*p3.z;
        float v123 = p1.x*p2.y*p3.z;
        return (1.0f/6.0f)*(-v321 + v231 + v312 - v132 - v213 + v123);
    }

    float volumeOfMesh() 
    {
        float vols = 0.0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(mesh.cloud,*cloud);
        for(int triangle=0;triangle<mesh.polygons.size();triangle++)
        {
            pcl::PointXYZ pt1 = cloud->points[mesh.polygons[triangle].vertices[0]];
            pcl::PointXYZ pt2 = cloud->points[mesh.polygons[triangle].vertices[1]];
            pcl::PointXYZ pt3 = cloud->points[mesh.polygons[triangle].vertices[2]];
            vols += signedVolumeOfTriangle(pt1, pt2, pt3);
        }
        cloud->clear();
        return abs(vols);
    }
        

};

