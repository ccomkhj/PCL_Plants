
#pragma once

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

template <class T, class P>
class Moment
{
public:
    P cloud;

    Moment(P &point_cloud)
    {
        cloud = point_cloud;
    }

    void process(std::vector<float>& moment_of_inertia, std::vector<float>& eccentricity, T& min_point_AABB, T& max_point_AABB, T& min_point_OBB, T& max_point_OBB, T& position_OBB, Eigen::Matrix3f& rotational_matrix_OBB, float& major_value, float& middle_value, float& minor_value, Eigen::Vector3f& major_vector, Eigen::Vector3f& middle_vector, Eigen::Vector3f& minor_vector, Eigen::Vector3f& mass_center)
    {
        feature_extractor.setInputCloud(cloud);
        feature_extractor.compute();
        feature_extractor.getMomentOfInertia(moment_of_inertia);
        feature_extractor.getEccentricity(eccentricity);
        feature_extractor.getAABB(min_point_AABB, max_point_AABB);
        feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        feature_extractor.getEigenValues(major_value, middle_value, minor_value);
        feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
        feature_extractor.getMassCenter(mass_center);
    }

private:
    pcl::MomentOfInertiaEstimation<T> feature_extractor;
};
