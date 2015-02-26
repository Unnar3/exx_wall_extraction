//Practice with inheritance, polymorphism, and Abstract Data Types
//header file for Polygon class

#ifndef COMPRESSMETHODS_H
#define COMPRESSMETHODS_H

#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <vector>

typedef pcl::PointXYZRGB PointT;

class compressMethods
{
    public:
    
    // Down samples the point cloud using VoxelGrid filter
    // to make computations easier.
    void voxelGridCloud(pcl::PointCloud<PointT>::Ptr cloud, float leaf_size)
    {
        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (leaf_size, leaf_size, leaf_size);
        sor.filter (*cloud);
    }


    // Plane segmentation using RANSAC
    // Create the segmentation object for the planar model and set all the parameters
    void removePlanes(pcl::PointCloud<PointT>::Ptr cloud, std::vector<pcl::PointCloud<PointT> > *planes, std::vector<pcl::ModelCoefficients > *coeffs)
    {
        pcl::SACSegmentation<PointT> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.02);

        int i=0, nr_points = (int) cloud->points.size ();
        while (cloud->points.size() > 0.3 * nr_points)
        {
            std::cout << "hmmmm" << std::endl;
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }

            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud (cloud);
            extract.setIndices (inliers);
            extract.setNegative (false);

            // Get the points associated with the planar surface
            extract.filter (*cloud_plane);

            // Remove the planar inliers, extract the rest
            extract.setNegative (true);
            extract.filter (*cloud);

            (*coeffs).push_back(*coefficients);
            (*planes).push_back(*cloud_plane);
        }
    }

    private:
};

#endif