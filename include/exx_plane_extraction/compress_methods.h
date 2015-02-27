//Practice with inheritance, polymorphism, and Abstract Data Types
//header file for Polygon class

#ifndef COMPRESSMETHODS_H
#define COMPRESSMETHODS_H

#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>

#include <vector>

typedef pcl::PointXYZ PointT;

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
    void removePlanes(pcl::PointCloud<PointT>::Ptr cloud, std::vector<pcl::PointCloud<PointT>::Ptr > *planes, std::vector<pcl::ModelCoefficients::Ptr > *coeffs)
    {
        pcl::SACSegmentation<PointT> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT> ());
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.02);

        int i=0, nr_points = (int) cloud->points.size ();
        while (cloud->points.size() > 0.3 * nr_points)
        {
            // Define for each plane we find
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());

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
            extract.filter (*cloud_f);
            cloud.swap (cloud_f);

            (*coeffs).push_back(coefficients);
            (*planes).push_back(cloud_plane);
        }
    }


    // Project inliers conforming to a plane model exactly on the plane
    // input:   vector of point clouds where each cloud contains only points 
    //          belonging to one plane and a vector of plane coefficients
    // output:  vector of point clouds where each cloud contains only points 
    //          exactly on one plane.
    void projectToPlane(std::vector<pcl::PointCloud<PointT>::Ptr > *planes, std::vector<pcl::ModelCoefficients::Ptr > *coeffs)
    {
        int n = planes->size();
        if(n == 0){
            std::cout << "No planes, nothing to do" << std::endl;
            return;
        }
        // Create the projection object
        pcl::ProjectInliers<PointT> proj;
        proj.setModelType (pcl::SACMODEL_PLANE);

        for (int i = 0; i < n; i++){ 
            proj.setInputCloud ( (*planes)[i] );
            proj.setModelCoefficients ( (*coeffs)[i]);
            proj.filter ( *(*planes)[i] );
        }
    }


    // Calculates the concave hull of planes.
    // Input:   vector containing point clouds where all points belong to a single plane  
    // Output:  vector containing point clouds where all points belong to the concave hull 
    //          of a single plane  
    void planeToConcaveHull(std::vector<pcl::PointCloud<PointT>::Ptr > *planes, std::vector<pcl::PointCloud<PointT>::Ptr > *hulls){

        int n = planes->size();
        if(n == 0){
            std::cout << "No planes, nothing to do" << std::endl;
            return;
        }

        // Create a Concave Hull representation of the projected inliers
        pcl::ConcaveHull<PointT> chull;
        for (int i = 0; i < n; i++){ 
            pcl::PointCloud<PointT>::Ptr cloud_hull (new pcl::PointCloud<PointT> ());
            chull.setInputCloud ( (*planes)[i] );
            chull.setAlpha (0.1);
            chull.reconstruct (*cloud_hull);
            (*hulls).push_back(cloud_hull);
            std::cout << "plane count: " << (*planes)[i]->points.size() << std::endl;
            std::cout << "hull count: " << cloud_hull->points.size() << std::endl;
        }
    }


    private:
};

#endif