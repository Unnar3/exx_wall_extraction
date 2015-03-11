// Class to make plane extraction easier.
// Uses both RANSAC from PCL as well as 
// OrganizedMultiPlaneSegmentation from PCL

#ifndef PLANEEXTRACTION_H
#define PLANEEXTRACTION_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::ModelCoefficients ModelCoeffT;
typedef pcl::PlanarRegion<PointT> PlanarRegT;


namespace exx{
	namespace compression{
		namespace planeExtraction{

			// Plane segmentation using RANSAC from PCL
			// input:
			// PointCloudT::Ptr cloud                 = input cloud, want to find planes in this cloud.
			// std::vector<PointCloudT::Ptr > *planes = empty vector of pointclouds. 
			// std::vector<ModelCoeffT::Ptr > *coeffs = empty vector of modelcoefficients.
			// int max_iterations                     = Maximum number of iterations to search for planes.  
			// double distance_threshold              = Maximum distance from plane to be counted as inlier.
			// int min_inliers                        = Minimum number of inliers to be counted as plane.
			// Output:
			// std::vector<PointCloudT::Ptr > *planes = Vector containing all planes as pointclouds. 
			// std::vector<ModelCoeffT::Ptr > *coeffs = Vector containing coefficients for all planes.
		    void RANSACPlanes(PointCloudT::Ptr cloud, std::vector<PointCloudT::Ptr > *planes, std::vector<ModelCoeffT::Ptr > *coeffs, int max_iterations, double distance_threshold, int min_inliers)
		    {
		        pcl::SACSegmentation<PointT> seg;
		        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		        PointCloudT::Ptr cloud_f (new PointCloudT ());
		        seg.setOptimizeCoefficients (true);
		        seg.setModelType (pcl::SACMODEL_PLANE);
		        seg.setMethodType (pcl::SAC_RANSAC);
		        seg.setMaxIterations (max_iterations);
		        seg.setDistanceThreshold (distance_threshold);

		        int i=0, nr_points = (int) cloud->points.size ();
		        while (i < 100 && cloud->points.size() > 0 && cloud->points.size() > 0.1 * nr_points)
		        {
		            // Define for each plane we find
		            ModelCoeffT::Ptr coefficients (new ModelCoeffT);
		            PointCloudT::Ptr cloud_plane (new PointCloudT ());

		            // Segment the largest planar component from the remaining cloud
		            seg.setInputCloud (cloud);
		            seg.segment (*inliers, *coefficients);
		            if (inliers->indices.size () == 0)
		            {
		                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
		                break;
		            }
		            if(inliers->indices.size() < min_inliers){
		                i++;
		                continue;
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
		            i++;
		        }
		    }

		    // Plane segmentation using RANSAC from PCL
		    // See definition above.
		    // Uses default values for max_iterations, distance_threshold, min_inliers.
		    void RANSACPlanes(PointCloudT::Ptr cloud, std::vector<PointCloudT::Ptr > *planes, std::vector<ModelCoeffT::Ptr > *coeffs)
			{
				static int max_iterations        = 100;
				static double distance_threshold = 0.02;
				static int min_inliers           = 200;
				RANSACPlanes(cloud, planes, coeffs, max_iterations, distance_threshold, min_inliers);
			} 


			// Plane segmentation using OrganizedMultiPlaneSegmentation in PCL.
			// NEEDS ORGANIZED POINT CLOUD.
			std::vector<PlanarRegT, Eigen::aligned_allocator< PlanarRegT > > MPSPlanes(PointCloudT::Ptr cloud)
			{
				pcl::NormalEstimation<PointT, pcl::Normal> n;
		        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
		        tree->setInputCloud (cloud);
		        n.setInputCloud (cloud);
		        n.setSearchMethod (tree);
		        n.setKSearch (20);
		        n.compute (*normals);


				pcl::OrganizedMultiPlaneSegmentation< PointT, pcl::Normal, pcl::Label > mps;
				mps.setMinInliers (200);
				mps.setAngularThreshold (0.017453 * 10.0); // 2 degrees
				mps.setDistanceThreshold (0.02); // 2cm
				mps.setInputNormals (normals);
				mps.setInputCloud (cloud);
				std::vector<PlanarRegT, Eigen::aligned_allocator< PlanarRegT > > regions; 
				mps.segmentAndRefine (regions);

				return regions;
			}
		}
	}
}

#endif

