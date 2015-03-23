// Methods for filtering the point cloud
// Based on PCL.

#ifndef CLOUDFILTERING_H
#define CLOUDFILTERING_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/supervoxel_clustering.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


#define LEAF_SIZE_DEFAULT 0.02

namespace exx{
	namespace compression{
		namespace cloudFiltering{

			// Down samples the point cloud using VoxelGrid filter
			// to make computations easier.
			void voxelGridFiltering(PointCloudT::Ptr cloud, float leaf_size_x, float leaf_size_y, float leaf_size_z)
			{
			    PointCloudT::Ptr cloud_voxel (new PointCloudT ());
			    pcl::VoxelGrid<PointT> sor;
			    sor.setInputCloud (cloud);
			    sor.setLeafSize (leaf_size_x, leaf_size_y, leaf_size_z);
			    sor.filter (*cloud_voxel);
			    *cloud = *cloud_voxel;
			}

			// Same as above, leaf size same for all dimensions.
			void voxelGridFiltering(PointCloudT::Ptr cloud, float leaf_size)
			{
				voxelGridFiltering(cloud, leaf_size, leaf_size, leaf_size);
			}

			// Same as above, leaf size same for all dimensions and using default value.
			void voxelGridFiltering(PointCloudT::Ptr cloud)
			{
				voxelGridFiltering(cloud, (float)LEAF_SIZE_DEFAULT, (float)LEAF_SIZE_DEFAULT, (float)LEAF_SIZE_DEFAULT);
			}



			std::vector<PointCloudT::Ptr > superVoxelClustering(std::vector<PointCloudT::Ptr > *planes, float voxel_res, float seed_res, float color_imp, float spatial_imp)
			{
			    std::vector<PointCloudT::Ptr > super_voxel_planes;

			    int number_of_planes = planes->size();
			    if(number_of_planes == 0){
			        std::cout << "No planes, nothing to do" << std::endl;
			        return super_voxel_planes;
			    }

			    bool use_transform = false;
			    float voxel_resolution = voxel_res;
			    float seed_resolution = seed_res;
			    float color_importance = color_imp;
			    float spatial_importance = spatial_imp;


			    // super.setNormalImportance (normal_importance);

			    for (int i = 0; i <  number_of_planes; i++)
			    {
			        pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution, use_transform);
			        super.setColorImportance (color_importance);
			        super.setSpatialImportance (spatial_importance);
			        PointCloudT::Ptr currentPlane = (*planes)[i];
			        super.setInputCloud (currentPlane);
			        std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
			        super.extract (supervoxel_clusters);

			        super_voxel_planes.push_back(super.getVoxelCentroidCloud ());
			    }

			    return super_voxel_planes;
			}

			std::vector<PointCloudT::Ptr > superVoxelClustering(std::vector<PointCloudT::Ptr > *planes)
			{
				bool use_transform = false;
			    float voxel_resolution = 0.1f;
			    float seed_resolution = 0.3f;
			    float color_importance = 0.5f;
			    float spatial_importance = 0.1f;
			    
			    return superVoxelClustering(planes, voxel_resolution, seed_resolution, color_importance, spatial_importance);
			}
		}
	}
}


#endif