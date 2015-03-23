// Methods for filtering the point cloud
// Based on PCL.

#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/gp3.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointNCloudT;

namespace exx{
	namespace compression{
		namespace triangulation{


			pcl::PolygonMesh greedyProjectionTriangulation(PointCloudT::Ptr plane)
			{
			    pcl::PolygonMesh triangle;

			    // Normal estimation*
			    pcl::NormalEstimation<PointT, pcl::Normal> n;
        		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
			    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
			    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
			    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

			    // Initialize objects
			    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;

			    // Set the maximum distance between connected points (maximum edge length)
			    gp3.setSearchRadius (0.3);

			    // Set typical values for the parameters
			    gp3.setMu (2.5);
			    gp3.setMaximumNearestNeighbors (100);
			    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
			    gp3.setMinimumAngle(M_PI/20); // 10 degrees
			    gp3.setMaximumAngle(2*M_PI/2.5); // 120 degrees
			    gp3.setNormalConsistency(false);

			    tree->setInputCloud (plane);
			    n.setInputCloud (plane);
			    n.setSearchMethod (tree);
			    n.setKSearch (20);
			    n.compute (*normals);

			    // Concatenate the XYZ and normal fields*
			    pcl::concatenateFields (*plane, *normals, *cloud_with_normals);

			    tree2->setInputCloud (cloud_with_normals);

			    // Get result
			    gp3.setInputCloud (cloud_with_normals);
			    gp3.setSearchMethod (tree2);
			    gp3.reconstruct (triangle);
			    return triangle;
			}

			std::vector<pcl::PolygonMesh> greedyProjectionTriangulation(std::vector<PointCloudT::Ptr > *planes)
			{
			    pcl::PolygonMesh mesh;
			    std::vector<pcl::PolygonMesh> triangles;

			    int number_of_planes = planes->size();
			    if(number_of_planes == 0){
			        std::cout << "No planes, nothing to do" << std::endl;
			        return triangles;
			    }

			    for (int i = 0; i < number_of_planes; i++){
			        triangles.push_back( greedyProjectionTriangulation( (*planes)[i] ) );
			    }

			    return triangles;
			}

		}
	}
}

#endif