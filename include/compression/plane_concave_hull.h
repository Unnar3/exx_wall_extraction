// Methods for filtering the point cloud
// Based on PCL.

#ifndef CONCAVEHULL_H
#define CONCAVEHULL_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


#define LEAF_SIZE_DEFAULT 0.02

namespace exx{
	namespace compression{
		namespace concaveHull{

			// Calculates the concave hull of planes.
			// Input:   vector containing point clouds where all points belong to a single plane
			// Output:  vector containing point clouds where all points belong to the concave hull
			//          of a single plane
			std::vector<PointCloudT::Ptr > planeToConcaveHull(std::vector<PointCloudT::Ptr > *planes)
			{
				std::vector<PointCloudT::Ptr > hulls;
			    int number_of_planes = planes->size();

			    // Create a Concave Hull representation of the projected inliers
			    pcl::ConcaveHull<PointT> chull;
			    for (int i = 0; i < number_of_planes; i++){
			        PointCloudT::Ptr cloud_hull (new PointCloudT ());
			        chull.setInputCloud ( (*planes)[i] );
			        chull.setAlpha (0.1);
			        chull.setKeepInformation (true);
			        chull.reconstruct (*cloud_hull);
			        hulls.push_back(cloud_hull);
			        std::cout << "plane count: " << (*planes)[i]->points.size() << std::endl;
			        std::cout << "hull count: " << cloud_hull->points.size() << std::endl;
			    }

			    return hulls;
			}

			// Calculates the distance from a point in 3D to a line.
			// input:   PointT points, current and next define a line and nextCheck is a point where we want to know
			//          the distance to the line.
			// output:  Double that defines the distance to the line in meters.
			double pointToLineDistance(PointT current, PointT next, PointT nextCheck)
			{
			    std::vector<float> x0x1;
			    x0x1.push_back(nextCheck.x-current.x);
			    x0x1.push_back(nextCheck.y-current.y);
			    x0x1.push_back(nextCheck.z-current.z);

			    std::vector<float> x0x2;
			    x0x2.push_back(nextCheck.x-next.x);
			    x0x2.push_back(nextCheck.y-next.y);
			    x0x2.push_back(nextCheck.z-next.z);

			    std::vector<float> x1x2;
			    x1x2.push_back(current.x-next.x);
			    x1x2.push_back(current.y-next.y);
			    x1x2.push_back(current.z-next.z);

			    std::vector<float> cross;
			    cross.push_back(x0x1[1]*x0x2[2] - x0x1[2]*x0x2[1]);
			    cross.push_back(x0x1[2]*x0x2[0] - x0x1[0]*x0x2[2]);
			    cross.push_back(x0x1[0]*x0x2[1] - x0x1[1]*x0x2[0]);

			    return sqrt(cross[0]*cross[0] + cross[1]*cross[1] + cross[2]*cross[2]) / sqrt(x1x2[0]*x1x2[0] + x1x2[1]*x1x2[1] + x1x2[2]*x1x2[2]);
			}
			

			// Simplifies the concave hull using the Reumann-Witkam algorithm
			// Input : Vector of clouds where each cloud represents the concave hull of a plane,
			// Output: Vector of clouds where each cloud represents the simplified concave hull of a plane,
			void reumannWitkamLineSimplification(std::vector<PointCloudT::Ptr > *hulls)
			{
			    int number_of_planes = hulls->size();
			    if(number_of_planes == 0){
			        std::cout << "No planes, nothing to do" << std::endl;
			        return;
			    }

			    double eps = 0.02;
			    double distToLine;
			    int numberOfPointsInPlane;
			    int j_current, j_next, j_nextCheck, j_last;
			    PointT current, next, last, nextCheck;
			    PointCloudT::Ptr currentPlane;

			    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
			    pcl::ExtractIndices<PointT> extract;

			    // Loop through all planes
			    for (int i = 0; i < number_of_planes; i++){
			        currentPlane = (*hulls)[i];
			        numberOfPointsInPlane = currentPlane->points.size();

			        j_current = 0;
			        j_next = j_current + 1;
			        j_last = j_next;
			        j_nextCheck = j_next + 1;
			        inliers->indices.clear();

			        // Loop through all points in the plane and find redundant points.
			        while(j_nextCheck < numberOfPointsInPlane){
			            current = currentPlane->points[j_current];
			            next = currentPlane->points[j_next];
			            last = currentPlane->points[j_last];
			            nextCheck = currentPlane->points[j_nextCheck];
			            distToLine = pointToLineDistance(current, next, nextCheck);
			            if ( distToLine < eps ){
			                if ( j_next != j_last ){
			                    inliers->indices.push_back(j_last);
			                }
			                j_last++;
			                j_nextCheck++;
			            } else {
			                inliers->indices.push_back(j_next);
			                j_current = j_nextCheck;
			                j_next = j_current + 1;
			                j_last = j_next;
			                j_nextCheck = j_next + 1;
			            }
			        }
			        // Remove the redundant points.
			        extract.setInputCloud (currentPlane);
			        extract.setIndices (inliers);
			        extract.setNegative (true);
			        extract.filter (*currentPlane);
			    }
			}
		}
	}
}

#endif