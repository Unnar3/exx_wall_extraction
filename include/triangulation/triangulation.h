#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/Vertices.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointNCloudT;

namespace EXX{
	namespace triangulation {

		struct cloudB
		{
		    PointCloudT::Ptr cloud;
		    int boundaryIndexLow;
		    int boundaryIndexHigh;
		};

		class triangulation {
			PointCloudT::Ptr cloud;
			double searchRadius;
			int KNearestNeighbours;
			std::vector<int> old_boundary_points;
			std::vector<int> boundary_points;
			std::vector<int> completed_points;
			std::vector<std::vector<int> > triangles;

		public:
			triangulation();
			~triangulation();
			void setInputCloud(cloudB input_cloud);
			int getCloudSize(){ return cloud->points.size(); };

			std::vector<std::vector<int> > triangulate();
			
			void setSearchRadius(double radius){ searchRadius = radius; };
			void setKNearestNeighbours(int K){ KNearestNeighbours = K; };


		private:

			PointT getSeedPoint();

			void printSelectedPoints(std::vector<int> points, PointT point);
			std::vector<double> orderPointsAccordingToAngle(std::vector<int> points);
			int findFirstNonBoundaryIndex(std::vector<int> indexes);
			int getIndexToBestBoundaryPoint(int internal, int next, int last);
			double distanceBetweenPoints(int a, int b);
			void addTriangle(int a, int b, int c);
			void printTriangles();
		};
	}
}

#endif