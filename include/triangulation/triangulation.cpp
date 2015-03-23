#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/Vertices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <math.h>
#include <numeric>

#include "triangulation.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace EXX {
namespace triangulation {

triangulation::triangulation()
{
	cloud = PointCloudT::Ptr (new PointCloudT ());
	searchRadius = 7;
	KNearestNeighbours = 7;
};
triangulation::~triangulation(){};

void triangulation::setInputCloud(cloudB input_cloud)
{
	*cloud = *input_cloud.cloud;
	for (size_t i = input_cloud.boundaryIndexLow; i <= input_cloud.boundaryIndexHigh; i++ )
	{
		boundary_points.push_back(i);
	}
}

std::vector<std::vector<int> > triangulation::triangulate()
{			
	// std::vector<std::vector<int> > triangles;
	// std::vector<int> triangle (3);
	int triangle_count = 0;
	bool justStarted = true;
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud (cloud);

	std::vector<int> pointIdxNKNSearch(KNearestNeighbours);
	std::vector<float> pointNKNSquaredDistance(KNearestNeighbours);
	PointT currPoint;

	int outer_loop_count = 0;
	std::vector<int>::iterator it;
	while(old_boundary_points.size() < cloud->points.size())
	{
		old_boundary_points.insert(old_boundary_points.end(), boundary_points.begin(), boundary_points.end());
		std::cout << "ytri lykkja" << std::endl;
		it = boundary_points.begin();
		while(it != --boundary_points.end())
		{
			std::cout << "hmm" << std::endl;
			currPoint = cloud->points[*it];
			int numberOfNeighbours = kdtree.nearestKSearch(currPoint, KNearestNeighbours, pointIdxNKNSearch, pointNKNSquaredDistance);
			// triangulation::printSelectedPoints(pointIdxNKNSearch, currPoint);
			int closestInternal = triangulation::findFirstNonBoundaryIndex(pointIdxNKNSearch);
			// Check if the closest point should be connected through next boundary 
			// or last boundary point
			int ind = 0;
			if (justStarted)
			{
				// ind = getIndexToBestBoundaryPoint(closestInternal, *it+1, boundary_points.back());
				justStarted = false;
				triangulation::addTriangle(*it, closestInternal, *(it+1));
				completed_points.push_back(closestInternal);	
				it++;				
			}
			else
			{
				// ind = getIndexToBestBoundaryPoint(closestInternal, *it, *it+1);
				if (closestInternal != -1 && triangulation::distanceBetweenPoints(closestInternal, *it) < triangulation::distanceBetweenPoints(completed_points.back(), *it+1))
				{
					triangulation::addTriangle(*it, completed_points.back(), closestInternal);
					completed_points.push_back(closestInternal);
				}
				else{
					triangulation::addTriangle(*it, completed_points.back(), *(it+1));
					it++;
				}
			}
			triangle_count++;

			// The loop has finished, need to close it;
			if (it == --boundary_points.end())
			{
				if(triangulation::distanceBetweenPoints(boundary_points.front(), completed_points.back()) < triangulation::distanceBetweenPoints(boundary_points.back(), completed_points.front()))
				{
					triangulation::addTriangle(*it, completed_points.back(), boundary_points.front());
					triangulation::addTriangle(*it, completed_points.back(), completed_points.front());
				}
				else
				{
					triangulation::addTriangle(boundary_points.back(), completed_points.back(), completed_points.front());
					triangulation::addTriangle(boundary_points.back(), completed_points.front(), boundary_points.front());;
				}
			}
		}
		boundary_points.clear();
		boundary_points.swap(completed_points);
		outer_loop_count++;
		justStarted = true;
	}

	triangulation::printTriangles();
	return triangles;
}

PointT triangulation::getSeedPoint()
{
	return cloud->points[8];
}

void triangulation::printSelectedPoints(std::vector<int> Points, PointT point)
{
	std::cout << "Center point is:" << std::endl;
	std::cout << point.x << " " << point.y << " " << point.z << std::endl;
	std::cout << "Surrounding points are:" << std::endl;
	for(size_t i = 0; i < Points.size(); i++)
	{
		std::cout << cloud->points[Points[i]].x << " ";
		std::cout << cloud->points[Points[i]].y << " ";
		std::cout << cloud->points[Points[i]].z << std::endl;
	}
}

int triangulation::findFirstNonBoundaryIndex(std::vector<int> indexes)
{
	std::vector<int>::iterator it_b;
	std::vector<int>::iterator it_i;
	for (size_t i = 0; i < indexes.size(); i++)
	{
		it_b = find(old_boundary_points.begin(), old_boundary_points.end(), indexes[i]);
		if (it_b == old_boundary_points.end())
		{
			it_i = find(completed_points.begin(), completed_points.end(), indexes[i]);
			if (it_i == completed_points.end())
			{
				return indexes[i];
			}
		}
	}
	return -1;
}
int triangulation::getIndexToBestBoundaryPoint(int internal, int next, int last)
{
	if (triangulation::distanceBetweenPoints(internal, next) > triangulation::distanceBetweenPoints(internal, last))
	{
		return last;
	}
	return next;
}

double triangulation::distanceBetweenPoints(int a, int b)
{
	double x = 0;
	double y = 0;
	double z = 0;
	x = cloud->points[a].x-cloud->points[b].x;
	y = cloud->points[a].y-cloud->points[b].y;
	z = cloud->points[a].z-cloud->points[b].z;
	return std::sqrt(x*x + y*y + z*z);
}

std::vector<double> triangulation::orderPointsAccordingToAngle(std::vector<int> Points)
{

	std::vector<float> a(3);			
	a[0] = cloud->points[Points[0]].x - cloud->points[Points[1]].x;
	a[1] = cloud->points[Points[0]].y - cloud->points[Points[1]].y;
	a[2] = cloud->points[Points[0]].z - cloud->points[Points[1]].z;

	double a_square = a[0]*a[0] + a[1]*a[1] + a[2]*a[2];
	
	// float dot = point.x*x2 + point.y*y2 + z1*z2
	// lenSq1 = x1*x1 + y1*y1 + z1*z1
	// lenSq2 = x2*x2 + y2*y2 + z2*z2
	// angle = acos(dot/sqrt(lenSq1 * lenSq2))

	std::cout << "order this shit" << std::endl;
	std::vector<double> b(3);
	double b_square;
	double dot;
	std::vector<double> angles;
	angles.push_back(0.0);
	for (size_t i = 2; i < Points.size(); i++)
	{
		b[0] = cloud->points[Points[0]].x - cloud->points[Points[i]].x;
		b[1] = cloud->points[Points[0]].y - cloud->points[Points[i]].y;
		b[2] = cloud->points[Points[0]].z - cloud->points[Points[i]].z;
		b_square = b[0]*b[0] + b[1]*b[1] + b[2]*b[2];
		dot = std::inner_product(a.begin(), a.end(), b.begin(   ), 0);
		angles.push_back(acos( dot / std::sqrt(a_square * b_square) ) );
	}
	return angles;
}

void triangulation::addTriangle(int a, int b, int c)
{
	// Check to see that the triangle is valid;
	if (a == b || a == c || b == c){ return; }
	if (a == -1 || b == -1 || c == -1) { return; }

	std::vector<int> triangle(3);
	triangle[0] = a;
	triangle[1] = b;
	triangle[2] = c;
	triangles.push_back(triangle);
}

void triangulation::printTriangles()
{
	for (size_t i = 0; i < triangles.size(); i++)
	{
		std::cout << "Triangle number: " << i << std::endl;
		std::cout << triangles[i][0] << " ";
		std::cout << triangles[i][1] << " ";
		std::cout << triangles[i][2] << " " << std::endl;
	}
}

}
}