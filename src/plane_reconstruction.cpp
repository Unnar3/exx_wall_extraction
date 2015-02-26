#include <ros/ros.h>
// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
//#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
// OTHER
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <cstdlib>
#include <vector>
#include <cmath>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */


typedef pcl::PointXYZ PointT;

int main(int argc, char **argv) {

  ros::init(argc, argv, "PLANE_RECONSTRUCTION");
  ros::NodeHandle nh;
  nh = ros::NodeHandle("~");
  ros::Rate loop_rate(10);



  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  // Fill in the cloud data
  cloud->width  = 100000;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  // Generate the data
  srand (time(NULL));
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1.0;
    //cloud->points[i].r =  255;//rand()%255;
    //cloud->points[i].g =  255;//rand()%255;
    //cloud->points[i].b =  0;//rand()%255;
  }
  pcl::io::savePCDFileASCII ("constructed_plane.pcd", *cloud);
  std::cerr << "Saved " << cloud->points.size () << " data points to constructed_plane.pcd." << std::endl;

  pcl::PointCloud <PointT>::Ptr cloud_voxel (new pcl::PointCloud <PointT>);
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (*cloud_voxel);
  pcl::io::savePCDFileASCII ("constructed_plane_voxel.pcd", *cloud_voxel);
  std::cerr << "Saved " << cloud_voxel->points.size () << " data points to constructed_plane_voxel.pcd." << std::endl;

  // Normal estimation*
  pcl::NormalEstimation<PointT, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud_voxel);
  n.setInputCloud (cloud_voxel);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud_voxel, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.2);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  pcl::io::saveVTKFile ("mesh.vtk", triangles);
}