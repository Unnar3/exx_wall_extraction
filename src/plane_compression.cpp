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
#include <exx_plane_extraction/compress_methods.h>

// DEFINITIONS
#define HZ                  10
#define BUFFER_SIZE         1
#define NODE_NAME           "plane_compression_node"

typedef pcl::PointXYZRGB PointT;

class PlaneCompression : public compressMethods
{
    ros::NodeHandle nh;

public:
    PlaneCompression()
    {
        nh = ros::NodeHandle("~");
        //add_params();
    }

    void cloudCompress(pcl::PointCloud<PointT>::Ptr cloud)
    {
        std::cout << "Point cloud size BEFORE voxel grid filtering:" << std::endl;
        std::cout <<  cloud->points.size () << " points." << std::endl;
        voxelGridCloud(cloud, 0.05);
        std::cout << "Point cloud size AFTER voxel grid filtering:" << std::endl;
        std::cout <<  cloud->points.size () << " points." << std::endl;


        // Extract planes
        std::vector<pcl::PointCloud<PointT> > planes; 
        std::vector<pcl::ModelCoefficients > coeffs;
        removePlanes(cloud, &planes, &coeffs);
        std::cout << "Number of planes: " << int(planes.size()) << std::endl;
        std::cout << "Number of coefficients: " << int(coeffs.size()) << std::endl;
        std::cout << "One plane AFTER plane extraction:" << std::endl;
        pcl::io::savePCDFileASCII ("compressed_one_plane.pcd", planes[0]);
    }

    private:
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "NODE_NAME");

    // Initialize the compressor class
    PlaneCompression compressor;
    ros::Rate loop_rate(HZ);
  
    // Create a point cloud containing a single plain 
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    // Fill in the cloud data
    cloud->width  = 200000;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    // Generate the data
    srand (time(NULL));
    for (size_t i = 0; i < cloud->points.size()/2; ++i)
    {
      cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud->points[i].z = 1.0;
      cloud->points[i].r = 255;
      cloud->points[i].g = 255;
      cloud->points[i].b = 50;
    }
    for (size_t i = cloud->points.size()/2; i < cloud->points.size(); ++i)
    {
      cloud->points[i].x = 1.0;
      cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud->points[i].r = 255;
      cloud->points[i].g = 50;
      cloud->points[i].b = 50;
    }

    pcl::io::savePCDFileASCII ("compressed_plane.pcd", *cloud);
    compressor.cloudCompress(cloud);
    pcl::io::savePCDFileASCII ("compressed_voxel_plane.pcd", *cloud);
    
    return 0;
}


