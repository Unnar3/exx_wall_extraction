#include <ros/ros.h>
#include <compression/plane_extraction.h>
#include <exx_common_node/Node.h>
// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
// OTHER
#include <pcl/console/parse.h>
#include <vector>
#include <compression.h>


// DEFINITIONS
#define HZ                  10
#define BUFFER_SIZE         1
#define NODE_NAME           "test_node"
const std::string METAROOM = "/home/unnar/catkin_ws/src/Metarooms/room_0/";

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::ModelCoefficients ModelCoeffT;
typedef pcl::PlanarRegion<PointT> PlanarRegT;
using namespace std;
using namespace exx::compression;

class PlaneTest// : public exx::Node
{

    ros::NodeHandle nh;

    double	voxel_leaf_size;
    double 	seg_distance, seg_percentage, normal_distance_weight;
    std::string point_cloud_name;

public:
    PlaneTest()
    {
        nh = ros::NodeHandle("~");
    }

    void testCompression()
    {
        int serial;
        std::cout << "testing that it works." << std::endl;
        nh.param<int>("serial", serial, 50);
        std::string savePath;
        nh.param<std::string>("savePath", savePath, "./");
        // nh.getParam("serial", serial);

        std::cout << "Parameter test: " << serial << std::endl;

        PointCloudT::Ptr cloud (new PointCloudT ());
        pcl::io::loadPCDFile ("/home/unnar/catkin_ws/src/Metarooms/room_2/complete_cloud.pcd", *cloud);
        // pcl::io::savePCDFileBinary ("input_cloud.pcd", *cloud);
        EXX::compression cmprs;
        cmprs.setInputCloud(cloud);
        cmprs.triangulate();
        cmprs.saveMesh(savePath);
        std::cout << "Cloud should be saved." << std::endl;
    }

private:
    void voxelGridCloud(pcl::PointCloud<PointT>::Ptr cloud, float leaf_size)
    {
        pcl::PointCloud <PointT>::Ptr cloud_voxel (new pcl::PointCloud <PointT> ());
        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (leaf_size, leaf_size, leaf_size);
        sor.filter (*cloud_voxel);
        *cloud = *cloud_voxel;
    }
    
};

int main(int argc, char **argv) {

    ros::init(argc, argv, NODE_NAME);

    PlaneTest test;
    ros::Rate loop_rate(HZ);


    test.testCompression();

    return 0;
}