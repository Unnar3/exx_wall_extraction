#include <ros/ros.h>
#include <compression/plane_extraction.h>
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
using namespace exx;

class PlaneTest
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

    void testPlanes(PointCloudT::Ptr cloud)
    {
        std::cout << "testing that it works." << std::endl;

        std::cout << cloud->width << "  " <<  cloud->height << std::endl; 

        voxelGridCloud(cloud, 0.02);
 
        std::vector<PointCloudT::Ptr > planes; 
        std::vector<ModelCoeffT::Ptr > coeffs;
        planeExtraction::RANSACPlanes(cloud, &planes, &coeffs);
        std::cout << "found " << planes.size() << " planes using RANSAC." << std::endl;

        /* Need organized point cloud which we don't have;
        std::vector<PlanarRegT, Eigen::aligned_allocator< PlanarRegT > > regions;
        regions =  exx::planes::MPSPlanes(cloud);
        std::cout << "found " << regions.size() << " planes using MPS." << std::endl;        
        */
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

    std::cout << "Creating point cloud" << std::endl;
    // Create a point cloud containing a single plain
    std::vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    std::string filename = METAROOM;
    if (filenames.size() != 0){
        filename += argv[filenames[0]];
    }
    if (pcl::io::loadPCDFile (filename, *cloud) < 0)  {
        std::cout << "Error loading point cloud " << std::endl;
        return 0;
    }

    test.testPlanes(cloud);

    return 0;
}