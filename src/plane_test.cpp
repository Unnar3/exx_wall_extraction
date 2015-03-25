#include <ros/ros.h>
#include <exx_common_node/Node.h>
#include <exx_compression/compression.h>
// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// OTHER
#include <pcl/console/parse.h>
#include <vector>


// DEFINITIONS
#define PRINT               1
#define HZ                  10
#define BUFFER_SIZE         1
#define NODE_NAME           "test_node"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
using namespace std;

class PlaneTest// : public exx::Node
{

    bool printParameters;
    double SVVoxelResolution;
    double SVSeedResolution;
    double RANSACDistanceThreshold;
    ros::NodeHandle nh;

    // Params
    std::string savePath;
    std::string cloudPath;

    std::string point_cloud_name;

public:
    PlaneTest() : printParameters(true)
    {
        nh = ros::NodeHandle("~");
        loadParams();
    }

    void testCompression()
    {
        std::cout << "testing that it works." << std::endl;

        PointCloudT::Ptr cloud (new PointCloudT ());
        pcl::io::loadPCDFile (cloudPath, *cloud);
        pcl::io::savePCDFileBinary ("input_cloud.pcd", *cloud);
        EXX::compression cmprs;
        cmprs.setInputCloud(cloud);
        cmprs.setSVVoxelResolution(SVVoxelResolution);
        cmprs.setSVSeedResolution(SVSeedResolution);
        cmprs.setRANSACDistanceThreshold(RANSACDistanceThreshold);
        cmprs.triangulate();
        std::vector<EXX::cloudMesh> cmesh;
        cmesh = cmprs.returnCloudMesh();
        pcl::io::savePCDFileASCII ("cmprs_output_cloud.pcd", *cmesh[0].cloud);
        pcl::io::saveVTKFile ("cmprs_output_mesh.vtk", cmesh[0].mesh);
        std::cout << "Cloud should be saved." << std::endl;
    }

private:
    void loadParams(){
        std::cout << "" << std::endl;
        std::cout << "##################" << std::endl;
        std::cout << "LOADING PARAMETERS" << std::endl;

        add_param( "printParameters", printParameters, true);
        add_param( "savePath", savePath, "./");
        add_param( "cloudPath", cloudPath, "./");
        add_param( "SVVoxelResolution", SVVoxelResolution, 0.1);
        add_param( "SVSeedResolution", SVSeedResolution, 0.3);
        add_param( "RANSACDistanceThreshold", RANSACDistanceThreshold, 0.04);

        std::cout << "##################" << std::endl;
        std::cout << "" << std::endl;
    }

    template <typename T, typename C>
    void add_param(const std::string & name, T & destination, const C & default_value) {
        nh.param<T>(name, destination, default_value);
        if (printParameters){
            if ( typeid(T) == typeid(bool) ){
                std::cout << std::boolalpha << name << ": " << destination << std::endl;
            } else {
                std::cout << name << ": " << destination << std::endl;
            }
        }
    }

    /**
     * Fix for above method when destination is a std::string but default value is a char[].
     */
    void add_param(const std::string & name, std::string & destination, const char default_value[]) {
        add_param(name, destination, std::string(default_value));
    }
};

int main(int argc, char **argv) {

    ros::init(argc, argv, NODE_NAME);

    PlaneTest test;
    ros::Rate loop_rate(HZ);


    test.testCompression();

    return 0;
}