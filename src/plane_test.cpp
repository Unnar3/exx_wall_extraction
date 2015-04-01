#include <ros/ros.h>
#include <exx_compression/compression.h>
// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
// OTHER
#include <pcl/console/parse.h>
#include <vector>
#include <stdlib.h>
#include <time.h>


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
    double ECClusterTolerance;
    double VoxelResolution;
    int MeanK;
    double stdDev;
    int ECMinClusterSize;
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
        PointCloudT::Ptr cloud_f (new PointCloudT ());
        pcl::io::loadPCDFile (cloudPath, *cloud);
        pcl::io::savePCDFileBinary (savePath + "input_cloud.pcd", *cloud);

        // Create the filtering object
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud (cloud);
        sor.setMeanK (MeanK);
        sor.setStddevMulThresh (stdDev);
        sor.filter (*cloud_f);

        EXX::compression cmprs;
        cmprs.setSVVoxelResolution(SVVoxelResolution);
        cmprs.setSVSeedResolution(SVSeedResolution);
        cmprs.setRANSACDistanceThreshold(RANSACDistanceThreshold);
        cmprs.setECClusterTolerance(ECClusterTolerance);
        cmprs.setECMinClusterSize(ECMinClusterSize);
        cmprs.setVoxelLeafSize(VoxelResolution);
        
        PointCloudT::Ptr voxel_cloud (new PointCloudT ());
        EXX::planesAndCoeffs pac;
        std::vector<PointCloudT::Ptr> c_planes;
        std::vector<PointCloudT::Ptr> hulls;
        std::vector<PointCloudT::Ptr> simplified_hulls;
        std::vector<PointCloudT::Ptr> super_planes;
        std::vector<EXX::cloudMesh> cm;
        clock_t t1,t2,t3,t4,t5,t6,t7,t8,t9;
        t1=clock();
        cmprs.voxelGridFilter(cloud_f, voxel_cloud);
        t2=clock();
        cmprs.extractPlanesRANSAC(voxel_cloud, &pac);
        t3=clock();
        cmprs.projectToPlane(&pac);
        t4=clock();
        cmprs.euclideanClusterPlanes(&pac.cloud, &c_planes);
        t5=clock();
        cmprs.planeToConcaveHull(&c_planes, &hulls);
        t6=clock();
        cmprs.reumannWitkamLineSimplification( &hulls, &simplified_hulls);
        t7=clock();
        cmprs.superVoxelClustering(&c_planes, &super_planes);
        t8=clock();
        cmprs.greedyProjectionTriangulationPlanes(voxel_cloud, &super_planes, &simplified_hulls, &cm);
        t9=clock();

        std::cout << "Total time: " << double(t9-t1) / CLOCKS_PER_SEC << std::endl;
        std::cout << "Voxel time: " << double(t2-t1) / CLOCKS_PER_SEC << std::endl;
        std::cout << "RANSAC time: " << double(t3-t2) / CLOCKS_PER_SEC << std::endl;
        std::cout << "Project time: " << double(t4-t3) / CLOCKS_PER_SEC << std::endl;
        std::cout << "EC time: " << double(t5-t4) / CLOCKS_PER_SEC << std::endl;
        std::cout << "Hull time: " << double(t6-t5) / CLOCKS_PER_SEC << std::endl;
        std::cout << "Simple Hull time: " << double(t7-t6) / CLOCKS_PER_SEC << std::endl;
        std::cout << "Super Voxel time: " << double(t8-t7) / CLOCKS_PER_SEC << std::endl;
        std::cout << "triangulation time: " << double(t9-t8) / CLOCKS_PER_SEC << std::endl;

        // cmprs.setInputCloud(cloud_f);
        
        // // cmprs.triangulate();
        // cmprs.voxelGridFilter();
        // cmprs.extractPlanesRANSAC();
        // cmprs.euclideanClusterPlanes();

        // std::vector<PointCloudT::Ptr > planes_original;
        // std::vector<PointCloudT::Ptr > planes;
        // planes_original = cmprs.returnPlanes();
        // planes = cmprs.returnECPlanes();

        // int r, g, b;
        // PointCloudT::Ptr colored_cloud (new PointCloudT ());
        // std::vector<PointCloudT::Ptr >::iterator it = planes.begin();
        // srand ( time(NULL) );
        // for ( ; it != planes.end() ; ++it ){
        //     r = rand () % 255;
        //     g = rand () % 255;
        //     b = rand () % 255;
        //     std::vector<PointT, Eigen::aligned_allocator_indirection<PointT> >::iterator pit = (*it)->points.begin();
        //     for ( ; pit != (*it)->points.end() ; ++pit ){
        //         (*pit).r = r;
        //         (*pit).g = g;
        //         (*pit).b = b;
        //     } 
        //     *colored_cloud += **it;
        // }

        // std::cout << "Original size: " << planes_original.size() << std::endl;
        // std::cout << "Segmented size: " << planes.size() << std::endl;
        
        // pcl::io::savePCDFileASCII (savePath + "colored_cloud.pcd", *colored_cloud);

        // std::vector<EXX::cloudMesh> cmesh;
        // cmesh = cmprs.returnCloudMesh();
        // pcl::io::savePCDFileASCII ("cmprs_output_cloud.pcd", *cmesh[0].cloud);
        // pcl::io::saveVTKFile ("cmprs_output_mesh.vtk", cmesh[0].mesh);
        // std::cout << "Cloud should be saved." << std::endl;
    }

private:

    std::vector<std::vector<int> > getColors(){
        std::vector<std::vector<int> > colors;

    }

    void loadParams(){
        std::cout << "" << std::endl;
        std::cout << "##################" << std::endl;
        std::cout << "LOADING PARAMETERS" << std::endl;

        add_param( "printParameters", printParameters, true);
        add_param( "savePath", savePath, "./");
        add_param( "cloudPath", cloudPath, "./");
        add_param( "VoxelResolution",VoxelResolution, 0.01);
        add_param( "SVVoxelResolution", SVVoxelResolution, 0.1);
        add_param( "SVSeedResolution", SVSeedResolution, 0.3);
        add_param( "RANSACDistanceThreshold", RANSACDistanceThreshold, 0.04);
        add_param( "ECClusterTolerance", ECClusterTolerance, 0.05);
        add_param( "ECMinClusterSize", ECMinClusterSize, 100);
        add_param( "MeanK", MeanK, 15);
        add_param( "stdDev", stdDev, 15);

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