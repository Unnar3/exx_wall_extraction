#include <ros/ros.h>
#include <exx_compression/compression.h>
// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "moment_of_inertia_estimation.h"
#include <pcl/visualization/cloud_viewer.h>
#include <ransac_primitives/primitive_core.h>
#include <ransac_primitives/plane_primitive.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
// OTHER
#include <pcl/console/parse.h>
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <sstream>


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

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

        std::vector<base_primitive*> primitives = { new plane_primitive() };

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, centroid);
        Eigen::Matrix4f transformFromOrigin = Eigen::Affine3f(Eigen::Translation3f( Eigen::Vector3f(centroid(0),centroid(1),centroid(2)))).matrix();
        Eigen::Matrix4f transformToOrigin = transformFromOrigin.inverse();
        pcl::transformPointCloud (*cloud, *cloud, transformToOrigin);
        cout<<"transformed to origin"<<endl;

        // Create the filtering object
        // pcl::StatisticalOutlierRemoval<PointT> sor;
        // sor.setInputCloud (cloud);
        // sor.setMeanK (MeanK);
        // sor.setStddevMulThresh (stdDev);
        // sor.filter (*cloud_f);

        EXX::compression cmprs;
        cmprs.setSVVoxelResolution(SVVoxelResolution);
        cmprs.setSVSeedResolution(SVSeedResolution);
        cmprs.setRANSACDistanceThreshold(RANSACDistanceThreshold);
        cmprs.setRANSACMaxIteration(10);
        cmprs.setECClusterTolerance(ECClusterTolerance);
        cmprs.setECMinClusterSize(ECMinClusterSize);
        cmprs.setVoxelLeafSize(VoxelResolution);
        cmprs.setRANSACMinInliers(20);
        
        PointCloudT::Ptr voxel_cloud (new PointCloudT ());
        EXX::planesAndCoeffs pac;
        std::vector<PointCloudT::Ptr> c_planes;
        std::vector<PointCloudT::Ptr> hulls;
        std::vector<PointCloudT::Ptr> simplified_hulls;
        std::vector<PointCloudT::Ptr> super_planes;
        std::vector<EXX::cloudMesh> cm;
        clock_t t1,t2,t3,t4,t5,t6,t7,t8,t9;
        t1=clock();
        cmprs.voxelGridFilter(cloud, voxel_cloud);
        t2=clock();


        // sphere_primitive and cylinder_primitive have not been ported to the new framework yet
        primitive_params params;
        params.number_disjoint_subsets = 10;
        params.octree_res = 0.1;
        params.normal_neigbourhood = 0.20;
        params.inlier_threshold = 0.04;
        params.angle_threshold = 0.2;
        params.add_threshold = 0.1;
        params.min_shape = cloud->points.size()*0.001;
        params.inlier_min = 10;//params.min_shape;
        params.connectedness_res = 0.06;
        params.distance_threshold = 0.0;

        //primitive_visualizer<pcl::PointXYZRGB> viewer;
        primitive_extractor<PointT> extractor(voxel_cloud, primitives, params, NULL);
        std::vector<base_primitive*> extracted;
        extractor.extract(extracted);

        std::vector<PointCloudT::Ptr> plane_vec;
        std::vector<int> ind;
        for (size_t j = 0; j < extracted.size(); ++j){
            ind = extracted[j]->supporting_inds;
            PointCloudT::Ptr test_cloud (new PointCloudT ());
            for (size_t i = 0; i < ind.size(); ++i){
                    test_cloud->points.push_back(voxel_cloud->points[ind[i]]);
            }
            std::cout << "pushing" << std::endl;
            plane_vec.push_back(test_cloud);
        }
        
        // pcl::io::savePCDFileBinary (savePath + "maybe_plane.pcd", *test_cloud   );

        
        
        // cmprs.extractPlanesRANSAC(voxel_cloud, &pac);
        // t3=clock();
        // cmprs.projectToPlane(&pac);
        // t4=clock();
        // cmprs.euclideanClusterPlanes(&pac.cloud, &c_planes);
        // t5=clock();
        cmprs.planeToConcaveHull(&plane_vec, &hulls);
        t6=clock();
        cmprs.reumannWitkamLineSimplification( &hulls, &simplified_hulls);
        t7=clock();
        cmprs.superVoxelClustering(&c_planes, &super_planes);
        t8=clock();
        cmprs.greedyProjectionTriangulationPlanes(voxel_cloud, &super_planes, &simplified_hulls, &cm);
        t9=clock();

        // std::cout << "Total time: " << double(t9-t1) / CLOCKS_PER_SEC << std::endl;
        // std::cout << "Voxel time: " << double(t2-t1) / CLOCKS_PER_SEC << std::endl;
        // std::cout << "RANSAC time: " << double(t3-t2) / CLOCKS_PER_SEC << std::endl;
        // std::cout << "Project time: " << double(t4-t3) / CLOCKS_PER_SEC << std::endl;
        // std::cout << "EC time: " << double(t5-t4) / CLOCKS_PER_SEC << std::endl;
        // std::cout << "Hull time: " << double(t6-t5) / CLOCKS_PER_SEC << std::endl;
        // std::cout << "Simple Hull time: " << double(t7-t6) / CLOCKS_PER_SEC << std::endl;
        // std::cout << "Super Voxel time: " << double(t8-t7) / CLOCKS_PER_SEC << std::endl;
        // std::cout << "triangulation time: " << double(t9-t8) / CLOCKS_PER_SEC << std::endl;
        
        PointCloudT::Ptr colored_cloud (new PointCloudT ());
        PointCloudT::Ptr tmp_cloud (new PointCloudT ());
        int r, g, b;
        for (size_t i = 0; i < plane_vec.size(); ++i){
            *tmp_cloud = *plane_vec[i] + *simplified_hulls[i];
            r = rand () % 155;
            g = rand () % 155;
            b = rand () % 155;
            for (size_t j = 0; j < tmp_cloud->points.size(); ++j){
                tmp_cloud->points[j].r = r;
                tmp_cloud->points[j].g = g;
                tmp_cloud->points[j].b = b;
            }
            *colored_cloud += *tmp_cloud;
        }
        
        pcl::io::savePCDFileASCII (savePath + "colored_cloud.pcd", *colored_cloud);

        // // Nýtt test
        // pcl::PointCloud<PointT>::Ptr cloud22 (new pcl::PointCloud<PointT> ());
        // pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
        // PointT min_point_OBB;
        // PointT max_point_OBB;
        // PointT position_OBB;
        // Eigen::Matrix3f rotational_matrix_OBB;  

        // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        // viewer->setBackgroundColor (0, 0, 0);
        // viewer->addCoordinateSystem (1.0);
        // viewer->initCameraParameters ();
        // boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
        // ostringstream convert;   
        // // string str = string(intStr);
        // float f;
        // for (size_t i = 0; i < c_planes.size(); ++i){
        //     r = rand () % 155;
        //     g = rand () % 155;
        //     b = rand () % 155;
        //     pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color (c_planes[i], r,g,b);
        //     convert << i;
        //     feature_extractor.setInputCloud (c_planes[i]);
        //     feature_extractor.compute ();
        //     feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        //     Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
        //     Eigen::Quaternionf quat (rotational_matrix_OBB);
        //     viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB"+convert.str());
        //     viewer->addPointCloud(c_planes[i], single_color, convert.str());
        //     f = pcl::calculatePolygonArea(*hulls[i]);
        //     std::cout << "Total area: " << f << std::endl;
        // }   
       
        // while(!viewer->wasStopped())
        // {
        //     viewer->spinOnce (100);
        //     boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        // }
    }

private:

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