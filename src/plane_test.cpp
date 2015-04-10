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
#include <simple_xml_parser.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
// OTHER
#include <pcl/console/parse.h>
#include <Eigen/Dense>
#include <complex>
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

struct planeDescriptor{
    std::vector<float> momentOfInertia;
    double boundingBoxArea;
    double hullArea;
    double hullBoundingBoxRatio;
    double widthLengthRatio;
    double distFromCentroid;
    Eigen::Vector4d normal;
};

class PlaneTest
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
        // pcl::io::loadPCDFile (cloudPath, *cloud);

        auto sweep = SimpleXMLParser<PointT>::loadRoomFromXML("/home/unnar/catkin_ws/src/Metarooms/room_2/room.xml");
        cloud = sweep.vIntermediateRoomClouds[43];
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
        cmprs.voxelGridFilter(cloud, voxel_cloud);


        // sphere_primitive and cylinder_primitive have not been ported to the new framework yet
        primitive_params params;
        params.number_disjoint_subsets = 10;
        params.octree_res = 2.0;
        params.normal_neigbourhood = 0.05;
        params.inlier_threshold = 0.1;
        params.angle_threshold = 0.3;
        params.add_threshold = 0.01;
        params.min_shape = cloud->points.size()*0.0001;
        params.inlier_min = params.min_shape;
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
        
        // pcl::io::savePCDFileBinary (savePath + "maybe_plane.pcd", *test_cloud);  
        // cmprs.extractPlanesRANSAC(voxel_cloud, &pac);
        // cmprs.projectToPlane(&pac);
        // cmprs.euclideanClusterPlanes(&plane_vec, &c_planes);
        cmprs.planeToConcaveHull(&plane_vec, &hulls);
        cmprs.reumannWitkamLineSimplification( &hulls, &simplified_hulls);
        cmprs.superVoxelClustering(&plane_vec, &super_planes);
        cmprs.greedyProjectionTriangulationPlanes(voxel_cloud, &super_planes, &simplified_hulls, &cm);
        
        
        // PointCloudT::Ptr colored_cloud (new PointCloudT ());
        // PointCloudT::Ptr tmp_cloud (new PointCloudT ());
        // int r, g, b;
        // for (size_t i = 0; i < plane_vec.size(); ++i){
        //     *tmp_cloud = *plane_vec[i] + *simplified_hulls[i];
        //     r = rand () % 155;
        //     g = rand () % 155;
        //     b = rand () % 155;
        //     for (size_t j = 0; j < tmp_cloud->points.size(); ++j){
        //         tmp_cloud->points[j].r = r;
        //         tmp_cloud->points[j].g = g;
        //         tmp_cloud->points[j].b = b;
        //     }
        //     *colored_cloud += *tmp_cloud;
        // }

        // pcl::io::savePCDFileASCII (savePath + "colored_cloud.pcd", *colored_cloud);

        // Nýtt test
        pcl::PointCloud<PointT>::Ptr cloud22 (new pcl::PointCloud<PointT> ());
        pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
        PointT min_point_OBB;
        PointT max_point_OBB;
        PointT position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;  
        std::vector<float> moment_of_inertia;

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
        boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
        ostringstream convert;   

        std::vector<planeDescriptor> vPlaneDescriptor;
        planeDescriptor pDescriptor;
        double area;
        double wlRatio;
        int pos = 3;
        Eigen::Vector4d normal;
        srand (time(NULL));
        for (auto i = 0; i < plane_vec.size(); ++i){
            // r = rand () % 155;
            // g = rand () % 155;
            // b = rand () % 155;
            // pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color (plane_vec[i], r,g,b);
            convert << i;
            feature_extractor.setInputCloud (plane_vec[i]);
            feature_extractor.compute ();
            feature_extractor.getMomentOfInertia (moment_of_inertia);
            feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
            Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
            Eigen::Quaternionf quat (rotational_matrix_OBB);
            if( i == pos ){
                viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB"+convert.str());
            }
            // viewer->addPointCloud(plane_vec[i], single_color, convert.str());
            pDescriptor.momentOfInertia = moment_of_inertia;
            getBiggestCubeArea(min_point_OBB, max_point_OBB, &area, &wlRatio);
            pDescriptor.boundingBoxArea = area;
            pDescriptor.hullArea = double( pcl::calculatePolygonArea(*hulls[i]) );
            pDescriptor.hullBoundingBoxRatio = pDescriptor.hullArea / pDescriptor.boundingBoxArea;
            pDescriptor.widthLengthRatio = wlRatio;
            Eigen::VectorXd data;
            extracted.at(i)->shape_data(data); 
            normal = data.segment<4>(0);
            pDescriptor.normal = normal;
            vPlaneDescriptor.push_back(pDescriptor);
        }  


        int red;
        int green;
        int blue;
        int j = 0;
        double eDist;
        std::vector<double> vDescriptor;
        for (auto i : vPlaneDescriptor){
            eDist = 0;
            vDescriptor.clear();
            // Check distance 
            vDescriptor.push_back( std::log( vPlaneDescriptor.at(pos).boundingBoxArea ) - std::log( i.boundingBoxArea ));
            vDescriptor.push_back( std::log( vPlaneDescriptor.at(pos).hullArea ) - std::log( i.hullArea ));
            vDescriptor.push_back( std::log( vPlaneDescriptor.at(pos).hullBoundingBoxRatio ) - std::log( i.hullBoundingBoxRatio ));
            vDescriptor.push_back( std::log( vPlaneDescriptor.at(pos).widthLengthRatio ) - std::log( i.widthLengthRatio ));
            vDescriptor.push_back( std::log( angleBetweenVectors( vPlaneDescriptor.at(pos).normal, i.normal )));
            for (auto j : vDescriptor){
                eDist += j * j;
                std::cout << ", " << j*j;
            }
            std::cout << "" << std::endl;
            eDist = std::sqrt( eDist );
            std::cout << "eDist: " << eDist << std::endl;
            getValueBetweenTwoFixedColors(eDist, &red, &green, &blue);
            pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color (plane_vec[j], red,green,blue);
            convert << j;
            viewer->addPointCloud(plane_vec[j], single_color, convert.str());
            j++;
        }

        while(!viewer->wasStopped())
        {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    }

private:
    // Takes in two vectors, returns angle between them in range 0 to 1  
    // where 1 means no difference, pi/2 is cnsidered maximum angle and pi as 0.
    double angleBetweenVectors(Eigen::Vector4d a, Eigen::Vector4d b){
        if (a == b) { return 1; }
        
        // Norm the vectors before calculation.
        a = a / a.squaredNorm();
        b = b / b.squaredNorm();

        double ang = std::acos( a.dot(b) ); 
        // Shift it around to return 1 for 0 degrees and 0 for 45 degrees.
        if ( std::isnan(ang) ){
            return 1;
        } else {
            return std::abs( (ang - M_PI/2) / (M_PI/2) );
        }
    }

    // Returns RGB value from red to green depending on value, low value results in blue,
    // high value results in red.
    void getValueBetweenTwoFixedColors(double value, int *red, int *green, int *blue)
    {
        if (value > 10.0 && value == INFINITY){ 
            value = 1.0; 
        } else {
            value = value / 10.0;
        }

        int aR = 0;   int aG = 0; int aB=255;  // RGB for our 1st color (blue in this case).
        int bR = 255; int bG = 0; int bB=0;    // RGB for our 2nd color (red in this case).

        *red   = double(bR - aR) * value + aR;      // Evaluated as -255*value + 255.
        *green = double(bG - aG) * value + aG;      // Evaluates as 0.
        *blue  = double(bB - aB) * value + aB;      // Evaluates as 255*value + 0.
    }

    void getBiggestCubeArea(PointT minPoint, PointT maxPoint, double *area, double *WLRatio){
        double x = maxPoint.x - minPoint.x;
        double y = maxPoint.y - minPoint.y;
        double z = maxPoint.z - minPoint.z;
        if (x > z && y > z){
            *area = x*y;
            *WLRatio = x/y;
        } else if (x > y && z > y) {
            *area = x*z;
            *WLRatio = x/z;
        }
        else {
            *area = y*z;
            *WLRatio = y/z;
        }
        if( *WLRatio > 1 ){
            *WLRatio = 1 / *WLRatio;
        }
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