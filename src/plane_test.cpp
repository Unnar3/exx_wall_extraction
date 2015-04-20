#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <exx_compression/compression.h>
#include <plane_features/plane_features.h>
// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "moment_of_inertia_estimation.h"
#include "color_gradient.h"
#include <pcl/visualization/cloud_viewer.h>
#include <ransac_primitives/primitive_core.h>
#include <ransac_primitives/plane_primitive.h>
#include <simple_xml_parser.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
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

// struct planeDescriptor{
//     std::vector<float> momentOfInertia;
//     double boundingBoxArea;
//     double hullArea;
//     double hullBoundingBoxRatio;
//     double widthLengthRatio;
//     double distFromCentroid;
//     Eigen::Vector4d normal;
// };

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
        pcl::io::loadPCDFile (cloudPath, *cloud);

        // Create the filtering object
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (-2.5, -1.0);
        pass.filter (*cloud);
        // pass.setInputCloud (cloud);
        // pass.setFilterFieldName ("z");
        // pass.setFilterLimits (0.0, );
        // pass.filter (*cloud);
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (1.2, 5.3);
        pass.filter (*cloud);
        pcl::io::savePCDFileBinary (savePath + "pass_cloud.pcd", *cloud);
        
        // auto sweep = SimpleXMLParser<PointT>::loadRoomFromXML("/home/unnar/catkin_ws/src/Metarooms/room_3/room.xml");
        // cloud = sweep.vIntermediateRoomClouds[7];
        // tf::StampedTransform rotationTF = sweep.vIntermediateRoomCloudTransforms[7];
        // pcl_ros::transformPointCloud(*cloud, *cloud, rotationTF);

        // std::vector<int> indices;
        // pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

        

        // Eigen::Vector4f centroid;
        // pcl::compute3DCentroid(*cloud, centroid);
        // Eigen::Matrix4f transformFromOrigin = Eigen::Affine3f(Eigen::Translation3f( Eigen::Vector3f(centroid(0),centroid(1),centroid(2)))).matrix();
        // Eigen::Matrix4f transformToOrigin = transformFromOrigin.inverse();
        // pcl::transformPointCloud (*cloud, *cloud, transformToOrigin);
        // cout<<"transformed to origin"<<endl;
        // pcl::io::savePCDFileBinary (savePath + "input_cloud.pcd", *cloud);

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
        params.normal_neigbourhood = 0.15;
        params.inlier_threshold = 0.2;
        params.angle_threshold = 0.5;
        params.add_threshold = 0.01;
        params.min_shape = cloud->points.size()*0.0001;
        params.inlier_min = params.min_shape;
        params.connectedness_res = 0.06;
        params.distance_threshold = 0.0;

        //primitive_visualizer<pcl::PointXYZRGB> viewer;
        std::vector<base_primitive*> primitives = { new plane_primitive() };
        primitive_extractor<PointT> extractor(voxel_cloud, primitives, params, NULL);
        std::vector<base_primitive*> extracted;
        extractor.extract(extracted);

        std::vector<PointCloudT::Ptr> plane_vec;
        std::vector<int> ind;
        std::vector<Eigen::Vector4d> normal;
        Eigen::VectorXd data;
        for (size_t j = 0; j < extracted.size(); ++j){
            ind = extracted[j]->supporting_inds;
            extracted.at(j)->shape_data(data); 
            normal.push_back(data.segment<4>(0));
            PointCloudT::Ptr test_cloud (new PointCloudT ());
            for (size_t i = 0; i < ind.size(); ++i){
                test_cloud->points.push_back(voxel_cloud->points[ind[i]]);
            }
            plane_vec.push_back(test_cloud);
        }
        
        // pcl::io::savePCDFileBinary (savePath + "maybe_plane.pcd", *test_cloud);  
        // cmprs.extractPlanesRANSAC(voxel_cloud, &pac);
        // cmprs.projectToPlane(&pac);
        std::vector<int> normalInd;
        std::vector<double> area;
        cmprs.euclideanClusterPlanes(&plane_vec, &c_planes, &normalInd);
        cmprs.setHULLAlpha(1.0);
        cmprs.planeToConvexHull(c_planes, hulls, area);
        cmprs.reumannWitkamLineSimplification( &hulls, &simplified_hulls);
        // cmprs.superVoxelClustering(&plane_vec, &super_planes);
        // cmprs.greedyProjectionTriangulationPlanes(voxel_cloud, &super_planes, &simplified_hulls, &cm);
        
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();

        std::cout << "calculating features" << std::endl;

        std::set<std::set<int> > sets;
        std::set<int> walls;
        std::set<int> floors;
        flann::Matrix<double> dataset;
        flann::Matrix<int> indices;
        std::vector<int> set_size;
        EXX::planeFeatures features;
        features.setViewer(viewer);
        features.loadFeatures(c_planes, simplified_hulls, normal, normalInd, area, dataset, walls, floors);
        features.matchFeatures(dataset, indices);
        features.groupFeatures(indices, sets);
        printSetOfSets(sets, "Sets");

        ColorGradient cGrad(5);
        int r,g,b;
        int k = 0, u = 0;
        for( auto j : sets ){  
            if (j.size() < 1) { 
                ++u;
                continue; 
            }     
            cGrad.getColorAtValue(double(++u)/double(sets.size()), r, g, b);
            for ( auto i : j ){
                pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color (c_planes[i], r,g,b);
                viewer->addPointCloud(c_planes[i], single_color, std::to_string(++k));
            }
        }

        while(!viewer->wasStopped())
        {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    }

private:
    void printSet(std::set<int> a, std::string name){
        std::cout << "printing set " << name << ": ";
        for ( auto i : a ){
            std::cout << i << " ";
        }
        std::cout << " " << std::endl;
    }

    void printSetOfSets(std::set<std::set<int> > a, std::string name){
        std::cout << "Printing set of sets: " << name << std::endl;
        int ite = 0;
        for ( auto i:a ){
            std::cout << "set " << ++ite << ": ";
            for ( auto j:i){
                std::cout << j << " ";
            }
            std::cout << " " << std::endl;
        }
    }

    int maxIndex(std::vector<int> a){
        std::cout << " " << std::endl;
        std::cout << "print vector:" << std::endl;
        for (auto i : a){
            std::cout << i << " ";
        }
        std::cout << " " << std::endl;
        int b = std::distance(a.begin(), std::max_element(a.begin(), a.end()));
        std::cout << "index: " << b << std::endl;
        std::cout << " " << std::endl;
        return b;
    }

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
        const int NUM_COLORS = 4;
        static float color[NUM_COLORS][3] = { {0,0,255}, {0,255,0}, {255,255,0}, {255,0,0} };
        // A static array of 4 colors:  (blue,   green,  yellow,  red) using {r,g,b} for each.

        int idx1;        // |-- Our desired color will be between these two indexes in "color".
        int idx2;        // |
        float fractBetween = 0;  // Fraction between "idx1" and "idx2" where our value is.

        if(value <= 0)      {  idx1 = idx2 = 0;            }    // accounts for an input <=0
        else if(value >= 1)  {  idx1 = idx2 = NUM_COLORS-1; }    // accounts for an input >=0
        else
        {
            value = value * (NUM_COLORS-1);        // Will multiply value by 3.
            idx1  = floor(value);                  // Our desired color will be after this index.
            idx2  = idx1+1;                        // ... and before this index (inclusive).
            fractBetween = value - float(idx1);    // Distance between the two indexes (0-1).
        }

        *red   = (color[idx2][0] - color[idx1][0])*fractBetween + color[idx1][0];
        *green = (color[idx2][1] - color[idx1][1])*fractBetween + color[idx1][1];
        *blue  = (color[idx2][2] - color[idx1][2])*fractBetween + color[idx1][2];
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