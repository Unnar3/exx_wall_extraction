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
#include <pcl/geometry/planar_polygon.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/console/parse.h>
// OTHER
#include <boost/thread/thread.hpp>
#include <vector>
#include <cmath>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <compression/compress_methods.h>
#include <compression/cloud_filtering.h>
#include <compression/plane_extraction.h>

using namespace exx::compression;

// DEFINITIONS
#define HZ                  10
#define BUFFER_SIZE         1
#define NODE_NAME           "plane_compression"
const std::string METAROOM = "/home/unnar/catkin_ws/src/Metarooms/room_0/";

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointNCloudT;

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
        cloudFiltering::voxelGridFiltering(cloud, 0.02);
        std::cout << "Point cloud size AFTER voxel grid filtering:" << std::endl;
        std::cout <<  cloud->points.size () << " points." << std::endl;

        // Extract planes
        std::vector<pcl::PointCloud<PointT>::Ptr > planes;
        std::vector<pcl::ModelCoefficients::Ptr > coeffs;
        removePlanes(cloud, &planes, &coeffs);
        std::cout << "Number of planes: " << int(planes.size()) << std::endl;
        savePCL("compressed_extracted_planes", planes);

        // Project points to the planes
        projectToPlane(&planes, &coeffs);
        savePCL("compressed_extracted_projected_planes", planes);

        // calculate concave hull
        std::vector<PointCloudT::Ptr > hulls;
        planeToConcaveHull(&planes, &hulls);
        savePCL("compressed_hulls_planes", hulls);


        // Simplify the concave hull.
        reumannWitkamLineSimplification(&hulls);
        savePCL("compressed_simplified_hulls_planes", hulls);

        // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        // pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_test;
        // viewer->setBackgroundColor (0, 0, 0);
        // viewer->addPointCloud(cloud, "samplecloud");
        // viewer->addPolygon(planar_poly, 255.0, 255.0, 50.0, "polygon1", 0);
        // viewer->addPolygon(planar_poly2, 255.0, 50.0, 50.0, "polygon2", 0);
        // while (!viewer->wasStopped ())
        // {
        //     viewer->spinOnce (100);
        // }

        // Create supervoxels from the cloud
        std::vector<PointCloudT::Ptr > super_voxel_planes = superVoxelClustering(&planes);
        savePCL("compressed_voxel_planes", super_voxel_planes);
        PointCloudT::Ptr cloud_triangle (new PointCloudT ());
        for (int i = 0; i < super_voxel_planes.size(); i++)
        {
            *cloud_triangle += *super_voxel_planes[i];
            //*cloud_triangle += *hulls[i];
            //*super_voxel_planes[i] += *hulls[i]; 
        }

        //std::vector<pcl::PolygonMesh> triangles;
        savePCL("Cloud_triangle", cloud_triangle);
        pcl::PolygonMesh triangles;
        triangles = greedyProjectionTriangulation(cloud_triangle);//&super_voxel_planes);
        saveVTK("mesh", triangles);
    }

    private:
    void savePCL(const std::string filename, std::vector<PointCloudT::Ptr > clouds)
    {
        if (clouds.size() == 0){
            std::cout << "no data to save (savePCL)" << std::endl;
        }

        PointCloudT::Ptr cloud_tmp (new PointCloudT ());
        for (int j = 0; j < clouds.size(); j++){
            *cloud_tmp += *clouds[j];
        }
        pcl::io::savePCDFileASCII (filename + ".pcd", *cloud_tmp);
    }

    void savePCL(const std::string filename, PointCloudT::Ptr cloud)
    {
        pcl::io::savePCDFileASCII (filename + ".pcd", *cloud);
    }

    void saveVTK(const std::string filename, pcl::PolygonMesh poly)
    {
        pcl::io::saveVTKFile (filename + ".vtk", poly);
    }
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "NODE_NAME");

    // Initialize the compressor class
    PlaneCompression compressor;
    ros::Rate loop_rate(HZ);


    std::cout << "Creating point cloud" << std::endl;
    // Create a point cloud containing a single plain
    std::vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    bool cloudLoaded = true;
    std::string filename = METAROOM;
    if (filenames.size() != 0){
        filename += argv[filenames[0]];
    }
    std::cout << "input: " << filename << std::endl;
    if (filenames.size() > 0 && pcl::io::loadPCDFile (filename, *cloud) < 0)  {
        std::cout << "Error loading point cloud " << std::endl;
        cloudLoaded = false;
    }
    if (cloudLoaded == false || filenames.size() == 0){

        std::cout << "Creating simulated data" << std::endl;
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
    }

    std::cout << "Point cloud created" << std::endl;

    pcl::io::savePCDFileASCII ("compressed_plane.pcd", *cloud);
    compressor.cloudCompress(cloud);
    //pcl::io::savePCDFileASCII ("compressed_voxel_plane.pcd", *cloud);

    return 0;
}
