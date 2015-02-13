#include <ros/ros.h>
#include <exx_plane_extraction/plane_extraction_node.h>
// PCL specific includes
#include <pcl/io/pcd_io.h>
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
// OTHER
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <cstdlib>
#include <vector>
#include <cmath>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */


// DEFINITIONS
#define HZ                  10
#define BUFFER_SIZE         1

#define NODE_NAME           		"plane_extraction_node"

typedef pcl::PointXYZRGB PointT;
using namespace std;
using namespace exx::plane_extraction_node;

class PlaneExtraction
{
    const int hz;

    ros::NodeHandle nh;

    double	voxel_leaf_size;
    double 	seg_distance, seg_percentage, normal_distance_weight;
    std::string point_cloud_name;

public:
    PlaneExtraction(int hz) : hz(hz)
    {
        add_params();
        nh = ros::NodeHandle("~");
    }

    void extractPlanes()
    {

        pcl::PointCloud <PointT>::Ptr cloud (new pcl::PointCloud <PointT>);
        if ( pcl::io::loadPCDFile <PointT> ("/home/unnar/Desktop/PointCloud/room_0/" + point_cloud_name, *cloud) == -1 )
        {
            std::cout << "Cloud reading failed." << std::endl;
            return;
        } else {
            std::cout << "Cloud reading successful." << std::endl;
            std::cerr << "has " << cloud->points.size () << " number of data points." << std::endl;
        }

        ros::Time begin = ros::Time::now();
        voxelGrid(cloud);
        
        pcl::PointCloud <PointT>::Ptr cloud_plane (new pcl::PointCloud <PointT>);
        removePlanes(cloud, cloud_plane);

        ros::Time end = ros::Time::now();
        ros::Duration duration = end - begin;
        ROS_INFO("Algorithm finished after %f seconds...", duration.toSec());

        *cloud += *cloud_plane;

        pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);
        pcl::io::savePCDFileASCII ("test_pcd_plane.pcd", *cloud_plane);
        std::cerr << "Saved " << cloud->points.size () << " data points to test_pcd.pcd." << std::endl;
        std::cerr << "Saved " << cloud_plane->points.size () << " data points to test_pcd_plane.pcd." << std::endl;

    }

private:
    // Removes outliers using a StatisticalOutlierRemoval filter
    void statisticalOutlierRemovalCloud(pcl::PointCloud<PointT>::Ptr cloud_stat)
    {
        pcl::StatisticalOutlierRemoval<PointT> sta;
        sta.setInputCloud (cloud_stat);
        sta.setMeanK (20);
        sta.setStddevMulThresh (1.0);
        sta.filter (*cloud_stat);
    }

    // Down samples the point cloud using VoxelGrid filter
    // to make computations easier.
    void voxelGrid(pcl::PointCloud<PointT>::Ptr cloud_stat)
    {
        pcl::PointCloud <PointT>::Ptr cloud_voxel (new pcl::PointCloud <PointT>);
        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud (cloud_stat);
        sor.setLeafSize ((float)voxel_leaf_size, (float)voxel_leaf_size, (float)voxel_leaf_size);
        sor.filter (*cloud_voxel);
        *cloud_stat = *cloud_voxel;
    }

    pcl::PointCloud<PointT>::Ptr removePlanes(pcl::PointCloud<PointT>::Ptr cloud_seg, pcl::PointCloud<PointT>::Ptr cloud_p)
    {
        pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_plane_tmp (new pcl::PointCloud<PointT>);
        pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

        pcl::SACSegmentation<PointT> seg;
        pcl::ExtractIndices<PointT> extract;


        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (seg_distance);
        seg.setMaxIterations (100);

        int i = 0, 
        nr_points = (int) cloud_seg->points.size ();
        // While 20% of the original cloud is still there
        int r,g,b;
        srand (time(NULL));
        while (i < 100 && cloud_seg->points.size() > 0 && cloud_seg->points.size() > 0.1 * nr_points)
        {
            //seg.setInputCloud (cloud);
            seg.setInputCloud (cloud_seg);
            seg.segment (*inliers, *coeff);
            if (inliers->indices.size () == 0)
            {
                break;
            }
            if(inliers->indices.size() < 200){
                i++;
                continue;
            }
            // Extract the planar inliers from the input cloud
            extract.setInputCloud (cloud_seg);
            extract.setIndices (inliers);

            extract.setNegative (false);
            extract.filter (*cloud_plane_tmp);
            // Loop through all points to find si
            r = rand()%255;
            for(int iter = 1; iter != cloud_plane_tmp->points.size(); ++iter)
            {
                cloud_plane_tmp->points[iter].r =  155;
                cloud_plane_tmp->points[iter].g =  r;
                cloud_plane_tmp->points[iter].b =  r;
            }
            // Create the filtering object
            pcl::ProjectInliers<PointT> proj;
            proj.setModelType (pcl::SACMODEL_PLANE);
            proj.setInputCloud (cloud_plane_tmp);
            proj.setModelCoefficients (coeff);
            proj.filter (*cloud_plane_tmp);
            
            // Create a Concave Hull representation of the projected inliers
            /*
            pcl::ConcaveHull<PointT> chull;
            chull.setInputCloud (cloud_plane_tmp);
            chull.setAlpha (0.1);
            chull.reconstruct (*cloud_plane_tmp);
            */  
            *cloud_p += *cloud_plane_tmp;


            extract.setNegative (true);
            extract.filter (*cloud_plane);
            extract.setNegative (true);
            extract.filter (*cloud_plane);
            cloud_seg.swap (cloud_plane);
            i++;
        }
        return cloud_seg;
    }

    void add_params()
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_json(CONFIG_DOC, pt);
        // point cloud name
        point_cloud_name = pt.get<string>("point_cloud_name");
        voxel_leaf_size = pt.get<double>("voxel_leaf_size");
        seg_distance = pt.get<double>("seg_distance");
        seg_percentage = pt.get<double>("seg_percentage");
        normal_distance_weight = pt.get<double>("normal_distance_weight");
    }
};

int main(int argc, char **argv) {

    ros::init(argc, argv, NODE_NAME);

    PlaneExtraction extractor(HZ);
    ros::Rate loop_rate(HZ);

    extractor.extractPlanes();

    return 0;
}