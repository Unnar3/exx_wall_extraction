#include <ros/ros.h>
#include <exx_plane_extraction/plane_extraction_node.h>
// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/surface/mls.h>
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


// DEFINITIONS
#define HZ                  10
#define BUFFER_SIZE         1

#define NODE_NAME           		"s8_object_detection_node"
#define TOPIC_POINT_CLOUD   		"/camera/depth_registered/points"
#define TOPIC_EXTRACTED_OBJECTS		"/s8/detectedObject"
#define TOPIC_IS_FRONT_WALL         "/s8/isFrontWall"
#define TOPIC_OBJECTS_POS		    "/s8/ip/detection/distPose"
#define CONFIG_DOC                  "/catkin_ws/src/s8_ip_detection/parameters/parameters.json"

// PARAMETERS
#define PARAM_FILTER_X_NAME						"filter_x"
#define PARAM_FILTER_X_DEFAULT					0.1
#define PARAM_FILTER_Z_NAME						"filter_z"
#define PARAM_FILTER_Z_DEFAULT					1.0
#define PARAM_FILTER_Y_NAME						"filter_y"
#define PARAM_FILTER_Y_DEFAULT					0.2
#define PARAM_FLOOR_EXTRACTION_DIST_NAME		"floor_extraction_dist"
#define PARAM_FLOOR_EXTRACTION_DIST_DEFAULT		0.02
#define PARAM_VOXEL_LEAF_SIZE_NAME				"voxel_leaf_size"
#define PARAM_VOXEL_LEAF_SIZE_DEFAULT			0.005
#define PARAM_SEG_DISTANCE_NAME					"seg_distance"
#define PARAM_SEG_DISTANCE_DEFAULT				0.01
#define PARAM_SEG_PERCENTAGE_NAME				"seg_percentage"
#define PARAM_SEG_PERCENTAGE_DEFAULT			0.2
#define PARAM_CAM_ANGLE_NAME				    "cam_angle"
#define PARAM_CAM_ANGLE_DEFAULT			        0.4

typedef pcl::PointXYZRGB PointT;
using namespace std;
using namespace exx::plane_extraction_node;

class PlaneExtraction
{
    const int hz;

    ros::NodeHandle nh;

    double	voxel_leaf_size;
    double 	seg_distance, seg_percentage, normal_distance_weight;

public:
    PlaneExtraction(int hz) : hz(hz)
    {
        add_params();
        //printParams();

        nh = ros::NodeHandle("~");

        //cloud = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>);
    }

    void extractPlanes()
    {

        pcl::PointCloud <pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);
        if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("/home/unnar/Desktop/PointCloud/room_0/intermediate_cloud0034.pcd", *cloud) == -1 )
        {
            std::cout << "Cloud reading failed." << std::endl;
            return;
        }

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
    void voxelGridCloud(pcl::PointCloud<PointT>::Ptr cloud_stat)
    {
        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud (cloud_stat);
        sor.setLeafSize ((float)voxel_leaf_size, (float)voxel_leaf_size, (float)voxel_leaf_size);
        sor.filter (*cloud_stat);
    }

    pcl::PointCloud<PointT>::Ptr removeWallsCloud(pcl::PointCloud<PointT>::Ptr cloud_seg)
    {
        pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);
        pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
        pcl::NormalEstimation<PointT, pcl::Normal> ne;

        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
        pcl::ExtractIndices<PointT> extract;

        // Estimate point normals
        ne.setSearchMethod (tree);
        ne.setKSearch (50);

        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (seg_distance);
        seg.setNormalDistanceWeight (normal_distance_weight);
        seg.setMaxIterations (1000);

        int i = 0, nr_points = (int) cloud_seg->points.size ();
        // While 20% of the original cloud is still there
        while (cloud_seg->points.size () > seg_percentage * nr_points && i < 10 && cloud_seg->points.size() > 0)
        {
            //seg.setInputCloud (cloud);
            ne.setInputCloud (cloud_seg);
            ne.compute (*cloud_normals);
            //seg.setInputCloud (cloud);
            seg.setInputCloud (cloud_seg);
            seg.setInputNormals (cloud_normals);
            seg.segment (*inliers, *coeff);
            if (inliers->indices.size () == 0)
            {
                break;
            }
            if(inliers->indices.size() < nr_points/20 || inliers->indices.size() < 10){
                i++;
                continue;
            }
            // Extract the planar inliers from the input cloud
            extract.setInputCloud (cloud_seg);
            extract.setIndices (inliers);
            extract.setNegative (true);
            extract.filter (*cloud_plane);
            cloud_seg.swap (cloud_plane);
            i++;
        }
        return cloud_seg;
    }

    void add_params()
    {
        //std::string home(::getenv("HOME"));
        //ROS_INFO("home: %s", CONFIG_DOC);
        //boost::property_tree::ptree pt;
        //boost::property_tree::read_json(home + CONFIG_DOC, pt);
        // CIRCLE
        //filter_x = pt.get<double>("filter_x");
    }
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "plane_extraction");

    PlaneExtraction extractor(HZ);
    ros::Rate loop_rate(HZ);

    extractor.extractPlanes();

    return 0;
}