//Practice with inheritance, polymorphism, and Abstract Data Types
//header file for Polygon class

#ifndef COMPRESSMETHODS_H
#define COMPRESSMETHODS_H

#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>


#include <vector>
#include <math.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class compressMethods
{
    public:
    
    // Down samples the point cloud using VoxelGrid filter
    // to make computations easier.
    void voxelGridCloud(pcl::PointCloud<PointT>::Ptr cloud, float leaf_size)
    {
        pcl::PointCloud <PointT>::Ptr cloud_voxel (new pcl::PointCloud <PointT> ()); 
        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (leaf_size, leaf_size, leaf_size);
        sor.filter (*cloud_voxel);
        *cloud = *cloud_voxel;
    }


    // Plane segmentation using RANSAC
    // Create the segmentation object for the planar model and set all the parameters
    void removePlanes(pcl::PointCloud<PointT>::Ptr cloud, std::vector<pcl::PointCloud<PointT>::Ptr > *planes, std::vector<pcl::ModelCoefficients::Ptr > *coeffs)
    {
        pcl::SACSegmentation<PointT> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT> ());
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.02);

        int i=0, nr_points = (int) cloud->points.size ();
        while (i < 100 && cloud->points.size() > 0 && cloud->points.size() > 0.1 * nr_points)
        {
            // Define for each plane we find
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());

            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }
            if(inliers->indices.size() < 200){
                i++;
                continue;
            }

            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud (cloud);
            extract.setIndices (inliers);
            extract.setNegative (false);

            // Get the points associated with the planar surface
            extract.filter (*cloud_plane);

            // Remove the planar inliers, extract the rest
            extract.setNegative (true);
            extract.filter (*cloud_f);
            cloud.swap (cloud_f);

            (*coeffs).push_back(coefficients);
            (*planes).push_back(cloud_plane);
            i++;
        }
    }


    // Project inliers conforming to a plane model exactly on the plane
    // input:   vector of point clouds where each cloud contains only points 
    //          belonging to one plane and a vector of plane coefficients
    // output:  vector of point clouds where each cloud contains only points 
    //          exactly on one plane.
    void projectToPlane(std::vector<pcl::PointCloud<PointT>::Ptr > *planes, std::vector<pcl::ModelCoefficients::Ptr > *coeffs)
    {
        int number_of_planes = planes->size();
        if(number_of_planes == 0){
            std::cout << "No planes, nothing to do" << std::endl;
            return;
        }
        // Create the projection object
        pcl::ProjectInliers<PointT> proj;
        proj.setModelType (pcl::SACMODEL_PLANE);

        for (int i = 0; i < number_of_planes; i++){ 
            proj.setInputCloud ( (*planes)[i] );
            proj.setModelCoefficients ( (*coeffs)[i]);
            proj.filter ( *(*planes)[i] );
        }
    }


    // Calculates the concave hull of planes.
    // Input:   vector containing point clouds where all points belong to a single plane  
    // Output:  vector containing point clouds where all points belong to the concave hull 
    //          of a single plane  
    void planeToConcaveHull(std::vector<pcl::PointCloud<PointT>::Ptr > *planes, std::vector<pcl::PointCloud<PointT>::Ptr > *hulls){

        int number_of_planes = planes->size();
        if(number_of_planes == 0){
            std::cout << "No planes, nothing to do" << std::endl;
            return;
        }

        // Create a Concave Hull representation of the projected inliers
        pcl::ConcaveHull<PointT> chull;
        for (int i = 0; i < number_of_planes; i++){ 
            pcl::PointCloud<PointT>::Ptr cloud_hull (new pcl::PointCloud<PointT> ());
            chull.setInputCloud ( (*planes)[i] );
            chull.setAlpha (0.05);
            chull.setKeepInformation (true);
            chull.reconstruct (*cloud_hull);
            (*hulls).push_back(cloud_hull);
            std::cout << "plane count: " << (*planes)[i]->points.size() << std::endl;
            std::cout << "hull count: " << cloud_hull->points.size() << std::endl;
        }
    }

    void rotatePointCloud(pcl::PointCloud<PointT>::Ptr inputCloud, pcl::PointCloud<PointT>::Ptr outputCloud, std::vector<float> theta)
    {
        if ( theta.size() != 3 ){
            std::cout << "transform vector of wrong size, nothing to be done." << std::endl;
            outputCloud = inputCloud;
            return;
        }

        // Create rotation matrix.
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate (Eigen::AngleAxisf (theta[0], Eigen::Vector3f::UnitX()));
        transform.rotate (Eigen::AngleAxisf (theta[1], Eigen::Vector3f::UnitY()));
        transform.rotate (Eigen::AngleAxisf (theta[2], Eigen::Vector3f::UnitZ()));

        std::cout << "about to rotate" << std::endl;
        // Transform the cloud and return it
        pcl::transformPointCloud (*inputCloud, *outputCloud, transform);
        std::cout << "rotated" << std::endl;
    }

    // Calculates the distance from a point in 3D to a line.
    // input:   PointT points, current and next define a line and nextCheck is a point where we want to know 
    //          the distance to the line.
    // output:  Double that defines the distance to the line in meters.
    double pointToLineDistance(PointT current, PointT next, PointT nextCheck)
    {
        std::vector<float> x0x1;
        x0x1.push_back(nextCheck.x-current.x);
        x0x1.push_back(nextCheck.y-current.y);
        x0x1.push_back(nextCheck.z-current.z);
        
        std::vector<float> x0x2;
        x0x2.push_back(nextCheck.x-next.x);
        x0x2.push_back(nextCheck.y-next.y);
        x0x2.push_back(nextCheck.z-next.z);
        
        std::vector<float> x1x2; 
        x1x2.push_back(current.x-next.x);
        x1x2.push_back(current.y-next.y);
        x1x2.push_back(current.z-next.z);  
        
        std::vector<float> cross; 
        cross.push_back(x0x1[1]*x0x2[2] - x0x1[2]*x0x2[1]);
        cross.push_back(x0x1[2]*x0x2[0] - x0x1[0]*x0x2[2]);
        cross.push_back(x0x1[0]*x0x2[1] - x0x1[1]*x0x2[0]);
        
        return sqrt(cross[0]*cross[0] + cross[1]*cross[1] + cross[2]*cross[2]) / sqrt(x1x2[0]*x1x2[0] + x1x2[1]*x1x2[1] + x1x2[2]*x1x2[2]);
    }


    // Simplifies the concave hull using the Reumann-Witkam algorithm
    // Input : Vector of clouds where each cloud represents the concave hull of a plane,
    // Output: Vector of clouds where each cloud represents the simplified concave hull of a plane,
    void reumannWitkamLineSimplification(std::vector<pcl::PointCloud<PointT>::Ptr > *hulls)
    {
        int number_of_planes = hulls->size();
        if(number_of_planes == 0){
            std::cout << "No planes, nothing to do" << std::endl;
            return;
        }

        double eps = 0.02;
        double distToLine;
        int numberOfPointsInPlane;
        int j_current, j_next, j_nextCheck, j_last;
        PointT current, next, last, nextCheck;
        pcl::PointCloud<PointT>::Ptr currentPlane;

        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        pcl::ExtractIndices<PointT> extract;
        
        // Loop through all planes
        for (int i = 0; i < number_of_planes; i++){
            currentPlane = (*hulls)[i];
            numberOfPointsInPlane = currentPlane->points.size();
            
            j_current = 0;
            j_next = j_current + 1;
            j_last = j_next;
            j_nextCheck = j_next + 1;
            inliers->indices.clear();
            
            // Loop through all points in the plane and find redundant points.
            while(j_nextCheck < numberOfPointsInPlane){
                current = currentPlane->points[j_current];
                next = currentPlane->points[j_next];
                last = currentPlane->points[j_last];
                nextCheck = currentPlane->points[j_nextCheck];
                distToLine = pointToLineDistance(current, next, nextCheck);
                if ( distToLine < eps ){
                    if ( j_next != j_last ){
                        inliers->indices.push_back(j_last);
                    }
                    j_last++; 
                    j_nextCheck++;
                } else {
                    inliers->indices.push_back(j_next);
                    j_current = j_nextCheck;
                    j_next = j_current + 1; 
                    j_last = j_next;
                    j_nextCheck = j_next + 1;
                }
            }
            // Remove the redundant points.
            extract.setInputCloud (currentPlane);
            extract.setIndices (inliers);
            extract.setNegative (true);
            extract.filter (*currentPlane);
        }
    }

    std::vector<PointCloudT::Ptr > superVoxelClustering(std::vector<PointCloudT::Ptr > *planes)
    {
        std::vector<PointCloudT::Ptr > super_voxel_planes;
        
        int number_of_planes = planes->size();
        if(number_of_planes == 0){
            std::cout << "No planes, nothing to do" << std::endl;
            return super_voxel_planes;
        }

        bool use_transform = false;
        float voxel_resolution = 0.2f;
        float seed_resolution = 0.2f;
        float color_importance = 0.5f;
        float spatial_importance = 0.4f;
        float normal_importance = 0.1f;

        
        // super.setNormalImportance (normal_importance);

        for (int i = 0; i <  number_of_planes; i++)
        {
            pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution, use_transform);
            super.setColorImportance (color_importance);
            super.setSpatialImportance (spatial_importance);
            PointCloudT::Ptr currentPlane = (*planes)[i];
            super.setInputCloud (currentPlane);
            std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
            super.extract (supervoxel_clusters);

            super_voxel_planes.push_back(super.getVoxelCentroidCloud ());
        }

        return super_voxel_planes;
    }

    std::vector<pcl::PolygonMesh> greedyProjectionTriangulation(std::vector<PointCloudT::Ptr > *planes)
    {
        pcl::PolygonMesh mesh;
        std::vector<pcl::PolygonMesh> triangles;
        
        int number_of_planes = planes->size();
        if(number_of_planes == 0){
            std::cout << "No planes, nothing to do" << std::endl;
            return triangles;
        }

        for (int i = 0; i < number_of_planes; i++){
            triangles.push_back( greedyProjectionTriangulation( (*planes)[i] ) );
        }

        return triangles; 
    }

    pcl::PolygonMesh greedyProjectionTriangulation(PointCloudT::Ptr plane)
    {
        pcl::PolygonMesh triangle;

        // Normal estimation*
        pcl::NormalEstimation<PointT, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

        // Initialize objects
        pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;

        // Set the maximum distance between connected points (maximum edge length)
        gp3.setSearchRadius (0.5);

        // Set typical values for the parameters
        gp3.setMu (2.5);
        gp3.setMaximumNearestNeighbors (300);
        gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
        gp3.setMinimumAngle(M_PI/10); // 10 degrees
        gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
        gp3.setNormalConsistency(false);

        pcl::PolygonMesh triangles_planes;

        tree->setInputCloud (plane);
        n.setInputCloud (plane);
        n.setSearchMethod (tree);
        n.setKSearch (20);
        n.compute (*normals);

        // Concatenate the XYZ and normal fields*
        pcl::concatenateFields (*plane, *normals, *cloud_with_normals);

        tree2->setInputCloud (cloud_with_normals);

        // Get result
        gp3.setInputCloud (cloud_with_normals);
        gp3.setSearchMethod (tree2);
        gp3.reconstruct (triangle);

        return triangle;
    }



    pcl::PolygonMesh meshAppend(pcl::PolygonMesh a, pcl::PolygonMesh b)
    {
        pcl::PolygonMesh c;
        //concatenateFields(a.cloud, b.cloud, c.cloud); 
        a.polygons.insert(a.polygons.end(), b.polygons.begin(), b.polygons.end());
        return a;
    }




    private:
};

#endif