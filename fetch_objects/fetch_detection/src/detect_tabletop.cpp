#include "ros/ros.h"

//import the pointcloud library
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

//import library for transforms
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/common/transforms.h>
#include "Eigen/Core"
#include "Eigen/Geometry"

//import the octomap library and dependancies
#include <octomap/octomap.h>
#include <pcl/octree/octree.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class SubscribePublish
{
public:
  SubscribePublish()
  {
    sub = n.subscribe("/camera/depth/points", 1, &SubscribePublish::return_tabletop, this);

    tabletop_pub = n.advertise<PointCloud>("/tabletop", 1);
    items_pub = n.advertise<PointCloud>("/items", 1);
  }

  void return_tabletop(const PointCloud::ConstPtr& msg)
  {
    PointCloud::Ptr cloud (new PointCloud), items_cloud (new PointCloud), table_cloud (new PointCloud);
    
    cloud->header = msg->header;
    cloud->width = msg->width;
    cloud->height = msg->height;
    cloud->points = msg->points;
    cloud->is_dense = msg->is_dense;
    
    Eigen::Affine3d transform_eigen;
    tf::StampedTransform transform_tf;
    
    try
    {
      ros::Time now = ros::Time::now();
      listener.waitForTransform("camera_link", msg->header.frame_id, now, ros::Duration(0.05));
      listener.lookupTransform("camera_link", msg->header.frame_id, now, transform_tf);
    }
    catch (tf::TransformException &ex)
    {
      ros::Duration(0.05).sleep();
    }
    tf::transformTFToEigen(transform_tf, transform_eigen);
    pcl::transformPointCloud (*cloud, *cloud, transform_eigen);

    cloud->header.frame_id = "camera_link";
    cloud->header.frame_id = msg->header.frame_id;

    //pass through filter
        //x range
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*cloud);
        //y range
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-0.5, 0.5);
    pass.filter (*cloud);
        //z range
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-0.5, 0.5);
    pass.filter (*cloud);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.02);
    seg.setMaxIterations (100.0);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    
    extract.filter (*table_cloud);

    extract.setNegative (true);
    extract.filter (*items_cloud);

    tabletop_pub.publish(table_cloud);
    items_pub.publish(items_cloud);
  }

private:
  ros::NodeHandle n;
  ros::Publisher tabletop_pub;
  ros::Publisher items_pub;
  ros::Publisher octo_pub;
  ros::Subscriber sub;
  tf::TransformListener listener;
};



int main(int argc, char **argv)
{
  //initialise the node
  ros::init(argc, argv, "detect_tabletop");
  ros::NodeHandle n;

  SubscribePublish SAPObject;

  ros::spin();

  return 0;
}
