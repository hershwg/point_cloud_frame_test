#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>

#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

tf::TransformListener* tf_listener;

void cloud_cb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  printf("pcl::PointCloud frame_id = '%s'.\n", cloud->header.frame_id.c_str());
}

void ros_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  printf("sensor_msgs::PointCloud2 frame_id = '%s'.\n", cloud->header.frame_id.c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_frame_test");
  ros::NodeHandle nh;

  ros::Subscriber pc_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >( "input", 1, cloud_cb );
  ros::Subscriber ros_sub = nh.subscribe<sensor_msgs::PointCloud2>( "input", 1, ros_cloud_cb );

  ros::spin();

  return 0;
}

