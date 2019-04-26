
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>

#include "pcl_ros/point_cloud.h"
#include <pcl/filters/voxel_grid.h>

#include "pcl_compression/pcl_conversions.h"


class PclCompression
{
  ros::NodeHandle nh_;
  ros::Subscriber pcl_sub_;
  ros::Publisher pcl_pub_;
  std::string input_topic_, out_topic_;
  double resolution_;

public:
  PclCompression(ros::NodeHandle *nh) {
    nh_ = *nh;

    nh_.getParam("input_topic", input_topic_);
    nh_.getParam("out_topic", out_topic_);
    nh_.getParam("resolution", resolution_);

    pcl_sub_ = nh_.subscribe(input_topic_, 1, &PclCompression::pcl_callback, this);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(out_topic_, 1);

    ROS_INFO("[pcl_compression] Input PCL topic: %s", pcl_sub_.getTopic().c_str());
    ROS_INFO("[pcl_compression] Output PCL topic: %s", pcl_pub_.getTopic().c_str());
    ROS_INFO("[pcl_compression] Compression voxel size: %f", resolution_);
  }

  ~PclCompression() {
    
  }

  void pcl_callback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*msg, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> grid;
    grid.setInputCloud(cloudPtr);
    grid.setLeafSize(resolution_, resolution_, resolution_);
    grid.filter(cloud_filtered);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);

    // Publish the data
    pcl_pub_.publish(output);
  }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_compression");
  ros::NodeHandle node("~");
  PclCompression obj(&node);

  ros::spin();

  return 0;
}