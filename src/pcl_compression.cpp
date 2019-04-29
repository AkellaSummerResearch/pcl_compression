
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>

#include "pcl_ros/point_cloud.h"
#include <pcl/filters/voxel_grid.h>


class PclCompression
{
  ros::NodeHandle nh_;
  ros::Subscriber pcl_sub_;
  ros::Publisher pcl_pub_;
  std::string input_topic_, out_topic_;
  double resolution_;
  int drop_factor_;
  int cur_idx_;
  float min_distance_sqr_, max_distance_sqr_;

public:
  PclCompression(ros::NodeHandle *nh) {
    nh_ = *nh;

    float min_distance, max_distance;
    nh_.getParam("input_topic", input_topic_);
    nh_.getParam("out_topic", out_topic_);
    nh_.getParam("resolution", resolution_);
    nh_.getParam("max_distance", max_distance);
    nh_.getParam("min_distance", min_distance);
    nh_.getParam("drop_factor", drop_factor_);

    if (drop_factor_ < 1) {
      drop_factor_ = 1;
      ROS_WARN("[pcl_compression] drop_factor has to be greater than or equal to 1. Setting drop_factor = 1");
    }
    cur_idx_ = 0;
    min_distance_sqr_ = min_distance*min_distance;
    max_distance_sqr_ = max_distance*max_distance;

    pcl_sub_ = nh_.subscribe(input_topic_, 1, &PclCompression::pcl_callback, this);
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(out_topic_, 1);

    ROS_INFO("[pcl_compression] Input PCL topic: %s", pcl_sub_.getTopic().c_str());
    ROS_INFO("[pcl_compression] Output PCL topic: %s", pcl_pub_.getTopic().c_str());
    ROS_INFO("[pcl_compression] Compression voxel size: %f", resolution_);
    ROS_INFO("[pcl_compression] Minimum point distance: %f", min_distance);
    ROS_INFO("[pcl_compression] Maximum point distance: %f", max_distance);
    ROS_INFO("[pcl_compression] Drop factor: %d", drop_factor_);
  }

  ~PclCompression() {
    
  }

  void pcl_callback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // Only execute the function when cur_idx == 0
    if (cur_idx_ == drop_factor_-1) {
      cur_idx_ = 0;
    } else {
      cur_idx_++;
      return;
    }

    // Container for original & filtered data
    // ros::Time t0 = ros::Time::now();
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert to PCL data type
    pcl_conversions::toPCL(*msg, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> grid;
    grid.setInputCloud(cloudPtr);
    grid.setLeafSize(resolution_, resolution_, resolution_);
    // grid.setFilterLimits(min_distance_, max_distance_);
    grid.filter(cloud_filtered);

    // Perform min/max distance filtering
    pcl::fromPCLPointCloud2(cloud_filtered, *pcl_cloud_filtered);
    pcl::PointCloud<pcl::PointXYZ> cloud_out;
    pcl::PointCloud<pcl::PointXYZ>::iterator it;
    for(it = pcl_cloud_filtered->begin(); it != pcl_cloud_filtered->end(); it++) {
      float point_dist_sqr = it->x*it->x + it->y*it->y + it->z*it->z;

      if((point_dist_sqr > min_distance_sqr_) &&(point_dist_sqr < max_distance_sqr_)) {
        cloud_out.push_back(*it);
      }
    } 

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_out, output);
    output.header.stamp = msg->header.stamp;
    output.header.frame_id = msg->header.frame_id;

    // Publish the data
    pcl_pub_.publish(output);
    // ros::Time tf = ros::Time::now();
    // std::cout << (tf - t0).toSec() << std::endl;
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