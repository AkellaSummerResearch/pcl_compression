
#ifndef PCL_COMPRESSION_CLASS_H_
#define PCL_COMPRESSION_CLASS_H_

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>

#include "pcl_ros/point_cloud.h"
#include <pcl/filters/voxel_grid.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace pcl_compression_ns {

class PclCompressionClass : public nodelet::Nodelet {
  ros::NodeHandle nh_;
  ros::Subscriber pcl_sub_;
  ros::Publisher pcl_pub_;
  std::string input_topic_, out_topic_;
  double resolution_;
  int drop_factor_;
  int cur_idx_;
  float min_distance_sqr_, max_distance_sqr_;

public:
  PclCompressionClass();

  ~PclCompressionClass();

  virtual void onInit();

  void pcl_callback(const sensor_msgs::PointCloud2::ConstPtr &msg);
  
};

}  // namespace pcl_compression_ns

#endif // PCL_COMPRESSION_CLASS_H_
