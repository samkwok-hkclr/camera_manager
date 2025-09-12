#ifndef DATA_SAVER_HPP__
#define DATA_SAVER_HPP__

#pragma once

#include <functional>
#include <memory>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class DataSaver : public rclcpp::Node
{
  using Image = sensor_msgs::msg::Image;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

public:
  explicit DataSaver(const rclcpp::NodeOptions& options);
  ~DataSaver();

  std::string get_home_path(void);
  std::string get_save_path(void);
  bool create_save_folder(void);
  bool create_date_folder(void);
  std::string get_date(void);
  std::string get_time(void);
  bool folder_validation(void);

  void image_save_cb(const Image::SharedPtr msg);
  void pc_save_cb(const PointCloud2::SharedPtr msg);

private:
  std::mutex mutex_;
  std::string ws_name_;
  std::string save_folder_name_;

  rclcpp::Subscription<Image>::SharedPtr image_save_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr pc_save_sub_;
  
};

#endif // DATA_SAVER_HPP__