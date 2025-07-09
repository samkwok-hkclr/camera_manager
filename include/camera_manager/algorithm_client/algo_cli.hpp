#ifndef ALGO_CLI_HPP__
#define ALGO_CLI_HPP__

#pragma once

#include <functional>
#include <future>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include "std_msgs/msg/int8.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "robotic_platform_msgs/msg/detected_object.hpp"
#include "robotic_platform_msgs/msg/detection_result.hpp"
#include "robotic_platform_msgs/msg/localization_param.hpp"
#include "robotic_platform_msgs/msg/object_pose.hpp"

#include "robotic_platform_msgs/srv/init_algo.hpp"
#include "robotic_platform_msgs/srv/get_object_pose.hpp"
#include "robotic_platform_msgs/srv/get_slot_state.hpp"

class AlgoCli : public rclcpp::Node
{
  using Int8 = std_msgs::msg::Int8;

  using Image = sensor_msgs::msg::Image;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  using DetectedObject = robotic_platform_msgs::msg::DetectedObject;
  using DetectionResult = robotic_platform_msgs::msg::DetectionResult;
  using LocalizationParam = robotic_platform_msgs::msg::LocalizationParam;
  using ObjectPose = robotic_platform_msgs::msg::ObjectPose;

  using InitAlgo = robotic_platform_msgs::srv::InitAlgo;
  using GetObjectPose = robotic_platform_msgs::srv::GetObjectPose;
  using GetSlotState = robotic_platform_msgs::srv::GetSlotState;

public:
  AlgoCli();
  AlgoCli(const rclcpp::NodeOptions& options);
  virtual ~AlgoCli() = default;

  void init_algo(void);

  virtual bool get_obj_pose(
    const Image::SharedPtr image, 
    const PointCloud2::SharedPtr pc,
    const robotic_platform_msgs::msg::LocalizationParam::SharedPtr param, 
    DetectionResult::SharedPtr detected_result);

  virtual bool get_slot_state(
    const Image::SharedPtr image, 
    const PointCloud2::SharedPtr pc,
    const LocalizationParam::SharedPtr param, 
    Int8::SharedPtr refill_qty);

  template <typename T>
  bool cli_wait_for_srv(
    typename rclcpp::Client<T>::SharedPtr cli, 
    const std::string srv_name) const;

private:
  uint8_t algo_id_;
  // std::string save_vis_path_;
  rclcpp::CallbackGroup::SharedPtr cli_cbg_;

  rclcpp::TimerBase::SharedPtr init_timer_;
  
  rclcpp::Client<InitAlgo>::SharedPtr init_algo_cli_;
  rclcpp::Client<GetObjectPose>::SharedPtr get_obj_pose_cli_;
  rclcpp::Client<GetSlotState>::SharedPtr get_slot_state_cli_;
  
  constexpr static uint8_t SRV_CLI_MAX_RETIES = 3;

};

#endif // ALGO_CLI_HPP__