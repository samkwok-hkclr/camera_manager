#ifndef FAKE_ALGO_CLI_HPP__
#define FAKE_ALGO_CLI_HPP__

#pragma once

#include "algo_cli.hpp"

/*
  NOT READY TO USE!!!
  NOT READY TO USE!!!
  NOT READY TO USE!!!
*/

class FakeAlgoCli : public AlgoCli
{
  using Int8 = std_msgs::msg::Int8;

  using Image = sensor_msgs::msg::Image;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  using DetectedObject = robotic_platform_msgs::msg::DetectedObject;
  using DetectionResult = robotic_platform_msgs::msg::DetectionResult;
  using LocalizationParam = robotic_platform_msgs::msg::LocalizationParam;
  using ObjectPose = robotic_platform_msgs::msg::ObjectPose;

public:
  FakeAlgoCli();
  virtual ~FakeAlgoCli() override = default;

  bool get_obj_pose(
    const Image::SharedPtr image, 
    const PointCloud2::SharedPtr pc,
    const robotic_platform_msgs::msg::LocalizationParam::SharedPtr param, 
    DetectionResult::SharedPtr detected_result);

  bool get_slot_state(
    const Image::SharedPtr image, 
    const PointCloud2::SharedPtr pc,
    const LocalizationParam::SharedPtr param, 
    Int8::SharedPtr refill_qty);

private:
  
};

#endif // FAKE_ALGO_CLI_HPP__