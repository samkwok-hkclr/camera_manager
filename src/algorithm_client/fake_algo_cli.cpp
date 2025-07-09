#include "vision_server/algorithm_client/fake_algo_cli.hpp"

FakeAlgoCli::FakeAlgoCli()
{

}

bool FakeAlgoCli::get_obj_pose(
  const Image::SharedPtr image, 
  const PointCloud2::SharedPtr pc,
  const robotic_platform_msgs::msg::LocalizationParam::SharedPtr param, 
  DetectionResult::SharedPtr detected_result)
{
  (void) image;
  (void) pc;
  (void) param;
  (void) detected_result;

  RCLCPP_INFO(get_logger(), "FakeAlgoCli getObjectPose");
  return true;
}

bool FakeAlgoCli::get_slot_state(
  const Image::SharedPtr image, 
  const PointCloud2::SharedPtr pc,
  const LocalizationParam::SharedPtr param, 
  Int8::SharedPtr refill_qty)
{
  (void) image;
  (void) pc;
  (void) param;
  (void) refill_qty;

  RCLCPP_INFO(get_logger(), "FakeAlgoCli getSlotState");
  return true;
}