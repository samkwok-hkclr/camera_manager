#ifndef VISION_SERVER_HPP__
#define VISION_SERVER_HPP__

#pragma once

#include <functional>
#include <future>
#include <memory>
#include <chrono>
#include <iterator>
#include <deque>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int8.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "robotic_platform_msgs/msg/detection_result.hpp"
#include "robotic_platform_msgs/msg/localization_param.hpp"

#include "robotic_platform_msgs/srv/get_object_pose_trigger.hpp"
#include "robotic_platform_msgs/srv/get_slot_state_trigger.hpp"
#include "robotic_platform_msgs/srv/save_camera_data.hpp"

#include "algorithm_client/algo_cli.hpp"
#include "algorithm_client/fake_algo_cli.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

enum CameraId: uint8_t
{
  LEFT = 1,
  RIGHT = 2,

  LAST // Caution: LAST should not be used
};

class CameraManager : public rclcpp::Node
{
  using Int8 = std_msgs::msg::Int8;

  using Image = sensor_msgs::msg::Image;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  using DetectionResult = robotic_platform_msgs::msg::DetectionResult;
  using LocalizationParam = robotic_platform_msgs::msg::LocalizationParam;

  using GetObjectPoseTrigger = robotic_platform_msgs::srv::GetObjectPoseTrigger;
  using GetSlotStateTrigger = robotic_platform_msgs::srv::GetSlotStateTrigger;
  using SaveCameraData = robotic_platform_msgs::srv::SaveCameraData;
  
public:
  explicit CameraManager(
    const rclcpp::NodeOptions& options, 
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec);
  ~CameraManager();

  void pub_status_cb(void);

  void image_recv_cb(const Image::SharedPtr msg, const CameraId id);
  void pc_recv_cb(const PointCloud2::SharedPtr msg, const CameraId id);

  void get_obj_pose_tri_cb(
    const std::shared_ptr<GetObjectPoseTrigger::Request> request, 
    std::shared_ptr<GetObjectPoseTrigger::Response> response);
  void get_slot_state_tri_cb(
    const std::shared_ptr<GetSlotStateTrigger::Request> request, 
    std::shared_ptr<GetSlotStateTrigger::Response> response);
  void save_cam_data_cb(
    const std::shared_ptr<SaveCameraData::Request> request, 
    std::shared_ptr<SaveCameraData::Response> response);

  template <typename T>
  bool save_cam_data(
    std::deque<typename T::SharedPtr>& buf, 
    std::mutex& mutex, 
    const CameraId id, 
    const rclcpp::Time target_time,
    typename rclcpp::Publisher<T>::SharedPtr pub) const;
  
  template <typename T>
  typename std::deque<typename T::SharedPtr>::reverse_iterator search_data(
    std::deque<typename T::SharedPtr>& buf, 
    const rclcpp::Time& time) const;

private:
  bool sim_;

  std::mutex mutex_;
  std::shared_ptr<AlgoCli> algo_cli_node_;

  std::unordered_map<CameraId, std::mutex> image_mutexes_;
  std::unordered_map<CameraId, std::mutex> pc_mutexes_;

  std::unordered_map<CameraId, std::string> image_topic_;
  std::unordered_map<CameraId, std::string> pc_topic_;

  size_t buf_max_size_;

  std::unordered_map<CameraId, std::deque<Image::SharedPtr>> image_buf_;
  std::unordered_map<CameraId, std::deque<PointCloud2::SharedPtr>> pc_buf_;
 
  std::unordered_map<CameraId, std::function<void(const Image::SharedPtr msg)>> image_cb_;
  std::unordered_map<CameraId, std::function<void(const PointCloud2::SharedPtr msg)>> pc_cb_;

  rclcpp::CallbackGroup::SharedPtr image_sub_cbg_;
  rclcpp::CallbackGroup::SharedPtr pc_sub_cbg_;
  rclcpp::CallbackGroup::SharedPtr srv_ser_cbg_;

  rclcpp::TimerBase::SharedPtr status_timer_;

  rclcpp::Publisher<Image>::SharedPtr status_pub_; // for debugging only
  rclcpp::Publisher<Image>::SharedPtr save_image_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr save_pc_pub_;

  std::unordered_map<CameraId, rclcpp::Subscription<Image>::SharedPtr> image_sub_;
  std::unordered_map<CameraId, rclcpp::Subscription<PointCloud2>::SharedPtr> pc_sub_;

  rclcpp::Service<GetObjectPoseTrigger>::SharedPtr get_obj_pose_tri_srv_;
  rclcpp::Service<GetSlotStateTrigger>::SharedPtr get_slot_state_tri_srv_;
  rclcpp::Service<SaveCameraData>::SharedPtr save_cam_data_srv_;
};

#endif // VISION_SERVER_HPP__