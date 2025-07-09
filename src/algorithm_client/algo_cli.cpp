#include "camera_manager/algorithm_client/algo_cli.hpp"

AlgoCli::AlgoCli(
  const rclcpp::NodeOptions& options)
: Node("algorithm_client", options)
{
  declare_parameter<int>("algo_id", 0);
  get_parameter("algo_id", algo_id_);

  cli_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  init_timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&AlgoCli::init_algo, this));

  init_algo_cli_ = create_client<InitAlgo>(
    "init_algo",
    rmw_qos_profile_services_default,
    cli_cbg_);

  get_obj_pose_cli_ = create_client<GetObjectPose>(
    "get_object_pose",
    rmw_qos_profile_services_default,
    cli_cbg_);

  get_slot_state_cli_ = create_client<GetSlotState>(
    "get_slot_state",
    rmw_qos_profile_services_default,
    cli_cbg_);

  RCLCPP_INFO(get_logger(), "Algorithm Client is up.");
}

void AlgoCli::init_algo(void)
{
  RCLCPP_INFO(get_logger(), "Start to init algo id: %d", algo_id_);

  if (!cli_wait_for_srv<InitAlgo>(init_algo_cli_, __FUNCTION__))
    return;

  auto request = std::make_shared<InitAlgo::Request>();
  request->header.stamp = get_clock()->now();
  request->localization_algo_id = algo_id_;
  // request->config_file = config_file;
  // request->save_vis_path = save_vis_path_;
    
  auto future = init_algo_cli_->async_send_request(request);
  std::future_status status = future.wait_for(std::chrono::seconds(10));

  if (status != std::future_status::ready)
  {
    return;
  }

  const auto& response = future.get();
  if (response->success)
  {
    init_timer_->cancel();
    RCLCPP_INFO(get_logger(), "Init algo successfully");
  }
}

bool AlgoCli::get_obj_pose(
  const sensor_msgs::msg::Image::SharedPtr image, 
  const sensor_msgs::msg::PointCloud2::SharedPtr pc,
  const robotic_platform_msgs::msg::LocalizationParam::SharedPtr param, 
  robotic_platform_msgs::msg::DetectionResult::SharedPtr detected_result)
{
  const uint8_t object_id = param->target_object_id;
  RCLCPP_INFO(get_logger(), "object_id : %d", object_id);
  
  if (!cli_wait_for_srv<GetObjectPose>(get_obj_pose_cli_, __FUNCTION__))
    return false;

  auto request = std::make_shared<GetObjectPose::Request>();

  request->localization_algo_id = algo_id_;
  request->target_object_id = object_id;
  // request->detect_bin_direction_flag = 2; // FIXME
  // request->detect_object_type_flag = 3; // FIXME
  // request->bin_type = 4; // FIXME
  request->image = std::move(*image);
  request->pointcloud = std::move(*pc);

  auto start_time = std::chrono::system_clock::now();

  auto future = get_obj_pose_cli_->async_send_request(request);
  std::future_status status = future.wait_for(std::chrono::seconds(30));

  auto end_time = std::chrono::system_clock::now();

  std::chrono::duration<double> elapsed_seconds = end_time - start_time;
  RCLCPP_INFO(get_logger(), "Client side total time for request to SensingServer %.2fs", elapsed_seconds.count());

  if (status != std::future_status::ready)
  {
    RCLCPP_INFO(get_logger(), "Failed to call service GetObjectPose due to %d", static_cast<int>(status));
    return false;
  }
  
  auto response = future.get();

  if (!response->success)
  {
    RCLCPP_INFO(get_logger(), "GetObjectPose is unsuccessful");
    return false;
  }

  for (const auto& object_pose : response->object_poses)
  {
    DetectedObject obj;

    obj.camera_pose = object_pose.pose;
    obj.object_id = object_pose.id;
    obj.size = object_pose.size;
    obj.gripper = object_pose.gripper;

    detected_result->detected_objects.emplace_back(std::move(obj));
  }

  RCLCPP_INFO(get_logger(), "Call GetObjectPose Service is successfully");
  return true;
}

bool AlgoCli::get_slot_state(
  const sensor_msgs::msg::Image::SharedPtr image, 
  const sensor_msgs::msg::PointCloud2::SharedPtr pc,
  const robotic_platform_msgs::msg::LocalizationParam::SharedPtr param, 
  Int8::SharedPtr refill_qty)
{
  const uint8_t object_id = param->target_object_id;
  RCLCPP_INFO(get_logger(), "object_id : %d", object_id);
  
  if (!cli_wait_for_srv<GetSlotState>(get_slot_state_cli_, __FUNCTION__))
    return false;

  auto request = std::make_shared<GetSlotState::Request>();

  request->target_object_id = object_id; 
  request->image = std::move(*image);
  request->pointcloud = std::move(*pc);

  auto start_time = std::chrono::system_clock::now();

  auto future = get_slot_state_cli_->async_send_request(request);
  std::future_status status = future.wait_for(std::chrono::seconds(30));

  auto end_time = std::chrono::system_clock::now();

  std::chrono::duration<double> elapsed_seconds = start_time - end_time;
  RCLCPP_INFO(get_logger(), "Client side total time for request to SensingServer %fs", elapsed_seconds.count());

  if (status != std::future_status::ready)
  {
    RCLCPP_INFO(get_logger(), "Failed to call service GetSlotState due to %d", static_cast<int>(status));
    return false;
  }

  auto response = future.get();
  if (!response->success)
  {
    RCLCPP_INFO(get_logger(), "GetSlotState is unsuccessful");
    return false;
  }

  refill_qty->data = response->turn_over_detected ? -1 : response->remain_qty;

  RCLCPP_INFO(get_logger(), "Call GetObjectPose Service is successfully. object_id [%d] replenish quantity: %d", object_id, refill_qty->data);
  return true;
}

template <typename T>
bool AlgoCli::cli_wait_for_srv(
  typename rclcpp::Client<T>::SharedPtr cli, 
  const std::string srv_name) const
{
  uint8_t retry = 0;

  while (rclcpp::ok() && !cli->wait_for_service(std::chrono::milliseconds(200)))
  {
    if (retry >= SRV_CLI_MAX_RETIES)
    {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }

    RCLCPP_WARN(get_logger(), "%s service not available, waiting again...", srv_name.c_str());
    retry++;
  }

  return true;
}