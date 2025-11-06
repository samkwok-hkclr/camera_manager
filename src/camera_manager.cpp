#include "camera_manager/camera_manager.hpp"

CameraManager::CameraManager(
  const rclcpp::NodeOptions& options,
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec)
: Node("camera_manager", options)
{
  declare_parameter<bool>("sim", false);
  declare_parameter<int64_t>("buf_max_size", 100);
  declare_parameter<std::vector<int>>("camera_id");
  declare_parameter<std::vector<std::string>>("image_topic");
  declare_parameter<std::vector<std::string>>("pointcloud_topic");

  get_parameter("sim", sim_);
  get_parameter("buf_max_size", buf_max_size_);

  std::vector<int64_t> cam_id_tmp = get_parameter("camera_id").as_integer_array();
  std::vector<std::string> image_topic_tmp = get_parameter("image_topic").as_string_array();
  std::vector<std::string> pc_topic_tmp = get_parameter("pointcloud_topic").as_string_array();

  if (cam_id_tmp.empty())
  {
    RCLCPP_ERROR(get_logger(), "No camera IDs provided. Shutting down.");
    rclcpp::shutdown();
    return;
  }

  if (!(cam_id_tmp.size() == image_topic_tmp.size() && image_topic_tmp.size() == pc_topic_tmp.size()))
  {
    RCLCPP_ERROR(get_logger(), "Array size mismatch: camera_id (%ld), image_topic (%ld), pointcloud_topic (%ld). Shutting down.",
      cam_id_tmp.size(), image_topic_tmp.size(), pc_topic_tmp.size());
    rclcpp::shutdown();
    return;
  }

  for (size_t i = 0; i < cam_id_tmp.size(); i++)
  {
    CameraId camera_id = static_cast<CameraId>(cam_id_tmp[i]);

    image_topic_[camera_id] = image_topic_tmp[i];
    pc_topic_[camera_id] = pc_topic_tmp[i];
  }

  algo_cli_node_ = std::make_shared<AlgoCli>(options);
  exec->add_node(algo_cli_node_->get_node_base_interface());

  image_sub_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  pc_sub_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  srv_ser_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  status_timer_ = create_wall_timer(500ms, std::bind(&CameraManager::pub_status_cb, this));

  rclcpp::SubscriptionOptions image_sub_options;
  image_sub_options.callback_group = image_sub_cbg_;
  rclcpp::SubscriptionOptions pc_sub_options;
  pc_sub_options.callback_group = pc_sub_cbg_;

  status_pub_ = create_publisher<Image>("/testing_image", 10); 
  save_image_pub_ = create_publisher<Image>("/save_image", 10); 
  save_pc_pub_ = create_publisher<PointCloud2>("/save_pointcloud", 10); 

  for (size_t i = 0; i < cam_id_tmp.size(); i++)
  {
    CameraId cam_id = static_cast<CameraId>(cam_id_tmp[i]);

    if (!image_topic_tmp[i].empty())
    {
      image_cb_[cam_id] = std::bind(&CameraManager::image_recv_cb, this, _1, cam_id);
      image_sub_[cam_id] = create_subscription<Image>(
        image_topic_[cam_id], 
        rclcpp::QoS(rclcpp::KeepLast(10)), 
        image_cb_[cam_id],
        image_sub_options);

      RCLCPP_INFO(get_logger(), "Created image subscription group of Camera: %d", cam_id);
    }
    
    if (!pc_topic_tmp[i].empty())
    {
      pc_cb_[cam_id] = std::bind(&CameraManager::pc_recv_cb, this, _1, cam_id);
      pc_sub_[cam_id] = create_subscription<PointCloud2>(
        pc_topic_[cam_id], 
        rclcpp::QoS(rclcpp::KeepLast(10)), 
        pc_cb_[cam_id],
        pc_sub_options);

      RCLCPP_INFO(get_logger(), "Created pointcloud subscription group of Camera: %d", cam_id);
    }
  }

  get_obj_pose_tri_srv_ = create_service<GetObjectPoseTrigger>(
    "get_object_pose_trigger", 
    std::bind(&CameraManager::get_obj_pose_tri_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  get_slot_state_tri_srv_ = create_service<GetSlotStateTrigger>(
    "get_slot_state_trigger", 
    std::bind(&CameraManager::get_slot_state_tri_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  save_cam_data_srv_ = create_service<SaveCameraData>(
    "save_camera_data", 
    std::bind(&CameraManager::save_cam_data_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  RCLCPP_INFO(get_logger(), "Camera Manager is up.");
}

CameraManager::~CameraManager()
{

}

void CameraManager::pub_status_cb(void)
{
  // TODO: publish server status

  // FIXME: publish image for debugging only!!!
  std::lock_guard<std::mutex> lock(image_mutexes_[CameraId::ONE]);
  
  if (image_buf_[CameraId::ONE].empty())
    return;
  
  status_pub_->publish(*image_buf_[CameraId::ONE].back());
}

void CameraManager::image_recv_cb(const Image::SharedPtr msg, const CameraId id)
{
  std::lock_guard<std::mutex> lock(image_mutexes_[id]);  

  auto& buf = image_buf_[id];
  buf.push_back(msg);

  if (buf_max_size_ > 0 && buf.size() > buf_max_size_) 
    buf.pop_front();
  
  RCLCPP_DEBUG(get_logger(), "Image received! %s <Size: %ld>", 
    id == CameraId::ONE ? "left" : "right", 
    buf.size());
}

void CameraManager::pc_recv_cb(const PointCloud2::SharedPtr msg, const CameraId id)
{
  std::lock_guard<std::mutex> lock(pc_mutexes_[id]);  

  auto& buf = pc_buf_[id];
  buf.push_back(msg);

  if (buf_max_size_ > 0 && buf.size() > buf_max_size_) 
    buf.pop_front();

  RCLCPP_DEBUG(get_logger(), "Pointcloud received! %s <Size: %ld>", 
    id == CameraId::ONE ? "left" : "right", 
    buf.size());
}

void CameraManager::get_obj_pose_tri_cb(
  const std::shared_ptr<GetObjectPoseTrigger::Request> request, 
  std::shared_ptr<GetObjectPoseTrigger::Response> response)
{
  if (request->camera_id < CameraId::ONE || request->camera_id >= CameraId::LAST)
    return;

  const CameraId id = static_cast<CameraId>(request->camera_id);

  auto param_msg = std::make_shared<LocalizationParam>();
  param_msg->target_object_id = request->target_object_id;
  RCLCPP_WARN(get_logger(), "target_object_id: %d", request->target_object_id);

  auto detect_result_msg = std::make_shared<DetectionResult>();

  const rclcpp::Time curr_time = get_clock()->now();

  auto image_copy = get_data_copy<Image>(image_buf_[id], image_mutexes_[id], id, curr_time);
  auto pc_copy = get_data_copy<PointCloud2>(pc_buf_[id], pc_mutexes_[id], id, curr_time);

  bool success{false};
  success = algo_cli_node_->get_obj_pose(image_copy, pc_copy, param_msg, detect_result_msg);
  if (!success)
  {
    RCLCPP_ERROR(get_logger(), "algo_cli_node_->get_obj_pose failed");
    return;
  }

  for (const auto& x : detect_result_msg->detected_objects)
  {
    ObjectPose msg;
    msg.object_id = request->target_object_id;
    msg.pose = x.camera_pose;
    response->object_poses.emplace_back(std::move(msg));
  }

  success = save_cam_data<Image>(image_copy, id, save_image_pub_) &&
            save_cam_data<PointCloud2>(pc_copy, id, save_pc_pub_);

  if (!success)
  {
    RCLCPP_ERROR(get_logger(), "Save camera data failed but continue to process");
  }

  RCLCPP_INFO(get_logger(), "return pose size: %ld", detect_result_msg->detected_objects.size());
  response->success = true;
}

void CameraManager::get_slot_state_tri_cb(
  const std::shared_ptr<GetSlotStateTrigger::Request> request, 
  std::shared_ptr<GetSlotStateTrigger::Response> response)
{
  if (request->camera_id < CameraId::ONE || request->camera_id >= CameraId::LAST)
    return;

  const CameraId id = static_cast<CameraId>(request->camera_id);

  auto param_msg = std::make_shared<LocalizationParam>();
  param_msg->target_object_id = request->target_object_id;

  auto qty_msg = std::make_shared<Int8>();

  const rclcpp::Time curr_time = get_clock()->now();

  auto image_copy = get_data_copy<Image>(image_buf_[id], image_mutexes_[id], id, curr_time);
  auto pc_copy = get_data_copy<PointCloud2>(pc_buf_[id], pc_mutexes_[id], id, curr_time);

  bool success{false};
  success = algo_cli_node_->get_slot_state(image_copy, pc_copy, param_msg, qty_msg);
  if (!success)
  {
    RCLCPP_ERROR(get_logger(), "algo_cli_node_->get_slot_state failed");
    return;
  }

  success = save_cam_data<Image>(image_copy, id, save_image_pub_) &&
            save_cam_data<PointCloud2>(pc_copy, id, save_pc_pub_);

  if (!success)
  {
    RCLCPP_ERROR(get_logger(), "Save camera data failed but continue to process");
  }

  RCLCPP_INFO(get_logger(), "replenish quantity: %d", qty_msg->data);
  response->success = true;
}

void CameraManager::save_cam_data_cb(
  const std::shared_ptr<SaveCameraData::Request> request, 
  std::shared_ptr<SaveCameraData::Response> response)
{
  const CameraId id = static_cast<CameraId>(request->camera_id);
  const auto& target_time = request->target_time;

  bool success = false;

  auto save_helper = [&](auto& buf, auto& mutex, auto& pub) {
    return save_cam_data<std::decay_t<decltype(*buf[0])>>(buf, mutex, id, target_time, pub);
  };

  switch (request->type)
  {
  case CameraDataType::IMAGE:
    /* code */
    success = save_helper(image_buf_[id], image_mutexes_[id], save_image_pub_);
    break;
  case CameraDataType::POINTCLOUD:
    success = save_helper(pc_buf_[id], pc_mutexes_[id], save_pc_pub_);
    break;  
  case CameraDataType::BOTH:
    // TODO: how to handle partial success???
    success = save_helper(image_buf_[id], image_mutexes_[id], save_image_pub_);
    success &= save_helper(pc_buf_[id], pc_mutexes_[id], save_pc_pub_);
    break; 
  default:
    RCLCPP_ERROR(get_logger(), "type does not defined.");
    return;
  }

  if (target_time.sec == 0 && target_time.nanosec == 0)
  {
    RCLCPP_INFO(get_logger(), "Save operation %s for Camera %d", 
      success ? "succeeded" : "failed", 
      id);
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Save operation %s for Camera %d at time %d.%d", 
      success ? "succeeded" : "failed", 
      id, 
      target_time.sec,
      target_time.nanosec);
  }

  response->success = success;
}

template <typename T>
std::shared_ptr<T> CameraManager::get_data_copy(
  std::deque<typename T::SharedPtr>& buf,
  std::mutex& mutex,
  const CameraId id,
  const rclcpp::Time target_time) const
{
  std::lock_guard<std::mutex> lock(mutex);

  if (buf.empty()) 
  {
    RCLCPP_ERROR(get_logger(), "Buffer for Camera %d is empty. No data available.", id);
    return nullptr;
  }

  auto target_it = search_data<T>(buf, target_time);
  if (target_it == buf.crend()) 
  {
    // This case should not be happened
    RCLCPP_WARN(get_logger(), "No image found for camera %d at target time", id);
    return nullptr;
  }
  if (target_it == buf.crbegin()) 
  {
    RCLCPP_DEBUG(get_logger(), "target is the lastest data");
  }

  std::shared_ptr<T> copy = std::make_shared<T>(**target_it);
  RCLCPP_DEBUG(get_logger(), "copy stamp second: %d", copy->header.stamp.sec);

  return copy;
}

template <typename T>
bool CameraManager::save_cam_data(
  std::deque<typename T::SharedPtr>& buf,
  std::mutex& mutex,
  const CameraId id,
  const rclcpp::Time target_time,
  typename rclcpp::Publisher<T>::SharedPtr pub) const
{
  std::lock_guard<std::mutex> lock(mutex);

  if (buf.empty())
  {
    RCLCPP_ERROR(get_logger(), "Buffer for Camera %d is empty. No data to save.", id);
    return false;
  }
    
  auto target_it = search_data<T>(buf, target_time);
  if (target_it == buf.crend()) 
  {
    RCLCPP_WARN(get_logger(), "No image found for camera %d at target time", id);
    return false;
  }

  if (!pub) 
  {
    RCLCPP_ERROR(get_logger(), "Publisher not initialized!");
    return false;
  }
  pub->publish(**target_it);

  RCLCPP_DEBUG(get_logger(), "Published data for camera %d at time %f", id, target_time.seconds());
  return true;
}

template <typename T>
bool CameraManager::save_cam_data(
  typename T::SharedPtr& msg, 
  const CameraId id, 
  typename rclcpp::Publisher<T>::SharedPtr pub) const
{
  if (!pub) 
  {
    RCLCPP_ERROR(get_logger(), "Publisher not initialized!");
    return false;
  }
  pub->publish(*msg);

  RCLCPP_DEBUG(get_logger(), "Published data for camera %d at time %d", id, msg->header.stamp.sec);
  return true;
}

template <typename T>
typename std::deque<typename T::SharedPtr>::const_reverse_iterator CameraManager::search_data(
  std::deque<typename T::SharedPtr>& buf,
  const rclcpp::Time& target_time) const
{
  auto comp_func = [](const typename T::SharedPtr data, const rclcpp::Time& time) {
    return rclcpp::Time(data->header.stamp) < time;
  };

  auto it = std::lower_bound(buf.crbegin(), buf.crend(), target_time, comp_func);

  if (it == buf.crbegin() || it == buf.crend())
  {
    RCLCPP_INFO(get_logger(), "Save the lastest data");
    return buf.crbegin();
  }
  
  // Compare with previous element to find the closest
  auto prev_it = std::prev(it);
  rclcpp::Time prev_time((*prev_it)->header.stamp);
  rclcpp::Time current_time((*it)->header.stamp);

  if (std::abs((target_time - prev_time).seconds()) < std::abs((target_time - current_time).seconds())) 
  {
    RCLCPP_INFO(get_logger(), "prev_it");
    return prev_it;
  }
  else
  {
    RCLCPP_INFO(get_logger(), "it");
    return it;
  }
}
