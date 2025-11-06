#include "camera_manager/data_saver.hpp"

DataSaver::DataSaver(const rclcpp::NodeOptions& options)
: Node("data_saver", options)
{
  declare_parameter<std::string>("workspace_name", "");
  declare_parameter<std::string>("save_folder_name", "");
  get_parameter("workspace_name", ws_name_);
  get_parameter("save_folder_name", save_folder_name_);

  image_save_sub_ = create_subscription<Image>(
    "/save_image", 
    10, 
    std::bind(&DataSaver::image_save_cb, this, _1));

  pc_save_sub_ = create_subscription<PointCloud2>( 
    "/save_pointcloud", 
    10, 
    std::bind(&DataSaver::pc_save_cb, this, _1));

  if (get_save_path().empty())
  {
    RCLCPP_INFO(get_logger(), "save path is empty!");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(get_logger(), "ws_name_: %s", ws_name_.c_str());
  RCLCPP_INFO(get_logger(), "Date folder name sample: %s", get_date().c_str());
  RCLCPP_INFO(get_logger(), "Data file name sample: image_%s.png", get_time().c_str());
  RCLCPP_INFO(get_logger(), "Data Saver is up.");
}

DataSaver::~DataSaver()
{

}

std::string DataSaver::get_home_path(void)
{
  const char* home_path = std::getenv("HOME");
  if (!home_path)
  {
    RCLCPP_ERROR(this->get_logger(), "HOME does not existed!!!");
    return "";
  }

  RCLCPP_DEBUG(get_logger(), "HOME: %s", home_path);
  return std::string(home_path);
}

std::string DataSaver::get_save_path(void)
{
  const char* ws_name = std::getenv("WS_NAME");
  std::string base_path = get_home_path();
  
  if (base_path.empty())
  {
    return "/" + save_folder_name_;
  }

  std::string save_path;
  if (!ws_name_.empty())
  {
    // Prioritize member variable ws_name_ if set
    save_path = base_path + "/" + ws_name_ + "/" + save_folder_name_;
    RCLCPP_DEBUG(this->get_logger(), "Using member variable workspace name: %s", ws_name_.c_str());
  }
  else if (ws_name)
  {
    // Use environment variable WS_NAME if ws_name_ is empty
    save_path = base_path + "/" + ws_name + "/" + save_folder_name_;
    RCLCPP_WARN(this->get_logger(), "Using environment variable WS_NAME: %s", ws_name);
  }
  else
  {
    save_path = base_path + "/" + save_folder_name_;
    RCLCPP_WARN(this->get_logger(), "Neither WS_NAME nor ws_name_ set. Using default save folder: %s", 
      save_path.c_str());
  }

  RCLCPP_DEBUG(get_logger(), "Save path: %s", save_path.c_str());
  return save_path;
}

bool DataSaver::create_save_folder(void)
{
  std::string save_path = get_save_path();
  if (save_path.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot create save folder: Invalid path");
    return false;
  }

  try
  {
    if (std::filesystem::exists(save_path))
    {
      RCLCPP_DEBUG(this->get_logger(), "Save folder already exists: %s", save_path.c_str());
      return true;
    }

    if (std::filesystem::create_directories(save_path))
    {
      RCLCPP_INFO(this->get_logger(), "Created save folder: %s", save_path.c_str());
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to create save folder: %s", save_path.c_str());
      return false;
    }
  }
  catch (const std::filesystem::filesystem_error& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Filesystem error while creating save folder: %s", e.what());
    return false;
  }

  return false;
}

bool DataSaver::create_date_folder(void)
{
  std::string save_path = get_save_path();
  if (save_path.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot create date folder: Invalid path");
    return false;
  }

  std::string date_path = save_path + "/" + get_date();
  try
  {
    if (std::filesystem::exists(date_path))
    {
      RCLCPP_DEBUG(this->get_logger(), "Date folder already exists: %s", date_path.c_str());
      return true;
    }

    if (std::filesystem::create_directory(date_path))
    {
      RCLCPP_WARN(this->get_logger(), "Created date folder: %s", date_path.c_str());
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to create date folder: %s", date_path.c_str());
      return false;
    }
  }
  catch (const std::filesystem::filesystem_error& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Filesystem error while creating date folder: %s", e.what());
    return false;
  }

  return false;
}

std::string DataSaver::get_date(void)
{
  auto now = std::chrono::system_clock::now();
  std::time_t curr_time = std::chrono::system_clock::to_time_t(now);
  
  std::stringstream ss;
  ss << std::put_time(std::localtime(&curr_time), "%Y_%m_%d");
  return ss.str();
}

std::string DataSaver::get_time(void)
{
  auto now = std::chrono::system_clock::now();
  std::time_t curr_time = std::chrono::system_clock::to_time_t(now);
  
  std::stringstream ss;
  ss << std::put_time(std::localtime(&curr_time), "%H_%M_%S");
  return ss.str();
}

bool DataSaver::folder_validation(void)
{
  if (!create_save_folder())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to create save folder for image saving");
    return false;
  }

  if (!create_date_folder())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to create date folder for image saving");
    return false;
  }

  return true;
}

void DataSaver::image_save_cb(const Image::SharedPtr msg)
{
  if (!folder_validation())
  {
    RCLCPP_ERROR(this->get_logger(), "%s failed", __FUNCTION__);
    return;
  }

  std::string save_path = get_save_path();
  if (save_path.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid save path for image saving");
    return;
  }

  std::string date_folder = save_path + "/" + get_date();
  std::string filename = date_folder + "/img_" + get_time() + ".png";

  try
  {
    RCLCPP_WARN(this->get_logger(), "ROS msg encoding: %s", msg->encoding.c_str());
    // FIXME: why the source is rgb8 but cv_bridge requires bgr8
    // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding.c_str());
    // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding.c_str());

    if (!cv_ptr)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to convert ROS2 Image to OpenCV image");
      return;
    }

    if (cv::imwrite(filename, cv_ptr->image))
    {
      RCLCPP_WARN(this->get_logger(), "Saved image to: %s", filename.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to save image to: %s", filename.c_str());
    }
  }
  catch (const cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error saving image: %s", e.what());
  }
}

void DataSaver::pc_save_cb(const PointCloud2::SharedPtr msg)
{
  if (!folder_validation())
  {
    RCLCPP_ERROR(this->get_logger(), "%s failed", __FUNCTION__);
    return;
  }

  std::string save_path = get_save_path();
  if (save_path.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid save path for point cloud saving");
    return;
  }

  std::string date_folder = save_path + "/" + get_date();
  // std::string filename = date_folder + "/pc_" + get_time() + ".pcd";
  std::string filename = date_folder + "/ply_" + get_time() + ".ply";

  try
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    // if (pcl::io::savePCDFileASCII(filename, *cloud) == 0)
    if (pcl::io::savePLYFileASCII(filename, *cloud) == 0)
    {
      RCLCPP_WARN(this->get_logger(), "Saved point cloud to: %s", filename.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud to: %s", filename.c_str());
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error saving point cloud: %s", e.what());
  }
}