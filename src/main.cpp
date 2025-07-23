#include "camera_manager/camera_manager.hpp"
#include "camera_manager/data_saver.hpp"
#include "camera_manager/algorithm_client/algo_cli.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = rclcpp::NodeOptions();

  auto manager_node = std::make_shared<CameraManager>(options, exec);
  auto saver_node = std::make_shared<DataSaver>(options);

  exec->add_node(manager_node->get_node_base_interface());
  exec->add_node(saver_node->get_node_base_interface());

  exec->spin();

  rclcpp::shutdown();
}