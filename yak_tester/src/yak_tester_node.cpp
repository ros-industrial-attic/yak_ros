#include "rclcpp/rclcpp.hpp"


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("yak_tester_node");
  //node->create_client<yak_ros_msgs::srv::GenerateMesh>("generate_mesh_service");
  rclcpp::shutdown();
  return 0;
}
