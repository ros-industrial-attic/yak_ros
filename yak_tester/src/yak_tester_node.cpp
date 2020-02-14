#include "rclcpp/rclcpp.hpp"
//#include "yak_ros_msgs/srv/generate_mesh.hpp"


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclpp::Node::make_shared("yak_tester_node");
  //node->create_client<yak_ros_msgs::srv::GenerateMesh>("generate_mesh_service");
  rclcpp::shutdown();
  return 0;
}
