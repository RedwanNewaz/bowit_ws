#include "libbow/bowit_ros_interface.h"




int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  printf("hello world bowit package\n");
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node1 = std::make_shared<BowitROSInteface>();
  executor.add_node(node1);
  executor.spin();
  rclcpp::shutdown();


  return 0;
}
