#include "viz_interface/bowit_viz.h"



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  printf("hello world bowit viz package\n");
  rclcpp::spin(std::make_shared<BowitViz>());
  rclcpp::shutdown();

  return 0;
}
