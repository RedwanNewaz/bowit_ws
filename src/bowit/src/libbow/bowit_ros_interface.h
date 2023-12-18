#ifndef BOWIT_ROS_INTERFACE_HPP
#define BOWIT_ROS_INTERFACE_HPP



#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <geometry_msgs/msg/pose_array.hpp>
#include "bowit_robot.h"

using namespace std::chrono_literals;

class BowitROSInteface: public rclcpp::Node
{
  using Stop_t = boost::fusion::vector<stop::MaxIterations<Params>>;
  using Stat_t = boost::fusion::vector<stat::Samples<Params>,
                 stat::BestObservations<Params>,
                 stat::AggregatedObservations<Params>>;
  using Mean_t = mean::Constant<Params>;
  using Kernel_t = kernel::Exp<Params>;
  using GP_t = model::GP<Params, Kernel_t, Mean_t>;
  using Constrained_GP_t = model::GP<Params, Kernel_t, Mean_t>;

  using Acqui_t = experimental::acqui::ECI<Params, GP_t, Constrained_GP_t>;
  using Init_t = init::RandomSampling<Params>;
  using TIME_POINT = std::chrono::time_point<std::chrono::high_resolution_clock>; 


public:
  BowitROSInteface();

  int getRobotID() const ;

protected:

  void comm_callback(geometry_msgs::msg::PoseArray::SharedPtr msg);

  void communication();

  void timer_callback();

  void mse_timer_callback();

  Eigen::VectorXd toState(const nav_msgs::msg::Odometry& odom);

  void publish_state();

  void state_callback(nav_msgs::msg::Odometry::SharedPtr msg);


private:
  int robotID_;
  TIME_POINT start_time_;
  
  experimental::bayes_opt::CBOptimizer<Params,
        modelfun<GP_t>,
        acquifun<Acqui_t>,
        statsfun<Stat_t>,
        initfun<Init_t>,
        stopcrit<Stop_t>,
        experimental::constraint_modelfun<Constrained_GP_t>>
                opt_;
  std::shared_ptr<BowitRobot> robot_; 
  rclcpp::TimerBase::SharedPtr timer_, mse_timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_sensor_data_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_sensor_data_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr msePub_;

  std::unordered_map<std::string, TIME_POINT> comm_tracker_;

  
};

#include "bowit_ros_interface.cpp"

#endif // BOWIT_ROS_INTERFACE_HPP