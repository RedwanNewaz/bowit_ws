#pragma once 
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <unordered_map> 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <Eigen/Dense>
#include "../libbow/motion_model.h"
#include "../libbow/collision_checker.h"
#include <mutex> 


static std::mutex vmu; 

class BowitViz: public rclcpp::Node
{
public:
    BowitViz();

private:
    int width_, height_; 
    double resolution_; 
    const int unexploredPixel = 50;
    const int exploredPixel = 0;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_state_, pub_frontier_, pub_trajectory_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occPub_;
    nav_msgs::msg::OccupancyGrid occGridMsg_;
    std::unordered_map<std::string, geometry_msgs::msg::Pose> otherRobots_;
    std::unordered_map<std::string, geometry_msgs::msg::Twist> otherRobotsVel_;
    MotionModelPtr mmodel_;
    std::shared_ptr<CollisionChecker> collisionChecker_;
protected:
    /// @brief Given a coordinate, update the occupancy grid 
    /// @param x x coord
    /// @param y y coord 
    void updateGridMap(double x, double y);
    /// @brief compute pose to state vector (x, y, theta)
    /// @param pose 
    /// @return state vector (x, y, theta)
    std::vector<double> computeState(const geometry_msgs::msg::Pose& pose) const ;

    /// @brief  given current state of the robot update occupancy grid 
    /// @param self robot state vector (x, y, theta)
    /// @param points (rviz points)
    void update_frontier(const std::vector<double>& self, std::vector<geometry_msgs::msg::Point>& points);

    /// @brief for a given distance range,  find a neighbor
    /// @param curr_frame 
    /// @param range 
    /// @return true if neighbor found otherwise false 
    bool check_comm_range(const std::string& curr_frame, double range);


    void state_callback(nav_msgs::msg::Odometry::SharedPtr msg);

    bool check_collision(const std::string& curr_frame);

    void compute_trajectory(nav_msgs::msg::Odometry::SharedPtr msg);
    
};

