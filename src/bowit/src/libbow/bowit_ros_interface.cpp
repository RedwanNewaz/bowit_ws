#include "bowit_ros_interface.h"

BowitROSInteface::BowitROSInteface():Node("bowit")
{
    this->declare_parameter("coord", "/home/airlab/CLionProjects/goal_free_planner/results/test_fiu_temp/inputs/exp2/coords.csv");
    this->declare_parameter("data", "/home/airlab/CLionProjects/goal_free_planner/results/test_fiu_temp/inputs/exp2/temp.csv");
    this->declare_parameter("dt", 0.20);
    this->declare_parameter("robotID", 1);
    this->declare_parameter("predTime", 3.0);
    this->declare_parameter("xi", 33.65856);
    this->declare_parameter("yi", 31.992462);

    // parameters 
    std::string inputCoordFile = this->get_parameter("coord").get_parameter_value().get<std::string>();
    std::string inputDataFile = this->get_parameter("data").get_parameter_value().get<std::string>();

    robotID_ = this->get_parameter("robotID").get_parameter_value().get<int>();
    double dt = this->get_parameter("dt").get_parameter_value().get<double>();
    double predTime = this->get_parameter("predTime").get_parameter_value().get<double>();
    double xi = this->get_parameter("xi").get_parameter_value().get<double>();
    double yi = this->get_parameter("yi").get_parameter_value().get<double>();


    
    // initialize robot 
    auto fieldMap = std::make_shared<MetricMap>(inputCoordFile, inputDataFile);
    const int state_dim = 5;
    auto mmodel = std::make_shared<MotionModel>(state_dim, dt, predTime);
    std::vector<double>controlRange{0,2,-M_PI, M_PI};
    robot_ = std::make_shared<BowitRobot>(xi, yi, mmodel, fieldMap, controlRange);

    // run loop
    int duration_ms = 1000 * dt; 
    timer_ = this->create_wall_timer(std::chrono::milliseconds(duration_ms), 
        std::bind(&BowitROSInteface::timer_callback, this));

    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/state", 10);

    state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/state", 10, std::bind(
            &BowitROSInteface::state_callback, this, std::placeholders::_1)
    );

    pub_sensor_data_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/comm_data", 10);
    sub_sensor_data_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/comm_data", 10, 
        std::bind(&BowitROSInteface::comm_callback, this, std::placeholders::_1)
    );
    start_time_ = std::chrono::high_resolution_clock::now();
    RCLCPP_INFO(get_logger(), " initiated for robot %d", robotID_);
}

int BowitROSInteface::getRobotID() const 
{
    return robotID_;
}

void BowitROSInteface::comm_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    auto frame_id = "robot" + std::to_string(robotID_);
    if(frame_id != msg->header.frame_id)
        return;

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_);

    // communicate every 5 seconds 
    if (duration.count() < 5000)
        return;

    RCLCPP_INFO_STREAM(this->get_logger(),  msg->header.frame_id <<" Elapsed time: " << duration.count() << " data size " << msg->poses.size());
    for(const auto& pose:msg->poses)
    {
        auto p = pose.position;
        robot_->update_visited_map(p.x, p.y, p.z);
    }
    robot_->updateGP();

    start_time_ = std::chrono::high_resolution_clock::now();
}


void BowitROSInteface::communication()
{
    auto neighbors = robot_->check_comm_range(2.0);
    if(neighbors.empty())
        return;

    for(const auto& ner : neighbors)
    {

        if(comm_tracker_.find(ner) != comm_tracker_.end())
        {
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - comm_tracker_[ner]);
            if (duration.count() < 5000)
                continue;
        }

        geometry_msgs::msg::PoseArray msg; 

        msg.header.frame_id = ner; 
        msg.header.stamp = this->get_clock()->now(); 
        std::vector<std::vector<double>> data; 

        robot_->getObservedData(data);

        auto ndata = DataCompression::compress(data, 500);
        RCLCPP_INFO_STREAM(get_logger(), "train data size " << robot_->trainSize() << " data size " << data.size() << " compress " << ndata.size() );
        
        for(auto& item:ndata)
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = item[0];
            pose.position.y = item[1];
            pose.position.z = item[2];
            msg.poses.push_back(pose);
        }
        pub_sensor_data_->publish(msg); 

        comm_tracker_[ner] = std::chrono::high_resolution_clock::now();
        
    }
}

void BowitROSInteface::timer_callback()
{
    opt_.optimize(*robot_);
    auto u = opt_.best_sample();
    communication();   
    robot_->setNormalizedControl(u);
    publish_state(); 
}

Eigen::VectorXd BowitROSInteface::toState(const nav_msgs::msg::Odometry& odom)
{
    Eigen::VectorXd xx(5); 

    xx(0) = odom.pose.pose.position.x;
    xx(1) = odom.pose.pose.position.y;

    tf2::Quaternion q; 
    q.setX(odom.pose.pose.orientation.x);
    q.setY(odom.pose.pose.orientation.y);
    q.setZ(odom.pose.pose.orientation.z);
    q.setW(odom.pose.pose.orientation.w);

    tf2::Matrix3x3 m(q); 
    double roll, pitch, yaw; 
    m.getRPY(roll, pitch, yaw); 
    xx(2) = yaw;

    xx(3) = odom.twist.twist.linear.x; 
    xx(4) = odom.twist.twist.angular.z;

    return xx; 
}

void BowitROSInteface::publish_state()  
{
    auto state = robot_->getState(); 
    nav_msgs::msg::Odometry odom; 

    odom.header.frame_id = "map"; 
    odom.child_frame_id = "robot" + std::to_string(robotID_);
    odom.header.stamp = get_clock()->now();

    odom.pose.pose.position.x = state[0];
    odom.pose.pose.position.y = state[1];
    odom.pose.pose.position.z = state[2];

    tf2::Quaternion q; 
    q.setRPY(0, 0, state[3]);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = state[4]; 
    odom.twist.twist.angular.z = state[5];
    publisher_->publish(odom);

    // RCLCPP_INFO(get_logger(), " publish state for robot %d: (%lf, %lf)", robotID_, state[0], state[1]);
}

void BowitROSInteface::state_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::string selfID = "robot" + std::to_string(robotID_);
    if(msg->child_frame_id == selfID)
        return;

    robot_->updateOtherRobots(msg->child_frame_id, toState(*msg.get()));

    // if(robot_->detectCollision())
    //   RCLCPP_ERROR_STREAM(get_logger(), " Collision detected with  " << msg->child_frame_id);

}