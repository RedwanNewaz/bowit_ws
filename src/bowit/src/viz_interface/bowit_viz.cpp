#include "bowit_viz.h"

BowitViz::BowitViz():Node("bowitViz")
{
    this->declare_parameter("width", 160);
    this->declare_parameter("height", 160);
    this->declare_parameter("resolution", 0.3125);

    width_ = this->get_parameter("width").get_parameter_value().get<int>();
    height_ = this->get_parameter("height").get_parameter_value().get<int>();
    resolution_ = this->get_parameter("resolution").get_parameter_value().get<double>();


    state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/state", 10, std::bind(
        &BowitViz::state_callback, this, std::placeholders::_1));
    pub_state_ = this->create_publisher<visualization_msgs::msg::Marker>("/bowit_robots", 10);
    pub_frontier_ = this->create_publisher<visualization_msgs::msg::Marker>("/bowit_frontiers", 10);
    pub_trajectory_ = this->create_publisher<visualization_msgs::msg::Marker>("/bowit_trajectories", 10);
    pclPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/bowit_local_sensing", 10);

    occPub_= this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    
    occGridMsg_.info.width = width_;
    occGridMsg_.info.height = height_;
    occGridMsg_.info.resolution = resolution_;
    occGridMsg_.info.origin.position.x = occGridMsg_.info.origin.position.y = occGridMsg_.info.origin.position.z = 0.0;
    occGridMsg_.info.origin.orientation.x = occGridMsg_.info.origin.orientation.y = occGridMsg_.info.origin.orientation.z = 0.0;
    occGridMsg_.info.origin.orientation.w = 1;  

    // unexplored cell represented by grey pixels 
    for (int i = 0; i < height_; i++)
        for (int j = 0; j < width_; j++)
            occGridMsg_.data.push_back(unexploredPixel);
    
    // configure motion model
    const int state_dim = 5; 
    const double dt = 0.20;
    const double predTime = 1.5;
    mmodel_ = std::make_shared<MotionModel>(state_dim, dt, predTime);
    collisionChecker_ = std::make_shared<CollisionChecker>(std::vector<double>{0.5,0.5,0.1});

   

    
}
bool BowitViz::updateGridMap(double x, double y)
{
    int row = y / resolution_;
    int column = x / resolution_;
    int index =  row * width_ + column;
    if(row < height_ && column < width_ && index < occGridMsg_.data.size())
    {
        occGridMsg_.data[index] = exploredPixel;
        return true; 
    }

    return false; 
}

std::vector<double> BowitViz::computeState(const geometry_msgs::msg::Pose& pose) const 
{
    double x = pose.position.x; 
    double y = pose.position.y; 
    tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w); 
    tf2::Matrix3x3 m(q); 
    double roll, pitch, yaw; 
    m.getRPY(roll, pitch, yaw); 
    return std::vector<double>{x, y, yaw};
}

std::vector<std::vector<double>> BowitViz::update_frontier(const std::vector<double>& self, std::vector<geometry_msgs::msg::Point>& points)
{     

    // identify frontier for visualization 
    double r = 1.5; 
    geometry_msgs::msg::Point p1, p2; 
    p1.x = r * cos(-0.785); 
    p1.y = r * sin(-0.785);
    p1.z = p2.z = 0.0; 
    points.emplace_back(p1);
    points.emplace_back(p1);
    p2.x = r * cos(0.785); 
    p2.y = r * sin(0.785);
    points.emplace_back(p2);
    points.emplace_back(p2); 
    
    // update occupancy grid 
    double theta = self[2] - 0.785;
    double dtheta = 0.0174533; // 1 deg increment 
    std::vector<std::vector<double>> cloud; 
    while (theta < self[2] + 0.785) 
    {
        // double r = std::min(1.0, std::max(1.0/ cos(theta), 1.0/ sin(theta)));
        for (double dr = 0; dr <= r; dr += 0.1)
        {
            double x = self[0] + dr * cos(theta); 
            double y = self[1] + dr * sin(theta);
            // check if x and y are valids 
            if(updateGridMap(x, y))
            {
                cloud.push_back({x, y, -0.001});
            }
        }

        theta += dtheta; 
    }

    return cloud;
}


bool BowitViz::check_comm_range(const std::string& curr_frame, double range)
{
    std::vector<double> self; 
    std::vector<std::vector<double>> others; 

    for(const auto& cand: otherRobots_)
    {
        auto pose = otherRobots_[cand.first];
        if(cand.first == curr_frame)
            self = computeState(pose);
        else 
            others.emplace_back(computeState(pose));
    }

    for(auto& other: others)
    {
        double dx = other[0] - self[0]; 
        double dy = other[1] - self[1];
        double dist = sqrt(dx * dx + dy * dy); 
        if(dist <= range)
            return true;  
    }

    return false; 
}


void BowitViz::state_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(vmu);
    // update unordered maps 
    otherRobots_[msg->child_frame_id] = msg->pose.pose;
    otherRobotsVel_[msg->child_frame_id] = msg->twist.twist;

    // viz data 
    visualization_msgs::msg::Marker vizMsg; 
    vizMsg.pose = msg->pose.pose;
    vizMsg.pose.position.z = 0.0; 
    vizMsg.header = occGridMsg_.header =  msg->header;  
    vizMsg.ns = msg->child_frame_id;  

    // update occupancy grid 
    auto robotState = computeState(vizMsg.pose);
    std::vector<geometry_msgs::msg::Point> points;
    auto cloudVec = update_frontier(robotState, points);
    cv::Vec3b rgbColor;
    scalarToHeatmapColor(msg->pose.pose.position.z, rgbColor);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    auto cp = msg->pose.pose.position;
    for(const auto& pc: cloudVec)
    {
        pcl::PointXYZRGB point;
        point.x = pc[0];
        point.y = pc[1];
        point.z = pc[2];
        point.r = rgbColor(0);
        point.g = rgbColor(1);
        point.b = rgbColor(2);
        cloud->points.push_back(point);
    }
    
    // Convert PCL point cloud to ROS PointCloud2
    sensor_msgs::msg::PointCloud2 rosCloud;
    pcl::toROSMsg(*cloud, rosCloud);
    rosCloud.header = msg->header; 
    pclPub_->publish(rosCloud);

    // check communication 
    bool comm = check_comm_range(msg->child_frame_id, 2.0);
    if(comm)
        vizMsg.color.r = vizMsg.color.b = 0.66; // arrow scale 0.2 roomba scale 1.0
    else 
        vizMsg.color.g = 0.66;
    vizMsg.color.a = 0.85;

    // viz occupancy grid 
    occPub_->publish(occGridMsg_);

    // viz frontier
    geometry_msgs::msg::Point def;
    def.x = def.y = def.z = 0.0; 
    vizMsg.points.push_back(def);
    std::copy(points.begin(), points.end(), std::back_inserter(vizMsg.points)); 
    vizMsg.points.push_back(def);

    vizMsg.type = visualization_msgs::msg::Marker::LINE_LIST; 
    vizMsg.scale.x = vizMsg.scale.y = vizMsg.scale.z = 0.1; // arrow scale 0.2 roomba scale 1.0 
    vizMsg.id = 101;
    pub_frontier_->publish(vizMsg);

    
    
    // viz robot pose
    vizMsg.type = visualization_msgs::msg::Marker::ARROW;
    vizMsg.scale.x = vizMsg.scale.y = vizMsg.scale.z = 0.345; // arrow scale 0.2 roomba scale 1.0 
    vizMsg.id = 100;
    pub_state_->publish(vizMsg);   

    compute_trajectory(msg); 


}

bool BowitViz::check_collision(const std::string& curr_frame)
{
    Eigen::VectorXd xx(5); 
    int n = otherRobots_.size() - 1; 
    Eigen::MatrixXd others(n, 5); 
    int rowCount = 0; 

    for(const auto& cand: otherRobots_)
    {
        auto pose = otherRobots_[cand.first];
        auto twist =  otherRobotsVel_[cand.first];
        auto state = computeState(pose);
    
        Eigen::VectorXd x(5);
        x << state[0], state[1], state[2], twist.linear.x, twist.angular.z;

        if(cand.first == curr_frame)
            xx = x;
        else 
            others.row(rowCount++) = x;
    }

    auto collision = collisionChecker_->checkTrajCollision(xx, others, mmodel_->getPtr());

    return collision; 
}

void BowitViz::compute_trajectory(nav_msgs::msg::Odometry::SharedPtr msg)
{
    Eigen::VectorXd xx(5);
    Eigen::Vector2d uu; 
    
    auto state = computeState(msg->pose.pose);
    uu << msg->twist.twist.linear.x, msg->twist.twist.angular.z;
    xx << state[0], state[1], state[2], uu(0), uu(1);
    auto traj = mmodel_->pred(xx, uu);

    


    visualization_msgs::msg::Marker vizMsg; 
    vizMsg.header = msg->header; 
    vizMsg.ns = msg->child_frame_id;  
    vizMsg.action = visualization_msgs::msg::Marker::ADD;
    vizMsg.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    vizMsg.scale.x = vizMsg.scale.y = vizMsg.scale.z = 0.25; // arrow scale 0.2 roomba scale 1.0 
    vizMsg.id = 102;
    if(check_collision(msg->child_frame_id))
        vizMsg.color.r = 1.0;
    else 
        vizMsg.color.g = 1.0;
    vizMsg.color.a = 1.0;
    vizMsg.pose.position.x = vizMsg.pose.position.y = vizMsg.pose.position.z = 0;
    vizMsg.pose.orientation.x = vizMsg.pose.orientation.y = vizMsg.pose.orientation.z = 0;
    vizMsg.pose.orientation.w = 1;

    for(int i = 0; i < traj.rows(); ++i)
    {
        auto x = traj.row(i);
        geometry_msgs::msg::Point p; 
        p.x = x(0); 
        p.y = x(1); 
        p.z = x(2); 
        vizMsg.points.emplace_back(p);
    }
    pub_trajectory_->publish(vizMsg);
}


void BowitViz::scalarToHeatmapColor(double scalar, cv::Vec3b& rgbColor) 
{
    // normalize scalar value 
    const double MAX_VAL = 25.9342;
    const double MIN_VAL = 25.764; 
    scalar -= MIN_VAL;
    scalar /= (MAX_VAL - MIN_VAL);
    // Define a colormap (you can choose any colormap from OpenCV)
    cv::Mat colormap;
    cv::applyColorMap(cv::Mat(1, 1, CV_8UC1, cv::Scalar(scalar * 255)), colormap, cv::COLORMAP_VIRIDIS);

    // Extract RGB color from the colormap
    rgbColor = colormap.at<cv::Vec3b>(0, 0);
}



