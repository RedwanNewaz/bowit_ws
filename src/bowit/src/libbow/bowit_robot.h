#pragma once 

#include "bowit_params.h"
#include "hypermap.h"
#include "collision_checker.h"
#include <random>
#include <utility>

extern std::mutex mu; 

class BowitRobot{
    using GKernel_t = kernel::MaternFiveHalves<GPParams>;
    using GMean_t = mean::Data<GPParams>;
    using GGP_t = model::GP<GPParams, GKernel_t, GMean_t>;
public:
  BO_PARAM(size_t, dim_in, 2);
  BO_PARAM(size_t, dim_out, 1);
  BO_PARAM(size_t, nb_constraints, 1);

  explicit BowitRobot(double px, double py, MotionModelPtr mmodel, std::shared_ptr<MetricMap> field, const std::vector<double>& controlRange);

  Eigen::VectorXd operator()(const Eigen::VectorXd& u) const;
  std::vector<double> getState() const ; 
  void updateOtherRobots(const std::string& robotName, const Eigen::VectorXd& x); 
  bool setNormalizedControl(const Eigen::Vector2d& u); 
  std::vector<std::string> check_comm_range(double range) const;
  void getObservedData(std::vector<std::vector<double>>& data) const;
  void update_visited_map(double x, double y, double z); 
  std::string toString() const; 
  void updateGP();
  double loss() const; 
  size_t trainSize() const;

private:
    std::unordered_map<int, double> observed_data_;
    std::shared_ptr<MetricMap> field_;
    std::shared_ptr<GGP_t> gp_;
    std::shared_ptr<CollisionChecker> collisionChecker_;
    MotionModelPtr mmodel_;
    /// @brief [vmin, vmax, ymin, ymax]
    std::vector<double> controlRange_;
    /// @brief field of view in radian 
    const double FOV_ = M_PI_2;   
    /// @brief frontier detection range in meter 
    const double detection_range_ = 1.5;
    const double one_deg =  0.0174533;
    // keep track of other robot states for collision checking 
    std::unordered_map<std::string, Eigen::VectorXd> otherRobots_;
    std::unordered_map<int, double> uniqueSamples_;  

protected:
  Eigen::VectorXd _x; //x, y , theta, v, w
  double _meas; // measurement 
  Eigen::Vector2d tfControl(const Eigen::VectorXd& act)const;
  std::vector<int> compute_frontier(const std::vector<double>& self) const; 
  double evaluateTrajectory(const Eigen::MatrixXd& traj) const;
  double evaluateTrajectoryV2(const Eigen::MatrixXd& traj) const;
  bool detectCollision(const Eigen::VectorXd& xx) const; 
  Eigen::MatrixXd getGroundTruth() const; 
  

};

#include "bowit_robot.cpp"

