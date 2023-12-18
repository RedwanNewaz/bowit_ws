#include "bowit_robot.h"
#include "data_compression.h"

BowitRobot::BowitRobot(double px, double py, MotionModelPtr mmodel, std::shared_ptr<MetricMap> field, const std::vector<double>& controlRange)
:mmodel_(mmodel), field_(field)
{
    _x.resize(mmodel->stateDim());
    _x << px, py, -M_PI_2, 0.0, 0.0;
    std::copy(controlRange.begin(), controlRange.end(), std::back_inserter(controlRange_));
    // gp_ = std::make_shared<GGP_t>(1, 1);
    collisionChecker_ = std::make_shared<CollisionChecker>(std::vector<double>{0.5,0.5,0.1});
    Eigen::Vector2d u(0, 0);
    setNormalizedControl(u);
}

bool BowitRobot::detectCollision(const Eigen::VectorXd& xx) const 
{
    int N = otherRobots_.size(); 
    Eigen::MatrixXd others(N, mmodel_->stateDim());
    int rid = 0; 
    for(auto& other: otherRobots_)
        others.row(rid++) = other.second; 
    
    return collisionChecker_->checkPointCollision(xx, others);
}

std::string BowitRobot::toString() const
{
    std::stringstream ss;
    ss << _x(0) << "," << _x(1) << "," << _x(2) << "," << _meas;
    return ss.str();
}


Eigen::MatrixXd BowitRobot::getGroundTruth() const 
{
    const size_t N = field_->size().first;
    Eigen::MatrixXd gt(N, N);
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            double x, y, z;
            std::tie(x, y, z) = field_->operator()(i, j);
            gt(i, j) = z;
        }
    }
    return gt;
}

double BowitRobot::loss() const
{
    // Calculate MSE
    Eigen::MatrixXd gt_field = getGroundTruth();
    size_t n(gt_field.rows()), m(gt_field.cols());
    Eigen::MatrixXd pred(n, m);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            int cellIndex = i * m + j;;
            Eigen::VectorXd v = tools::make_vector(cellIndex);
            Eigen::VectorXd mu;
            double sigma;
            std::tie(mu, sigma) = gp_->query(v);
            pred(i, j) = mu(0);
        }
    }
    double mse = (gt_field - pred).array().square().sum() / (n * m);

    return mse;
}

size_t BowitRobot::trainSize() const
{
    return uniqueSamples_.size();
}

void BowitRobot::updateGP()
{
    // std::lock_guard<std::mutex> lk(mu);

    
    // std::vector<std::vector<double>> trainData; 
    // for(auto& sample: uniqueSamples_)
    // {
    //     auto samples = field_->getSub(sample.first);
    //     samples.push_back(sample.second);
    //     trainData.push_back(samples);
    // }

 
    // // gp_ = std::make_shared<GGP_t>(1, 1);

    
    // trainData = DataCompression::compress(trainData, 500);

    // std::vector<Eigen::VectorXd> samples(trainData.size()), observations(trainData.size());    
    // for(int i = 0; i <trainData.size(); ++i)
    // {
    //     int cellIndex = field_->getIndex(trainData[i][0], trainData[i][1]);
    //     samples[i] = tools::make_vector(cellIndex); 
    //     observations[i] = tools::make_vector(trainData[i][2]);
    // }

    // gp_->compute(samples, observations);
}

std::vector<std::string> BowitRobot::check_comm_range(double range) const 
{
    std::vector<std::string> res;
    for(const auto& cand: otherRobots_)
    {
        auto other = cand.second;
        double dx = other(0) - _x(0); 
        double dy = other(1) - _x(1);
        double dist = sqrt(dx * dx + dy * dy); 
        if(dist <= range)
        {
            res.emplace_back(cand.first);
        }  
    }
    return res; 
}

void BowitRobot::getObservedData(std::vector<std::vector<double>>& data) const
{
    std::lock_guard<std::mutex> lk(mu);
    for(auto & obs: uniqueSamples_)
    {
        auto d = field_->getSub(obs.first); 
        d.push_back(obs.second);
        data.emplace_back(d);
    }    
}

void BowitRobot::update_visited_map(double x, double y, double z)
{
    std::lock_guard<std::mutex> lk(mu);
    // update visited map 
    int cellIndex = field_->getIndex(x, y);
    uniqueSamples_[cellIndex] = z; 
    auto frontiers = compute_frontier(std::vector<double>{_x(0), _x(1), _x(2)});
    for(auto index :frontiers)
    {
        observed_data_[index] = z;
    }
}

bool BowitRobot::setNormalizedControl(const Eigen::Vector2d& act)
{
    // std::lock_guard<std::mutex> lk(mu);
    auto u = tfControl(act);

    auto xx = mmodel_->get(_x, u, mmodel_->dt());
    if(!field_->isValidCoord(xx(0), xx(1)) || detectCollision(xx))
    {
        u(0) = 0.0;
        u(1) = 2.0 * abs(u(1));
    }

    _x = mmodel_->get(_x, u, mmodel_->dt());
    _meas = field_->getObsAlongTraj(std::vector<Eigen::VectorXd>{_x})[0];
    update_visited_map(_x(0), _x(1), _meas);
    updateGP();


    return true; 
}

void BowitRobot::updateOtherRobots(const std::string& robotName, const Eigen::VectorXd& x)
{
    std::lock_guard<std::mutex> lk(mu);
    otherRobots_[robotName] = x;
}


Eigen::VectorXd  BowitRobot::operator()(const Eigen::VectorXd& act) const
{
    Eigen::Vector2d res;
    Eigen::Vector2d u = tfControl(act);
    
    //satisfibility check  0: infeasible 1: feasible
    res(1) = 1;
    auto xx = mmodel_->get(_x, u, mmodel_->dt());
    bool inside = field_->isValidCoord(xx(0), xx(1));
    
    int N = otherRobots_.size();
    if(N > 0) 
    {
        Eigen::MatrixXd others(N, mmodel_->stateDim());
        int rid = 0; 
        for(auto& other: otherRobots_)
            others.row(rid++) = other.second; 
        
        auto collision = collisionChecker_->checkTrajCollision(xx, others, mmodel_->getPtr());    
        res(1) = inside && !collision; 
    }

    auto traj = mmodel_->pred(_x, u); 
    res(0) = evaluateTrajectoryV2(traj);    
    return res; 
    
}

std::vector<double> BowitRobot::getState() const 
{
    return std::vector<double>{_x(0), _x(1), _meas, _x(2), _x(3), _x(4)};
}

Eigen::Vector2d BowitRobot::tfControl(const Eigen::VectorXd& u) const
{
    Eigen::Vector2d uu;
    uu(0) =  controlRange_[0] + (controlRange_[1] - controlRange_[0]) * u(0); // max_speed = 0.345;
    uu(1) =  controlRange_[2] + (controlRange_[3] - controlRange_[2]) * u(1);
    return uu;
}

std::vector<int> BowitRobot::compute_frontier(const std::vector<double>& self) const 
{
    double theta = self[2] - FOV_ / 2.0;
    std::vector<int> results; 
    while (theta < self[2] + FOV_ / 2.0) 
    {
        double r = detection_range_; 
        for (double dr = 0; dr <= r; dr += 0.1)
        {
            double x = self[0] + dr * cos(theta); 
            double y = self[1] + dr * sin(theta);
            if(field_->isValidCoord(x, y))
            {
                int cellIndex = field_->getIndex(x, y);
                results.push_back(cellIndex);
            }
        }

        theta += one_deg; 
    }

    return results;
}

double BowitRobot::evaluateTrajectoryV2(const Eigen::MatrixXd& traj) const
{
    int unseen = 0; 
    for(int i = 0; i < traj.rows(); ++i)
    {
        auto tx = traj.row(i); 
        if(!field_->isValidCoord(tx(0), tx(1)))
            continue;
        
        int cellIndex = field_->getIndex(tx(0), tx(1));

        if(observed_data_.find(cellIndex) == observed_data_.end())
        {
            unseen += 1;
        }

    }
    return unseen;

}

double BowitRobot::evaluateTrajectory(const Eigen::MatrixXd& traj) const
{
    double reward = 0.0;
    double gamma = 0.85;
    int t = 0;
    double beta = std::sqrt(2.0 * std::log( M_PI * M_PI / 0.1));
    int seen = 1;
 
    for(int i = 0; i < traj.rows(); ++i)
    {
        auto tx = traj.row(i); 
        int cellIndex = field_->getIndex(tx(0), tx(1));
        if(observed_data_.find(cellIndex) != observed_data_.end())
        {
            seen += 1;
        }
        auto gmu = gp_->mean_vector();

        Eigen::VectorXd v = tools::make_vector(cellIndex);
        Eigen::VectorXd tmu;
        double sigma;
        std::tie(tmu, sigma) = gp_->query(v);

        reward += pow(gamma, t++) *  (tmu(0) +  beta * sigma);
        // reward += pow(gamma, t++) *  sigma;
    }
    return reward / seen;

}