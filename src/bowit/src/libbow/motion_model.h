#pragma once 
#include <memory>
#include <Eigen/Dense>

class MotionModel;
typedef std::shared_ptr<MotionModel> MotionModelPtr;


class MotionModel: public std::enable_shared_from_this<MotionModel>
{
public:
    MotionModel(size_t state_dim, double dt, double predTime = 3.0): state_dim_(state_dim), dt_(dt), predTime_(predTime)
    {

    }

    MotionModelPtr getPtr() 
    {
        return shared_from_this(); 
    }

    double dt() const 
    {
        return dt_; 
    }
    size_t stateDim() const 
    {
        return state_dim_; 
    }

    double predTime() const 
    {
        return predTime_;
    }

    Eigen::VectorXd get(const Eigen::VectorXd& xx, const Eigen::Vector2d& u, double dt) const
    {
        Eigen::VectorXd nx(state_dim_);

        nx(2) = xx(2) + u(1) * dt;
        nx(2) = fmod(nx(2) + M_PI, 2 * M_PI) - M_PI;

        nx(0) = xx(0) + u(0) * std::cos(nx(2)) * dt;
        nx(1) = xx(1) + u(0) * std::sin(nx(2)) * dt;
        nx(3) = u(0);
        nx(4) = u(1);
        return nx;
    }

    Eigen::MatrixXd pred(const Eigen::VectorXd& xx, const Eigen::Vector2d& uu) const 
    {
        size_t N = predTime_ / dt_ + 1;
        Eigen::MatrixXd traj (N, state_dim_);
        for(size_t t = 0; t < N; ++t)
        {
            traj.row(t) = get(xx, uu, t * dt_);
        }
        return traj;
    }

    Eigen::MatrixXd sample(const Eigen::VectorXd& xx, const Eigen::Vector2d& uu, size_t size) const 
    {
        Eigen::MatrixXd traj (size, state_dim_);
        
        double time = 0; 
        double delta = predTime() / size; 
        int index = 0; 
        while(time < predTime())
        {
            auto next = get(xx, uu, time);
            traj.row(index++) = next;
            time += delta; 
        }
        return traj;
    }

private:
    size_t state_dim_;
    double dt_;
    double predTime_;  
};