#pragma once 

#include <cassert>
#include <fcl/config.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree_array.h>
#include <fcl/broadphase/default_broadphase_callbacks.h>
#include <fcl/geometry/geometric_shape_to_BVH_model.h>
#include <fcl/broadphase/broadphase_SSaP.h>
#include "motion_model.h"
#include <mutex>

static std::mutex mu;

class CollisionChecker
{
public:
    CollisionChecker(const std::vector<double>& geom)
    {
        assert(geom.size() == 3); 
        geom_ = std::make_shared<fcl::Boxd>(geom[0], geom[1], geom[2]);
    } 

    /// @brief given current robot state and a set of other robots state, check collision 
    /// @param xx : current robot state (self m vector)
    /// @param others other robot states (nxm matrix)
    /// @return true if collision else false 
    bool checkPointCollision(const Eigen::VectorXd& xx, const Eigen::MatrixXd& others) const 
    {
        std::lock_guard<std::mutex> lk(mu);
        std::vector<fcl::CollisionObjectd*> env;
        for(int i = 0; i < others.rows(); ++i)
        {
            env.emplace_back(getCollisionObj(others.row(i)));
        }
     
        auto self = getCollisionObj(xx);
        fcl::DynamicAABBTreeCollisionManagerd manager;
        manager.registerObjects(env);
        manager.setup();

        fcl::DefaultCollisionData<double> cdata;
        cdata.request.num_max_contacts = 40.0;
        manager.collide(self, &cdata, fcl::DefaultCollisionFunction);
        return cdata.result.isCollision();
    }

    bool checkTrajCollision(const Eigen::VectorXd& xx, const Eigen::MatrixXd& others, MotionModelPtr model) const 
    {
        Eigen::Vector2d uu(xx(3), xx(4));  

        double robot_size = 0.5;
        int sample_size = xx(3) * model->predTime() / robot_size;
        sample_size = std::max(1, sample_size); 
        
        auto samples = model->sample(xx, uu, sample_size);
        for(int i = 0; i < samples.rows(); ++i)
        {
            if(checkPointCollision(samples.row(i), others)) 
                return true; 
        }


        return false;
    }
    
    /// @brief check collision with robot trajectories
    /// @param xx : current robot state (x, y, theta, u, v)
    /// @param others other robot states (nxm matrix)
    /// @return true if collision else false 
    bool checkTrajCollisionV2(const Eigen::VectorXd& xx, const Eigen::MatrixXd& others, MotionModelPtr model) const 
    {
        std::lock_guard<std::mutex> lk(mu);
        Eigen::Vector2d uu(xx(3), xx(4));  
        auto gx = model->get(xx, uu, model->predTime());

        auto getGoalTf = [](const Eigen::VectorXd& xx)
        {
            fcl::Transform3d tf_goal;
            tf_goal.rotate(Eigen::AngleAxisd(xx(2), Eigen::Vector3d::UnitZ()).matrix());
            tf_goal.translate(Eigen::Vector3d(xx(0), xx(1), 0.000000));
            return tf_goal;
        };

        auto o1 = getCollisionObj(xx);
        auto g1 = getGoalTf(gx);
      
        for(int i = 0; i < others.rows(); ++i)
        {
            auto x = others.row(i);
            Eigen::Vector2d u(x(3), x(4));  
            auto g = model->get(x, u, model->predTime());
            auto o2 = getCollisionObj(x);
            auto g2 = getGoalTf(g);

            fcl::ContinuousCollisionRequestd request;
            // result will be returned via the continuous collision result structure
            fcl::ContinuousCollisionResultd result;
            // perform continuous collision test
            fcl::continuousCollide(o1, g1, o2, g2, request, result);
            if(result.is_collide)
                return true; 
        }

        return false;
    }

private:
    std::shared_ptr<fcl::Boxd> geom_;
    

protected:
    fcl::ContinuousCollisionObjectd * getCCO(const Eigen::VectorXd& xx, MotionModelPtr model) const
    {
        Eigen::Vector2d uu(xx(3), xx(4));
        const size_t Knotsize = 4;
        auto sample_traj = model->sample(xx, uu, Knotsize);
        std::vector<fcl::Vector3d> t(4), r(4);

        t[0] = fcl::Vector3d(sample_traj(0, 0), sample_traj(0, 1), 0.0);
        t[1] = fcl::Vector3d(sample_traj(1, 0), sample_traj(1, 1), 0.0);
        t[2] = fcl::Vector3d(sample_traj(2, 0), sample_traj(2, 1), 0.0);
        t[3] = fcl::Vector3d(sample_traj(3, 0), sample_traj(3, 1), 0.0);

        r[0] = fcl::Vector3d(0, 0, sample_traj(0, 2));
        r[1] = fcl::Vector3d(0, 0, sample_traj(1, 2));
        r[2] = fcl::Vector3d(0, 0, sample_traj(2, 2));
        r[3] = fcl::Vector3d(0, 0, sample_traj(3, 2));

        auto motion_a = fcl::make_aligned_shared<fcl::SplineMotion<double>>(
                t[0], t[1], t[2], t[3],
                        r[0], r[1], r[2], r[3]);
        return new fcl::ContinuousCollisionObjectd(geom_,motion_a);
    }

    fcl::CollisionObjectd* getCollisionObj(const Eigen::VectorXd& x) const 
    {
        auto obj_collision = new fcl::CollisionObjectd(
                geom_, Eigen::AngleAxisd(x(2), Eigen::Vector3d::UnitZ()).matrix(),
                Eigen::Vector3d(x(0), x(1), 0.000000));
        return obj_collision;
    }

};