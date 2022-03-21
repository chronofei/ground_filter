#ifndef GROUND_FILTER_MODELS_CSF_PARTICLE_HPP_
#define GROUND_FILTER_MODELS_CSF_PARTICLE_HPP_

// C/C++
#include <vector>

// Eigen
#include <Eigen/Core>

// project
#include "ground_filter/common/common.hpp"

namespace ground_filter
{

#define DAMPING    0.01

const double singleMove1[15] = { 0, 0.3, 0.51, 0.657, 0.7599, 0.83193, 0.88235, 0.91765, 0.94235, 0.95965, 0.97175, 0.98023, 0.98616, 0.99031, 0.99322 };
const double doubleMove1[15] = { 0, 0.3, 0.42, 0.468, 0.4872, 0.4949, 0.498, 0.4992, 0.4997, 0.4999, 0.4999, 0.5, 0.5, 0.5, 0.5 };

class Particle
{
public:
    Particle(const Eigen::Vector3d& position, const Eigen::Vector3d& acceleration, const double time_step);

    bool IsMovable();
    void MakeUnmovable();

    Eigen::Vector3d& GetPos();
    void OffsetPos(const Eigen::Vector3d v);

    void TimeStep();
    void SatisfyConstraintSelf(int rigidness);

public:
    Eigen::Vector3d pos_;
    Eigen::Vector3d old_pos_;
    bool            is_visited_;
    int             pos_x_;
    int             pos_y_;
    int             c_pos_;

    std::vector<Particle*> neighbors_;

    std::vector<Point> corresponding_lidar_points_;
    double             nearest_point_height_;
    double             tmp_dist_;

private:
    bool            movable_;
    Eigen::Vector3d acceleration_;
    double          time_step2_;

}; // end class Particle

} // end namespace ground_filter


#endif // ifndef _PARTICLE_H_
