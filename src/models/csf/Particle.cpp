#include "ground_filter/models/csf/Particle.hpp"

namespace ground_filter
{

Particle::Particle(const double& acceleration, const double time_step)
{
    shift_by_acc_         = acceleration * time_step * time_step;

    Clear();
}

void Particle::TimeStep()
{
    if (movable_)
    {
        double temp  = curr_height_;
        curr_height_ = curr_height_ + (curr_height_ - last_height_) * 0.99 + shift_by_acc_;
        last_height_ = temp;
    }
}

void Particle::Clear()
{
    movable_              = true;
    curr_height_          = std::numeric_limits<double>::min();
    last_height_          = std::numeric_limits<double>::min();
    corresponding_lidar_points_.clear();
    nearest_point_height_ = std::numeric_limits<double>::min();

    tmp_dist_             = std::numeric_limits<double>::max();
}

} // end namespace ground_filter