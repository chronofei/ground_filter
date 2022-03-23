#include "ground_filter/models/csf/Particle.hpp"

namespace ground_filter
{

Particle::Particle()
{
    Clear();
}

void Particle::Clear()
{
    movable_              = true;
    curr_height_          = HEIGHT_THRESHOLD;
    last_height_          = HEIGHT_THRESHOLD;
    lidar_points_.clear();
    nearest_point_height_ = HEIGHT_THRESHOLD;

    is_visited_           = false;
    c_pos_                = -1;

    tmp_dist_             = std::numeric_limits<double>::max();
}

} // end namespace ground_filter