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

#define HEIGHT_THRESHOLD -10

class Particle
{
public:
    Particle();

    Eigen::Vector2d GetPosition(){return position_;}
    void SetPosition(const Eigen::Vector2d& position){position_=position;}

    Eigen::Vector3d GetPosition3D(){return Eigen::Vector3d(position_[0], position_[1], curr_height_);}

    Eigen::Vector2i GetRasterIndex(){return raster_index_;}
    void SetRasterIndex(const Eigen::Vector2i& raster_index){raster_index_=raster_index;}

    int GetNeighbor(int index){return neighbors_[index];}
    void AddNeighbor(int neighbor){neighbors_.emplace_back(neighbor);}
    size_t GetNeighborSize(){return neighbors_.size();}

    void Clear();

public:
    bool               movable_;
    double             curr_height_;
    double             last_height_;
    double             nearest_point_height_;

    std::vector<Point> lidar_points_;
    bool               is_visited_;
    double             tmp_dist_;
    int                c_pos_;

private:
    double             shift_by_acc_;
    Eigen::Vector2d    position_;
    Eigen::Vector2i    raster_index_;
    std::vector<int>   neighbors_;

}; // end class Particle

} // end namespace ground_filter


#endif // ifndef _PARTICLE_H_
