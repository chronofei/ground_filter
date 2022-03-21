#ifndef GROUND_FILTER_MODELS_CSF_CLOTH_HPP_
#define GROUND_FILTER_MODELS_CSF_CLOTH_HPP_

// C/C++
#include <math.h>
#include <vector>
#include <iostream>
#include <omp.h>
#include <sstream>
#include <list>
#include <cmath>
#include <string>
#include <queue>
#include <fstream>

// Eigen
#include <Eigen/Core>

// project
#include "ground_filter/models/csf/Particle.hpp"
#include "ground_filter/common/common.hpp"
#include "ground_filter/tools/tools.hpp"

namespace ground_filter
{

#define MAX_PARTICLE_FOR_POSTPROCESSIN    50

struct XY
{
    XY(int x1, int y1) 
    {
        x = x1; y = y1;
    }

    int x;
    int y;
};

class Cloth
{
public:
    Cloth(const PointCloud& cloud, const Eigen::Vector3d& acceleration, const double& time_step,
          const double& cloth_resolution, const double& smooth_threshold, const double& height_threshold, 
          const int& rigidness, const double& class_treshold);

    double TimeStep();
    void TerrCollision();

    void MovableFilter();

    void GetCloth(PointCloud& cloth);
    void GetFilterResult(PointCloud& ground, PointCloud& nonground);

private:
    void ComputeBoundingBox(const PointCloud& cloud, Eigen::Vector3d& min_boundary, Eigen::Vector3d& max_boundary);
    Particle* GetParticle(int x, int y);
    Particle* GetParticle(int index);
    void MakeConstraint(Particle *p1, Particle *p2);

    double FindHeightValByScanline(Particle *p);
    double FindHeightValByNeighbor(Particle *p);
    void   RasterTerrian(const PointCloud& cloud, std::vector<double>& heightVal);

    std::vector<int> FindUnmovablePoint(std::vector<XY> connected);
    void HandleSlopConnected(std::vector<int> edgePoints,
                             std::vector<XY> connected,
                             std::vector<std::vector<int>> neibors);

private:
    PointCloud cloud_;

    Eigen::Vector3d origin_;

    double cloth_resolution_;
    double smooth_threshold_;
    double height_threshold_;
    int    rigidness_;
    double class_threshold_;

    int num_particles_width_;
    int num_particles_height_;
    std::vector<Particle> particles_;
    std::vector<double> heightvals_;

}; // end class Cloth

}  // end namespace ground_filter

#endif
