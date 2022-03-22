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

const double singleMove1[15] = { 0, 0.3, 0.51, 0.657, 0.7599, 0.83193, 0.88235, 0.91765, 0.94235, 0.95965, 0.97175, 0.98023, 0.98616, 0.99031, 0.99322 };
const double doubleMove1[15] = { 0, 0.3, 0.42, 0.468, 0.4872, 0.4949, 0.498, 0.4992, 0.4997, 0.4999, 0.4999, 0.5, 0.5, 0.5, 0.5 };

class Cloth
{
public:
    Cloth(const Eigen::Vector2d cloth_size, const double& cloth_resolution,
          const double& acceleration,       const double& time_step,
          const double& smooth_threshold,   const double& height_threshold,
          const int& rigidness,             const double& class_threshold);

    void Clear();

    void RasterizePointCloud(const PointCloud& cloud);

    // double TimeStep();
    // void TerrCollision();

    // void MovableFilter();

    void GetCloth(PointCloud& cloth);
    void GetFilterResult(PointCloud& ground, PointCloud& nonground);

private:
    int GetIndex1D(int indexX, int indexY);
    void MakeConstraint(int p1, int p2);

    // double FindHeightValByScanline(Particle *p);
    // double FindHeightValByNeighbor(Particle *p);
    // void   RasterTerrian(const PointCloud& cloud, std::vector<double>& heightVal);

    // std::vector<int> FindUnmovablePoint(std::vector<XY> connected);
    // void HandleSlopConnected(std::vector<int> edgePoints,
    //                          std::vector<XY> connected,
    //                          std::vector<std::vector<int>> neibors);

    // void SatisfyConstraintSelf(int p, int rigidness);

private:
    Eigen::Vector2d cloth_origin_;
    int             cloth_width_;
    int             cloth_height_;
    double          cloth_resolution_;

    std::vector<Particle> particles_;

    std::vector<double> heightvals_;

    double smooth_threshold_;
    double height_threshold_;
    int    rigidness_;
    double class_threshold_;

}; // end class Cloth

}  // end namespace ground_filter

#endif
