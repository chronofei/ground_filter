#ifndef GROUND_FILTER_MODELS_CSF_CSF_H_
#define GROUND_FILTER_MODELS_CSF_CSF_H_

// C/C++
#include <vector>
#include <string>
#include <fstream>

// Eigen
#include <Eigen/Core>

// project
#include "ground_filter/models/ground_filter_base.hpp"
#include "ground_filter/models/csf/Cloth.hpp"

#include "ground_filter/common/common.hpp"
#include "ground_filter/tools/tools.hpp"

namespace ground_filter
{

class CSF : public GroundFilterBase
{
public:
    CSF();
    ~CSF();

    bool Filter(CompoundData& data);

private:
    bool    sloop_smooth_;
    bool    export_cloth_;
    int     iterations_;

    double  gravity_;
    double  time_step_;
    double  cloth_resolution_;
    double  smooth_threshold_;
    double  height_threshold_;
    int     rigidness_;
    double  class_threshold_;  

};  // end class CSF

}  // end namespace ground_filter

#endif