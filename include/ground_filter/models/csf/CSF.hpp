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
    int     iterations_;

    std::shared_ptr<Cloth> cloth_;

    int    max_particle_for_postprocess_;

};  // end class CSF

}  // end namespace ground_filter

#endif