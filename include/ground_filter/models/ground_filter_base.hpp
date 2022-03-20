#ifndef GROUND_FILTER_MODELS_GROUND_FILTER_BASE_HPP_
#define GROUND_FILTER_MODELS_GROUND_FILTER_BASE_HPP_

// C/C++
#include <fstream>
#include <mutex>
#include <thread>
#include <vector>

// project
#include "ground_filter/sensor_data/sensor_data.hpp"
#include "ground_filter/common/common.hpp"
#include "ground_filter/tools/tools.hpp"

namespace ground_filter 
{

class GroundFilterBase
{
public:
  GroundFilterBase();
  ~GroundFilterBase();

  virtual bool Filte(CompoundData& data) =0;

}; // end class GroundFilterBase

}  // end namespace ground_filter

#endif