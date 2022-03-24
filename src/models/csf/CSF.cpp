#include "ground_filter/models/csf/CSF.hpp"

namespace ground_filter
{

CSF::CSF()
{
    sloop_smooth_     = ParseConfig::GetInstance()->getConfig("models")["csf"]["sloop_smooth"].asBool();
    iterations_       = ParseConfig::GetInstance()->getConfig("models")["csf"]["iterations"].asInt();
    max_particle_for_postprocess_ = ParseConfig::GetInstance()->getConfig("models")["csf"]["max_particle_for_postprocess"].asInt();

    double cloth_width      = ParseConfig::GetInstance()->getConfig("models")["csf"]["cloth_width"].asDouble();
    double cloth_height     = ParseConfig::GetInstance()->getConfig("models")["csf"]["cloth_height"].asDouble();
    double acceleration     = ParseConfig::GetInstance()->getConfig("models")["csf"]["acceleration"].asDouble();
    double time_step        = ParseConfig::GetInstance()->getConfig("models")["csf"]["time_step"].asDouble();
    double cloth_resolution = ParseConfig::GetInstance()->getConfig("models")["csf"]["cloth_resolution"].asDouble();
    double smooth_threshold = ParseConfig::GetInstance()->getConfig("models")["csf"]["smooth_threshold"].asDouble();
    double height_threshold = ParseConfig::GetInstance()->getConfig("models")["csf"]["height_threshold"].asDouble();
    double class_threshold  = ParseConfig::GetInstance()->getConfig("models")["csf"]["class_threshold"].asDouble();

    cloth_ = std::make_shared<Cloth>(Eigen::Vector2d(cloth_width, cloth_height), cloth_resolution, acceleration, time_step, 
                                     smooth_threshold, height_threshold, class_threshold);
}

CSF::~CSF()
{
    // TODO
}

bool CSF::Filter(CompoundData& data)
{
    if (data.cloud.empty())
        return false;

    TicToc csf;

    cloth_->Clear();
    cloth_->RasterizePointCloud(data.cloud);
    
    for (int i = 0; i < iterations_; i++)
    {
        cloth_->TimeStep(i);
        cloth_->TerrCollision();
    }

    if (sloop_smooth_)
        cloth_->MovableFilter(max_particle_for_postprocess_);

    cloth_->GetFilterResult(data.ground, data.nonground, data.cloth);

    csf.toc("CSF");
    return true;
}

} // end namespace ground_filter