#include "ground_filter/models/csf/CSF.hpp"

namespace ground_filter
{

CSF::CSF()
{
    sloop_smooth_     = ParseConfig::GetInstance()->getConfig("models")["csf"]["sloop_smooth"].asBool();
    export_cloth_     = ParseConfig::GetInstance()->getConfig("models")["csf"]["export_cloth"].asBool();
    iterations_       = ParseConfig::GetInstance()->getConfig("models")["csf"]["iterations"].asInt();
    class_threshold_  = ParseConfig::GetInstance()->getConfig("models")["csf"]["class_threshold"].asDouble();
    max_particle_for_postprocess_ = ParseConfig::GetInstance()->getConfig("models")["csf"]["max_particle_for_postprocess"].asInt();

    double cloth_width      = ParseConfig::GetInstance()->getConfig("models")["csf"]["cloth_width"].asDouble();
    double cloth_height     = ParseConfig::GetInstance()->getConfig("models")["csf"]["cloth_height"].asDouble();
    double acceleration     = ParseConfig::GetInstance()->getConfig("models")["csf"]["acceleration"].asDouble();
    double time_step        = ParseConfig::GetInstance()->getConfig("models")["csf"]["time_step"].asDouble();
    double cloth_resolution = ParseConfig::GetInstance()->getConfig("models")["csf"]["cloth_resolution"].asDouble();
    double smooth_threshold = ParseConfig::GetInstance()->getConfig("models")["csf"]["smooth_threshold"].asDouble();
    double height_threshold = ParseConfig::GetInstance()->getConfig("models")["csf"]["height_threshold"].asDouble();
    int    rigidness        = ParseConfig::GetInstance()->getConfig("models")["csf"]["rigidness"].asInt();
    

    cloth_ = std::make_shared<Cloth>(Eigen::Vector2d(cloth_width, cloth_height), cloth_resolution, acceleration, time_step, 
                                     smooth_threshold, height_threshold, rigidness);
}

CSF::~CSF()
{
    // TODO
}

bool CSF::Filter(CompoundData& data)
{
    if (data.cloud.empty())
        return false;

    TicToc filter;

    cloth_->Clear();
    cloth_->RasterizePointCloud(data.cloud);

    for (int i = 0; i < iterations_; i++)
    {
        double maxDiff = cloth_->TimeStep();
        cloth_->TerrCollision();
        if ((maxDiff != 0) && (maxDiff < 0.005))
            break;
    }

    if (sloop_smooth_)
        cloth_->MovableFilter(max_particle_for_postprocess_);

    if (export_cloth_)
        cloth_->GetCloth(data.cloth);

    cloth_->GetFilterResult(class_threshold_, data.ground, data.nonground);

    filter.toc("CSF");

    return true;
}

} // end namespace ground_filter