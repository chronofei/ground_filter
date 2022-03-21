#include "ground_filter/models/csf/CSF.hpp"

namespace ground_filter
{

CSF::CSF()
{
    sloop_smooth_     = ParseConfig::GetInstance()->getConfig("models")["csf"]["sloop_smooth"].asBool();
    export_cloth_     = ParseConfig::GetInstance()->getConfig("models")["csf"]["export_cloth"].asBool();
    iterations_       = ParseConfig::GetInstance()->getConfig("models")["csf"]["iterations"].asInt();

    gravity_          = ParseConfig::GetInstance()->getConfig("models")["csf"]["gravity"].asDouble();
    time_step_        = ParseConfig::GetInstance()->getConfig("models")["csf"]["time_step"].asDouble();
    cloth_resolution_ = ParseConfig::GetInstance()->getConfig("models")["csf"]["cloth_resolution"].asDouble();
    smooth_threshold_ = ParseConfig::GetInstance()->getConfig("models")["csf"]["smooth_threshold"].asDouble();
    height_threshold_ = ParseConfig::GetInstance()->getConfig("models")["csf"]["height_threshold"].asDouble();
    rigidness_        = ParseConfig::GetInstance()->getConfig("models")["csf"]["rigitness"].asInt();
    class_threshold_  = ParseConfig::GetInstance()->getConfig("models")["csf"]["class_threshold"].asDouble();
}

CSF::~CSF()
{
    // TODO
}

bool CSF::Filter(CompoundData& data)
{
    if (data.cloud.empty())
        return false;

    Cloth cloth(data.cloud, Eigen::Vector3d(0,0,gravity_), time_step_, cloth_resolution_, 
                smooth_threshold_, height_threshold_, rigidness_, class_threshold_);

    for (int i = 0; i < iterations_; i++)
    {
        double maxDiff = cloth.TimeStep();
        cloth.TerrCollision();
        if ((maxDiff != 0) && (maxDiff < 0.005))
            break;
    }

    if (sloop_smooth_)
        cloth.MovableFilter();

    if (export_cloth_)
        cloth.GetCloth(data.cloth);

    cloth.GetFilterResult(data.ground, data.nonground);

    return true;
}

} // end namespace ground_filter