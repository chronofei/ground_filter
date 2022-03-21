#include "ground_filter/models/csf/Particle.hpp"

namespace ground_filter
{

Particle::Particle(const Eigen::Vector3d& position, const Eigen::Vector3d& acceleration, const double time_step)
{
    movable_      = true;
    acceleration_ = acceleration;
    time_step2_   = time_step * time_step;

    pos_     = position;
    old_pos_ = position;

    isVisited_            = false;
    pos_x_                = 0;
    pos_y_                = 0;
    c_pos_                = 0;
    nearest_point_height_ = std::numeric_limits<double>::min();
    tmpDist_              = std::numeric_limits<double>::max();
}

bool Particle::IsMovable()
{
    return movable_;
}

void Particle::MakeUnmovable()
{
    movable_ = false;
}

Eigen::Vector3d& Particle::GetPos()
{
    return pos_;
}

void Particle::OffsetPos(const Eigen::Vector3d v)
{
    if (movable_)
        pos_ += v;
}

void Particle::TimeStep()
{
    if (movable_)
    {
        Eigen::Vector3d temp = pos_;
        pos_                 = pos_ + (pos_ - old_pos_) * (1.0 - DAMPING) + acceleration_ * time_step2_;
        old_pos_             = temp;
    }
}

void Particle::SatisfyConstraintSelf(int rigidness)
{
    Particle *p1 = this;

    for (std::size_t i = 0; i < neighbors_.size(); i++)
    {
        Particle *p2 = neighbors_[i];
        Eigen::Vector3d correctionVector(0, 0, p2->pos_[2] - p1->pos_[2]);

        if (p1->IsMovable() && p2->IsMovable())
        {
            Eigen::Vector3d correctionVectorHalf = correctionVector * (rigidness > 14 ? 0.5 : doubleMove1[rigidness]);
            p1->OffsetPos(correctionVectorHalf);
            p2->OffsetPos(-correctionVectorHalf);
        }
        else if (p1->IsMovable() && !p2->IsMovable())
        {
            Eigen::Vector3d correctionVectorHalf = correctionVector * (rigidness > 14 ? 1 : singleMove1[rigidness]);
            p1->OffsetPos(correctionVectorHalf);
        }
        else if (!p1->IsMovable() && p2->IsMovable())
        {
            Eigen::Vector3d correctionVectorHalf = correctionVector * (rigidness > 14 ? 1 : singleMove1[rigidness]);
            p2->OffsetPos(-correctionVectorHalf);
        }
    }
}

} // end namespace ground_filter