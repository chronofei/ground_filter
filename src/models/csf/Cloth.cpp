#include "ground_filter/models/csf/Cloth.hpp"

namespace ground_filter
{

Cloth::Cloth(const Eigen::Vector2d cloth_size, const double& cloth_resolution,
             const double& acceleration,       const double& time_step,
             const double& smooth_threshold,   const double& height_threshold,
             const int& rigidness,             const double& class_threshold)
{
    cloth_resolution_ = cloth_resolution;

    cloth_origin_ = Eigen::Vector2d( 0.0             - 2* cloth_resolution_,
                                    -cloth_size[1]/2 - 2 * cloth_resolution_);

    cloth_width_  = static_cast<int>(std::floor(cloth_size[0] / cloth_resolution_)) + 2 * 2;
    cloth_height_ = static_cast<int>(std::floor(cloth_size[1] / cloth_resolution_)) + 2 * 2;

    for (int row = 0; row < cloth_height_; row++)
    {
        for (int col = 0; col < cloth_width_; col++)
        {
            particles_.emplace_back(Particle(acceleration, time_step));
            particles_.back().SetPosition(cloth_origin_ + Eigen::Vector2d(col * cloth_resolution_, row * cloth_resolution_));
            particles_.back().SetRasterIndex(Eigen::Vector2i(col, row));
        }
    }

    for (int row = 0; row < cloth_height_-1; row++)
    {
        for (int col = 0; col < cloth_width_-1; col++)
        {
            if (col < cloth_width_ -2)
            {
                MakeConstraint(GetIndex1D(col,row), GetIndex1D(col+2, row));
            }

            if (row < cloth_height_ - 2)
            {
                MakeConstraint(GetIndex1D(col, row), GetIndex1D(col, row + 2));
            }

            if ((col < cloth_width_ - 2) && (row < cloth_height_ - 2))
            {
                MakeConstraint(GetIndex1D(col, row), GetIndex1D(col + 2, row + 2));
                MakeConstraint(GetIndex1D(col + 2, row), GetIndex1D(col, row + 2));
            }

            MakeConstraint(GetIndex1D(col, row), GetIndex1D(col + 1, row));
            MakeConstraint(GetIndex1D(col, row), GetIndex1D(col, row + 1));
            MakeConstraint(GetIndex1D(col, row), GetIndex1D(col + 1, row + 1));
            MakeConstraint(GetIndex1D(col + 1, row), GetIndex1D(col, row + 1));
        }
    }

    heightvals_.resize(cloth_height_ * cloth_width_, -10.0);

    smooth_threshold_ = smooth_threshold;
    height_threshold_ = height_threshold;
    rigidness_        = rigidness;
    class_threshold_  = class_threshold;
}

int Cloth::GetIndex1D(int col, int row)
{
    return col + row * cloth_width_;
}

void Cloth::MakeConstraint(int p1, int p2)
{
    particles_[p1].AddNeighbor(p2);
    particles_[p2].AddNeighbor(p1);
}

void Cloth::Clear()
{
    for (size_t i=0; i<particles_.size(); i++)
    {
        particles_[i].Clear();
    }

    heightvals_.clear();
    heightvals_.resize(cloth_height_ * cloth_width_, -10.0);
}

void Cloth::RasterizePointCloud(const PointCloud& cloud)
{
    for (size_t i=0; i<cloud.size(); i++)
    {
        auto point = cloud[i];

        int col = int((point.x - cloth_origin_[0]) / cloth_resolution_ + 0.5);
        int row = int((point.y - cloth_origin_[1]) / cloth_resolution_ + 0.5);

        if (col>=0 && col<cloth_width_ && row>=0 && row<cloth_height_)
        {
            auto& particle = particles_[GetIndex1D(col, row)];

            particle.corresponding_lidar_points_.emplace_back(i);

            double pc_2_particle_dist = SquareDistanceXY(Eigen::Vector2d(point.x, point.y), particle.GetPosition());

            if (pc_2_particle_dist < particle.tmp_dist_)
            {
                particle.tmp_dist_             = pc_2_particle_dist;
                particle.nearest_point_height_ = cloud[i].z;
            }
        }
    }

    for (size_t i = 0; i < particles_.size(); i++)
    {
        double nearestHeight = particles_[i].nearest_point_height_;

        if (nearestHeight > std::numeric_limits<double>::min())
        {
            heightvals_[i] = nearestHeight;
        }
        // else
        // {
        //     heightVal[i] = FindHeightValByScanline(pcur);
        // }
    }
}

// double Cloth::FindHeightValByScanline(Particle *p)
// {
//     int xpos = p->pos_x_;
//     int ypos = p->pos_y_;

//     for (int i = xpos + 1; i < num_particles_width_; i++) {
//         double crresHeight = GetParticle(i, ypos)->nearest_point_height_;

//         if (crresHeight > std::numeric_limits<double>::min())
//             return crresHeight;
//     }

//     for (int i = xpos - 1; i >= 0; i--) {
//         double crresHeight = GetParticle(i, ypos)->nearest_point_height_;

//         if (crresHeight > std::numeric_limits<double>::min())
//             return crresHeight;
//     }

//     for (int j = ypos - 1; j >= 0; j--) {
//         double crresHeight = GetParticle(xpos, j)->nearest_point_height_;

//         if (crresHeight > std::numeric_limits<double>::min())
//             return crresHeight;
//     }

//     for (int j = ypos + 1; j < num_particles_height_; j++) {
//         double crresHeight = GetParticle(xpos, j)->nearest_point_height_;

//         if (crresHeight > std::numeric_limits<double>::min())
//             return crresHeight;
//     }

//     return FindHeightValByNeighbor(p);
// }

// double Cloth::FindHeightValByNeighbor(Particle *p)
// {
//     std::queue<Particle *>  nqueue;
//     std::vector<Particle *> pbacklist;
//     int neiborsize = p->neighbors_.size();

//     for (int i = 0; i < neiborsize; i++) {
//         p->is_visited_ = true;
//         nqueue.push(p->neighbors_[i]);
//     }

//     // iterate over the nqueue
//     while (!nqueue.empty()) {
//         Particle *pneighbor = nqueue.front();
//         nqueue.pop();
//         pbacklist.push_back(pneighbor);

//         if (pneighbor->nearest_point_height_ > std::numeric_limits<double>::min()) {
//             for (std::size_t i = 0; i < pbacklist.size(); i++)
//                 pbacklist[i]->is_visited_ = false;

//             while (!nqueue.empty()) {
//                 Particle *pp = nqueue.front();
//                 pp->is_visited_ = false;
//                 nqueue.pop();
//             }

//             return pneighbor->nearest_point_height_;
//         } else {
//             int nsize = pneighbor->neighbors_.size();

//             for (int i = 0; i < nsize; i++) {
//                 Particle *ptmp = pneighbor->neighbors_[i];

//                 if (!ptmp->is_visited_) {
//                     ptmp->is_visited_ = true;
//                     nqueue.push(ptmp);
//                 }
//             }
//         }
//     }

//     return std::numeric_limits<double>::min();
// }



// double Cloth::TimeStep()
// {
//     int particleCount = static_cast<int>(particles_.size());
//     for (int i = 0; i < particleCount; i++)
//     {
//         particles_[i].TimeStep();
//     }

//     for (int j = 0; j < particleCount; j++)
//     {
//         SatisfyConstraintSelf(j, rigidness_);
//     }

//     double maxDiff = 0;

//     for (int i = 0; i < particleCount; i++)
//     {
//         if (particles_[i].IsMovable())
//         {
//             double diff = fabs(particles_[i].old_pos_[2] - particles_[i].pos_[2]);

//             if (diff > maxDiff)
//                 maxDiff = diff;
//         }
//     }

//     return maxDiff;
// }

// void Cloth::SatisfyConstraintSelf(int p, int rigidness)
// {
//     Particle& p1 = particles_[p];

//     for (std::size_t i = 0; i < p1.neighbors_.size(); i++)
//     {
//         Particle& p2 = particles_[neighbors_[i]];
//         Eigen::Vector3d correctionVector(0, 0, p2.pos_[2] - p1.pos_[2]);

//         if (p1.IsMovable() && p2.IsMovable())
//         {
//             Eigen::Vector3d correctionVectorHalf = correctionVector * (rigidness > 14 ? 0.5 : doubleMove1[rigidness]);
//             p1.OffsetPos(correctionVectorHalf);
//             p2.OffsetPos(-correctionVectorHalf);
//         }
//         else if (p1.IsMovable() && !p2.IsMovable())
//         {
//             Eigen::Vector3d correctionVectorHalf = correctionVector * (rigidness > 14 ? 1 : singleMove1[rigidness]);
//             p1.OffsetPos(correctionVectorHalf);
//         }
//         else if (!p1.IsMovable() && p2.IsMovable())
//         {
//             Eigen::Vector3d correctionVectorHalf = correctionVector * (rigidness > 14 ? 1 : singleMove1[rigidness]);
//             p2.OffsetPos(-correctionVectorHalf);
//         }
//     }
// }

// void Cloth::TerrCollision()
// {
//     int particleCount = static_cast<int>(particles_.size());
//     for (int i = 0; i < particleCount; i++)
//     {
//         double height = particles_[i].pos_[2];

//         if (height < heightvals_[i])
//         {
//             particles_[i].OffsetPos(Eigen::Vector3d(0, 0, heightvals_[i] - height));
//             particles_[i].MakeUnmovable();
//         }
//     }
// }

// void Cloth::MovableFilter()
// {
//     std::vector<Particle> tmpParticles;

//     for (int x = 0; x < num_particles_width_; x++) {
//         for (int y = 0; y < num_particles_height_; y++) {
//             Particle *ptc = GetParticle(x, y);

//             if (ptc->IsMovable() && !ptc->is_visited_) {
//                 std::queue<int> que;
//                 std::vector<XY> connected; // store the connected component
//                 std::vector<std::vector<int> > neibors;
//                 int sum   = 1;
//                 int index = y * num_particles_width_ + x;

//                 // visit the init node
//                 connected.push_back(XY(x, y));
//                 particles_[index].is_visited_ = true;

//                 // enqueue the init node
//                 que.push(index);

//                 while (!que.empty()) {
//                     Particle *ptc_f = &particles_[que.front()];
//                     que.pop();
//                     int cur_x = ptc_f->pos_x_;
//                     int cur_y = ptc_f->pos_y_;
//                     std::vector<int> neibor;

//                     if (cur_x > 0) {
//                         Particle *ptc_left = GetParticle(cur_x - 1, cur_y);

//                         if (ptc_left->IsMovable()) {
//                             if (!ptc_left->is_visited_) {
//                                 sum++;
//                                 ptc_left->is_visited_ = true;
//                                 connected.push_back(XY(cur_x - 1, cur_y));
//                                 que.push(num_particles_width_ * cur_y + cur_x - 1);
//                                 neibor.push_back(sum - 1);
//                                 ptc_left->c_pos_ = sum - 1;
//                             } else {
//                                 neibor.push_back(ptc_left->c_pos_);
//                             }
//                         }
//                     }

//                     if (cur_x < num_particles_width_ - 1) {
//                         Particle *ptc_right = GetParticle(cur_x + 1, cur_y);

//                         if (ptc_right->IsMovable()) {
//                             if (!ptc_right->is_visited_) {
//                                 sum++;
//                                 ptc_right->is_visited_ = true;
//                                 connected.push_back(XY(cur_x + 1, cur_y));
//                                 que.push(num_particles_width_ * cur_y + cur_x + 1);
//                                 neibor.push_back(sum - 1);
//                                 ptc_right->c_pos_ = sum - 1;
//                             } else {
//                                 neibor.push_back(ptc_right->c_pos_);
//                             }
//                         }
//                     }

//                     if (cur_y > 0) {
//                         Particle *ptc_bottom = GetParticle(cur_x, cur_y - 1);

//                         if (ptc_bottom->IsMovable()) {
//                             if (!ptc_bottom->is_visited_) {
//                                 sum++;
//                                 ptc_bottom->is_visited_ = true;
//                                 connected.push_back(XY(cur_x, cur_y - 1));
//                                 que.push(num_particles_width_ * (cur_y - 1) + cur_x);
//                                 neibor.push_back(sum - 1);
//                                 ptc_bottom->c_pos_ = sum - 1;
//                             } else {
//                                 neibor.push_back(ptc_bottom->c_pos_);
//                             }
//                         }
//                     }

//                     if (cur_y < num_particles_height_ - 1) {
//                         Particle *ptc_top = GetParticle(cur_x, cur_y + 1);

//                         if (ptc_top->IsMovable()) {
//                             if (!ptc_top->is_visited_) {
//                                 sum++;
//                                 ptc_top->is_visited_ = true;
//                                 connected.push_back(XY(cur_x, cur_y + 1));
//                                 que.push(num_particles_width_ * (cur_y + 1) + cur_x);
//                                 neibor.push_back(sum - 1);
//                                 ptc_top->c_pos_ = sum - 1;
//                             } else {
//                                 neibor.push_back(ptc_top->c_pos_);
//                             }
//                         }
//                     }
//                     neibors.push_back(neibor);
//                 }

//                 if (sum > MAX_PARTICLE_FOR_POSTPROCESSIN) {
//                     std::vector<int> edgePoints = FindUnmovablePoint(connected);
//                     HandleSlopConnected(edgePoints, connected, neibors);
//                 }
//             }
//         }
//     }
// }

// std::vector<int> Cloth::FindUnmovablePoint(std::vector<XY> connected)
// {
//     std::vector<int> edgePoints;

//     for (std::size_t i = 0; i < connected.size(); i++) {
//         int x         = connected[i].x;
//         int y         = connected[i].y;
//         int index     = y * num_particles_width_ + x;
//         Particle *ptc = GetParticle(x, y);

//         if (x > 0) {
//             Particle *ptc_x = GetParticle(x - 1, y);

//             if (!ptc_x->IsMovable()) {
//                 int index_ref = y * num_particles_width_ + x - 1;

//                 if ((fabs(heightvals_[index] - heightvals_[index_ref]) < smooth_threshold_) &&
//                     (ptc->pos_[2] - heightvals_[index] < height_threshold_)) {
//                     Eigen::Vector3d offsetVec(0, 0, heightvals_[index] - ptc->pos_[2]);
//                     particles_[index].OffsetPos(offsetVec);
//                     ptc->MakeUnmovable();
//                     edgePoints.push_back(i);
//                     continue;
//                 }
//             }
//         }

//         if (x < num_particles_width_ - 1) {
//             Particle *ptc_x = GetParticle(x + 1, y);

//             if (!ptc_x->IsMovable()) {
//                 int index_ref = y * num_particles_width_ + x + 1;

//                 if ((fabs(heightvals_[index] - heightvals_[index_ref]) < smooth_threshold_) &&
//                     (ptc->pos_[2] - heightvals_[index] < height_threshold_)) {
//                     Eigen::Vector3d offsetVec(0, 0, heightvals_[index] - ptc->pos_[2]);
//                     particles_[index].OffsetPos(offsetVec);
//                     ptc->MakeUnmovable();
//                     edgePoints.push_back(i);
//                     continue;
//                 }
//             }
//         }

//         if (y > 0) {
//             Particle *ptc_y = GetParticle(x, y - 1);

//             if (!ptc_y->IsMovable()) {
//                 int index_ref = (y - 1) * num_particles_width_ + x;

//                 if ((fabs(heightvals_[index] - heightvals_[index_ref]) < smooth_threshold_) &&
//                     (ptc->pos_[2] - heightvals_[index] < height_threshold_)) {
//                     Eigen::Vector3d offsetVec(0, 0, heightvals_[index] - ptc->pos_[2]);
//                     particles_[index].OffsetPos(offsetVec);
//                     ptc->MakeUnmovable();
//                     edgePoints.push_back(i);
//                     continue;
//                 }
//             }
//         }

//         if (y < num_particles_height_ - 1) {
//             Particle *ptc_y = GetParticle(x, y + 1);

//             if (!ptc_y->IsMovable()) {
//                 int index_ref = (y + 1) * num_particles_width_ + x;

//                 if ((fabs(heightvals_[index] - heightvals_[index_ref]) < smooth_threshold_) &&
//                     (ptc->pos_[2] - heightvals_[index] < height_threshold_)) {
//                     Eigen::Vector3d offsetVec(0, 0, heightvals_[index] - ptc->pos_[2]);
//                     particles_[index].OffsetPos(offsetVec);
//                     ptc->MakeUnmovable();
//                     edgePoints.push_back(i);
//                     continue;
//                 }
//             }
//         }
//     }

//     return edgePoints;
// }

// void Cloth::HandleSlopConnected(std::vector<int> edgePoints, std::vector<XY> connected, std::vector<std::vector<int> > neibors)
// {
//     std::vector<bool> visited;

//     for (std::size_t i = 0; i < connected.size(); i++) visited.push_back(false);

//     std::queue<int> que;

//     for (std::size_t i = 0; i < edgePoints.size(); i++) {
//         que.push(edgePoints[i]);
//         visited[edgePoints[i]] = true;
//     }

//     while (!que.empty()) {
//         int index = que.front();
//         que.pop();

//         int index_center = connected[index].y * num_particles_width_ + connected[index].x;

//         for (std::size_t i = 0; i < neibors[index].size(); i++) {
//             int index_neibor = connected[neibors[index][i]].y * num_particles_width_ + connected[neibors[index][i]].x;

//             if ((fabs(heightvals_[index_center] - heightvals_[index_neibor]) < smooth_threshold_) &&
//                 (fabs(particles_[index_neibor].pos_[2] - heightvals_[index_neibor]) < height_threshold_)) {
//                 Eigen::Vector3d offsetVec(0, 0, heightvals_[index_neibor] - particles_[index_neibor].pos_[2]);
//                 particles_[index_neibor].OffsetPos(offsetVec);
//                 particles_[index_neibor].MakeUnmovable();

//                 if (visited[neibors[index][i]] == false) {
//                     que.push(neibors[index][i]);
//                     visited[neibors[index][i]] = true;
//                 }
//             }
//         }
//     }
// }

void Cloth::GetCloth(PointCloud& cloth)
{
    cloth.clear();

    for (std::size_t i = 0; i < particles_.size(); i++)
    {
        auto position = particles_[i].GetPosition3D();
        Point temp;
        temp.x = position[0];
        temp.y = position[1];
        temp.z = position[2];

        cloth.push_back(temp);
    }
}

// void Cloth::GetFilterResult(PointCloud& ground, PointCloud& nonground)
// {
//     ground.clear();
//     nonground.clear();

//     int total_particles = num_particles_width_ * num_particles_height_;

//     for (int i = 0; i < total_particles; i++)
//     {
//         Particle* pcur = GetParticle(i);

//         int col = pcur->pos_x_;
//         int row = pcur->pos_y_;

//         double h00 = GetParticle(col    , row    )->pos_[2];
//         double h10 = GetParticle(col + 1, row    )->pos_[2];
//         double h11 = GetParticle(col + 1, row + 1)->pos_[2];
//         double h01 = GetParticle(col    , row + 1)->pos_[2];

//         for (size_t j=0; j<pcur->corresponding_lidar_points_.size(); j++)
//         {
//             auto point = pcur->corresponding_lidar_points_[j];

//             double subdeltaX = (point.x - pcur->pos_[0]) / cloth_resolution_;
//             double subdeltaY = (point.y - pcur->pos_[1]) / cloth_resolution_;

//             double fxy = h00 * (1 - subdeltaX) * (1 - subdeltaY) +
//                          h01 * (1 - subdeltaX) *      subdeltaY  +
//                          h11 *      subdeltaX  *      subdeltaY  +
//                          h10 *      subdeltaX  * (1 - subdeltaY);

//             double height_var = fxy - point.z;

//             if (std::fabs(height_var) < class_threshold_)
//                 ground.push_back(point);
//             else
//                 nonground.push_back(point);
//         }
//     }
// }

} // end namespace ground_filter