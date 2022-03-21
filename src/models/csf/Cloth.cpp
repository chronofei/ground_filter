#include "ground_filter/models/csf/Cloth.hpp"

namespace ground_filter
{

Cloth::Cloth(const PointCloud& cloud, const Eigen::Vector3d& acceleration, const double& time_step,
             const double& cloth_resolution, const double& smooth_threshold, const double& height_threshold, 
             const int& rigidness, const double& class_threshold) 
{
    cloud_ = cloud;

    cloth_resolution_ = cloth_resolution;
    smooth_threshold_ = smooth_threshold;
    height_threshold_ = height_threshold;
    rigidness_        = rigidness;
    class_threshold_  = class_threshold;

    Eigen::Vector3d min_boundary, max_boundary;
    ComputeBoundingBox(cloud_, min_boundary, max_boundary);

    origin_ = Eigen::Vector3d(min_boundary[0] - 2 * cloth_resolution_,
                              min_boundary[1] - 2 * cloth_resolution_,
                              max_boundary[2] - 0.05);

    int num_particles_width_  = static_cast<int>(std::floor((max_boundary[0] - min_boundary[0]) / cloth_resolution_)) + 2 * 2;
    int num_particles_height_ = static_cast<int>(std::floor((max_boundary[1] - min_boundary[0]) / cloth_resolution_)) + 2 * 2;

    for (int i = 0; i < num_particles_width_; i++)
    {
        for (int j = 0; j < num_particles_height_; j++)
        {
            Eigen::Vector3d pos(origin_[0] + i *cloth_resolution_,
                                origin_[1] + j *cloth_resolution_,
                                origin_[2]);
            particles_.emplace_back(Particle(pos, acceleration, time_step));
            particles_.back().pos_x_ = i;
            particles_.back().pos_y_ = j;
        }
    }

    for (int x = 0; x < num_particles_width_; x++)
    {
        for (int y = 0; y < num_particles_height_; y++)
        {
            if (x < num_particles_width_ - 1)
                MakeConstraint(GetParticle(x, y), GetParticle(x + 1, y));

            if (y < num_particles_height_ - 1)
                MakeConstraint(GetParticle(x, y), GetParticle(x, y + 1));

            if ((x < num_particles_width_ - 1) && (y < num_particles_height_ - 1))
                MakeConstraint(GetParticle(x, y), GetParticle(x + 1, y + 1));

            if ((x < num_particles_width_ - 1) && (y < num_particles_height_ - 1))
                MakeConstraint(GetParticle(x + 1, y), GetParticle(x, y + 1));
        }
    }

    for (int x = 0; x < num_particles_width_; x++)
    {
        for (int y = 0; y < num_particles_height_; y++)
        {
            if (x < num_particles_width_ - 2)
                MakeConstraint(GetParticle(x, y), GetParticle(x + 2, y));

            if (y < num_particles_height_ - 2)
                MakeConstraint(GetParticle(x, y), GetParticle(x, y + 2));

            if ((x < num_particles_width_ - 2) && (y < num_particles_height_ - 2))
                MakeConstraint(GetParticle(x, y), GetParticle(x + 2, y + 2));

            if ((x < num_particles_width_ - 2) && (y < num_particles_height_ - 2))
                MakeConstraint(GetParticle(x + 2, y), GetParticle(x, y + 2));
        }
    }

    RasterTerrian(cloud_, heightvals_);
}

void Cloth::ComputeBoundingBox(const PointCloud& cloud, Eigen::Vector3d& min_boundary, Eigen::Vector3d& max_boundary)
{
    min_boundary[0] = std::numeric_limits<double>::max();
    min_boundary[1] = std::numeric_limits<double>::max();
    min_boundary[2] = std::numeric_limits<double>::max();

    max_boundary[0] = std::numeric_limits<double>::min();
    max_boundary[1] = std::numeric_limits<double>::min();
    max_boundary[2] = std::numeric_limits<double>::min();

    for (size_t i = 0; i < cloud.size(); i++)
    {
        Point p = cloud[i];

        if (p.x < min_boundary[0])
            min_boundary[0] = p.x;
        else if (p.x > max_boundary[0])
            max_boundary[0] = p.x;

        if (p.y < min_boundary[1])
            min_boundary[1] = p.y;
        else if (p.y > max_boundary[1])
            max_boundary[1] = p.y;

        if (p.z < min_boundary[2])
            min_boundary[2] = p.z;
        else if (p.z > max_boundary[2])
            max_boundary[2] = p.z;
    }
}

Particle* Cloth::GetParticle(int x, int y)
{
    return &particles_[y * num_particles_width_ + x];
}

Particle* Cloth::GetParticle(int index)
{
    return &particles_[index];
}

void Cloth::MakeConstraint(Particle *p1, Particle *p2)
{
    p1->neighbors_.push_back(p2);
    p2->neighbors_.push_back(p1);
}

double Cloth::FindHeightValByScanline(Particle *p)
{
    int xpos = p->pos_x_;
    int ypos = p->pos_y_;

    for (int i = xpos + 1; i < num_particles_width_; i++) {
        double crresHeight = GetParticle(i, ypos)->nearest_point_height_;

        if (crresHeight > std::numeric_limits<double>::min())
            return crresHeight;
    }

    for (int i = xpos - 1; i >= 0; i--) {
        double crresHeight = GetParticle(i, ypos)->nearest_point_height_;

        if (crresHeight > std::numeric_limits<double>::min())
            return crresHeight;
    }

    for (int j = ypos - 1; j >= 0; j--) {
        double crresHeight = GetParticle(xpos, j)->nearest_point_height_;

        if (crresHeight > std::numeric_limits<double>::min())
            return crresHeight;
    }

    for (int j = ypos + 1; j < num_particles_height_; j++) {
        double crresHeight = GetParticle(xpos, j)->nearest_point_height_;

        if (crresHeight > std::numeric_limits<double>::min())
            return crresHeight;
    }

    return FindHeightValByNeighbor(p);
}

double Cloth::FindHeightValByNeighbor(Particle *p)
{
    std::queue<Particle *>  nqueue;
    std::vector<Particle *> pbacklist;
    int neiborsize = p->neighbors_.size();

    for (int i = 0; i < neiborsize; i++) {
        p->isVisited_ = true;
        nqueue.push(p->neighbors_[i]);
    }

    // iterate over the nqueue
    while (!nqueue.empty()) {
        Particle *pneighbor = nqueue.front();
        nqueue.pop();
        pbacklist.push_back(pneighbor);

        if (pneighbor->nearest_point_height_ > std::numeric_limits<double>::min()) {
            for (std::size_t i = 0; i < pbacklist.size(); i++)
                pbacklist[i]->isVisited_ = false;

            while (!nqueue.empty()) {
                Particle *pp = nqueue.front();
                pp->isVisited_ = false;
                nqueue.pop();
            }

            return pneighbor->nearest_point_height_;
        } else {
            int nsize = pneighbor->neighbors_.size();

            for (int i = 0; i < nsize; i++) {
                Particle *ptmp = pneighbor->neighbors_[i];

                if (!ptmp->isVisited_) {
                    ptmp->isVisited_ = true;
                    nqueue.push(ptmp);
                }
            }
        }
    }

    return std::numeric_limits<double>::min();
}

void Cloth::RasterTerrian(const PointCloud& cloud, std::vector<double>& heightVal)
{
    for (size_t i = 0; i < cloud.size(); i++)
    {
        double pc_x = cloud[i].x;
        double pc_y = cloud[i].y;

        double deltaX = pc_x - origin_[0];
        double deltaZ = pc_y - origin_[1];
        int    col    = int(deltaX / cloth_resolution_ + 0.5);
        int    row    = int(deltaZ / cloth_resolution_ + 0.5);

        if ((col >= 0) && (row >= 0))
        {
            Particle *pt = GetParticle(col, row);
            pt->corresponding_lidar_point_.push_back(i);
            double pc2particleDist = SquareDistanceXY(pc_x, pc_y, pt->pos_[0], pt->pos_[1]);

            if (pc2particleDist < pt->tmpDist_)
            {
                pt->tmpDist_              = pc2particleDist;
                pt->nearest_point_height_ = cloud[i].z;
            }
        }
    }

    int total_particles = num_particles_width_ * num_particles_height_;
    heightVal.resize(total_particles);

    for (int i = 0; i < total_particles; i++)
    {
        Particle *pcur          = GetParticle(i);
        double    nearestHeight = pcur->nearest_point_height_;

        if (nearestHeight > std::numeric_limits<double>::min())
        {
            heightVal[i] = nearestHeight;
        }
        else
        {
            heightVal[i] = FindHeightValByScanline(pcur);
        }
    }
}

double Cloth::TimeStep()
{
    int particleCount = static_cast<int>(particles_.size());
    for (int i = 0; i < particleCount; i++)
    {
        particles_[i].TimeStep();
    }

    for (int j = 0; j < particleCount; j++)
    {
        particles_[j].SatisfyConstraintSelf(rigidness_);
    }

    double maxDiff = 0;

    for (int i = 0; i < particleCount; i++)
    {
        if (particles_[i].IsMovable())
        {
            double diff = fabs(particles_[i].old_pos_[2] - particles_[i].pos_[2]);

            if (diff > maxDiff)
                maxDiff = diff;
        }
    }

    return maxDiff;
}

void Cloth::TerrCollision()
{
    int particleCount = static_cast<int>(particles_.size());
    for (int i = 0; i < particleCount; i++)
    {
        double height = particles_[i].pos_[2];

        if (height < heightvals_[i])
        {
            particles_[i].OffsetPos(Eigen::Vector3d(0, 0, heightvals_[i] - height));
            particles_[i].MakeUnmovable();
        }
    }
}

void Cloth::MovableFilter()
{
    std::vector<Particle> tmpParticles;

    for (int x = 0; x < num_particles_width_; x++) {
        for (int y = 0; y < num_particles_height_; y++) {
            Particle *ptc = GetParticle(x, y);

            if (ptc->IsMovable() && !ptc->isVisited_) {
                std::queue<int> que;
                std::vector<XY> connected; // store the connected component
                std::vector<std::vector<int> > neibors;
                int sum   = 1;
                int index = y * num_particles_width_ + x;

                // visit the init node
                connected.push_back(XY(x, y));
                particles_[index].isVisited_ = true;

                // enqueue the init node
                que.push(index);

                while (!que.empty()) {
                    Particle *ptc_f = &particles_[que.front()];
                    que.pop();
                    int cur_x = ptc_f->pos_x_;
                    int cur_y = ptc_f->pos_y_;
                    std::vector<int> neibor;

                    if (cur_x > 0) {
                        Particle *ptc_left = GetParticle(cur_x - 1, cur_y);

                        if (ptc_left->IsMovable()) {
                            if (!ptc_left->isVisited_) {
                                sum++;
                                ptc_left->isVisited_ = true;
                                connected.push_back(XY(cur_x - 1, cur_y));
                                que.push(num_particles_width_ * cur_y + cur_x - 1);
                                neibor.push_back(sum - 1);
                                ptc_left->c_pos_ = sum - 1;
                            } else {
                                neibor.push_back(ptc_left->c_pos_);
                            }
                        }
                    }

                    if (cur_x < num_particles_width_ - 1) {
                        Particle *ptc_right = GetParticle(cur_x + 1, cur_y);

                        if (ptc_right->IsMovable()) {
                            if (!ptc_right->isVisited_) {
                                sum++;
                                ptc_right->isVisited_ = true;
                                connected.push_back(XY(cur_x + 1, cur_y));
                                que.push(num_particles_width_ * cur_y + cur_x + 1);
                                neibor.push_back(sum - 1);
                                ptc_right->c_pos_ = sum - 1;
                            } else {
                                neibor.push_back(ptc_right->c_pos_);
                            }
                        }
                    }

                    if (cur_y > 0) {
                        Particle *ptc_bottom = GetParticle(cur_x, cur_y - 1);

                        if (ptc_bottom->IsMovable()) {
                            if (!ptc_bottom->isVisited_) {
                                sum++;
                                ptc_bottom->isVisited_ = true;
                                connected.push_back(XY(cur_x, cur_y - 1));
                                que.push(num_particles_width_ * (cur_y - 1) + cur_x);
                                neibor.push_back(sum - 1);
                                ptc_bottom->c_pos_ = sum - 1;
                            } else {
                                neibor.push_back(ptc_bottom->c_pos_);
                            }
                        }
                    }

                    if (cur_y < num_particles_height_ - 1) {
                        Particle *ptc_top = GetParticle(cur_x, cur_y + 1);

                        if (ptc_top->IsMovable()) {
                            if (!ptc_top->isVisited_) {
                                sum++;
                                ptc_top->isVisited_ = true;
                                connected.push_back(XY(cur_x, cur_y + 1));
                                que.push(num_particles_width_ * (cur_y + 1) + cur_x);
                                neibor.push_back(sum - 1);
                                ptc_top->c_pos_ = sum - 1;
                            } else {
                                neibor.push_back(ptc_top->c_pos_);
                            }
                        }
                    }
                    neibors.push_back(neibor);
                }

                if (sum > MAX_PARTICLE_FOR_POSTPROCESSIN) {
                    std::vector<int> edgePoints = FindUnmovablePoint(connected);
                    Handle_slop_connected(edgePoints, connected, neibors);
                }
            }
        }
    }
}

std::vector<int> Cloth::FindUnmovablePoint(std::vector<XY> connected)
{
    std::vector<int> edgePoints;

    for (std::size_t i = 0; i < connected.size(); i++) {
        int x         = connected[i].x;
        int y         = connected[i].y;
        int index     = y * num_particles_width_ + x;
        Particle *ptc = GetParticle(x, y);

        if (x > 0) {
            Particle *ptc_x = GetParticle(x - 1, y);

            if (!ptc_x->IsMovable()) {
                int index_ref = y * num_particles_width_ + x - 1;

                if ((fabs(heightvals_[index] - heightvals_[index_ref]) < smooth_threshold_) &&
                    (ptc->pos_[2] - heightvals_[index] < height_threshold_)) {
                    Eigen::Vector3d offsetVec(0, 0, heightvals_[index] - ptc->pos_[2]);
                    particles_[index].OffsetPos(offsetVec);
                    ptc->MakeUnmovable();
                    edgePoints.push_back(i);
                    continue;
                }
            }
        }

        if (x < num_particles_width_ - 1) {
            Particle *ptc_x = GetParticle(x + 1, y);

            if (!ptc_x->IsMovable()) {
                int index_ref = y * num_particles_width_ + x + 1;

                if ((fabs(heightvals_[index] - heightvals_[index_ref]) < smooth_threshold_) &&
                    (ptc->pos_[2] - heightvals_[index] < height_threshold_)) {
                    Eigen::Vector3d offsetVec(0, 0, heightvals_[index] - ptc->pos_[2]);
                    particles_[index].OffsetPos(offsetVec);
                    ptc->MakeUnmovable();
                    edgePoints.push_back(i);
                    continue;
                }
            }
        }

        if (y > 0) {
            Particle *ptc_y = GetParticle(x, y - 1);

            if (!ptc_y->IsMovable()) {
                int index_ref = (y - 1) * num_particles_width_ + x;

                if ((fabs(heightvals_[index] - heightvals_[index_ref]) < smooth_threshold_) &&
                    (ptc->pos_[2] - heightvals_[index] < height_threshold_)) {
                    Eigen::Vector3d offsetVec(0, 0, heightvals_[index] - ptc->pos_[2]);
                    particles_[index].OffsetPos(offsetVec);
                    ptc->MakeUnmovable();
                    edgePoints.push_back(i);
                    continue;
                }
            }
        }

        if (y < num_particles_height_ - 1) {
            Particle *ptc_y = GetParticle(x, y + 1);

            if (!ptc_y->IsMovable()) {
                int index_ref = (y + 1) * num_particles_width_ + x;

                if ((fabs(heightvals_[index] - heightvals_[index_ref]) < smooth_threshold_) &&
                    (ptc->pos_[2] - heightvals_[index] < height_threshold_)) {
                    Eigen::Vector3d offsetVec(0, 0, heightvals_[index] - ptc->pos_[2]);
                    particles_[index].OffsetPos(offsetVec);
                    ptc->MakeUnmovable();
                    edgePoints.push_back(i);
                    continue;
                }
            }
        }
    }

    return edgePoints;
}

void Cloth::Handle_slop_connected(std::vector<int> edgePoints, std::vector<XY> connected, std::vector<std::vector<int> > neibors)
{
    std::vector<bool> visited;

    for (std::size_t i = 0; i < connected.size(); i++) visited.push_back(false);

    std::queue<int> que;

    for (std::size_t i = 0; i < edgePoints.size(); i++) {
        que.push(edgePoints[i]);
        visited[edgePoints[i]] = true;
    }

    while (!que.empty()) {
        int index = que.front();
        que.pop();

        int index_center = connected[index].y * num_particles_width_ + connected[index].x;

        for (std::size_t i = 0; i < neibors[index].size(); i++) {
            int index_neibor = connected[neibors[index][i]].y * num_particles_width_ + connected[neibors[index][i]].x;

            if ((fabs(heightvals_[index_center] - heightvals_[index_neibor]) < smooth_threshold_) &&
                (fabs(particles_[index_neibor].pos_[2] - heightvals_[index_neibor]) < height_threshold_)) {
                Eigen::Vector3d offsetVec(0, 0, heightvals_[index_neibor] - particles_[index_neibor].pos_[2]);
                particles_[index_neibor].OffsetPos(offsetVec);
                particles_[index_neibor].MakeUnmovable();

                if (visited[neibors[index][i]] == false) {
                    que.push(neibors[index][i]);
                    visited[neibors[index][i]] = true;
                }
            }
        }
    }
}

void Cloth::GetCloth(PointCloud& cloth)
{
    cloth.clear();

    for (std::size_t i = 0; i < particles_.size(); i++)
    {
        Point temp;
        temp.x = particles_[i].pos_[0];
        temp.y = particles_[i].pos_[1];
        temp.z = particles_[i].pos_[2];

        cloth.push_back(temp);
    }
}

void Cloth::GetFilterResult(PointCloud& ground, PointCloud& nonground)
{
    ground.clear();
    nonground.clear();

    for (size_t i = 0; i < cloud_.size(); i++)
    {
        Point p = cloud_[i];

        double pc_x = p.x;
        double pc_y = p.y;

        double deltaX = pc_x - origin_[0];
        double deltaY = pc_y - origin_[1];

        int col0 = int(deltaX / cloth_resolution_);
        int row0 = int(deltaY / cloth_resolution_);
        int col1 = col0 + 1;
        int row1 = row0;
        int col2 = col0 + 1;
        int row2 = row0 + 1;
        int col3 = col0;
        int row3 = row0 + 1;

        double subdeltaX = (deltaX - col0 * cloth_resolution_) / cloth_resolution_;
        double subdeltaY = (deltaY - row0 * cloth_resolution_) / cloth_resolution_;

        double fxy
            = GetParticle(col0, row0)->pos_[2] * (1 - subdeltaX) * (1 - subdeltaY) +
              GetParticle(col3, row3)->pos_[2] * (1 - subdeltaX) * subdeltaY +
              GetParticle(col2, row2)->pos_[2] * subdeltaX * subdeltaY +
              GetParticle(col1, row1)->pos_[2] * subdeltaX * (1 - subdeltaY);
        double height_var = fxy - p.z;

        if (std::fabs(height_var) < class_threshold_)
            ground.push_back(p);
        else
            nonground.push_back(p);
    }
}

} // end namespace ground_filter