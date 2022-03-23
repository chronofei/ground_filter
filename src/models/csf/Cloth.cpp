#include "ground_filter/models/csf/Cloth.hpp"

namespace ground_filter
{

Cloth::Cloth(const Eigen::Vector2d cloth_size, const double& cloth_resolution,
             const double& acceleration,       const double& time_step,
             const double& smooth_threshold,   const double& height_threshold,
             const int& rigidness)
{
    shift_by_acc_     = acceleration * time_step * time_step;

    cloth_resolution_ = cloth_resolution;

    cloth_origin_     = Eigen::Vector2d( 0.0             - 2* cloth_resolution_,
                                        -cloth_size[1]/2 - 2 * cloth_resolution_);

    cloth_width_      = static_cast<int>(std::floor(cloth_size[0] / cloth_resolution_)) + 2 * 2;
    cloth_height_     = static_cast<int>(std::floor(cloth_size[1] / cloth_resolution_)) + 2 * 2;

    particle_number_  = cloth_width_ * cloth_height_;

    particles_.resize(particle_number_);
    for (int row = 0; row < cloth_height_; row++)
    {
        for (int col = 0; col < cloth_width_; col++)
        {
            particles_[GetIndex1D(col, row)].SetPosition(cloth_origin_ + Eigen::Vector2d(col * cloth_resolution_, row * cloth_resolution_));
            particles_[GetIndex1D(col, row)].SetRasterIndex(Eigen::Vector2i(col, row));
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

    smooth_threshold_ = smooth_threshold;
    height_threshold_ = height_threshold;
    rigidness_        = rigidness;
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
    double lowest_height = std::numeric_limits<double>::max();

    for (size_t i=0; i<cloud.size(); i++)
    {
        auto point = cloud[i];

        int col = int((point.x - cloth_origin_[0]) / cloth_resolution_ + 0.5);
        int row = int((point.y - cloth_origin_[1]) / cloth_resolution_ + 0.5);

        if (col>=0 && col<cloth_width_ && row>=0 && row<cloth_height_)
        {
            auto& particle = particles_[GetIndex1D(col, row)];

            if (point.z < lowest_height)
                lowest_height = point.z;

            particle.lidar_points_.emplace_back(point);

            double pc_2_particle_dist = SquareDistanceXY(Eigen::Vector2d(point.x, point.y), particle.GetPosition());

            if (pc_2_particle_dist < particle.tmp_dist_)
            {
                particle.tmp_dist_             = pc_2_particle_dist;
                particle.nearest_point_height_ = cloud[i].z;
            }
        }
    }

    for (int i = 0; i < particle_number_; i++)
    {
        auto& particle = particles_[i];

        particle.curr_height_ = particle.last_height_ = lowest_height;

        if (particle.nearest_point_height_ < HEIGHT_THRESHOLD + 1.0e-6)
            particles_[i].nearest_point_height_ = FindHeightValByScanline(i);
    }
}

double Cloth::FindHeightValByScanline(int particle)
{
    int xpos = particles_[particle].GetRasterIndex()[0];
    int ypos = particles_[particle].GetRasterIndex()[1];

    for (int i = xpos + 1; i < cloth_width_; i++)
    {
        double crresHeight = particles_[GetIndex1D(i, ypos)].nearest_point_height_;

        if (crresHeight > HEIGHT_THRESHOLD + 1.0e-6)
            return crresHeight;
    }

    for (int i = xpos - 1; i >= 0; i--)
    {
        double crresHeight = particles_[GetIndex1D(i, ypos)].nearest_point_height_;

        if (crresHeight > HEIGHT_THRESHOLD + 1.0e-6)
            return crresHeight;
    }

    for (int j = ypos - 1; j >= 0; j--)
    {
        double crresHeight = particles_[GetIndex1D(xpos, j)].nearest_point_height_;

        if (crresHeight > HEIGHT_THRESHOLD + 1.0e-6)
            return crresHeight;
    }

    for (int j = ypos + 1; j < cloth_height_; j++)
    {
        double crresHeight = particles_[GetIndex1D(xpos, j)].nearest_point_height_;

        if (crresHeight > HEIGHT_THRESHOLD + 1.0e-6)
            return crresHeight;
    }

    return FindHeightValByNeighbor(particle);
}

double Cloth::FindHeightValByNeighbor(int particle)
{
    std::queue<int>  nqueue;
    std::vector<int> pbacklist;

    size_t neiborsize = particles_[particle].GetNeighborSize();

    for (size_t i = 0; i < neiborsize; i++)
    {
        auto neibor = particles_[particle].GetNeighbor(i);
        particles_[neibor].is_visited_ = true;
        nqueue.push(neibor);
    }

    while (!nqueue.empty())
    {
        int pneighbor = nqueue.front();
        nqueue.pop();
        pbacklist.push_back(pneighbor);

        if (particles_[pneighbor].nearest_point_height_ > HEIGHT_THRESHOLD + 1.0e-6)
        {
            for (size_t i = 0; i < pbacklist.size(); i++)
                particles_[pbacklist[i]].is_visited_ = false;

            while (!nqueue.empty())
            {
                int pp = nqueue.front();
                particles_[pp].is_visited_ = false;
                nqueue.pop();
            }

            return particles_[pneighbor].nearest_point_height_;
        }
        else
        {
            int nsize = particles_[pneighbor].GetNeighborSize();

            for (int i = 0; i < nsize; i++)
            {
                int ptmp = particles_[pneighbor].GetNeighbor(i);

                if (!particles_[ptmp].is_visited_)
                {
                    particles_[ptmp].is_visited_ = true;
                    nqueue.push(ptmp);
                }
            }
        }
    }

    return HEIGHT_THRESHOLD;
}

double Cloth::TimeStep()
{
    for (int i = 0; i < particle_number_; i++)
    {
        auto& particle = particles_[i];

        if (particle.movable_)
        {
            double temp = particle.curr_height_;
            particle.curr_height_ = particle.curr_height_ + (particle.curr_height_ - particle.last_height_) * 0.99 + shift_by_acc_;
            particle.last_height_ = temp;
        }
    }

    for (int j = 0; j < particle_number_; j++)
    {
        SatisfyConstraintSelf(j);
    }

    double maxDiff = 0;

    for (int i = 0; i < particle_number_; i++)
    {
        auto& particle = particles_[i];

        if (particle.movable_)
        {
            double diff = fabs(particle.curr_height_ - particle.last_height_);

            if (diff > maxDiff)
                maxDiff = diff;
        }
    }

    return maxDiff;
}

void Cloth::SatisfyConstraintSelf(int particle)
{
    auto& p1 = particles_[particle];

    for (std::size_t i = 0; i < p1.GetNeighborSize(); i++)
    {
        Particle& p2 = particles_[p1.GetNeighbor(i)];

        double diff = p2.curr_height_ - p1.curr_height_;

        if (p1.movable_ && p2.movable_)
        {
            double diff_half = diff * (rigidness_ > 14 ? 0.5 : doubleMove1[rigidness_]);
            p1.curr_height_ += diff_half;
            p2.curr_height_ -= diff_half;
        }
        else if (p1.movable_ && !p2.movable_)
        {
            double diff_half = diff * (rigidness_ > 14 ? 0.5 : doubleMove1[rigidness_]);
            p1.curr_height_ += diff_half;
        }
        else if (!p1.movable_ && p2.movable_)
        {
            double diff_half = diff * (rigidness_ > 14 ? 0.5 : doubleMove1[rigidness_]);
            p2.curr_height_ -= diff_half;
        }
    }
}

void Cloth::TerrCollision()
{
    for (int i = 0; i < particle_number_; i++)
    {
        auto& particle = particles_[i];

        if (particle.curr_height_ > particle.nearest_point_height_)
        {
            particle.curr_height_ = particle.nearest_point_height_;
            particle.movable_     = false;
        }
    }
}

void Cloth::MovableFilter(const int& max_particle_for_postprocess)
{
    for (int x = 0; x < cloth_width_; x++)
    {
        for (int y = 0; y < cloth_height_; y++)
        {
            auto& particle = particles_[GetIndex1D(x,y)];

            if (particle.movable_ && !particle.is_visited_)
            {
                particle.is_visited_ = true;

                std::queue<int> que;
                std::vector<int> connected;
                std::vector<std::vector<int>> neibors;

                int sum   = 1;
                int index = GetIndex1D(x,y);

                connected.push_back(index);
                particle.c_pos_ = 0;

                que.push(index);

                while (!que.empty())
                {
                    auto& ptc_f = particles_[que.front()];
                    que.pop();

                    int cur_x = ptc_f.GetRasterIndex()[0];
                    int cur_y = ptc_f.GetRasterIndex()[1];

                    std::vector<int> neibor;

                    if (cur_x > 0)
                    {
                        int ptc_left_index = GetIndex1D(cur_x - 1, cur_y);
                        auto& ptc_left = particles_[ptc_left_index];

                        if (ptc_left.movable_)
                        {
                            if (!ptc_left.is_visited_)
                            {
                                sum++;
                                ptc_left.is_visited_ = true;
                                connected.push_back(ptc_left_index);
                                que.push(ptc_left_index);
                                neibor.push_back(sum - 1);
                                ptc_left.c_pos_ = sum - 1;
                            }
                            else
                            {
                                neibor.push_back(ptc_left.c_pos_);
                            }
                        }
                    }

                    if (cur_x < cloth_width_ - 1)
                    {
                        int ptc_right_index = GetIndex1D(cur_x+1, cur_y);
                        auto& ptc_right = particles_[ptc_right_index];

                        if (ptc_right.movable_)
                        {
                            if (!ptc_right.is_visited_)
                            {
                                sum++;
                                ptc_right.is_visited_ = true;
                                connected.push_back(ptc_right_index);
                                que.push(ptc_right_index);
                                neibor.push_back(sum - 1);
                                ptc_right.c_pos_ = sum - 1;
                            }
                            else
                            {
                                neibor.push_back(ptc_right.c_pos_);
                            }
                        }
                    }

                    if (cur_y > 0)
                    {
                        int ptc_bottom_index = GetIndex1D(cur_x, cur_y - 1);
                        auto& ptc_bottom = particles_[ptc_bottom_index];

                        if (ptc_bottom.movable_)
                        {
                            if (!ptc_bottom.is_visited_)
                            {
                                sum++;
                                ptc_bottom.is_visited_ = true;
                                connected.push_back(ptc_bottom_index);
                                que.push(ptc_bottom_index);
                                neibor.push_back(sum - 1);
                                ptc_bottom.c_pos_ = sum - 1;
                            }
                            else
                            {
                                neibor.push_back(ptc_bottom.c_pos_);
                            }
                        }
                    }

                    if (cur_y < cloth_height_ - 1)
                    {
                        int ptc_top_index = GetIndex1D(cur_x, cur_y + 1);
                        auto& ptc_top = particles_[ptc_top_index];

                        if (ptc_top.movable_)
                        {
                            if (!ptc_top.is_visited_)
                            {
                                sum++;
                                ptc_top.is_visited_ = true;
                                connected.push_back(ptc_top_index);
                                que.push(ptc_top_index);
                                neibor.push_back(sum - 1);
                                ptc_top.c_pos_ = sum - 1;
                            }
                            else
                            {
                                neibor.push_back(ptc_top.c_pos_);
                            }
                        }
                    }
                    neibors.push_back(neibor);
                }

                if (sum > max_particle_for_postprocess)
                {
                    std::vector<int> edgePoints = FindUnmovablePoint(connected);
                    HandleSlopConnected(edgePoints, connected, neibors);
                }
            }
        }
    }
}

std::vector<int> Cloth::FindUnmovablePoint(const std::vector<int>& connected)
{
    std::vector<int> edgePoints;

    for (std::size_t i = 0; i < connected.size(); i++)
    {
        int index      = connected[i];
        auto& particle = particles_[index];
        int x          = particle.GetRasterIndex()[0];
        int y          = particle.GetRasterIndex()[1];

        if (x > 0)
        {
            int ptc_left_index = GetIndex1D(x-1, y);
            auto& ptc_left     = particles_[ptc_left_index];

            if (!ptc_left.movable_)
            {
                if ((std::fabs(particle.nearest_point_height_ - ptc_left.nearest_point_height_) < smooth_threshold_) &&
                    (std::fabs(particle.nearest_point_height_ - particle.curr_height_         ) < height_threshold_))
                {
                    particle.curr_height_ = particle.nearest_point_height_;
                    particle.movable_     = false;
                    edgePoints.push_back(i);
                    continue;
                }
            }
        }

        if (x < cloth_width_ - 1)
        {
            int ptc_right_index = GetIndex1D(x+1, y);
            auto& ptc_right     = particles_[ptc_right_index];

            if (!ptc_right.movable_)
            {
                if ((std::fabs(particle.nearest_point_height_ - ptc_right.nearest_point_height_) < smooth_threshold_) &&
                    (std::fabs(particle.nearest_point_height_ - particle.curr_height_          ) < height_threshold_))
                {
                    particle.curr_height_ = particle.nearest_point_height_;
                    particle.movable_     = false;
                    edgePoints.push_back(i);
                    continue;
                }
            }
        }

        if (y > 0)
        {
            int ptc_bottom_index = GetIndex1D(x, y-1);
            auto& ptc_bottom     = particles_[ptc_bottom_index];

            if (!ptc_bottom.movable_)
            {
                if ((std::fabs(particle.nearest_point_height_ - ptc_bottom.nearest_point_height_) < smooth_threshold_) &&
                    (std::fabs(particle.nearest_point_height_ - particle.curr_height_           ) < height_threshold_))
                {
                    particle.curr_height_ = particle.nearest_point_height_;
                    particle.movable_     = false;
                    edgePoints.push_back(i);
                    continue;
                }
            }
        }

        if (y < cloth_height_ - 1)
        {
            int ptc_top_index = GetIndex1D(x, y+1);
            auto& ptc_top     = particles_[ptc_top_index];

            if (!ptc_top.movable_)
            {
                if ((std::fabs(particle.nearest_point_height_ - ptc_top.nearest_point_height_) < smooth_threshold_) &&
                    (std::fabs(particle.nearest_point_height_ - particle.curr_height_        ) < height_threshold_))
                {
                    particle.curr_height_ = particle.nearest_point_height_;
                    particle.movable_     = false;
                    edgePoints.push_back(i);
                    continue;
                }
            }
        }
    }

    return edgePoints;
}

void Cloth::HandleSlopConnected(std::vector<int> edgePoints, std::vector<int> connected, std::vector<std::vector<int> > neibors)
{
    std::vector<bool> visited;
    visited.resize(connected.size(), false);

    std::queue<int> que;

    for (std::size_t i = 0; i < edgePoints.size(); i++)
    {
        que.push(edgePoints[i]);
        visited[edgePoints[i]] = true;
    }

    while (!que.empty())
    {
        int index = que.front();
        que.pop();

        int index_center = connected[index];

        for (std::size_t i = 0; i < neibors[index].size(); i++)
        {
            int index_neibor = connected[neibors[index][i]];

            if ((std::fabs(particles_[index_center].nearest_point_height_ - particles_[index_neibor].nearest_point_height_) < smooth_threshold_) &&
                (std::fabs(particles_[index_neibor].curr_height_          - particles_[index_neibor].nearest_point_height_) < height_threshold_))
            {
                particles_[index_neibor].curr_height_ = particles_[index_neibor].nearest_point_height_;
                particles_[index_neibor].movable_     = false;

                if (visited[neibors[index][i]] == false)
                {
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
        auto position = particles_[i].GetPosition3D();
        Point temp;
        temp.x = position[0];
        temp.y = position[1];
        temp.z = position[2];

        cloth.push_back(temp);
    }
}

void Cloth::GetFilterResult(const double& class_threshold, PointCloud& ground, PointCloud& nonground)
{
    ground.clear();
    nonground.clear();

    for (int i = 0; i < particle_number_; i++)
    {
        auto& particle = particles_[i];

        int col = particle.GetRasterIndex()[0];
        int row = particle.GetRasterIndex()[1];

        double h00 = particles_[GetIndex1D(col  , row  )].curr_height_;
        double h10 = particles_[GetIndex1D(col+1, row  )].curr_height_;
        double h11 = particles_[GetIndex1D(col+1, row+1)].curr_height_;
        double h01 = particles_[GetIndex1D(col  , row+1)].curr_height_;

        for (size_t j=0; j<particle.lidar_points_.size(); j++)
        {
            auto point = particle.lidar_points_[j];

            double subdeltaX = (point.x - particle.GetPosition()[0]) / cloth_resolution_;
            double subdeltaY = (point.y - particle.GetPosition()[1]) / cloth_resolution_;

            double fxy = h00 * (1 - subdeltaX) * (1 - subdeltaY) +
                         h01 * (1 - subdeltaX) *      subdeltaY  +
                         h11 *      subdeltaX  *      subdeltaY  +
                         h10 *      subdeltaX  * (1 - subdeltaY);

            double height_var = fxy - point.z;

            if (std::fabs(height_var) < class_threshold)
                ground.push_back(point);
            else
                nonground.push_back(point);
        }
    }
}

} // end namespace ground_filter