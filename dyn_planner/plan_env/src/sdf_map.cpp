#include "plan_env/sdf_map.h"

namespace dyn_planner
{
SDFMap::SDFMap(Eigen::Vector3d ori, double resolution, Eigen::Vector3d size)
{
  this->origin_ = ori;
  this->resolution_sdf_ = resolution;
  this->resolution_inv_ = 1 / resolution_sdf_;
  this->map_size_ = size;
  for (int i = 0; i < 3; ++i)
    grid_size_(i) = ceil(map_size_(i) / resolution_sdf_);
  // cout << "grid num:" << grid_size_.transpose() << endl;
  min_range_ = origin_;
  max_range_ = origin_ + map_size_;
  min_vec_ = Eigen::Vector3i::Zero();
  max_vec_ = grid_size_ - Eigen::Vector3i::Ones();

  // initialize size of buffer
  occupancy_buffer_.resize(grid_size_(0) * grid_size_(1) * grid_size_(2));
  distance_buffer_.resize(grid_size_(0) * grid_size_(1) * grid_size_(2));
  tmp_buffer1_.resize(grid_size_(0) * grid_size_(1) * grid_size_(2));
  tmp_buffer2_.resize(grid_size_(0) * grid_size_(1) * grid_size_(2));

  fill(distance_buffer_.begin(), distance_buffer_.end(), 10000);
  fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), 0.0);
}

void SDFMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{
  min_pos(0) = max(min_pos(0), min_range_(0));
  min_pos(1) = max(min_pos(1), min_range_(1));
  min_pos(2) = max(min_pos(2), min_range_(2));

  max_pos(0) = min(max_pos(0), max_range_(0));
  max_pos(1) = min(max_pos(1), max_range_(1));
  max_pos(2) = min(max_pos(2), max_range_(2));

  Eigen::Vector3i min_id, max_id;

  posToIndex(min_pos, min_id);
  posToIndex(max_pos - Eigen::Vector3d(resolution_sdf_ / 2, resolution_sdf_ / 2, resolution_sdf_ / 2), max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        occupancy_buffer_[x * grid_size_(1) * grid_size_(2) + y * grid_size_(2) + z] = 0.0;
        distance_buffer_[x * grid_size_(1) * grid_size_(2) + y * grid_size_(2) + z] = 10000;
      }
}

bool SDFMap::isInMap(Eigen::Vector3d pos)
{
  if (pos(0) < min_range_(0) + 1e-4 || pos(1) < min_range_(1) + 1e-4 || pos(2) < min_range_(2) + 1e-4)
  {
    // cout << "less than min range!" << endl;
    return false;
  }

  if (pos(0) > max_range_(0) - 1e-4 || pos(1) > max_range_(1) - 1e-4 || pos(2) > max_range_(2) - 1e-4)
  {
    // cout << "larger than max range!" << endl;
    return false;
  }

  return true;
}

void SDFMap::posToIndex(Eigen::Vector3d pos, Eigen::Vector3i& id)
{
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - origin_(i)) * resolution_inv_);
}

void SDFMap::indexToPos(Eigen::Vector3i id, Eigen::Vector3d& pos)
{
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * resolution_sdf_ + origin_(i);
}

void SDFMap::setOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
  {
    cout << "occ value error!" << endl;
    return;
  }

  if (!isInMap(pos))
    return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  // (x, y, z) -> x*ny*nz + y*nz + z
  // cout << "..."
  //      << id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2)
  //      << endl;
  // cout << "..." << occupancy_buffer_.size() << endl;
  occupancy_buffer_[id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2)] = occ;
}

int SDFMap::getOccupancy(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  // (x, y, z) -> x*ny*nz + y*nz + z
  return occupancy_buffer_[id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2)];
}

int SDFMap::getOccupancy(Eigen::Vector3i id)
{
  if (id(0) < 0 || id(0) >= grid_size_(0) || id(1) < 0 || id(1) >= grid_size_(1) || id(2) < 0 || id(2) >= grid_size_(2))
    return -1;

  // (x, y, z) -> x*ny*nz + y*nz + z
  return occupancy_buffer_[id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2)];
}

void SDFMap::getOccupancyMarker(visualization_msgs::Marker& m, int id, Eigen::Vector4d color)
{
  m.header.frame_id = "world";
  m.id = id;
  m.type = visualization_msgs::Marker::CUBE_LIST;
  m.action = visualization_msgs::Marker::MODIFY;
  m.scale.x = resolution_sdf_ * 0.9;
  m.scale.y = resolution_sdf_ * 0.9;
  m.scale.z = resolution_sdf_ * 0.9;
  m.color.a = color(3);
  m.color.r = color(0);
  m.color.g = color(1);
  m.color.b = color(2);

  // iterate the map
  for (int x = 0; x < grid_size_(0); ++x)
    for (int y = 0; y < grid_size_(1); ++y)
      for (int z = 0; z < grid_size_(2); ++z)
      {
        if (1 != occupancy_buffer_[x * grid_size_(1) * grid_size_(2) + y * grid_size_(2) + z])
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);

        geometry_msgs::Point p;
        p.x = pos(0);
        p.y = pos(1);
        p.z = pos(2);
        m.points.push_back(p);
      }
}

double SDFMap::getDistance(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  // (x, y, z) -> x*ny*nz + y*nz + z
  return distance_buffer_[id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2)];
}

double SDFMap::getDistance(Eigen::Vector3i id)
{
  id(0) = max(min(id(0), grid_size_(0) - 1), 0);
  id(1) = max(min(id(1), grid_size_(1) - 1), 0);
  id(2) = max(min(id(2), grid_size_(2) - 1), 0);

  // (x, y, z) -> x*ny*nz + y*nz + z
  return distance_buffer_[id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2)];
}

double SDFMap::getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d& grad)
{
  if (!isInMap(pos))
    return -1;

  /* use trilinear interpolation */
  Eigen::Vector3d pos_m = pos - 0.5 * resolution_sdf_ * Eigen::Vector3d::Ones();

  Eigen::Vector3i idx;
  posToIndex(pos_m, idx);

  Eigen::Vector3d idx_pos, diff;
  indexToPos(idx, idx_pos);

  diff = (pos - idx_pos) * resolution_inv_;

  double values[2][2][2];
  for (int x = 0; x < 2; x++)
  {
    for (int y = 0; y < 2; y++)
    {
      for (int z = 0; z < 2; z++)
      {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        values[x][y][z] = getDistance(current_idx);
      }
    }
  }

  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];

  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;

  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  grad[2] = (v1 - v0) * resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);

  grad[0] *= resolution_inv_;

  return dist;
}

double SDFMap::getDistTrilinear(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return -1;

  /* use trilinear interpolation */
  Eigen::Vector3d pos_m = pos - 0.5 * resolution_sdf_ * Eigen::Vector3d::Ones();

  Eigen::Vector3i idx;
  posToIndex(pos_m, idx);

  Eigen::Vector3d idx_pos, diff;
  indexToPos(idx, idx_pos);

  diff = (pos - idx_pos) * resolution_inv_;

  double values[2][2][2];
  for (int x = 0; x < 2; x++)
  {
    for (int y = 0; y < 2; y++)
    {
      for (int z = 0; z < 2; z++)
      {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        values[x][y][z] = getDistance(current_idx);
      }
    }
  }

  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];

  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;

  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  return dist;
}

void SDFMap::setUpdateRange(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{
  /* chou gou shi! */
  // if (!isInMap(min_pos)) min_pos = min_range_;
  // if (!isInMap(max_pos)) max_pos = max_range_;
  min_pos(0) = max(min_pos(0), min_range_(0));
  min_pos(1) = max(min_pos(1), min_range_(1));
  min_pos(2) = max(min_pos(2), min_range_(2));

  max_pos(0) = min(max_pos(0), max_range_(0));
  max_pos(1) = min(max_pos(1), max_range_(1));
  max_pos(2) = min(max_pos(2), max_range_(2));

  posToIndex(min_pos, min_vec_);
  posToIndex(max_pos - Eigen::Vector3d(resolution_sdf_ / 2, resolution_sdf_ / 2, resolution_sdf_ / 2), max_vec_);
}

template <typename F_get_val, typename F_set_val>
void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim)
{
  int v[grid_size_(dim)];
  double z[grid_size_(dim) + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++)
  {
    k++;
    double s;

    do
    {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);

    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++)
  {
    while (z[k + 1] < q)
      k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void SDFMap::updateESDF3d(bool neg)
{
  for (int x = min_vec_[0]; x <= max_vec_[0]; x++)
  {
    for (int y = min_vec_[1]; y <= max_vec_[1]; y++)
    {
      fillESDF(
          [&](int z) {
            return occupancy_buffer_[x * grid_size_(1) * grid_size_(2) + y * grid_size_(2) + z] == 1 ?
                       0 :
                       std::numeric_limits<double>::max();
          },
          [&](int z, double val) { tmp_buffer1_[x * grid_size_(1) * grid_size_(2) + y * grid_size_(2) + z] = val; },
          min_vec_[2], max_vec_[2], 2);
    }
  }

  for (int x = min_vec_[0]; x <= max_vec_[0]; x++)
  {
    for (int z = min_vec_[2]; z <= max_vec_[2]; z++)
    {
      fillESDF(
          [&](int y) {
            // cout << "get xyz:" << x << ", " << y << ", " << z << endl;
            return tmp_buffer1_[x * grid_size_(1) * grid_size_(2) + y * grid_size_(2) + z];
          },
          [&](int y, double val) {
            // cout << "set xyz:" << x << ", " << y << ", " << z << endl;
            // cout << "index:" << x * grid_size_(1) * grid_size_(2) + y *
            // grid_size_(2) + z << endl; cout << "buffer length:" <<
            // tmp_buffer2_.size() << endl;
            tmp_buffer2_[x * grid_size_(1) * grid_size_(2) + y * grid_size_(2) + z] = val;
          },
          min_vec_[1], max_vec_[1], 1);
    }
  }

  for (int y = min_vec_[1]; y <= max_vec_[1]; y++)
  {
    for (int z = min_vec_[2]; z <= max_vec_[2]; z++)
    {
      fillESDF([&](int x) { return tmp_buffer2_[x * grid_size_(1) * grid_size_(2) + y * grid_size_(2) + z]; },
               [&](int x, double val) {
                 distance_buffer_[x * grid_size_(1) * grid_size_(2) + y * grid_size_(2) + z] =
                     resolution_sdf_ * std::sqrt(val);
               },
               min_vec_[0], max_vec_[0], 0);
    }
  }

  if (!neg)
  {
    min_vec_ = Eigen::Vector3i::Zero();
    max_vec_ = grid_size_ - Eigen::Vector3i::Ones();
    return;
  }

  /* ============================== negative distance field ============================== */
  tmp_buffer1_.clear();
  tmp_buffer2_.clear();

  ros::Time t1, t2;

  for (int x = min_vec_[0]; x <= max_vec_[0]; x++)
  {
    for (int y = min_vec_[1]; y <= max_vec_[1]; y++)
    {
      fillESDF(
          [&](int z) {
            return occupancy_buffer_[x * grid_size_(1) * grid_size_(2) + y * grid_size_(2) + z] == 0 ?
                       0 :
                       std::numeric_limits<double>::max();
          },
          [&](int z, double val) { tmp_buffer1_[x * grid_size_(1) * grid_size_(2) + y * grid_size_(2) + z] = val; },
          min_vec_[2], max_vec_[2], 2);
    }
  }

  for (int x = min_vec_[0]; x <= max_vec_[0]; x++)
  {
    for (int z = min_vec_[2]; z <= max_vec_[2]; z++)
    {
      fillESDF(
          [&](int y) { return tmp_buffer1_[x * grid_size_(1) * grid_size_(2) + y * grid_size_(2) + z]; },
          [&](int y, double val) { tmp_buffer2_[x * grid_size_(1) * grid_size_(2) + y * grid_size_(2) + z] = val; },
          min_vec_[1], max_vec_[1], 1);
    }
  }

  for (int y = min_vec_[1]; y <= max_vec_[1]; y++)
  {
    for (int z = min_vec_[2]; z <= max_vec_[2]; z++)
    {
      fillESDF([&](int x) { return tmp_buffer2_[x * grid_size_(1) * grid_size_(2) + y * grid_size_(2) + z]; },
               [&](int x, double val) {
                 distance_buffer_neg_[x * grid_size_(1) * grid_size_(2) + y * grid_size_(2) + z] =
                     resolution_sdf_ * std::sqrt(val);
               },
               min_vec_[0], max_vec_[0], 0);
    }
  }

  /* ========== combine pos and neg DT ========== */
  for (int x = min_vec_(0); x <= max_vec_(0); ++x)
    for (int y = min_vec_(1); y <= max_vec_(1); ++y)
      for (int z = min_vec_(2); z <= max_vec_(2); ++z)
      {
        int idx = x * grid_size_(1) * grid_size_(2) + y * grid_size_(2) + z;

        if (distance_buffer_neg_[idx] > 0.0)
          distance_buffer_[idx] = distance_buffer_[idx] - distance_buffer_neg_[idx] + resolution_sdf_;
      }

  min_vec_ = Eigen::Vector3i::Zero();
  max_vec_ = grid_size_ - Eigen::Vector3i::Ones();
}

void SDFMap::getESDFMarker(vector<visualization_msgs::Marker>& markers, int id, Eigen::Vector3d color)
{
  double max_dist = getMaxDistance();

  // get marker in several distance level
  const int level = ceil(max_dist * resolution_inv_);

  for (int i = 0; i < level; ++i)
  {
    visualization_msgs::Marker m;
    m.header.frame_id = "world";
    m.id = i + level * id;
    m.type = visualization_msgs::Marker::CUBE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = resolution_sdf_ * 0.9;
    m.scale.y = resolution_sdf_ * 0.9;
    m.scale.z = resolution_sdf_ * 0.9;
    m.color.r = color(0);
    m.color.g = color(1);
    m.color.b = color(2);

    // transparency and distance conversion
    double min_a = 0.05, max_a = 0.25;
    double da = (max_a - min_a) / (level - 1);
    m.color.a = max_a - da * i;
    // cout << "alpha:" << m.color.a << endl;

    // distance level
    double delta_d = max_dist / level;
    double min_d = i * delta_d - 1e-3;
    double max_d = (i + 1) * delta_d - 1e-3;

    // iterate the map
    for (int x = 0; x < grid_size_(0); ++x)
      for (int y = 0; y < grid_size_(1); ++y)
        for (int z = 0; z < grid_size_(2) - 15; ++z)
        {
          double dist = distance_buffer_[x * grid_size_(1) * grid_size_(2) + y * grid_size_(2) + z];
          bool in_range = dist < max_d && dist >= min_d;
          if (!in_range)
            continue;

          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);

          geometry_msgs::Point p;
          p.x = pos(0);
          p.y = pos(1);
          p.z = pos(2);
          m.points.push_back(p);
        }
    markers.push_back(m);
  }
}

double SDFMap::getMaxDistance()
{
  // get the max distance
  double max_dist = -1;
  for (int i = 0; i < int(distance_buffer_.size()); ++i)
  {
    if (distance_buffer_[i] > max_dist)
      max_dist = distance_buffer_[i];
  }
  // cout << "Max distance is:" << max_dist << endl;
  return max_dist;
}

void SDFMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  /* need odom_ for center radius sensing */
  if (!have_odom_)
  {
    cout << "no odom_" << endl;
    return;
  }

  pcl::fromROSMsg(*msg, latest_cloud_);

  // if ((int)latest_cloud_.points.size() == 0) return;

  new_map_ = true;
}

void SDFMap::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  if (msg->child_frame_id == "X" || msg->child_frame_id == "O")
    return;

  odom_ = *msg;
  odom_.header.frame_id = "world";
  have_odom_ = true;
}

void SDFMap::updateCallback(const ros::TimerEvent& e)
{
  if (!new_map_)
  {
    // cout << "no new map." << endl;
    return;
  }
  map_valid_ = true;
  new_map_ = false;

  if (latest_cloud_.points.size() == 0)
    return;

  Eigen::Vector3d center(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);
  if (isnan(center(0)) || isnan(center(1)) || isnan(center(2)))
    return;

  /* ---------- inflate cloud and insert to SDFMap ---------- */
  Eigen::Vector3d disp(update_range_, update_range_, update_range_ / 2.0);
  this->resetBuffer(center - disp, center + disp);

  cloud_inflate_vis_.clear();
  pcl::PointXYZ pt, pt_inf;
  Eigen::Vector3d p3d, p3d_inf;
  const int ifn = ceil(inflate_ * resolution_inv_);

  for (size_t i = 0; i < latest_cloud_.points.size(); ++i)
  {
    pt = latest_cloud_.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

    /* point inside update range */
    if ((center - p3d).norm() < update_range_)
    {
      /* inflate the point */
      for (int x = -ifn; x <= ifn; ++x)
        for (int y = -ifn; y <= ifn; ++y)
          for (int z = -ifn; z <= ifn; ++z)
          {
            p3d_inf(0) = pt_inf.x = pt.x + x * resolution_sdf_;
            p3d_inf(1) = pt_inf.y = pt.y + y * resolution_sdf_;
            p3d_inf(2) = pt_inf.z = pt.z + 0.5 * z * resolution_sdf_;

            this->setOccupancy(p3d_inf);

            if (pt_inf.z < 2.0)
              cloud_inflate_vis_.push_back(pt_inf);
          }
    }
  }
  cloud_inflate_vis_.width = cloud_inflate_vis_.points.size();
  cloud_inflate_vis_.height = 1;
  cloud_inflate_vis_.is_dense = true;
  cloud_inflate_vis_.header.frame_id = "world";
  cloud_inflate_vis_.header.seq = latest_cloud_.header.seq;
  cloud_inflate_vis_.header.stamp = latest_cloud_.header.stamp;
  sensor_msgs::PointCloud2 map_inflate_vis;
  pcl::toROSMsg(cloud_inflate_vis_, map_inflate_vis);

  inflate_cloud_pub_.publish(map_inflate_vis);

  /* ---------- add ceil ---------- */
  if (ceil_height_ > 0.0)
  {
    for (double cx = center(0) - update_range_; cx <= center(0) + update_range_; cx += resolution_sdf_)
      for (double cy = center(1) - update_range_; cy <= center(1) + update_range_; cy += resolution_sdf_)
      {
        this->setOccupancy(Eigen::Vector3d(cx, cy, ceil_height_));
      }
  }

  /* ---------- update ESDF ---------- */
  this->setUpdateRange(center - disp, center + disp);
  this->updateESDF3d(true);
}

void SDFMap::init(ros::NodeHandle& nh)
{
  node_ = nh;

  /* ---------- param ---------- */
  node_.param("sdf_map/origin_x", origin_(0), -20.0);
  node_.param("sdf_map/origin_y", origin_(1), -20.0);
  node_.param("sdf_map/origin_z", origin_(2), 0.0);

  node_.param("sdf_map/map_size_x", map_size_(0), 40.0);
  node_.param("sdf_map/map_size_y", map_size_(1), 40.0);
  node_.param("sdf_map/map_size_z", map_size_(2), 5.0);

  node_.param("sdf_map/resolution_sdf", resolution_sdf_, 0.2);
  node_.param("sdf_map/ceil_height", ceil_height_, 2.0);
  node_.param("sdf_map/update_rate", update_rate_, 10.0);
  node_.param("sdf_map/update_range", update_range_, 5.0);
  node_.param("sdf_map/inflate", inflate_, 0.2);
  node_.param("sdf_map/radius_ignore", radius_ignore_, 0.2);

  cout << "origin_: " << origin_.transpose() << endl;
  cout << "map size: " << map_size_.transpose() << endl;
  cout << "resolution: " << resolution_sdf_ << endl;

  /* ---------- sub and pub ---------- */
  odom_sub_ = node_.subscribe<nav_msgs::Odometry>("/odom_world", 10, &SDFMap::odomCallback, this);
  cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1, &SDFMap::cloudCallback, this);

  update_timer_ = node_.createTimer(ros::Duration(0.1), &SDFMap::updateCallback, this);

  inflate_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/inflate_cloud", 1);

  /* ---------- setting ---------- */
  have_odom_ = false;
  new_map_ = false;
  map_valid_ = false;

  resolution_inv_ = 1 / resolution_sdf_;
  for (int i = 0; i < 3; ++i)
    grid_size_(i) = ceil(map_size_(i) / resolution_sdf_);
  // SETY << "grid num:" << grid_size_.transpose() << REC;
  min_range_ = origin_;
  max_range_ = origin_ + map_size_;
  min_vec_ = Eigen::Vector3i::Zero();
  max_vec_ = grid_size_ - Eigen::Vector3i::Ones();

  // initialize size of buffer
  occupancy_buffer_.resize(grid_size_(0) * grid_size_(1) * grid_size_(2));
  distance_buffer_.resize(grid_size_(0) * grid_size_(1) * grid_size_(2));
  distance_buffer_neg_.resize(grid_size_(0) * grid_size_(1) * grid_size_(2));
  tmp_buffer1_.resize(grid_size_(0) * grid_size_(1) * grid_size_(2));
  tmp_buffer2_.resize(grid_size_(0) * grid_size_(1) * grid_size_(2));

  fill(distance_buffer_.begin(), distance_buffer_.end(), 10000);
  fill(distance_buffer_neg_.begin(), distance_buffer_neg_.end(), 10000);
  fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), 0.0);
}

void SDFMap::getInterpolationData(const Eigen::Vector3d& pos, vector<Eigen::Vector3d>& pos_vec, Eigen::Vector3d& diff)
{
  if (!isInMap(pos))
  {
    // cout << "pos invalid for interpolation." << endl;
  }

  /* interpolation position */
  Eigen::Vector3d pos_m = pos - 0.5 * resolution_sdf_ * Eigen::Vector3d::Ones();

  Eigen::Vector3i idx;
  posToIndex(pos_m, idx);

  Eigen::Vector3d idx_pos;
  indexToPos(idx, idx_pos);

  diff = (pos - idx_pos) * resolution_inv_;

  pos_vec.clear();

  for (int x = 0; x < 2; x++)
    for (int y = 0; y < 2; y++)
      for (int z = 0; z < 2; z++)
      {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        Eigen::Vector3d current_pos;
        indexToPos(current_idx, current_pos);
        pos_vec.push_back(current_pos);
      }
}

// SDFMap::
}  // namespace dyn_planner
