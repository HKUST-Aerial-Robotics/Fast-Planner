#ifndef _SDF_MAP_H
#define _SDF_MAP_H

#include <visualization_msgs/Marker.h>
#include <Eigen/Eigen>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>

using namespace std;

namespace dyn_planner
{
class SDFMap
{
private:
  // data are saved in vector
  std::vector<int> occupancy_buffer_;  // 0 is free, 1 is occupied
  std::vector<double> distance_buffer_;
  std::vector<double> distance_buffer_neg_;
  std::vector<double> tmp_buffer1_, tmp_buffer2_;

  // map property
  Eigen::Vector3d min_range_, max_range_;  // map range in pos
  Eigen::Vector3i grid_size_;              // map range in index
  Eigen::Vector3i min_vec_, max_vec_;      // the min and max updated range, unit is 1

  bool isInMap(Eigen::Vector3d pos);
  void posToIndex(Eigen::Vector3d pos, Eigen::Vector3i& id);
  void indexToPos(Eigen::Vector3i id, Eigen::Vector3d& pos);

  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

  /* ---------- parameter ---------- */
  double inflate_, update_range_, radius_ignore_;
  Eigen::Vector3d origin_, map_size_;
  double resolution_sdf_, resolution_inv_;
  double ceil_height_;
  double update_rate_;

  /* ---------- callback ---------- */
  nav_msgs::Odometry odom_;
  bool have_odom_;

  pcl::PointCloud<pcl::PointXYZ> latest_cloud_, cloud_inflate_vis_;
  bool new_map_, map_valid_;

  ros::NodeHandle node_;
  ros::Subscriber odom_sub_, cloud_sub_;
  ros::Publisher inflate_cloud_pub_;
  ros::Timer update_timer_;

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void updateCallback(const ros::TimerEvent& e);

  /* --------------------------------- */

public:
  SDFMap() {}
  SDFMap(Eigen::Vector3d origin, double resolution, Eigen::Vector3d map_size);
  ~SDFMap() {}
  void init(ros::NodeHandle& nh);

  /* get state */
  bool odomValid() { return have_odom_; }
  bool mapValid() { return map_valid_; }
  nav_msgs::Odometry getOdom() { return odom_; }
  void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) { ori = origin_, size = map_size_; }
  double getResolution() { return resolution_sdf_; }
  double getIgnoreRadius() { return radius_ignore_; }
  void getInterpolationData(const Eigen::Vector3d& pos, vector<Eigen::Vector3d>& pos_vec,
                            Eigen::Vector3d& diff);

  // occupancy management
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);
  void setOccupancy(Eigen::Vector3d pos, int occ = 1);
  int getOccupancy(Eigen::Vector3d pos);
  int getOccupancy(Eigen::Vector3i id);
  void getOccupancyMarker(visualization_msgs::Marker& m, int id, Eigen::Vector4d color);

  // distance field management
  double getDistance(Eigen::Vector3d pos);
  double getDistance(Eigen::Vector3i id);
  double getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d& grad);
  double getDistTrilinear(Eigen::Vector3d pos);
  void setUpdateRange(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos);
  void updateESDF3d(bool neg = false);
  void getESDFMarker(vector<visualization_msgs::Marker>& markers, int id, Eigen::Vector3d color);
  double getMaxDistance();

  void publishESDF();

  typedef shared_ptr<SDFMap> Ptr;
};

}  // namespace dyn_planner

#endif