#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseArray.h>
#include <laser_geometry/laser_geometry.h>
#include <pose_utils.h>
#include <multi_map_server/MultiOccupancyGrid.h>
#include <multi_map_server/MultiSparseMap3D.h>
#include <multi_map_server/Map2D.h>
#include <multi_map_server/Map3D.h>

using namespace arma;
using namespace std;
#define MAX_MAP_CNT 25

ros::Publisher pub1;
ros::Publisher pub2;

// 2D Map
int maps2dCnt = 0;
Map2D maps2d[MAX_MAP_CNT];
map_server_3d_new::MultiOccupancyGrid grids2d;
// 3D Map
int maps3dCnt = 0;
Map3D maps3d[MAX_MAP_CNT];
// Map origin from UKF
vector<geometry_msgs::Pose> maps_origin;   

void map2d_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  // Update msg and publish
  if (grids2d.maps.size() == 0)
  {
    // Init msg
    grids2d.maps.push_back(*msg);
    maps2dCnt++;
  }
  else if (grids2d.maps.back().info.map_load_time != msg->info.map_load_time)
  {
    // Add Costmap
    nav_msgs::OccupancyGrid m;
    maps2d[maps2dCnt-1].get_map(m);
    mapMatcher.AddCostMap(m);
    // Increase msg size
    grids2d.maps.back().data.clear();
    grids2d.maps.push_back(*msg);
    maps2dCnt++;
  }
  else
  {
    grids2d.maps.back() = *msg;
  }
  pub1.publish(grids2d);
  // Update internval map
  maps2d[maps2dCnt-1].update(*msg);
}

void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  // Get Map Status
  static bool isSLAMValid = false;
  if (msg->intensities[10])
  {
    if (!isSLAMValid)
    {
      maps3dCnt++;
      mapLinks.push_back(zeros<colvec>(3));
    }
    isSLAMValid = true;
  }
  else
  {
    isSLAMValid = false;
    return;
  }
  // Scan cnt
  mapLinks.back()(2)++;
  // Get Current Pose
  colvec pose(6);
  pose(0) = msg->intensities[0];
  pose(1) = msg->intensities[1];
  pose(2) = msg->intensities[2];
  pose(3) = msg->intensities[3];
  pose(4) = msg->intensities[4];
  pose(5) = msg->intensities[5];
  colvec pose2D(3);
  pose2D(0) = msg->intensities[0];
  pose2D(1) = msg->intensities[1];
  pose2D(2) = msg->intensities[3];
  double currFloor  = msg->intensities[7];
  double currLaserH = msg->intensities[8];
  // Horizontal laser scans
  sensor_msgs::LaserScan scan = *msg;
  scan.intensities.clear();
  laser_geometry::LaserProjection projector;
  sensor_msgs::PointCloud scanCloud;
  projector.projectLaser(scan, scanCloud);
  mat scanMat(3, scanCloud.points.size());
  // Get scan in body frame
  for (int k = 0; k < scanCloud.points.size(); k++)
  {
    scanMat(0,k) = scanCloud.points[k].x;
    scanMat(1,k) = scanCloud.points[k].y;
    scanMat(2,k) = 0.215 - 0.09;
  }
  // Downsample Laser scan to map resolution
  double resolution = maps3d[maps3dCnt-1].GetResolution();
  double px = NUM_INF;
  double py = NUM_INF;
  int cnt = 0;
  mat scanMatDown = zeros<mat>(3, scanMat.n_cols);
  for (int k = 0; k < scanMat.n_cols; k++)
  {
    double x = scanMat(0,k);
    double y = scanMat(1,k);
    double dist = (x-px)*(x-px) + (y-py)*(y-py); 
    if (dist > resolution * resolution)
    {
      scanMatDown.col(cnt) = scanMat.col(k);
      px = x;
      py = y;
      cnt++;
    }
  }
  if (cnt > 0)
    scanMat = scanMatDown.cols(0, cnt-1);
  else
    scanMat = scanMatDown.cols(0,cnt);
  // Transform to map local frame
  scanMat = ypr_to_R(pose.rows(3,5)) * scanMat + pose.rows(0,2)*ones<rowvec>(scanMat.n_cols);
  // Update map
  for (int k = 0; k < scanMat.n_cols; k++)
    maps3d[maps3dCnt-1].SetOccupancyFromWorldFrame(scanMat(0,k), scanMat(1,k), scanMat(2,k), PROB_LASER_OCCUPIED);
  // downward facing laser scans
  if (currLaserH > 0)
  {
    colvec pt(3);
    pt(0) = 0;
    pt(1) = 0;
    pt(2) = -currLaserH;
    pt =  ypr_to_R(pose.rows(3,5)) * pt + pose.rows(0,2);
    double resolution = maps3d[maps3dCnt-1].GetResolution();
    for (double x = -0.1; x <= 0.1; x+=resolution)
      for (double y = -0.1; y <= 0.1; y+=resolution)
        maps3d[maps3dCnt-1].SetOccupancyFromWorldFrame(pt(0)+x, pt(1)+y, pt(2), PROB_LASER_OCCUPIED);          
  }
  // Floor Levels
  maps3d[maps3dCnt-1].SetFloor(currFloor);
}

void maps_origin_callback(const geometry_msgs::PoseArray::ConstPtr &msg)
{
  maps_origin = msg->poses;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_map_server_3d");
  ros::NodeHandle n("~");

  ros::Subscriber sub1 = n.subscribe("dmap2d",       100, map2d_callback);
  ros::Subscriber sub2 = n.subscribe("scan",         100, scan_callback);
  ros::Subscriber sub3 = n.subscribe("/maps_origin", 100, maps_origin_callback);
  pub1 = n.advertise<multi_map_server::MultiOccupancyGrid>("dmaps2d", 10, true); 
  pub2 = n.advertise<multi_map_server::MultiSparseMap3D>(  "dmaps3d", 10, true); 

  ros::Rate r(100.0);  
  int cnt = 0;
  while (n.ok())
  {
    cnt++;
    if (cnt > 100)
    {
      cnt = 0;
      map_server_3d_new::MultiSparseMap3D msg;
      msg.maps_origin = maps_origin;
      for (int k = 0; k < maps3dCnt; k++)
      {
        msg.maps_active.push_back((bool)(!mapLinks[k](0)) && mapLinks[k](2) > 50);
        map_server_3d_new::SparseMap3D m;
        maps3d[k].GetSparseMap3DMsg(m);
        m.header.seq = k;
        msg.maps.push_back(m);
      }
      pub2.publish(msg);
    }
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
