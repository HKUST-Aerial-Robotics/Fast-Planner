#include <iostream>
#include <fstream>
#include <vector>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <dynamic_reconfigure/server.h>
#include <cloud_banchmark/cloud_banchmarkConfig.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <eigen3/Eigen/Dense>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "depth_render.cuh"

using namespace cv;
using namespace std;
using namespace Eigen;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::TransformStamped> approx_policy;

int *depth_hostptr;
cv::Mat depth_mat;

int width, height;
cv::Mat cv_K, cv_D;
double fx,fy,cx,cy;
cv::Mat undist_map1, undist_map2;
bool is_distorted(false);

DepthRender depthrender;
ros::Publisher pub_depth;
ros::Publisher pub_color;
ros::Publisher pub_posedimage;

Matrix4d vicon2body;
Matrix4d cam02body;
Matrix4d cam2world;
Matrix4d vicon2leica;

ros::Time receive_stamp;

cv::Mat undistorted_image;

struct PoseInfo
{
  Matrix4d pose;
  double time;
};

vector<PoseInfo> gt_pose_vect;

void render_currentpose();

bool read_pose(fstream &file)
{
  int count = 0;
  bool good = true;
  double para[16];
  while(good)
  {
    count ++;
    for(int i = 0; i < 8 && good; i++)
    {
      good = good && file >> para[i];
    }
    if(good)
    {
      Eigen::Vector3d request_position;
      Eigen::Quaterniond request_pose;
      request_position.x() = para[1];
      request_position.y() = para[2];
      request_position.z() = para[3];
      request_pose.w() = para[4];
      request_pose.x() = para[5];
      request_pose.y() = para[6];
      request_pose.z() = para[7];
  
      PoseInfo info;
      info.time = para[0];
      info.pose = Matrix4d::Identity();
      info.pose.block<3,3>(0,0) = request_pose.toRotationMatrix();
      info.pose(0,3) = para[1];
      info.pose(1,3) = para[2];
      info.pose(2,3) = para[3];
      gt_pose_vect.push_back(info);
    }
  }
  printf("we have %d poses.\n", count);
  return true;
}

vector<cv::Point3f> pts_3;
vector<cv::Point2f> pts_2;
void imageBackFunc(int event, int x, int y, int flags, void* userdata)
{
  if( event == EVENT_LBUTTONDOWN )
  {
    cout << "image is clicked - position (" << x << ", " << y << ")" << endl;
    pts_2.push_back(cv::Point2f(x,y));
  }
}
void depthBackFunc(int event, int x, int y, int flags, void* userdata)
{
  if( event == EVENT_LBUTTONDOWN )
  {
    cout << "depth is clicked - position (" << x << ", " << y << ")" << endl;
    double depth = depth_mat.at<float>(y,x);
    double space_x = (x - cx) * depth / fx;
    double space_y = (y - cy) * depth / fy;
    double space_z = depth;
    pts_3.push_back(cv::Point3f(space_x, space_y, space_z));
  }
}

void solve_pnp()
{
//   translation : 
//   0.994976 -0.0431638  0.0903361 -0.0338185
//  0.0444475   0.998937  -0.012246  0.0541652
// -0.0897114  0.0161996   0.995836  0.0384018
//          0          0          0          1
  printf("we have %d pair points.\n", pts_3.size());
  if(pts_3.size() < 5 || pts_2.size() < 5)
  {
    return;
  }
  if(pts_3.size() != pts_2.size())
  {
    printf("error, not equal!\n");
    return;
  }

  cv::Mat r, rvec, t;
  cv::solvePnP(pts_3, pts_2, cv_K, cv::Mat::zeros(4,1,CV_32FC1), rvec, t);
  cv::Rodrigues(rvec, r);
  Matrix3d R_ref;
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
      {
          R_ref(i,j) = r.at<double>(i, j);
      }
  Matrix4d pnp_result = Matrix4d::Identity();
  pnp_result.block<3,3>(0,0) = R_ref;
  pnp_result(0,3) = t.at<double>(0, 0);
  pnp_result(1,3) = t.at<double>(1, 0);
  pnp_result(2,3) = t.at<double>(2, 0);

  vicon2leica = pnp_result.inverse();
  cout << "translation : " << endl << pnp_result << endl;
}

void image_pose_callback(
    const sensor_msgs::ImageConstPtr &image_input,
    const geometry_msgs::TransformStampedConstPtr &pose_input)
{
  //time diff
  double time_diff = fabs(image_input->header.stamp.toSec() - pose_input->header.stamp.toSec()) * 1000.0;
  printf("time diff is %lf ms.\n", time_diff);

  //pose
  Matrix4d Pose_receive = Matrix4d::Identity();

  //using vicon
  // Eigen::Vector3d request_position;
  // Eigen::Quaterniond request_pose;
  // request_position.x() = pose_input->transform.translation.x;
  // request_position.y() = pose_input->transform.translation.y;
  // request_position.z() = pose_input->transform.translation.z;
  // request_pose.x() = pose_input->transform.rotation.x;
  // request_pose.y() = pose_input->transform.rotation.y;
  // request_pose.z() = pose_input->transform.rotation.z;
  // request_pose.w() = pose_input->transform.rotation.w;
  // Pose_receive.block<3,3>(0,0) = request_pose.toRotationMatrix();
  // Pose_receive(0,3) = request_position(0);
  // Pose_receive(1,3) = request_position(1);
  // Pose_receive(2,3) = request_position(2);

  //using ground truth
  double image_time = image_input->header.stamp.toSec();
  double min_time_diff = 999.9;
  int min_time_index = 0;
  for(int i = 1; i < gt_pose_vect.size(); i++)
  {
    double time_diff = fabs(image_time - gt_pose_vect[i].time);
    if(time_diff < min_time_diff)
    {
      min_time_diff = time_diff;
      min_time_index = i;
    }
  }
  printf("min time diff index %d, with diff time %lf ms.\n", min_time_index, min_time_diff*1000.0f);
  Pose_receive = gt_pose_vect[min_time_index].pose;

  //convert to body pose
  // Matrix4d body_pose = Pose_receive * vicon2body.inverse();
  Matrix4d body_pose = Pose_receive;

  //convert to cam pose
  cam2world = body_pose * cam02body * vicon2leica;

  receive_stamp = pose_input->header.stamp;

  //image
  cv_bridge::CvImageConstPtr cv_img_ptr = cv_bridge::toCvShare(image_input, sensor_msgs::image_encodings::MONO8);
  cv::Mat img_8uC1 = cv_img_ptr->image;
  undistorted_image.create(height, width, CV_8UC1);
  if(is_distorted)
  {
    cv::remap(img_8uC1, undistorted_image, undist_map1, undist_map2, CV_INTER_LINEAR);
  }
  else
    undistorted_image = img_8uC1;

  render_currentpose();
}

void render_currentpose()
{
  solve_pnp();

  double this_time = ros::Time::now().toSec();

  Matrix4d cam_pose = cam2world.inverse();

  depthrender.render_pose(cam_pose, depth_hostptr);

  depth_mat = cv::Mat::zeros(height, width, CV_32FC1);
  double min = 0.5;
  double max = 1.0f;
  for(int i = 0; i < height; i++)
  	for(int j = 0; j < width; j++)
  	{
  		float depth = (float)depth_hostptr[i * width + j] / 1000.0f;
  		depth = depth < 500.0f ? depth : 0;
  		max = depth > max ? depth : max;
  		depth_mat.at<float>(i,j) = depth;
  	}
  ROS_INFO("render cost %lf ms.", (ros::Time::now().toSec() - this_time) * 1000.0f);
  printf("max_depth %lf.\n", max);

  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = receive_stamp;
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  out_msg.image = depth_mat.clone();
  pub_depth.publish(out_msg.toImageMsg());

  cv::Mat adjMap;
  depth_mat.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min);
  cv::Mat falseColorsMap;
  cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_RAINBOW);
  cv::Mat bgr_image;
  cv::cvtColor(undistorted_image, bgr_image, cv::COLOR_GRAY2BGR);
  cv::addWeighted(bgr_image, 0.2, falseColorsMap, 0.8, 0.0, falseColorsMap);
  cv_bridge::CvImage cv_image_colored;
  cv_image_colored.header.frame_id = "depthmap";
  cv_image_colored.encoding = sensor_msgs::image_encodings::BGR8;
  cv_image_colored.image = falseColorsMap;
  pub_color.publish(cv_image_colored.toImageMsg());

  cv::imshow("bluefox_image", bgr_image);
  cv::imshow("depth_image", adjMap);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_banchmark");
  ros::NodeHandle nh("~");

  nh.getParam("cam_width", width);
  nh.getParam("cam_height", height);
  nh.getParam("cam_fx", fx);
  nh.getParam("cam_fy", fy);
  nh.getParam("cam_cx", cx);
  nh.getParam("cam_cy", cy);

  depthrender.set_para(fx, fy, cx, cy, width, height);

  cv_K = (cv::Mat_<float>(3, 3) << fx, 0.0f, cx, 0.0f, fy, cy, 0.0f, 0.0f, 1.0f);
  if(nh.hasParam("cam_k1") &&
     nh.hasParam("cam_k2") &&
     nh.hasParam("cam_r1") &&
     nh.hasParam("cam_r2") )
  {
    float k1, k2, r1, r2;
    nh.getParam("cam_k1", k1);
    nh.getParam("cam_k2", k2);
    nh.getParam("cam_r1", r1);
    nh.getParam("cam_r2", r2);
    cv_D = (cv::Mat_<float>(1, 4) << k1, k2, r1, r2);
    cv::initUndistortRectifyMap(
        cv_K,
        cv_D,
        cv::Mat_<double>::eye(3,3),
        cv_K,
        cv::Size(width, height),
        CV_16SC2,
        undist_map1, undist_map2);
    is_distorted = true;
  }
  if(is_distorted)
    printf("need to rectify.\n");
  else
    printf("do not need to rectify.\n");

  vicon2body << 0.33638, -0.01749,  0.94156,  0.06901,
                -0.02078, -0.99972, -0.01114, -0.02781,
                0.94150, -0.01582, -0.33665, -0.12395,
                0.0,      0.0,      0.0,      1.0;
  cam02body <<  0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
                0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
                -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
                0.0, 0.0, 0.0, 1.0;
  cam2world = Matrix4d::Identity();

  string cloud_path;
  nh.getParam("cloud_path", cloud_path);
  printf("cloud file %s\n", cloud_path.c_str());
  std::fstream data_file;
  data_file.open(cloud_path.c_str(), ios::in);
  vector<float> cloud_data;
  double x, y, z, i, r, g, b;
	while(data_file >> x >> y >> z >> i >> r >> g >> b)
	{
		cloud_data.push_back(x);
		cloud_data.push_back(y);
		cloud_data.push_back(z);
	}
  data_file.close();
  printf("has points %d.\n", cloud_data.size() / 3 );


  string groundtruth_path = string("/home/denny/Downloads/wkx_bag/data.txt");
  std::fstream gt_file;
  gt_file.open(groundtruth_path.c_str(), ios::in);
  read_pose(gt_file);
  gt_file.close();

  //pass cloud_data to depth render
  depthrender.set_data(cloud_data);
  depth_hostptr = (int*) malloc(width * height * sizeof(int));

  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/cam0/image_raw", 30);
  message_filters::Subscriber<geometry_msgs::TransformStamped> pose_sub(nh, "/vicon/firefly_sbx/firefly_sbx", 30);
  message_filters::Synchronizer<approx_policy> sync2(approx_policy(100), image_sub, pose_sub);
  sync2.registerCallback(boost::bind(image_pose_callback, _1, _2));

  //publisher depth image and color image
  pub_depth = nh.advertise<sensor_msgs::Image>("depth",1000);
  pub_color = nh.advertise<sensor_msgs::Image>("colordepth",1000);
  // pub_posedimage = nh.advertise<sensor_msgs::Image>("posedimage",1000);

  undistorted_image.create(height, width, CV_8UC1);

  cv::namedWindow("bluefox_image",1);
  cv::namedWindow("depth_image",1);
  setMouseCallback("bluefox_image", imageBackFunc, NULL);
  setMouseCallback("depth_image", depthBackFunc, NULL);
  vicon2leica = Matrix4d::Identity();

  while(ros::ok())
  {
    ros::spinOnce();
    cv::waitKey(30);
  }
}