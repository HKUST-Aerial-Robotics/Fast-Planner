#include <gtest/gtest.h>
#include <uav_utils/utils.h>

using namespace uav_utils;
using namespace Eigen;

#define DOUBLE_EPS 1.0e-15
#define FLOAT_EPS 1.0e-6

TEST(GeometryUtilsDouble, Rotation) {
    Vector3d v(0.1, 0.2, 0.3);
    ASSERT_TRUE(v.isApprox(R_to_ypr(ypr_to_R(v)),DOUBLE_EPS));
    ASSERT_TRUE(v.isApprox(quaternion_to_ypr(ypr_to_quaternion(v)),DOUBLE_EPS));
    ASSERT_TRUE(v.isApprox(R_to_ypr(rotz(v(0)) * roty(v(1)) * rotx(v(2))),DOUBLE_EPS));

    double yaw = 30.0;
    ASSERT_DOUBLE_EQ(30.0, toDeg(get_yaw_from_quaternion(yaw_to_quaternion(toRad(yaw)))));
}

TEST(GeometryUtilsFloat, Rotation) {
    Vector3f v(0.1, 0.2, 0.3);
    ASSERT_TRUE(v.isApprox(R_to_ypr(ypr_to_R(v)),FLOAT_EPS));
    ASSERT_TRUE(v.isApprox(quaternion_to_ypr(ypr_to_quaternion(v)),FLOAT_EPS));
    ASSERT_TRUE(v.isApprox(R_to_ypr(rotz(v(0)) * roty(v(1)) * rotx(v(2))),FLOAT_EPS));

    float yaw = 30.0;
    ASSERT_FLOAT_EQ(30.0, toDeg(get_yaw_from_quaternion(yaw_to_quaternion(toRad(yaw)))));
}

TEST(GeometryUtilsDouble, Skew) {
    double v1 = 0.1;
    double v2 = 0.2;
    double v3 = 0.3;
    Vector3d v(v1, v2, v3);
    Matrix3d M;
    M <<  .0, -v3,  v2, 
          v3,  .0, -v1,
         -v2,  v1,  .0;
    
    ASSERT_TRUE(M.isApprox(get_skew_symmetric(v), DOUBLE_EPS));
    ASSERT_TRUE(v.isApprox(from_skew_symmetric(M), DOUBLE_EPS));
}

TEST(GeometryUtilsFloat, Skew) {
    float v1 = 0.1;
    float v2 = 0.2;
    float v3 = 0.3;
    Vector3f v(v1, v2, v3);
    Matrix3f M;
    M <<  .0, -v3,  v2, 
          v3,  .0, -v1,
         -v2,  v1,  .0;
    
    ASSERT_TRUE(M.isApprox(get_skew_symmetric(v), FLOAT_EPS));
    ASSERT_TRUE(v.isApprox(from_skew_symmetric(M), FLOAT_EPS));
}

TEST(GeometryUtilsDouble, Angle) {
    double a = toRad(179.0);
    double b = toRad(2.0);
    ASSERT_DOUBLE_EQ(-179.0, toDeg(angle_add(a, b)));
    ASSERT_DOUBLE_EQ(-179.0, toDeg(yaw_add(a, b)));
    ASSERT_DOUBLE_EQ(179.0, toDeg(angle_add(-a, -b)));
    ASSERT_DOUBLE_EQ(179.0, toDeg(yaw_add(-a, -b)));
    ASSERT_DOUBLE_EQ(177.0, toDeg(angle_add(a, -b)));
    ASSERT_DOUBLE_EQ(177.0, toDeg(yaw_add(a, -b)));
    ASSERT_NEAR(toRad(-2.0), normalize_angle(toRad(358.0)), DOUBLE_EPS);
}

TEST(GeometryUtilsFloat, Angle) {
    float a = toRad(179.0);
    float b = toRad(2.0);
    ASSERT_FLOAT_EQ(-179.0, toDeg(angle_add(a, b)));
    ASSERT_FLOAT_EQ(-179.0, toDeg(yaw_add(a, b)));
    ASSERT_FLOAT_EQ(179.0, toDeg(angle_add(-a, -b)));
    ASSERT_FLOAT_EQ(179.0, toDeg(yaw_add(-a, -b)));
    ASSERT_FLOAT_EQ(177.0, toDeg(angle_add(a, -b)));
    ASSERT_FLOAT_EQ(177.0, toDeg(yaw_add(a, -b)));
    ASSERT_NEAR(-2.0, toDeg(normalize_angle(toRad(358.0))),FLOAT_EPS);
}

TEST(ConverterDouble, Equality) {
    nav_msgs::OdometryPtr pOdom(new nav_msgs::Odometry());

    pOdom->pose.pose.position.x = 1.0;
    pOdom->pose.pose.position.y = 2.0;
    pOdom->pose.pose.position.z = 3.0;

    pOdom->pose.pose.orientation.w = 0.5;
    pOdom->pose.pose.orientation.x = -0.5;
    pOdom->pose.pose.orientation.y = 0.5;
    pOdom->pose.pose.orientation.z = -0.5;

    pOdom->twist.twist.linear.x = -1.0;
    pOdom->twist.twist.linear.y = -2.0;
    pOdom->twist.twist.linear.z = -3.0;

    pOdom->twist.twist.angular.x = -0.1;
    pOdom->twist.twist.angular.y = -0.2;
    pOdom->twist.twist.angular.z = -0.3;

    Eigen::Vector3d p, v, w;
    Eigen::Quaterniond q;

    nav_msgs::Odometry odom_ = *pOdom;

    extract_odometry(pOdom, p, v, q, w);

    ASSERT_TRUE(v.isApprox(from_vector3_msg(pOdom->twist.twist.linear)));
    ASSERT_TRUE(w.isApprox(from_vector3_msg(pOdom->twist.twist.angular)));
    ASSERT_TRUE(p.isApprox(from_point_msg(pOdom->pose.pose.position)));
    ASSERT_TRUE(q.isApprox(from_quaternion_msg(pOdom->pose.pose.orientation)));

    ASSERT_TRUE(v.isApprox(from_vector3_msg(to_vector3_msg(v))));
    ASSERT_TRUE(p.isApprox(from_point_msg(to_point_msg(p))));
    ASSERT_TRUE(q.isApprox(from_quaternion_msg(to_quaternion_msg(q))));
}

int main(int argc, char* argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}