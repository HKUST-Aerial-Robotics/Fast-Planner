#ifndef __GEOMETRY_UTILS_H
#define __GEOMETRY_UTILS_H

#include <Eigen/Dense>

/* clang-format off */
namespace uav_utils {

template <typename Scalar_t>
Scalar_t toRad(const Scalar_t& x) {
    return x / 180.0 * M_PI;
}

template <typename Scalar_t>
Scalar_t toDeg(const Scalar_t& x) {
    return x * 180.0 / M_PI;
}

template <typename Scalar_t>
Eigen::Matrix<Scalar_t, 3, 3> rotx(Scalar_t t) {
    Eigen::Matrix<Scalar_t, 3, 3> R;
    R(0, 0) = 1.0;
    R(0, 1) = 0.0;
    R(0, 2) = 0.0;
    R(1, 0) = 0.0;
    R(1, 1) = std::cos(t);
    R(1, 2) = -std::sin(t);
    R(2, 0) = 0.0;
    R(2, 1) = std::sin(t);
    R(2, 2) = std::cos(t);

    return R;
}

template <typename Scalar_t>
Eigen::Matrix<Scalar_t, 3, 3> roty(Scalar_t t) {
    Eigen::Matrix<Scalar_t, 3, 3> R;
    R(0, 0) = std::cos(t);
    R(0, 1) = 0.0;
    R(0, 2) = std::sin(t);
    R(1, 0) = 0.0;
    R(1, 1) = 1.0;
    R(1, 2) = 0;
    R(2, 0) = -std::sin(t);
    R(2, 1) = 0.0;
    R(2, 2) = std::cos(t);

    return R;
}

template <typename Scalar_t>
Eigen::Matrix<Scalar_t, 3, 3> rotz(Scalar_t t) {
    Eigen::Matrix<Scalar_t, 3, 3> R;
    R(0, 0) = std::cos(t);
    R(0, 1) = -std::sin(t);
    R(0, 2) = 0.0;
    R(1, 0) = std::sin(t);
    R(1, 1) = std::cos(t);
    R(1, 2) = 0.0;
    R(2, 0) = 0.0;
    R(2, 1) = 0.0;
    R(2, 2) = 1.0;

    return R;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr_to_R(const Eigen::DenseBase<Derived>& ypr) {
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
    EIGEN_STATIC_ASSERT(Derived::RowsAtCompileTime == 3, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    EIGEN_STATIC_ASSERT(Derived::ColsAtCompileTime == 1, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

    typename Derived::Scalar c, s;

    Eigen::Matrix<typename Derived::Scalar, 3, 3> Rz = Eigen::Matrix<typename Derived::Scalar, 3, 3>::Zero();
    typename Derived::Scalar y = ypr(0);
    c = cos(y);
    s = sin(y);
    Rz(0, 0) = c;
    Rz(1, 0) = s;
    Rz(0, 1) = -s;
    Rz(1, 1) = c;
    Rz(2, 2) = 1;

    Eigen::Matrix<typename Derived::Scalar, 3, 3> Ry = Eigen::Matrix<typename Derived::Scalar, 3, 3>::Zero();
    typename Derived::Scalar p = ypr(1);
    c = cos(p);
    s = sin(p);
    Ry(0, 0) = c;
    Ry(2, 0) = -s;
    Ry(0, 2) = s;
    Ry(2, 2) = c;
    Ry(1, 1) = 1;

    Eigen::Matrix<typename Derived::Scalar, 3, 3> Rx = Eigen::Matrix<typename Derived::Scalar, 3, 3>::Zero();
    typename Derived::Scalar r = ypr(2);
    c = cos(r);
    s = sin(r);
    Rx(1, 1) = c;
    Rx(2, 1) = s;
    Rx(1, 2) = -s;
    Rx(2, 2) = c;
    Rx(0, 0) = 1;

    Eigen::Matrix<typename Derived::Scalar, 3, 3> R = Rz * Ry * Rx;
    return R;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> R_to_ypr(const Eigen::DenseBase<Derived>& R) {
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
    EIGEN_STATIC_ASSERT(Derived::RowsAtCompileTime == 3, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    EIGEN_STATIC_ASSERT(Derived::ColsAtCompileTime == 3, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

    Eigen::Matrix<typename Derived::Scalar, 3, 1> n = R.col(0);
    Eigen::Matrix<typename Derived::Scalar, 3, 1> o = R.col(1);
    Eigen::Matrix<typename Derived::Scalar, 3, 1> a = R.col(2);

    Eigen::Matrix<typename Derived::Scalar, 3, 1> ypr(3);
    typename Derived::Scalar y = atan2(n(1), n(0));
    typename Derived::Scalar p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    typename Derived::Scalar r =
        atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr;
}

template <typename Derived>
Eigen::Quaternion<typename Derived::Scalar> ypr_to_quaternion(const Eigen::DenseBase<Derived>& ypr) {
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
    EIGEN_STATIC_ASSERT(Derived::RowsAtCompileTime == 3, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    EIGEN_STATIC_ASSERT(Derived::ColsAtCompileTime == 1, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

    const typename Derived::Scalar cy = cos(ypr(0) / 2.0);
    const typename Derived::Scalar sy = sin(ypr(0) / 2.0);
    const typename Derived::Scalar cp = cos(ypr(1) / 2.0);
    const typename Derived::Scalar sp = sin(ypr(1) / 2.0);
    const typename Derived::Scalar cr = cos(ypr(2) / 2.0);
    const typename Derived::Scalar sr = sin(ypr(2) / 2.0);

    Eigen::Quaternion<typename Derived::Scalar> q;

    q.w() = cr * cp * cy + sr * sp * sy;
    q.x() = sr * cp * cy - cr * sp * sy;
    q.y() = cr * sp * cy + sr * cp * sy;
    q.z() = cr * cp * sy - sr * sp * cy;

    return q;
}

template <typename Scalar_t>
Eigen::Matrix<Scalar_t, 3, 1> quaternion_to_ypr(const Eigen::Quaternion<Scalar_t>& q_) {
    Eigen::Quaternion<Scalar_t> q = q_.normalized();

    Eigen::Matrix<Scalar_t, 3, 1> ypr;
    ypr(2) = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
    ypr(1) = asin(2 * (q.w() * q.y() - q.z() * q.x()));
    ypr(0) = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));

    return ypr;
}

template <typename Scalar_t>
Scalar_t get_yaw_from_quaternion(const Eigen::Quaternion<Scalar_t>& q) {
    return quaternion_to_ypr(q)(0);
}

template <typename Scalar_t>
Eigen::Quaternion<Scalar_t> yaw_to_quaternion(Scalar_t yaw) {
    return Eigen::Quaternion<Scalar_t>(rotz(yaw));
}

template <typename Scalar_t>
Scalar_t normalize_angle(Scalar_t a) {
    int cnt = 0;
    while (true) {
        cnt++;

        if (a < -M_PI) {
            a += M_PI * 2.0;
        } else if (a > M_PI) {
            a -= M_PI * 2.0;
        }

        if (-M_PI <= a && a <= M_PI) {
            break;
        };

        assert(cnt < 10 && "[uav_utils/geometry_msgs] INVALID INPUT ANGLE");
    }

    return a;
}

template <typename Scalar_t>
Scalar_t angle_add(Scalar_t a, Scalar_t b) {
    Scalar_t c = a + b;
    c = normalize_angle(c);
    assert(-M_PI <= c && c <= M_PI);
    return c;
}

template <typename Scalar_t>
Scalar_t yaw_add(Scalar_t a, Scalar_t b) {
    return angle_add(a, b);
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> get_skew_symmetric(const Eigen::DenseBase<Derived>& v) {
	EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
    EIGEN_STATIC_ASSERT(Derived::RowsAtCompileTime == 3, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    EIGEN_STATIC_ASSERT(Derived::ColsAtCompileTime == 1, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

    Eigen::Matrix<typename Derived::Scalar, 3, 3> M;
    M.setZero();
    M(0, 1) = -v(2);
    M(0, 2) = v(1);
    M(1, 0) = v(2);
    M(1, 2) = -v(0);
    M(2, 0) = -v(1);
    M(2, 1) = v(0);
    return M;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> from_skew_symmetric(const Eigen::DenseBase<Derived>& M) {
	EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
    EIGEN_STATIC_ASSERT(Derived::RowsAtCompileTime == 3, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    EIGEN_STATIC_ASSERT(Derived::ColsAtCompileTime == 3, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

    Eigen::Matrix<typename Derived::Scalar, 3, 1> v;
    v(0) = M(2, 1);
    v(1) = -M(2, 0);
    v(2) = M(1, 0);
    
    assert(v.isApprox(Eigen::Matrix<typename Derived::Scalar, 3, 1>(-M(1, 2), M(0, 2), -M(0, 1))));
    
    return v;
}


}  // end of namespace uav_utils
/* clang-format on */

#endif
