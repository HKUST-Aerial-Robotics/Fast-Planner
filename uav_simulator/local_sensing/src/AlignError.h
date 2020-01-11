#include <Eigen/Eigen>
#include <ceres/ceres.h>

struct AlignError {
    AlignError(  const Eigen::Quaterniond camera_pose, const Eigen::Vector3d camera_trans,
                 const Eigen::Quaterniond velodyne_pose, const Eigen::Vector3d velodyne_trans)
    {
      camera_q[0] = camera_pose.x();
      camera_q[1] = camera_pose.y();
      camera_q[2] = camera_pose.z();
      camera_q[3] = camera_pose.w();
      camera_t[0] = camera_trans.x();
      camera_t[1] = camera_trans.y();
      camera_t[2] = camera_trans.z();

      velodyne_q[0] = velodyne_pose.x();
      velodyne_q[1] = velodyne_pose.y();
      velodyne_q[2] = velodyne_pose.z();
      velodyne_q[3] = velodyne_pose.w();
      velodyne_t[0] = velodyne_trans.x();
      velodyne_t[1] = velodyne_trans.y();
      velodyne_t[2] = velodyne_trans.z();
    }
 
    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create( const Eigen::Quaterniond camera_pose, const Eigen::Vector3d camera_trans,
                                        const Eigen::Quaterniond velodyne_pose, const Eigen::Vector3d velodyne_trans)
    {
        return (new ceres::AutoDiffCostFunction<AlignError, 6, 4, 3, 4, 3>(
            new AlignError(camera_pose, camera_trans, velodyne_pose, velodyne_trans)));
    }
 
    template <typename T>
    bool operator()(const T* const world_rotation, const T* const world_translation,
                    const T* const v2c_rotation, const T* const v2c_translation,
                    T* residuals) const 
    { 
        Eigen::Quaternion<T> q_world = Eigen::Map< const Eigen::Quaternion<T> >(world_rotation);
        Eigen::Matrix<T,3,1> t_world = Eigen::Map< const Eigen::Matrix<T,3,1> >(world_translation);

        Eigen::Quaternion<T> q_v2c = Eigen::Map< const Eigen::Quaternion<T> >(v2c_rotation);
        Eigen::Matrix<T,3,1> t_v2c = Eigen::Map< const Eigen::Matrix<T,3,1> >(v2c_translation);

        Eigen::Quaternion<T> q_c;
        Eigen::Matrix<T,3,1> t_c;
        q_c.x() = T(camera_q[0]);
        q_c.y() = T(camera_q[1]);
        q_c.z() = T(camera_q[2]);
        q_c.w() = T(camera_q[3]);
        t_c << T(camera_t[0]), T(camera_t[1]), T(camera_t[2]); 

        Eigen::Quaternion<T> q_v;
        Eigen::Matrix<T,3,1> t_v;
        q_v.x() = T(velodyne_q[0]);
        q_v.y() = T(velodyne_q[1]);
        q_v.z() = T(velodyne_q[2]);
        q_v.w() = T(velodyne_q[3]);
        t_v << T(velodyne_t[0]), T(velodyne_t[1]), T(velodyne_t[2]);

        Eigen::Quaternion<T> q_left; 
        Eigen::Matrix<T,3,1> t_left;

        q_left = q_world * q_c * q_v2c;
        t_left = q_world * q_c * t_v2c + q_world * t_c + t_world;

        Eigen::Quaternion<T> q_diff; 
        Eigen::Matrix<T,3,1> t_diff;
        q_diff = q_left * q_v.inverse();
        // t_diff = t_left - q_left * q_v.inverse() * t_v;
        t_diff = t_left - t_v;

        residuals[0] = q_diff.x();
        residuals[1] = q_diff.y();
        residuals[2] = q_diff.z();
        residuals[3] = t_diff(0);
        residuals[4] = t_diff(1);
        residuals[5] = t_diff(2);
 
        return true;
    }

    double camera_q[4];
    double camera_t[3];
    double velodyne_q[4];
    double velodyne_t[3];
};
