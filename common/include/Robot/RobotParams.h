#ifndef ROBOT_PARAMS_H
#define ROBOT_PARAMS_H

#include <string>
#include <Eigen/StdVector>

#include "Types.h"

// //? gonna use these namespaces?
// /*!
//  * Basic parameters for MIT humanoid robot
//  */
// namespace MIThumanoid {
//     constexpr size_t num_act_joint = 18;
//     constexpr size_t num_q = 12;
//     constexpr size_t num_v = 11;
//     constexpr size_t num_leg = 2;
//     constexpr size_t num_arm = 2;
//     constexpr size_t num_arm_joint = 4;
//     constexpr size_t num_leg_joint = 5;
// }  // namespace MIT humanoid

enum class Side { Left, Right, FL, FR, BL, BR };

template <typename T>
struct JointGroupParams {
    std::vector<int> q_idx;
    std::vector<int> qd_idx;
    std::vector<int> actuator_idx;

    DVec<T> motorTauMax;
    DVec<T> damping;
    DVec<T> dryFriction;
};

template <typename T>
struct LegParams {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Side side{Side::Left};
    JointGroupParams<T> joints;

    Vec3<T> hipLocationFromBody = Vec3<T>::Zero();

    vectorAligned<Vec3<T>> jointLocation_offsets;
};

template <typename T>
struct ArmParams {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Side side{Side::Left};
    JointGroupParams<T> joints;
};

template <typename T>
struct RobotParams {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RobotType roboType{RobotType::MIT_HUMANOID};

  int nq{0};
  int nv{0};
  int nu{0};

  T bodyMass{0.0};
  Mat3<T> bodyInertia = Mat3<T>::Zero();  // reduced-body frame, yaw aligned
  Vec3<T> bodyComLocation = Vec3<T>::Zero();  // torso root -> reduced COM, yaw aligned

  DVec<T> default_qpos;

  vectorAligned<LegParams<T>> legs;
  vectorAligned<ArmParams<T>> arms;
};

#endif  // ROBOT_PARAMS_H
