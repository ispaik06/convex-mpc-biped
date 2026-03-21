#ifndef ROBOT_PARAMS_H
#define ROBOT_PARAMS_H

#include <string>
#include <Eigen/StdVector>

#include "Types.h"

//? gonna use these namespaces?
/*!
 * Basic parameters for MIT humanoid robot
 */
namespace MIThumanoid {
    constexpr size_t num_act_joint = 18;
    constexpr size_t num_q = 12;
    constexpr size_t num_v = 11;
    constexpr size_t num_leg = 2;
    constexpr size_t num_arm = 2;
    constexpr size_t num_arm_joint = 4;
    constexpr size_t num_leg_joint = 5;
}  // namespace MIT humanoid

/*!
 * Basic parameters for UNITREE G1
 */
namespace UnitreeG1 {
    constexpr size_t num_act_joint = 23;
    constexpr size_t num_q = 24;
    constexpr size_t num_v = 23;
    constexpr size_t num_leg = 2;
    constexpr size_t num_arm = 2;
    constexpr size_t num_arm_joint = 4;
    constexpr size_t num_leg_joint = 6;
}

enum class Side { Left, Right, FL, FR, BL, BR };

struct EndEffectorParams {
    std::string body_name;
    std::string site_name;
    int body_id{-1};
    int site_id{-1};
};

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
    EndEffectorParams foot;

    Vec3<T> hipLocation_from_body = Vec3<T>::Zero();

    vectorAligned<Vec3<T>> jointLocation_offsets;
};

template <typename T>
struct ArmParams {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Side side{Side::Left};
    JointGroupParams<T> joints;
    EndEffectorParams hand;
};

template <typename T>
struct RobotParams {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RobotType roboType{RobotType::MIT_HUMANOID};
  std::string baseBodyName;
  int baseBodyId{-1};

  int nq{0};
  int nv{0};
  int nu{0};

  T bodyMass{0.0};
  Mat3<T> bodyInertia = Mat3<T>::Zero();
  Vec3<T> bodyComLocation = Vec3<T>::Zero();

  DVec<T> default_qpos;

  vectorAligned<LegParams<T>> legs;
  vectorAligned<ArmParams<T>> arms;
};

#endif  // ROBOT_PARAMS_H
