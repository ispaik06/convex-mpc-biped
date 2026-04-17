#ifndef LEG_CONTROLLER_H
#define LEG_CONTROLLER_H

#include <cstddef>

#include <Eigen/StdVector>

#include "Robot/RobotModel.h"

enum class LegControlMode {
    JointPd,
    SwingFoot,
    StanceWrench,
};

template <typename T>
struct LegControllerCommand {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LegControllerCommand() = delete;
    explicit LegControllerCommand(Eigen::Index dof);

    void resize(Eigen::Index dof);
    void zero();

    Eigen::Index dof() const;

    LegControlMode mode = LegControlMode::JointPd;
    DVec<T> tauFeedForward;
    Vec3<T> forceFeedForward = Vec3<T>::Zero();
    Vec3<T> momentFeedForward = Vec3<T>::Zero();

    DVec<T> qDes;
    DVec<T> qdDes;
    Vec3<T> pDes = Vec3<T>::Zero();
    Vec3<T> vDes = Vec3<T>::Zero();
    Vec3<T> aDes = Vec3<T>::Zero();

    Mat3<T> kpCartesian = Mat3<T>::Zero();
    Mat3<T> kdCartesian = Mat3<T>::Zero();
    DMat<T> kpJoint;
    DMat<T> kdJoint;
};

template <typename T>
struct LegControllerData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LegControllerData() = delete;
    explicit LegControllerData(Eigen::Index dof);

    void resize(Eigen::Index dof);
    void zero();

    Eigen::Index dof() const;

    DVec<T> q;
    DVec<T> qd;

    Vec3<T> pWorld = Vec3<T>::Zero();
    Vec3<T> vWorld = Vec3<T>::Zero();
    DMat<T> JvWorld;
    DMat<T> JvDotWorld;
    DMat<T> JwWorld;
    DMat<T> massMatrix;
    DVec<T> bias;

    DVec<T> tauEstimate;
    bool hasFootData = false;
    bool hasDynamicsData = false;
};

template <typename T>
class LegController {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit LegController(const RobotModel<T>& model);

    const RobotModel<T>& model() const;
    std::size_t numLegs() const;

    void zeroCommand();
    void zeroData();

    void setEnabled(bool enabled);
    bool enabled() const;

    void updateJointData(const DVec<T>& q, const DVec<T>& qd);
    void updateJointData(const DVec<T>& q, const DVec<T>& qd, const DVec<T>& tauEstimate);

    void setLegJointData(int leg, const DVec<T>& q, const DVec<T>& qd);
    void setLegTauEstimate(int leg, const DVec<T>& tauEstimate);
    void setLegCartesianData(int leg,
                             const Vec3<T>& pWorld,
                             const Vec3<T>& vWorld,
                             const DMat<T>& JvWorld,
                             const DMat<T>& JvDotWorld,
                             const DMat<T>& JwWorld);
    void setLegDynamicsData(int leg, const DMat<T>& massMatrix, const DVec<T>& bias);
    void clearLegCartesianData(int leg);
    void clearLegDynamicsData(int leg);

    DVec<T> computeLegTorque(int leg) const;
    DVec<T> computeJointPdTorque(int leg) const;
    DVec<T> computeSwingLegTorque(int leg) const;
    DVec<T> computeStanceLegTorque(int leg) const;

    void updateCommand(DVec<T>& tauAll) const;
    DVec<T> updateCommand() const;

    vectorAligned<LegControllerCommand<T>> commands;
    vectorAligned<LegControllerData<T>> datas;

private:
    static DVec<T> extractIndexed(const DVec<T>& src,
                                  const std::vector<int>& indices,
                                  const char* name);

    void resizeFromModel();
    void checkLegIndex(int leg) const;
    void validateLegShape(std::size_t leg) const;

    const RobotModel<T>* _robotModel = nullptr;
    bool _legsEnabled = false;
};

#endif  // LEG_CONTROLLER_H
