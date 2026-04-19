#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <cstddef>

#include <Eigen/StdVector>

#include "Robot/RobotModel.h"

template <typename T>
struct ArmControllerCommand {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ArmControllerCommand() = delete;
    explicit ArmControllerCommand(Eigen::Index dof);

    void resize(Eigen::Index dof);
    void zero();

    Eigen::Index dof() const;

    DVec<T> tauFeedForward;
    Vec3<T> forceFeedForward_W = Vec3<T>::Zero();

    DVec<T> qDes;
    DVec<T> qdDes;
    Vec3<T> pDes_W = Vec3<T>::Zero();
    Vec3<T> vDes_W = Vec3<T>::Zero();

    Mat3<T> kpCartesian = Mat3<T>::Zero();
    Mat3<T> kdCartesian = Mat3<T>::Zero();
    DMat<T> kpJoint;
    DMat<T> kdJoint;
};

template <typename T>
struct ArmControllerData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ArmControllerData() = delete;
    explicit ArmControllerData(Eigen::Index dof);

    void resize(Eigen::Index dof);
    void zero();

    Eigen::Index dof() const;

    DVec<T> q;
    DVec<T> qd;

    Vec3<T> p_W = Vec3<T>::Zero();
    Vec3<T> v_W = Vec3<T>::Zero();
    DMat<T> J_W;

    DVec<T> tauEstimate;
    bool hasCartesianData = false;
};

template <typename T>
class ArmController {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit ArmController(const RobotModel<T>& model);

    const RobotModel<T>& model() const;
    std::size_t numArms() const;

    void zeroCommand();
    void zeroData();

    void setEnabled(bool enabled);
    bool enabled() const;

    void updateJointData(const DVec<T>& q, const DVec<T>& qd);
    void updateJointData(const DVec<T>& q, const DVec<T>& qd, const DVec<T>& tauEstimate);

    void setArmJointData(int arm, const DVec<T>& q, const DVec<T>& qd);
    void setArmTauEstimate(int arm, const DVec<T>& tauEstimate);
    void setArmCartesianData(int arm, const Vec3<T>& p_W, const Vec3<T>& v_W, const DMat<T>& J_W);
    void clearArmCartesianData(int arm);

    DVec<T> computeArmTorque(int arm) const;

    void updateCommand(DVec<T>& tauAll) const;
    DVec<T> updateCommand() const;

    vectorAligned<ArmControllerCommand<T>> commands;
    vectorAligned<ArmControllerData<T>> datas;

private:
    static DVec<T> extractIndexed(const DVec<T>& src,
                                  const std::vector<int>& indices,
                                  const char* name);

    void resizeFromModel();
    void checkArmIndex(int arm) const;
    void validateArmShape(std::size_t arm) const;

    const RobotModel<T>* _robotModel = nullptr;
    bool _armsEnabled = false;
};

#endif  // ARM_CONTROLLER_H
