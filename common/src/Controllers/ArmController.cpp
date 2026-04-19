#include <stdexcept>
#include <string>

#include "Controllers/ArmController.h"

template <typename T>
ArmControllerCommand<T>::ArmControllerCommand(Eigen::Index dof) {
    resize(dof);
}

template <typename T>
void ArmControllerCommand<T>::resize(Eigen::Index dof) {
    if (dof < 0) {
        throw std::invalid_argument("ArmControllerCommand dof must be non-negative");
    }

    tauFeedForward.setZero(dof);
    qDes.setZero(dof);
    qdDes.setZero(dof);
    kpJoint.setZero(dof, dof);
    kdJoint.setZero(dof, dof);

    forceFeedForward_W.setZero();
    pDes_W.setZero();
    vDes_W.setZero();
    kpCartesian.setZero();
    kdCartesian.setZero();
}

template <typename T>
void ArmControllerCommand<T>::zero() {
    tauFeedForward.setZero(tauFeedForward.size());
    qDes.setZero(qDes.size());
    qdDes.setZero(qdDes.size());
    kpJoint.setZero(kpJoint.rows(), kpJoint.cols());
    kdJoint.setZero(kdJoint.rows(), kdJoint.cols());

    forceFeedForward_W.setZero();
    pDes_W.setZero();
    vDes_W.setZero();
    kpCartesian.setZero();
    kdCartesian.setZero();
}

template <typename T>
Eigen::Index ArmControllerCommand<T>::dof() const {
    return qDes.size();
}

template <typename T>
ArmControllerData<T>::ArmControllerData(Eigen::Index dof) {
    resize(dof);
}

template <typename T>
void ArmControllerData<T>::resize(Eigen::Index dof) {
    if (dof < 0) {
        throw std::invalid_argument("ArmControllerData dof must be non-negative");
    }

    q.setZero(dof);
    qd.setZero(dof);
    tauEstimate.setZero(dof);
    J_W.setZero(3, dof);

    p_W.setZero();
    v_W.setZero();
    hasCartesianData = false;
}

template <typename T>
void ArmControllerData<T>::zero() {
    q.setZero(q.size());
    qd.setZero(qd.size());
    tauEstimate.setZero(tauEstimate.size());
    J_W.setZero(J_W.rows(), J_W.cols());

    p_W.setZero();
    v_W.setZero();
    hasCartesianData = false;
}

template <typename T>
Eigen::Index ArmControllerData<T>::dof() const {
    return q.size();
}

template <typename T>
ArmController<T>::ArmController(const RobotModel<T>& model) : _robotModel(&model) {
    resizeFromModel();
}

template <typename T>
const RobotModel<T>& ArmController<T>::model() const {
    return *_robotModel;
}

template <typename T>
std::size_t ArmController<T>::numArms() const {
    return commands.size();
}

template <typename T>
void ArmController<T>::setEnabled(bool enabled) {
    _armsEnabled = enabled;
}

template <typename T>
bool ArmController<T>::enabled() const {
    return _armsEnabled;
}

template <typename T>
void ArmController<T>::zeroCommand() {
    for (auto& cmd : commands) {
        cmd.zero();
    }
    _armsEnabled = false;
}

template <typename T>
void ArmController<T>::zeroData() {
    for (auto& data : datas) {
        data.zero();
    }
}

template <typename T>
void ArmController<T>::updateJointData(const DVec<T>& q, const DVec<T>& qd) {
    for (std::size_t arm = 0; arm < datas.size(); ++arm) {
        datas[arm].q = _robotModel->getArmQ(q, static_cast<int>(arm));
        datas[arm].qd = _robotModel->getArmQd(qd, static_cast<int>(arm));
    }
}

template <typename T>
void ArmController<T>::updateJointData(const DVec<T>& q,
                                       const DVec<T>& qd,
                                       const DVec<T>& tauEstimate) {
    updateJointData(q, qd);

    for (std::size_t arm = 0; arm < datas.size(); ++arm) {
        datas[arm].tauEstimate =
            extractIndexed(tauEstimate, _robotModel->armActuatorIndices(static_cast<int>(arm)),
                           "tauEstimate");
    }
}

template <typename T>
void ArmController<T>::setArmJointData(int arm, const DVec<T>& q, const DVec<T>& qd) {
    checkArmIndex(arm);
    const Eigen::Index dof = datas[static_cast<std::size_t>(arm)].dof();
    if (q.size() != dof || qd.size() != dof) {
        throw std::invalid_argument("Arm joint data size does not match arm dof");
    }

    datas[static_cast<std::size_t>(arm)].q = q;
    datas[static_cast<std::size_t>(arm)].qd = qd;
}

template <typename T>
void ArmController<T>::setArmTauEstimate(int arm, const DVec<T>& tauEstimate) {
    checkArmIndex(arm);
    const std::size_t idx = static_cast<std::size_t>(arm);
    if (tauEstimate.size() != datas[idx].tauEstimate.size()) {
        throw std::invalid_argument("Arm torque estimate size does not match arm dof");
    }

    datas[idx].tauEstimate = tauEstimate;
}

template <typename T>
void ArmController<T>::setArmCartesianData(int arm,
                                           const Vec3<T>& p_W,
                                           const Vec3<T>& v_W,
                                           const DMat<T>& J_W) {
    checkArmIndex(arm);
    const std::size_t idx = static_cast<std::size_t>(arm);
    const Eigen::Index dof = datas[idx].dof();

    if (J_W.rows() != 3 || J_W.cols() != dof) {
        throw std::invalid_argument("Arm Jacobian must be 3 x arm dof");
    }

    datas[idx].p_W = p_W;
    datas[idx].v_W = v_W;
    datas[idx].J_W = J_W;
    datas[idx].hasCartesianData = true;
}

template <typename T>
void ArmController<T>::clearArmCartesianData(int arm) {
    checkArmIndex(arm);
    const std::size_t idx = static_cast<std::size_t>(arm);
    const Eigen::Index dof = datas[idx].dof();

    datas[idx].p_W.setZero();
    datas[idx].v_W.setZero();
    datas[idx].J_W.setZero(3, dof);
    datas[idx].hasCartesianData = false;
}

template <typename T>
DVec<T> ArmController<T>::computeArmTorque(int arm) const {
    checkArmIndex(arm);
    const std::size_t idx = static_cast<std::size_t>(arm);
    validateArmShape(idx);

    DVec<T> armTorque = commands[idx].tauFeedForward;
    armTorque += commands[idx].kpJoint * (commands[idx].qDes - datas[idx].q);
    armTorque += commands[idx].kdJoint * (commands[idx].qdDes - datas[idx].qd);

    if (datas[idx].hasCartesianData) {
        Vec3<T> handForce_W = commands[idx].forceFeedForward_W;
        handForce_W += commands[idx].kpCartesian * (commands[idx].pDes_W - datas[idx].p_W);
        handForce_W += commands[idx].kdCartesian * (commands[idx].vDes_W - datas[idx].v_W);
        armTorque += datas[idx].J_W.transpose() * handForce_W;
    }

    return armTorque;
}

template <typename T>
void ArmController<T>::updateCommand(DVec<T>& tauAll) const {
    if (!_armsEnabled) {
        return;
    }

    for (std::size_t arm = 0; arm < commands.size(); ++arm) {
        const DVec<T> armTorque = computeArmTorque(static_cast<int>(arm));
        _robotModel->setArmTau(static_cast<int>(arm), armTorque, tauAll);
    }
}

template <typename T>
DVec<T> ArmController<T>::updateCommand() const {
    DVec<T> tauAll = DVec<T>::Zero(_robotModel->nu());
    updateCommand(tauAll);
    return tauAll;
}

template <typename T>
DVec<T> ArmController<T>::extractIndexed(const DVec<T>& src,
                                         const std::vector<int>& indices,
                                         const char* name) {
    DVec<T> out(static_cast<Eigen::Index>(indices.size()));
    for (Eigen::Index i = 0; i < out.size(); ++i) {
        const int srcIdx = indices[static_cast<std::size_t>(i)];
        if (srcIdx < 0 || srcIdx >= src.size()) {
            throw std::out_of_range(std::string(name) + " index is out of range");
        }
        out[i] = src[srcIdx];
    }
    return out;
}

template <typename T>
void ArmController<T>::resizeFromModel() {
    commands.clear();
    datas.clear();

    commands.reserve(_robotModel->numArms());
    datas.reserve(_robotModel->numArms());

    for (std::size_t arm = 0; arm < _robotModel->numArms(); ++arm) {
        const auto qCount =
            static_cast<Eigen::Index>(_robotModel->armQIndices(static_cast<int>(arm)).size());
        const auto qdCount =
            static_cast<Eigen::Index>(_robotModel->armQdIndices(static_cast<int>(arm)).size());
        const auto actuatorCount = static_cast<Eigen::Index>(
            _robotModel->armActuatorIndices(static_cast<int>(arm)).size());

        if (qCount != qdCount) {
            throw std::invalid_argument("ArmController requires q and qd index counts to match");
        }

        if (qCount != actuatorCount) {
            throw std::invalid_argument(
                "ArmController currently requires actuator count to match joint count");
        }

        commands.emplace_back(qCount);
        datas.emplace_back(qCount);
    }
}

template <typename T>
void ArmController<T>::checkArmIndex(int arm) const {
    if (arm < 0 || static_cast<std::size_t>(arm) >= commands.size()) {
        throw std::out_of_range("Invalid arm index");
    }
}

template <typename T>
void ArmController<T>::validateArmShape(std::size_t arm) const {
    const Eigen::Index dof = datas[arm].dof();

    if (commands[arm].tauFeedForward.size() != dof || commands[arm].qDes.size() != dof ||
        commands[arm].qdDes.size() != dof) {
        throw std::invalid_argument("Arm command vector size does not match arm dof");
    }

    if (commands[arm].kpJoint.rows() != dof || commands[arm].kpJoint.cols() != dof ||
        commands[arm].kdJoint.rows() != dof || commands[arm].kdJoint.cols() != dof) {
        throw std::invalid_argument("Arm joint gain matrix size does not match arm dof");
    }

    if (datas[arm].q.size() != dof || datas[arm].qd.size() != dof ||
        datas[arm].tauEstimate.size() != dof) {
        throw std::invalid_argument("Arm data vector size does not match arm dof");
    }

    if (datas[arm].hasCartesianData &&
        (datas[arm].J_W.rows() != 3 || datas[arm].J_W.cols() != dof)) {
        throw std::invalid_argument("Arm Jacobian size does not match Cartesian task shape");
    }
}

template struct ArmControllerCommand<float>;
template struct ArmControllerCommand<double>;

template struct ArmControllerData<float>;
template struct ArmControllerData<double>;

template class ArmController<float>;
template class ArmController<double>;
