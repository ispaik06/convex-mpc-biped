#include <stdexcept>
#include <string>

#include "Controllers/LegController.h"
#include "Dynamics/OperationalSpaceDynamics.h"

template <typename T>
LegControllerCommand<T>::LegControllerCommand(Eigen::Index dof) {
    resize(dof);
}

template <typename T>
void LegControllerCommand<T>::resize(Eigen::Index dof) {
    if (dof < 0) {
        throw std::invalid_argument("LegControllerCommand dof must be non-negative");
    }

    mode = LegControlMode::JointPd;
    tauFeedForward.setZero(dof);
    qDes.setZero(dof);
    qdDes.setZero(dof);
    kpJoint.setZero(dof, dof);
    kdJoint.setZero(dof, dof);

    forceFeedForward_W.setZero();
    momentFeedForward_W.setZero();
    pDes_W.setZero();
    vDes_W.setZero();
    aDes_W.setZero();
    kpCartesian.setZero();
    kdCartesian.setZero();
}

template <typename T>
void LegControllerCommand<T>::zero() {
    mode = LegControlMode::JointPd;
    tauFeedForward.setZero(tauFeedForward.size());
    qDes.setZero(qDes.size());
    qdDes.setZero(qdDes.size());
    kpJoint.setZero(kpJoint.rows(), kpJoint.cols());
    kdJoint.setZero(kdJoint.rows(), kdJoint.cols());

    forceFeedForward_W.setZero();
    momentFeedForward_W.setZero();
    pDes_W.setZero();
    vDes_W.setZero();
    aDes_W.setZero();
    kpCartesian.setZero();
    kdCartesian.setZero();
}

template <typename T>
Eigen::Index LegControllerCommand<T>::dof() const {
    return qDes.size();
}

template <typename T>
LegControllerData<T>::LegControllerData(Eigen::Index dof) {
    resize(dof);
}

template <typename T>
void LegControllerData<T>::resize(Eigen::Index dof) {
    if (dof < 0) {
        throw std::invalid_argument("LegControllerData dof must be non-negative");
    }

    q.setZero(dof);
    qd.setZero(dof);
    tauEstimate.setZero(dof);
    Jv_W.setZero(3, dof);
    JvDot_W.setZero(3, dof);
    Jw_W.setZero(3, dof);
    massMatrix.setZero(dof, dof);
    bias.setZero(dof);

    p_W.setZero();
    v_W.setZero();
    hasFootData = false;
    hasDynamicsData = false;
}

template <typename T>
void LegControllerData<T>::zero() {
    q.setZero(q.size());
    qd.setZero(qd.size());
    tauEstimate.setZero(tauEstimate.size());
    Jv_W.setZero(Jv_W.rows(), Jv_W.cols());
    JvDot_W.setZero(JvDot_W.rows(), JvDot_W.cols());
    Jw_W.setZero(Jw_W.rows(), Jw_W.cols());
    massMatrix.setZero(massMatrix.rows(), massMatrix.cols());
    bias.setZero(bias.size());

    p_W.setZero();
    v_W.setZero();
    hasFootData = false;
    hasDynamicsData = false;
}

template <typename T>
Eigen::Index LegControllerData<T>::dof() const {
    return q.size();
}

template <typename T>
LegController<T>::LegController(const RobotModel<T>& model) : _robotModel(&model) {
    resizeFromModel();
}

template <typename T>
const RobotModel<T>& LegController<T>::model() const {
    return *_robotModel;
}

template <typename T>
std::size_t LegController<T>::numLegs() const {
    return commands.size();
}

template <typename T>
void LegController<T>::setEnabled(bool enabled) {
    _legsEnabled = enabled;
}

template <typename T>
bool LegController<T>::enabled() const {
    return _legsEnabled;
}

template <typename T>
void LegController<T>::zeroCommand() {
    for (auto& cmd : commands) {
        cmd.zero();
    }
    _legsEnabled = false;
}

template <typename T>
void LegController<T>::zeroData() {
    for (auto& data : datas) {
        data.zero();
    }
}

template <typename T>
void LegController<T>::updateJointData(const DVec<T>& q, const DVec<T>& qd) {
    for (std::size_t leg = 0; leg < datas.size(); ++leg) {
        datas[leg].q = _robotModel->getLegQ(q, static_cast<int>(leg));
        datas[leg].qd = _robotModel->getLegQd(qd, static_cast<int>(leg));
    }
}

template <typename T>
void LegController<T>::updateJointData(const DVec<T>& q,
                                       const DVec<T>& qd,
                                       const DVec<T>& tauEstimate) {
    updateJointData(q, qd);

    for (std::size_t leg = 0; leg < datas.size(); ++leg) {
        datas[leg].tauEstimate =
            extractIndexed(tauEstimate, _robotModel->legActuatorIndices(static_cast<int>(leg)),
                           "tauEstimate");
    }
}

template <typename T>
void LegController<T>::setLegJointData(int leg, const DVec<T>& q, const DVec<T>& qd) {
    checkLegIndex(leg);
    const Eigen::Index dof = datas[static_cast<std::size_t>(leg)].dof();
    if (q.size() != dof || qd.size() != dof) {
        throw std::invalid_argument("Leg joint data size does not match leg dof");
    }

    datas[static_cast<std::size_t>(leg)].q = q;
    datas[static_cast<std::size_t>(leg)].qd = qd;
}

template <typename T>
void LegController<T>::setLegTauEstimate(int leg, const DVec<T>& tauEstimate) {
    checkLegIndex(leg);
    const std::size_t idx = static_cast<std::size_t>(leg);
    if (tauEstimate.size() != datas[idx].tauEstimate.size()) {
        throw std::invalid_argument("Leg torque estimate size does not match leg dof");
    }

    datas[idx].tauEstimate = tauEstimate;
}

template <typename T>
void LegController<T>::setLegCartesianData(int leg,
                             const Vec3<T>& p_W,
                             const Vec3<T>& v_W,
                             const DMat<T>& Jv_W,
                             const DMat<T>& JvDot_W,
                             const DMat<T>& Jw_W) {
    checkLegIndex(leg);
    const std::size_t idx = static_cast<std::size_t>(leg);
    const Eigen::Index dof = datas[idx].dof();

    if (Jv_W.rows() != 3 || JvDot_W.rows() != 3 || Jw_W.rows() != 3 ||
        Jv_W.cols() != dof || JvDot_W.cols() != dof || Jw_W.cols() != dof) {
        throw std::invalid_argument("Leg Jacobians must be 3 x leg dof");
    }

    datas[idx].p_W = p_W;
    datas[idx].v_W = v_W;
    datas[idx].Jv_W = Jv_W;
    datas[idx].JvDot_W = JvDot_W;
    datas[idx].Jw_W = Jw_W;
    datas[idx].hasFootData = true;
}

template <typename T>
void LegController<T>::setLegDynamicsData(int leg, const DMat<T>& massMatrix, const DVec<T>& bias) {
    checkLegIndex(leg);
    const std::size_t idx = static_cast<std::size_t>(leg);
    const Eigen::Index dof = datas[idx].dof();

    if (massMatrix.rows() != dof || massMatrix.cols() != dof || bias.size() != dof) {
        throw std::invalid_argument("Leg dynamics data size does not match leg dof");
    }

    datas[idx].massMatrix = massMatrix;
    datas[idx].bias = bias;
    datas[idx].hasDynamicsData = true;
}

template <typename T>
void LegController<T>::clearLegCartesianData(int leg) {
    checkLegIndex(leg);
    const std::size_t idx = static_cast<std::size_t>(leg);
    datas[idx].p_W.setZero();
    datas[idx].v_W.setZero();
    datas[idx].Jv_W.setZero(datas[idx].Jv_W.rows(), datas[idx].Jv_W.cols());
    datas[idx].JvDot_W.setZero(datas[idx].JvDot_W.rows(), datas[idx].JvDot_W.cols());
    datas[idx].Jw_W.setZero(datas[idx].Jw_W.rows(), datas[idx].Jw_W.cols());
    datas[idx].hasFootData = false;
}

template <typename T>
void LegController<T>::clearLegDynamicsData(int leg) {
    checkLegIndex(leg);
    const std::size_t idx = static_cast<std::size_t>(leg);
    datas[idx].massMatrix.setZero(datas[idx].massMatrix.rows(), datas[idx].massMatrix.cols());
    datas[idx].bias.setZero(datas[idx].bias.size());
    datas[idx].hasDynamicsData = false;
}

template <typename T>
DVec<T> LegController<T>::computeJointPdTorque(int leg) const {
    checkLegIndex(leg);
    const std::size_t idx = static_cast<std::size_t>(leg);
    validateLegShape(idx);

    DVec<T> legTorque = commands[idx].tauFeedForward;
    legTorque += commands[idx].kpJoint * (commands[idx].qDes - datas[idx].q);
    legTorque += commands[idx].kdJoint * (commands[idx].qdDes - datas[idx].qd);
    return legTorque;
}

template <typename T>
DVec<T> LegController<T>::computeSwingLegTorque(int leg) const {
    checkLegIndex(leg);
    const std::size_t idx = static_cast<std::size_t>(leg);
    validateLegShape(idx);

    if (!datas[idx].hasFootData) {
        throw std::runtime_error("Swing leg control requires foot operational-space data");
    }
    if (!datas[idx].hasDynamicsData) {
        throw std::runtime_error("Swing leg control requires leg joint-space dynamics data");
    }

    DVec<T> legTorque = computeSwingLegJointTorque(
        datas[idx].Jv_W,
        datas[idx].JvDot_W,
        datas[idx].massMatrix,
        datas[idx].qd,
        datas[idx].bias,
        commands[idx].pDes_W,
        commands[idx].vDes_W,
        commands[idx].aDes_W,
        datas[idx].p_W,
        datas[idx].v_W,
        commands[idx].kpCartesian,
        commands[idx].kdCartesian,
        commands[idx].forceFeedForward_W);

    legTorque += commands[idx].tauFeedForward;
    return legTorque;
}

template <typename T>
DVec<T> LegController<T>::computeStanceLegTorque(int leg) const {
    checkLegIndex(leg);
    const std::size_t idx = static_cast<std::size_t>(leg);
    validateLegShape(idx);

    if (!datas[idx].hasFootData) {
        throw std::runtime_error("Stance leg control requires foot operational-space data");
    }

    DVec<T> legTorque = computeStanceLegJointTorque(
        datas[idx].Jv_W,
        datas[idx].Jw_W,
        commands[idx].forceFeedForward_W,
        commands[idx].momentFeedForward_W);

    legTorque += commands[idx].tauFeedForward;
    return legTorque;
}

template <typename T>
DVec<T> LegController<T>::computeLegTorque(int leg) const {
    checkLegIndex(leg);
    const auto mode = commands[static_cast<std::size_t>(leg)].mode;

    switch (mode) {
        case LegControlMode::JointPd:
            return computeJointPdTorque(leg);
        case LegControlMode::SwingFoot:
            return computeSwingLegTorque(leg);
        case LegControlMode::StanceWrench:
            return computeStanceLegTorque(leg);
    }

    throw std::runtime_error("Unsupported leg control mode");
}

template <typename T>
void LegController<T>::updateCommand(DVec<T>& tauAll) const {
    if (!_legsEnabled) {
        return;
    }

    for (std::size_t leg = 0; leg < commands.size(); ++leg) {
        const DVec<T> legTorque = computeLegTorque(static_cast<int>(leg));
        _robotModel->setLegTau(static_cast<int>(leg), legTorque, tauAll);
    }
}

template <typename T>
DVec<T> LegController<T>::updateCommand() const {
    DVec<T> tauAll = DVec<T>::Zero(_robotModel->nu());
    updateCommand(tauAll);
    return tauAll;
}

template <typename T>
DVec<T> LegController<T>::extractIndexed(const DVec<T>& src,
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
void LegController<T>::resizeFromModel() {
    commands.clear();
    datas.clear();

    commands.reserve(_robotModel->numLegs());
    datas.reserve(_robotModel->numLegs());
    for (std::size_t leg = 0; leg < _robotModel->numLegs(); ++leg) {
        const auto qCount =
            static_cast<Eigen::Index>(_robotModel->legQIndices(static_cast<int>(leg)).size());
        const auto qdCount =
            static_cast<Eigen::Index>(_robotModel->legQdIndices(static_cast<int>(leg)).size());
        const auto actuatorCount = static_cast<Eigen::Index>(
            _robotModel->legActuatorIndices(static_cast<int>(leg)).size());

        if (qCount != qdCount) {
            throw std::invalid_argument("LegController requires q and qd index counts to match");
        }

        if (qCount != actuatorCount) {
            throw std::invalid_argument(
                "LegController currently requires actuator count to match joint count");
        }

        commands.emplace_back(qCount);
        datas.emplace_back(qCount);
    }
}

template <typename T>
void LegController<T>::checkLegIndex(int leg) const {
    if (leg < 0 || static_cast<std::size_t>(leg) >= commands.size()) {
        throw std::out_of_range("Invalid leg index");
    }
}

template <typename T>
void LegController<T>::validateLegShape(std::size_t leg) const {
    const Eigen::Index dof = datas[leg].dof();

    if (commands[leg].tauFeedForward.size() != dof || commands[leg].qDes.size() != dof ||
        commands[leg].qdDes.size() != dof) {
        throw std::invalid_argument("Leg command vector size does not match leg dof");
    }

    if (commands[leg].kpJoint.rows() != dof || commands[leg].kpJoint.cols() != dof ||
        commands[leg].kdJoint.rows() != dof || commands[leg].kdJoint.cols() != dof) {
        throw std::invalid_argument("Leg joint gain matrix size does not match leg dof");
    }

    if (datas[leg].q.size() != dof || datas[leg].qd.size() != dof ||
        datas[leg].tauEstimate.size() != dof) {
        throw std::invalid_argument("Leg data vector size does not match leg dof");
    }

    if (datas[leg].hasFootData &&
        (datas[leg].Jv_W.rows() != 3 || datas[leg].JvDot_W.rows() != 3 ||
         datas[leg].Jw_W.rows() != 3 || datas[leg].Jv_W.cols() != dof ||
         datas[leg].JvDot_W.cols() != dof || datas[leg].Jw_W.cols() != dof)) {
        throw std::invalid_argument("Leg Jacobian size does not match leg dof");
    }

    if (datas[leg].hasDynamicsData &&
        (datas[leg].massMatrix.rows() != dof || datas[leg].massMatrix.cols() != dof ||
         datas[leg].bias.size() != dof)) {
        throw std::invalid_argument("Leg dynamics size does not match leg dof");
    }
}

template struct LegControllerCommand<float>;
template struct LegControllerCommand<double>;

template struct LegControllerData<float>;
template struct LegControllerData<double>;

template class LegController<float>;
template class LegController<double>;
