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

    forceFeedForward.setZero();
    momentFeedForward.setZero();
    pDes.setZero();
    vDes.setZero();
    aDes.setZero();
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

    forceFeedForward.setZero();
    momentFeedForward.setZero();
    pDes.setZero();
    vDes.setZero();
    aDes.setZero();
    kpCartesian.setZero();
    kdCartesian.setZero();
}

template <typename T>
Eigen::Index LegControllerCommand<T>::dof() const {
    return qDes.size();
}

template <typename T>
LegControllerData<T>::LegControllerData(Eigen::Index dof, Eigen::Index nv) {
    resize(dof, nv);
}

template <typename T>
void LegControllerData<T>::resize(Eigen::Index dof, Eigen::Index nv) {
    if (dof < 0) {
        throw std::invalid_argument("LegControllerData dof must be non-negative");
    }
    if (nv < 0) {
        throw std::invalid_argument("LegControllerData nv must be non-negative");
    }

    q.setZero(dof);
    qd.setZero(dof);
    tauEstimate.setZero(dof);
    JvWorld.setZero(3, nv);
    JvDotWorld.setZero(3, nv);
    JwWorld.setZero(3, nv);

    pWorld.setZero();
    vWorld.setZero();
    hasFootData = false;
}

template <typename T>
void LegControllerData<T>::zero() {
    q.setZero(q.size());
    qd.setZero(qd.size());
    tauEstimate.setZero(tauEstimate.size());
    JvWorld.setZero(JvWorld.rows(), JvWorld.cols());
    JvDotWorld.setZero(JvDotWorld.rows(), JvDotWorld.cols());
    JwWorld.setZero(JwWorld.rows(), JwWorld.cols());

    pWorld.setZero();
    vWorld.setZero();
    hasFootData = false;
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
    clearWholeBodyDynamicsData();
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
                                           const Vec3<T>& pWorld,
                                           const Vec3<T>& vWorld,
                                           const DMat<T>& JvWorld,
                                           const DMat<T>& JvDotWorld,
                                           const DMat<T>& JwWorld) {
    checkLegIndex(leg);
    const std::size_t idx = static_cast<std::size_t>(leg);
    const Eigen::Index nv = _robotModel->nv();

    if (JvWorld.rows() != 3 || JvDotWorld.rows() != 3 || JwWorld.rows() != 3 ||
        JvWorld.cols() != nv || JvDotWorld.cols() != nv || JwWorld.cols() != nv) {
        throw std::invalid_argument("Leg Jacobians must be 3 x full nv");
    }

    datas[idx].pWorld = pWorld;
    datas[idx].vWorld = vWorld;
    datas[idx].JvWorld = JvWorld;
    datas[idx].JvDotWorld = JvDotWorld;
    datas[idx].JwWorld = JwWorld;
    datas[idx].hasFootData = true;
}

template <typename T>
void LegController<T>::clearLegCartesianData(int leg) {
    checkLegIndex(leg);
    const std::size_t idx = static_cast<std::size_t>(leg);
    datas[idx].pWorld.setZero();
    datas[idx].vWorld.setZero();
    datas[idx].JvWorld.setZero(datas[idx].JvWorld.rows(), datas[idx].JvWorld.cols());
    datas[idx].JvDotWorld.setZero(datas[idx].JvDotWorld.rows(), datas[idx].JvDotWorld.cols());
    datas[idx].JwWorld.setZero(datas[idx].JwWorld.rows(), datas[idx].JwWorld.cols());
    datas[idx].hasFootData = false;
}

template <typename T>
void LegController<T>::setWholeBodyDynamicsData(const DVec<T>& qdFull,
                                                const DVec<T>& biasFull,
                                                const DMat<T>& massMatrixFull) {
    const Eigen::Index nv = _robotModel->nv();
    if (qdFull.size() != nv || biasFull.size() != nv ||
        massMatrixFull.rows() != nv || massMatrixFull.cols() != nv) {
        throw std::invalid_argument("Whole-body dynamics data size does not match RobotModel nv");
    }

    _qdFull = qdFull;
    _biasFull = biasFull;
    _massMatrixFull = massMatrixFull;
    _hasWholeBodyDynamics = true;
}

template <typename T>
void LegController<T>::clearWholeBodyDynamicsData() {
    const Eigen::Index nv = _robotModel->nv();
    _qdFull.setZero(nv);
    _biasFull.setZero(nv);
    _massMatrixFull.setZero(nv, nv);
    _hasWholeBodyDynamics = false;
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
    validateWholeBodyDynamics();

    if (!datas[idx].hasFootData) {
        throw std::runtime_error("Swing leg control requires foot operational-space data");
    }

    DVec<T> generalizedForce = computeSwingLegGeneralizedForce(
        datas[idx].JvWorld,
        datas[idx].JvDotWorld,
        _massMatrixFull,
        _qdFull,
        _biasFull,
        commands[idx].pDes,
        commands[idx].vDes,
        commands[idx].aDes,
        datas[idx].pWorld,
        datas[idx].vWorld,
        commands[idx].kpCartesian,
        commands[idx].kdCartesian,
        commands[idx].forceFeedForward);

    DVec<T> legTorque = extractIndexed(
        generalizedForce,
        _robotModel->legQdIndices(leg),
        "leg generalized force");
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

    const DVec<T> generalizedForce = computeStanceLegGeneralizedForce(
        datas[idx].JvWorld,
        datas[idx].JwWorld,
        commands[idx].forceFeedForward,
        commands[idx].momentFeedForward);

    DVec<T> legTorque = extractIndexed(
        generalizedForce,
        _robotModel->legQdIndices(leg),
        "leg generalized force");
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
    const Eigen::Index nv = _robotModel->nv();

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
        datas.emplace_back(qCount, nv);
    }

    clearWholeBodyDynamicsData();
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
    const Eigen::Index nv = _robotModel->nv();

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
        (datas[leg].JvWorld.rows() != 3 || datas[leg].JvDotWorld.rows() != 3 ||
         datas[leg].JwWorld.rows() != 3 || datas[leg].JvWorld.cols() != nv ||
         datas[leg].JvDotWorld.cols() != nv || datas[leg].JwWorld.cols() != nv)) {
        throw std::invalid_argument("Leg Jacobian size does not match RobotModel nv");
    }
}

template <typename T>
void LegController<T>::validateWholeBodyDynamics() const {
    const Eigen::Index nv = _robotModel->nv();
    if (!_hasWholeBodyDynamics) {
        throw std::runtime_error("Whole-body dynamics data is required for swing leg control");
    }

    if (_qdFull.size() != nv || _biasFull.size() != nv ||
        _massMatrixFull.rows() != nv || _massMatrixFull.cols() != nv) {
        throw std::invalid_argument("Whole-body dynamics data size does not match RobotModel nv");
    }
}

template struct LegControllerCommand<float>;
template struct LegControllerCommand<double>;

template struct LegControllerData<float>;
template struct LegControllerData<double>;

template class LegController<float>;
template class LegController<double>;
