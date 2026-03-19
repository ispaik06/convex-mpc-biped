#include <stdexcept>
#include <string>

#include "Controllers/LegController.h"

template <typename T>
LegControllerCommand<T>::LegControllerCommand(Eigen::Index dof) {
    resize(dof);
}

template <typename T>
void LegControllerCommand<T>::resize(Eigen::Index dof) {
    if (dof < 0) {
        throw std::invalid_argument("LegControllerCommand dof must be non-negative");
    }

    tauFeedForward.setZero(dof);
    qDes.setZero(dof);
    qdDes.setZero(dof);
    kpJoint.setZero(dof, dof);
    kdJoint.setZero(dof, dof);

    forceFeedForward.setZero();
    pDes.setZero();
    vDes.setZero();
    kpCartesian.setZero();
    kdCartesian.setZero();
}

template <typename T>
void LegControllerCommand<T>::zero() {
    tauFeedForward.setZero(tauFeedForward.size());
    qDes.setZero(qDes.size());
    qdDes.setZero(qdDes.size());
    kpJoint.setZero(kpJoint.rows(), kpJoint.cols());
    kdJoint.setZero(kdJoint.rows(), kdJoint.cols());

    forceFeedForward.setZero();
    pDes.setZero();
    vDes.setZero();
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
    J.setZero(3, dof);

    p.setZero();
    v.setZero();
    hasCartesianData = false;
}

template <typename T>
void LegControllerData<T>::zero() {
    q.setZero(q.size());
    qd.setZero(qd.size());
    tauEstimate.setZero(tauEstimate.size());
    J.setZero(J.rows(), J.cols());

    p.setZero();
    v.setZero();
    hasCartesianData = false;
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
                                           const Vec3<T>& p,
                                           const Vec3<T>& v,
                                           const DMat<T>& J) {
    checkLegIndex(leg);
    const std::size_t idx = static_cast<std::size_t>(leg);
    const Eigen::Index dof = datas[idx].dof();

    if (J.rows() != 3 || J.cols() != dof) {
        throw std::invalid_argument("Leg Jacobian must be 3 x leg dof");
    }

    datas[idx].p = p;
    datas[idx].v = v;
    datas[idx].J = J;
    datas[idx].hasCartesianData = true;
}

template <typename T>
void LegController<T>::clearLegCartesianData(int leg) {
    checkLegIndex(leg);
    const std::size_t idx = static_cast<std::size_t>(leg);
    const Eigen::Index dof = datas[idx].dof();

    datas[idx].p.setZero();
    datas[idx].v.setZero();
    datas[idx].J.setZero(3, dof);
    datas[idx].hasCartesianData = false;
}

template <typename T>
DVec<T> LegController<T>::computeLegTorque(int leg) const {
    checkLegIndex(leg);
    const std::size_t idx = static_cast<std::size_t>(leg);
    validateLegShape(idx);

    DVec<T> legTorque = commands[idx].tauFeedForward;
    legTorque += commands[idx].kpJoint * (commands[idx].qDes - datas[idx].q);
    legTorque += commands[idx].kdJoint * (commands[idx].qdDes - datas[idx].qd);

    if (datas[idx].hasCartesianData) {
        Vec3<T> footForce = commands[idx].forceFeedForward;
        footForce += commands[idx].kpCartesian * (commands[idx].pDes - datas[idx].p);
        footForce += commands[idx].kdCartesian * (commands[idx].vDes - datas[idx].v);
        legTorque += datas[idx].J.transpose() * footForce;
    }

    return legTorque;
}

template <typename T>
void LegController<T>::updateCommand(DVec<T>& tauAll) const {
    tauAll.setZero(_robotModel->nu());
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
    DVec<T> tauAll;
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
        const auto jointCount =
            static_cast<Eigen::Index>(_robotModel->legJointIndices(static_cast<int>(leg)).size());
        const auto actuatorCount = static_cast<Eigen::Index>(
            _robotModel->legActuatorIndices(static_cast<int>(leg)).size());

        // if (jointCount != actuatorCount) {
        //     throw std::invalid_argument(
        //         "LegController currently requires actuator count to match joint count");
        // }

        commands.emplace_back(jointCount);
        datas.emplace_back(jointCount);
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

    if (datas[leg].hasCartesianData && (datas[leg].J.rows() != 3 || datas[leg].J.cols() != dof)) {
        throw std::invalid_argument("Leg Jacobian size does not match Cartesian task shape");
    }
}

template struct LegControllerCommand<float>;
template struct LegControllerCommand<double>;

template struct LegControllerData<float>;
template struct LegControllerData<double>;

template class LegController<float>;
template class LegController<double>;
