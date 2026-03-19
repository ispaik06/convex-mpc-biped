#include "RobotModel.h"

#include <stdexcept>

namespace {
template <typename T>
const LegParams<T>& checkedLeg(const RobotParams<T>* params, int leg) {
    if (!params) {
        throw std::runtime_error("RobotModel has null RobotParams");
    }
    if (leg < 0 || static_cast<std::size_t>(leg) >= params->legs.size()) {
        throw std::out_of_range("Invalid leg index");
    }
    return params->legs.at(static_cast<std::size_t>(leg));
}

template <typename T>
void checkVectorIndex(const DVec<T>& vec, int idx, const char* name) {
    if (idx < 0 || idx >= vec.size()) {
        throw std::out_of_range(std::string(name) + " index is out of range");
    }
}
}  // namespace

template <typename T>
bool RobotModel<T>::validate() const {
    if (_params == nullptr) {
        return false;
    }

    if (_params->nq < 0 || _params->nv < 0 || _params->nu < 0) {
        return false;
    }

    if (_params->bodyMass < T(0)) {
        return false;
    }

    if (_params->default_qpos.size() != 0 && _params->nq != 0 &&
        _params->default_qpos.size() != _params->nq) {
        return false;
    }

    for (const auto& leg : _params->legs) {
        const auto joint_count = static_cast<Eigen::Index>(leg.joints.joint_idx.size());
        const auto actuator_count = static_cast<Eigen::Index>(leg.joints.actuator_idx.size());
        if (joint_count == 0) {
            return false;
        }
        if (leg.joints.motorTauMax.size() != 0 &&
            leg.joints.motorTauMax.size() != (actuator_count == 0 ? joint_count : actuator_count)) {
            return false;
        }
        if (leg.joints.damping.size() != 0 && leg.joints.damping.size() != joint_count) {
            return false;
        }
        if (leg.joints.dryFriction.size() != 0 && leg.joints.dryFriction.size() != joint_count) {
            return false;
        }
    }

    for (const auto& arm : _params->arms) {
        const auto joint_count = static_cast<Eigen::Index>(arm.joints.joint_idx.size());
        const auto actuator_count = static_cast<Eigen::Index>(arm.joints.actuator_idx.size());
        if (joint_count == 0) {
            return false;
        }
        if (arm.joints.motorTauMax.size() != 0 &&
            arm.joints.motorTauMax.size() != (actuator_count == 0 ? joint_count : actuator_count)) {
            return false;
        }
        if (arm.joints.damping.size() != 0 && arm.joints.damping.size() != joint_count) {
            return false;
        }
        if (arm.joints.dryFriction.size() != 0 && arm.joints.dryFriction.size() != joint_count) {
            return false;
        }
    }

    return true;
}

template <typename T>
const std::vector<int>& RobotModel<T>::legJointIndices(int leg) const {
    return checkedLeg(_params, leg).joints.joint_idx;
}

template <typename T>
const std::vector<int>& RobotModel<T>::legActuatorIndices(int leg) const {
    const auto& leg_params = checkedLeg(_params, leg);
    if (!leg_params.joints.actuator_idx.empty()) {
        return leg_params.joints.actuator_idx;
    }
    return leg_params.joints.joint_idx;
}

template <typename T>
DVec<T> RobotModel<T>::getLegQ(const DVec<T>& q, int leg) const {
    const auto& joint_idx = legJointIndices(leg);
    DVec<T> q_leg(static_cast<Eigen::Index>(joint_idx.size()));
    for (Eigen::Index i = 0; i < q_leg.size(); ++i) {
        checkVectorIndex(q, joint_idx[static_cast<std::size_t>(i)], "q");
        q_leg[i] = q[joint_idx[static_cast<std::size_t>(i)]];
    }
    return q_leg;
}

template <typename T>
DVec<T> RobotModel<T>::getLegQd(const DVec<T>& qd, int leg) const {
    const auto& joint_idx = legJointIndices(leg);
    DVec<T> qd_leg(static_cast<Eigen::Index>(joint_idx.size()));
    for (Eigen::Index i = 0; i < qd_leg.size(); ++i) {
        checkVectorIndex(qd, joint_idx[static_cast<std::size_t>(i)], "qd");
        qd_leg[i] = qd[joint_idx[static_cast<std::size_t>(i)]];
    }
    return qd_leg;
}

template <typename T>
void RobotModel<T>::setLegTau(int leg, const DVec<T>& tau_leg, DVec<T>& tau_all) const {
    const auto& actuator_idx = legActuatorIndices(leg);
    if (tau_leg.size() != static_cast<Eigen::Index>(actuator_idx.size())) {
        throw std::invalid_argument("tau_leg size does not match leg actuator count");
    }

    for (Eigen::Index i = 0; i < tau_leg.size(); ++i) {
        const int idx = actuator_idx[static_cast<std::size_t>(i)];
        checkVectorIndex(tau_all, idx, "tau_all");
        tau_all[idx] = tau_leg[i];
    }
}

template <typename T>
int RobotModel<T>::footBodyId(int leg) const {
    return checkedLeg(_params, leg).foot.body_id;
}

template <typename T>
int RobotModel<T>::footSiteId(int leg) const {
    return checkedLeg(_params, leg).foot.site_id;
}

template <typename T>
const Vec3<T>& RobotModel<T>::hipLocationFromBody(int leg) const {
    return checkedLeg(_params, leg).hipLocation_from_body;
}

template class RobotModel<float>;
template class RobotModel<double>;
