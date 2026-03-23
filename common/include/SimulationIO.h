#ifndef SIMULATION_IO_H
#define SIMULATION_IO_H

#include "Robot/RobotParams.h"

template <typename T>
struct RobotLegState {
    DVec<T> q;
    DVec<T> qd;
    DVec<T> tauEstimate;
    Vec3<T> footPosWorld = Vec3<T>::Zero();
    Vec3<T> footVelWorld = Vec3<T>::Zero();
    bool contact = true;

    void resize(Eigen::Index q_size, Eigen::Index qd_size, Eigen::Index tau_size) {
        q.resize(q_size);
        qd.resize(qd_size);
        tauEstimate.resize(tau_size);
        contact = true;
    }
};

template <typename T>
struct RobotArmState {
    DVec<T> q;
    DVec<T> qd;
    DVec<T> tauEstimate;
    Vec3<T> handPosWorld = Vec3<T>::Zero();
    Vec3<T> handVelWorld = Vec3<T>::Zero();
    bool contact = true;

    void resize(Eigen::Index q_size, Eigen::Index qd_size, Eigen::Index tau_size) {
        q.resize(q_size);
        qd.resize(qd_size);
        tauEstimate.resize(tau_size);
        contact = true;
    }
};

template <typename T>
struct CheaterState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double time{0.0};
    Vec3<T> basePos = Vec3<T>::Zero();
    Quat<T> baseQuat = Quat<T>::Zero();
    Vec3<T> baseLinVel = Vec3<T>::Zero();
    Vec3<T> baseAngVel = Vec3<T>::Zero();
    Vec3<T> baseLinAcc = Vec3<T>::Zero();
    Vec3<T> baseAngAcc = Vec3<T>::Zero();
    vectorAligned<RobotLegState<T>> legs;
    vectorAligned<RobotArmState<T>> arms;

    void resize(const RobotParams<T>& params) {
        legs.resize(params.legs.size());
        for (std::size_t leg = 0; leg < params.legs.size(); ++leg) {
            const auto& joints = params.legs[leg].joints;
            const Eigen::Index q_size = static_cast<Eigen::Index>(joints.q_idx.size());
            const Eigen::Index qd_size = static_cast<Eigen::Index>(joints.qd_idx.size());
            const Eigen::Index tau_size = static_cast<Eigen::Index>(
                joints.actuator_idx.empty() ? joints.qd_idx.size() : joints.actuator_idx.size());
            legs[leg].resize(q_size, qd_size, tau_size);
        }

        arms.resize(params.arms.size());
        for (std::size_t arm = 0; arm < params.arms.size(); ++arm) {
            const auto& joints = params.arms[arm].joints;
            const Eigen::Index q_size = static_cast<Eigen::Index>(joints.q_idx.size());
            const Eigen::Index qd_size = static_cast<Eigen::Index>(joints.qd_idx.size());
            const Eigen::Index tau_size = static_cast<Eigen::Index>(
                joints.actuator_idx.empty() ? joints.qd_idx.size() : joints.actuator_idx.size());
            arms[arm].resize(q_size, qd_size, tau_size);
        }
    }
};

template <typename T>
struct RobotCommand {
    DVec<T> tau;

    void resize(int nu) {
        tau.resize(nu);
    }
};

#endif  // SIMULATION_IO_H
