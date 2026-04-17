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
    DMat<T> JvWorld;
    DMat<T> JvDotWorld;
    DMat<T> JwWorld;
    DMat<T> massMatrix;
    DVec<T> bias;
    bool hasFootKinematics = false;
    bool hasLegDynamics = false;
    bool contact = true;

    bool matchesLayout(Eigen::Index q_size, Eigen::Index qd_size, Eigen::Index tau_size) const {
        return q.size() == q_size &&
               qd.size() == qd_size &&
               tauEstimate.size() == tau_size &&
               JvWorld.rows() == 3 &&
               JvWorld.cols() == qd_size &&
               JvDotWorld.rows() == 3 &&
               JvDotWorld.cols() == qd_size &&
               JwWorld.rows() == 3 &&
               JwWorld.cols() == qd_size &&
               massMatrix.rows() == qd_size &&
               massMatrix.cols() == qd_size &&
               bias.size() == qd_size;
    }

    void resize(Eigen::Index q_size, Eigen::Index qd_size, Eigen::Index tau_size) {
        if (matchesLayout(q_size, qd_size, tau_size)) {
            return;
        }

        q.resize(q_size);
        qd.resize(qd_size);
        tauEstimate.resize(tau_size);
        JvWorld.setZero(3, qd_size);
        JvDotWorld.setZero(3, qd_size);
        JwWorld.setZero(3, qd_size);
        massMatrix.setZero(qd_size, qd_size);
        bias.setZero(qd_size);
        hasFootKinematics = false;
        hasLegDynamics = false;
        contact = true;
    }
};

template <typename T>
struct WholeBodyDynamicsState {
    DVec<T> qd;
    DVec<T> bias;
    DMat<T> massMatrix;

    bool matchesLayout(Eigen::Index nv) const {
        return qd.size() == nv && bias.size() == nv &&
               massMatrix.rows() == nv && massMatrix.cols() == nv;
    }

    void resize(Eigen::Index nv) {
        if (matchesLayout(nv)) {
            return;
        }

        qd.setZero(nv);
        bias.setZero(nv);
        massMatrix.setZero(nv, nv);
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

    bool matchesLayout(Eigen::Index q_size, Eigen::Index qd_size, Eigen::Index tau_size) const {
        return q.size() == q_size && qd.size() == qd_size && tauEstimate.size() == tau_size;
    }

    void resize(Eigen::Index q_size, Eigen::Index qd_size, Eigen::Index tau_size) {
        if (matchesLayout(q_size, qd_size, tau_size)) {
            return;
        }

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
    Quat<T> baseQuat = Quat<T>::Identity();
    Vec3<T> baseLinVel = Vec3<T>::Zero();
    Vec3<T> baseAngVel = Vec3<T>::Zero();
    Vec3<T> baseLinAcc = Vec3<T>::Zero();
    Vec3<T> baseAngAcc = Vec3<T>::Zero();
    vectorAligned<RobotLegState<T>> legs;
    vectorAligned<RobotArmState<T>> arms;
    WholeBodyDynamicsState<T> dynamics;

    void resize(const RobotParams<T>& params) {
        dynamics.resize(params.nv);

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
