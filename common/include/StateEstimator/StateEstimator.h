#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include "SimulationIO.h"

enum class StateEstimatorMode {
    Cheater,
    Estimated,  // TODO: Implement non-cheater estimator modes when sensors are wired in.
};

template <typename T>
struct StateEstimate {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double time{0.0};
    Vec3<T> torsoPos_W = Vec3<T>::Zero();
    Quat<T> torsoQuat_W = Quat<T>::Identity();
    T psi{0};
    Vec3<T> torsoLinVel_W = Vec3<T>::Zero();
    Vec3<T> torsoAngVel_W = Vec3<T>::Zero();
    Vec3<T> torsoLinAcc_W = Vec3<T>::Zero();
    Vec3<T> torsoAngAcc_W = Vec3<T>::Zero();
    vectorAligned<RobotLegState<T>> legs;
    vectorAligned<RobotArmState<T>> arms;
    StandingFootKinematics<T> standingFeet;
    WholeBodyDynamicsState<T> dynamics;

    void resize(const RobotParams<T>& params) {
        dynamics.resize(params.nv);

        Eigen::Index total_leg_qd_size = 0;
        legs.resize(params.legs.size());
        for (std::size_t leg = 0; leg < params.legs.size(); ++leg) {
            const auto& joints = params.legs[leg].joints;
            const Eigen::Index q_size = static_cast<Eigen::Index>(joints.q_idx.size());
            const Eigen::Index qd_size = static_cast<Eigen::Index>(joints.qd_idx.size());
            const Eigen::Index tau_size = static_cast<Eigen::Index>(
                joints.actuator_idx.empty() ? joints.qd_idx.size() : joints.actuator_idx.size());
            legs[leg].resize(q_size, qd_size, tau_size);
            total_leg_qd_size += qd_size;
        }
        standingFeet.resize(total_leg_qd_size);

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

    void copyFrom(const CheaterState<T>& cheater_state) {
        time = cheater_state.time;
        torsoPos_W = cheater_state.torsoPos_W;
        torsoQuat_W = cheater_state.torsoQuat_W;
        torsoLinVel_W = cheater_state.torsoLinVel_W;
        torsoAngVel_W = cheater_state.torsoAngVel_W;
        torsoLinAcc_W = cheater_state.torsoLinAcc_W;
        torsoAngAcc_W = cheater_state.torsoAngAcc_W;
        legs = cheater_state.legs;
        arms = cheater_state.arms;
        dynamics = cheater_state.dynamics;
    }
};

template <typename T>
class StateEstimator {
public:
    explicit StateEstimator(StateEstimatorMode mode = StateEstimatorMode::Cheater);

    void update(const CheaterState<T>& cheater_state, StateEstimate<T>& state_estimate) const;

private:
    void updateCheater(const CheaterState<T>& cheater_state,
                       StateEstimate<T>& state_estimate) const;

    StateEstimatorMode _mode{StateEstimatorMode::Cheater};
};

#endif  // STATE_ESTIMATOR_H
