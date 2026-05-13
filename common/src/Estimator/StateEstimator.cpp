#include "StateEstimator/StateEstimator.h"

#include <cmath>

#include "Utilities/AngleUtils.h"

template <typename T>
StateEstimator<T>::StateEstimator(StateEstimatorMode mode) : _mode(mode) {}

template <typename T>
void StateEstimator<T>::update(const CheaterState<T>& cheater_state,
                               StateEstimate<T>& state_estimate) const {
    switch (_mode) {
        case StateEstimatorMode::Cheater:
            updateCheater(cheater_state, state_estimate);
            break;
        case StateEstimatorMode::Estimated:
            // TODO: Cheater mode is the only supported path for now.
            // Implement non-cheater estimator modes once the sensor pipeline exists.
            break;
    }
}

template <typename T>
void StateEstimator<T>::updateCheater(const CheaterState<T>& cheater_state,
                                      StateEstimate<T>& state_estimate) const {
    state_estimate.copyFrom(cheater_state);

    const Mat3<T> R_WT = state_estimate.torsoQuat_W.toRotationMatrix();
    const double yawWrappedNow = std::atan2(static_cast<double>(R_WT(1, 0)),
                                            static_cast<double>(R_WT(0, 0)));
    const double timeNow = cheater_state.time;

    if (!_hasYawHistory || !std::isfinite(timeNow) || timeNow < (_lastYawTime - 1e-9)) {
        state_estimate.yaw_W_wrapped = static_cast<T>(yawWrappedNow);
        state_estimate.yaw_W_unwrapped = static_cast<T>(yawWrappedNow);
        state_estimate.yawRate_W = static_cast<T>(0.0);
    } else {
        const double yawUnwrappedNow =
            unwrapAngle(yawWrappedNow, _lastYawWrapped, _lastYawUnwrapped);
        const double dt = timeNow - _lastYawTime;
        state_estimate.yaw_W_wrapped = static_cast<T>(yawWrappedNow);
        state_estimate.yaw_W_unwrapped = static_cast<T>(yawUnwrappedNow);
        state_estimate.yawRate_W = static_cast<T>(
            (dt > 0.0) ? ((yawUnwrappedNow - _lastYawUnwrapped) / dt) : 0.0);
    }
    state_estimate.psi = state_estimate.yaw_W_unwrapped;

    _lastYawWrapped = static_cast<double>(state_estimate.yaw_W_wrapped);
    _lastYawUnwrapped = static_cast<double>(state_estimate.yaw_W_unwrapped);
    _lastYawTime = timeNow;
    _hasYawHistory = true;
}

template class StateEstimator<float>;
template class StateEstimator<double>;
