#include "StateEstimator/StateEstimator.h"

#include <cmath>

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
            // TODO: Implement non-cheater estimator modes.
            break;
    }
}

template <typename T>
void StateEstimator<T>::updateCheater(const CheaterState<T>& cheater_state,
                                      StateEstimate<T>& state_estimate) const {
    state_estimate.copyFrom(cheater_state);

    Quat<T> baseQuat = state_estimate.baseQuat;
    baseQuat.normalize();

    const T w = baseQuat.w();
    const T x = baseQuat.x();
    const T y = baseQuat.y();
    const T z = baseQuat.z();

    state_estimate.psi =
        std::atan2(T(2) * (w * z + x * y), T(1) - T(2) * (y * y + z * z));
}

template class StateEstimator<float>;
template class StateEstimator<double>;
