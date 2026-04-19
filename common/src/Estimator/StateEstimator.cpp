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
    state_estimate.psi = std::atan2(R_WT(1, 0), R_WT(0, 0));
}

template class StateEstimator<float>;
template class StateEstimator<double>;
