#include "StateEstimator/StateEstimator.h"

#include <Eigen/Geometry>

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

    baseQuat.normalize();
    state_estimate.R_WB = baseQuat.toRotationMatrix();
    state_estimate.R_BW = state_estimate.R_WB.transpose();
}

template class StateEstimator<float>;
template class StateEstimator<double>;
