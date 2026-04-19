#include "LegPosInitializer.h"

#include <stdexcept>
#include <vector>

#include "InitialPoseConfig.h"

template <typename T>
LegPosInitializer<T>::LegPosInitializer(const RobotParams<T>* params, T end_time, float dt)
    : _params(params),
      _end_time(end_time),
      _curr_time(0),
      _dt(static_cast<T>(dt)) {
    if (_params == nullptr) {
        throw std::invalid_argument("LegPosInitializer received null RobotParams");
    }

    _totalLegDof = totalLegDof();
    if (_totalLegDof == 0) {
        throw std::invalid_argument("LegPosInitializer requires at least one leg joint");
    }

    _jpos_trj.setDimension(static_cast<int>(_totalLegDof));
}

template <typename T>
bool LegPosInitializer<T>::IsInitialized(LegController<T>* leg_ctrl) {
    if (leg_ctrl == nullptr) {
        throw std::invalid_argument("LegPosInitializer received null LegController");
    }

    if (!_splineInitialized) {
        initializeSpline(*leg_ctrl);
    }

    _curr_time += _dt;

    std::vector<T> jpos(_totalLegDof, T(0));
    if (!_jpos_trj.getCurvePoint(_curr_time, jpos.data())) {
        throw std::runtime_error("LegPosInitializer failed to evaluate joint spline");
    }

    std::size_t joint_idx = 0;
    for (std::size_t leg = 0; leg < leg_ctrl->numLegs(); ++leg) {
        auto& command = leg_ctrl->commands[leg];
        for (Eigen::Index jidx = 0; jidx < command.dof(); ++jidx) {
            if (joint_idx >= _totalLegDof) {
                throw std::runtime_error("LegPosInitializer leg dof exceeds configured spline size");
            }
            command.tauFeedForward[jidx] = T(0);
            command.qDes[jidx] = jpos[joint_idx];
            command.qdDes[jidx] = T(0);
            ++joint_idx;
        }
    }

    return _curr_time >= _end_time;
}

template <typename T>
void LegPosInitializer<T>::initializeSpline(const LegController<T>& leg_ctrl) {
    std::vector<T> ini(3 * _totalLegDof, T(0));
    std::vector<T> fin(3 * _totalLegDof, T(0));
    std::vector<T> mid_storage(_totalLegDof, T(0));
    T* mid[1] = {mid_storage.data()};

    std::size_t joint_idx = 0;
    for (std::size_t leg = 0; leg < leg_ctrl.numLegs(); ++leg) {
        const auto& q = leg_ctrl.datas[leg].q;
        const auto& q_idx = leg_ctrl.model().legQIndices(static_cast<int>(leg));

        for (Eigen::Index jidx = 0; jidx < q.size(); ++jidx) {
            if (joint_idx >= _totalLegDof) {
                throw std::runtime_error("LegPosInitializer leg dof exceeds configured spline size");
            }

            const int qpos_idx = q_idx[static_cast<std::size_t>(jidx)];
            if (qpos_idx < 0 || qpos_idx >= _params->default_qpos.size()) {
                throw std::out_of_range("LegPosInitializer q index is out of range");
            }

            ini[joint_idx] = q[jidx];
            fin[joint_idx] = _params->default_qpos[qpos_idx];
            mid_storage[joint_idx] = fin[joint_idx];
            ++joint_idx;
        }
    }

    applyConfiguredJointOffsets(fin,
                                mid_storage,
                                _params->legs,
                                getInitialPoseConfig().legJointOffsets);

    if (!_jpos_trj.SetParam(ini.data(), fin.data(), mid, _end_time)) {
        throw std::runtime_error("LegPosInitializer failed to initialize joint spline");
    }

    _splineInitialized = true;
}

template <typename T>
std::size_t LegPosInitializer<T>::totalLegDof() const {
    if (_params == nullptr) {
        throw std::runtime_error("LegPosInitializer requires RobotParams");
    }

    std::size_t total_dof = 0;
    for (const auto& leg : _params->legs) {
        total_dof += static_cast<std::size_t>(leg.joints.q_idx.size());
    }

    return total_dof;
}

template class LegPosInitializer<float>;
template class LegPosInitializer<double>;
