#include "LegPosInitializer.h"

#include <array>
#include <stdexcept>

template <typename T>
LegPosInitializer<T>::LegPosInitializer(const RobotParams<T>* params, T end_time, float dt)
    : _params(params),
      _end_time(end_time),
      _curr_time(0),
      _dt(static_cast<T>(dt)) {
    if (_params == nullptr) {
        throw std::invalid_argument("LegPosInitializer received null RobotParams");
    }
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

    std::array<T, MIThumanoid::num_leg_joint * 2> jpos{};
    if (!_jpos_trj.getCurvePoint(_curr_time, jpos.data())) {
        throw std::runtime_error("LegPosInitializer failed to evaluate joint spline");
    }

    std::size_t joint_idx = 0;
    for (std::size_t leg = 0; leg < leg_ctrl->numLegs(); ++leg) {
        auto& command = leg_ctrl->commands[leg];
        for (Eigen::Index jidx = 0; jidx < command.dof(); ++jidx) {
            if (joint_idx >= MIThumanoid::num_leg_joint * 2) {
                throw std::runtime_error(
                    "LegPosInitializer currently assumes MIThumanoid::num_leg_joint * 2");
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
    std::array<T, 3 * MIThumanoid::num_leg_joint * 2> ini{};
    std::array<T, 3 * MIThumanoid::num_leg_joint * 2> fin{};
    std::array<T, MIThumanoid::num_leg_joint * 2> mid_storage{};
    T* mid[1] = {mid_storage.data()};

    std::size_t joint_idx = 0;
    for (std::size_t leg = 0; leg < leg_ctrl.numLegs(); ++leg) {
        const auto& q = leg_ctrl.datas[leg].q;
        const auto& q_idx = leg_ctrl.model().legQIndices(static_cast<int>(leg));

        for (Eigen::Index jidx = 0; jidx < q.size(); ++jidx) {
            if (joint_idx >= MIThumanoid::num_leg_joint * 2) {
                throw std::runtime_error(
                    "LegPosInitializer currently assumes MIThumanoid::num_leg_joint * 2");
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

    if (!_jpos_trj.SetParam(ini.data(), fin.data(), mid, _end_time)) {
        throw std::runtime_error("LegPosInitializer failed to initialize joint spline");
    }

    _splineInitialized = true;
}

template class LegPosInitializer<float>;
template class LegPosInitializer<double>;
