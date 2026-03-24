#include "ArmPosInitializer.h"

#include <stdexcept>
#include <vector>

template <typename T>
ArmPosInitializer<T>::ArmPosInitializer(const RobotParams<T>* params, T end_time, float dt)
    : _params(params),
      _end_time(end_time),
      _curr_time(0),
      _dt(static_cast<T>(dt)) {
    if (_params == nullptr) {
        throw std::invalid_argument("ArmPosInitializer received null RobotParams");
    }

    _totalArmDof = totalArmDof();
    if (_totalArmDof == 0) {
        throw std::invalid_argument("ArmPosInitializer requires at least one arm joint");
    }

    _jpos_trj.setDimension(static_cast<int>(_totalArmDof));
}

template <typename T>
bool ArmPosInitializer<T>::IsInitialized(ArmController<T>* arm_ctrl) {
    if (arm_ctrl == nullptr) {
        throw std::invalid_argument("ArmPosInitializer received null ArmController");
    }

    if (!_splineInitialized) {
        initializeSpline(*arm_ctrl);
    }

    _curr_time += _dt;

    std::vector<T> jpos(_totalArmDof, T(0));
    if (!_jpos_trj.getCurvePoint(_curr_time, jpos.data())) {
        throw std::runtime_error("ArmPosInitializer failed to evaluate joint spline");
    }

    std::size_t joint_idx = 0;
    for (std::size_t arm = 0; arm < arm_ctrl->numArms(); ++arm) {
        auto& command = arm_ctrl->commands[arm];
        for (Eigen::Index jidx = 0; jidx < command.dof(); ++jidx) {
            if (joint_idx >= _totalArmDof) {
                throw std::runtime_error("ArmPosInitializer arm dof exceeds configured spline size");
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
void ArmPosInitializer<T>::initializeSpline(const ArmController<T>& arm_ctrl) {
    std::vector<T> ini(3 * _totalArmDof, T(0));
    std::vector<T> fin(3 * _totalArmDof, T(0));
    std::vector<T> mid_storage(_totalArmDof, T(0));
    T* mid[1] = {mid_storage.data()};

    std::size_t joint_idx = 0;
    for (std::size_t arm = 0; arm < arm_ctrl.numArms(); ++arm) {
        const auto& q = arm_ctrl.datas[arm].q;
        const auto& q_idx = arm_ctrl.model().armQIndices(static_cast<int>(arm));

        for (Eigen::Index jidx = 0; jidx < q.size(); ++jidx) {
            if (joint_idx >= _totalArmDof) {
                throw std::runtime_error("ArmPosInitializer arm dof exceeds configured spline size");
            }

            const int qpos_idx = q_idx[static_cast<std::size_t>(jidx)];
            if (qpos_idx < 0 || qpos_idx >= _params->default_qpos.size()) {
                throw std::out_of_range("ArmPosInitializer q index is out of range");
            }

            ini[joint_idx] = q[jidx];
            fin[joint_idx] = _params->default_qpos[qpos_idx];
            mid_storage[joint_idx] = fin[joint_idx];
            ++joint_idx;
        }
    }

    std::size_t flat_idx = 0;
    for (const auto& arm : _params->arms) {
        const std::size_t arm_dof = arm.joints.q_idx.size();
        if (arm_dof > 3) {
            fin[flat_idx + 3] -= 0.66;
            mid_storage[flat_idx + 3] -= 0.66;
        }
        flat_idx += arm_dof;
    }

    if (!_jpos_trj.SetParam(ini.data(), fin.data(), mid, _end_time)) {
        throw std::runtime_error("ArmPosInitializer failed to initialize joint spline");
    }

    _splineInitialized = true;
}

template <typename T>
std::size_t ArmPosInitializer<T>::totalArmDof() const {
    if (_params == nullptr) {
        throw std::runtime_error("ArmPosInitializer requires RobotParams");
    }

    std::size_t total_dof = 0;
    for (const auto& arm : _params->arms) {
        total_dof += static_cast<std::size_t>(arm.joints.q_idx.size());
    }

    return total_dof;
}

template class ArmPosInitializer<float>;
template class ArmPosInitializer<double>;
