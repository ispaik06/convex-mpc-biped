#include <stdexcept>
#include <iostream>

#include "RobotRunner.h"

void RobotRunner::init(RobotParams<double>* params, double timestep, const UserCommand* userCommand) {
    _params = params;
    _model = RobotModel<double>(_params);
    _tiemstep = timestep;
    _robot_ctrl->_userCommand = userCommand;
    if(!_model.validate()) {
        throw std::runtime_error("Invalid RobotParams");
    }
    _robotType = _params->roboType;
    _legController = std::make_unique<LegController<double>>(_model);
    _armController = std::make_unique<ArmController<double>>(_model);
    _legPosInitializer = std::make_unique<LegPosInitializer<double>>(_params, 1.1, _tiemstep);
    _armPosInitializer = std::make_unique<ArmPosInitializer<double>>(_params, 1.0, _tiemstep);
    initializeJointTrackingGains();
}

void RobotRunner::run(const StateEstimate<double>& state, RobotCommand<double>& command) {
    if (!_legController) {
        throw std::runtime_error("RobotRunner::init must be called before run");
    }

    setupStep(state);

    const bool armPosInitialized = _armPosInitializer->IsInitialized(_armController.get());
    for (auto& armCommand : _armController->commands) {
        _armJointTrackingGains.applyTo(armCommand);
    }

    if(1 | !_legPosInitializer->IsInitialized(_legController.get())) {
        // Initial pose control
        for (auto& legCommand : _legController->commands) {
            _legJointTrackingGains.applyTo(legCommand);
        }
    }
    else {
        // Run main Controller
        _robot_ctrl->runController();
    }

    _legController->setEnabled(true);
    _armController->setEnabled(true);
    composeCommand(command);

    finalizeStep();
}

void RobotRunner::composeCommand(RobotCommand<double>& command) const {
    command.tau.setZero(_model.nu());
    _legController->updateCommand(command.tau);
    _armController->updateCommand(command.tau);
}

void RobotRunner::initializeJointTrackingGains() {
    if (_params == nullptr) {
        throw std::runtime_error("RobotRunner::init must set RobotParams before gains");
    }

    switch (_robotType) {
        case RobotType::MIT_HUMANOID:
            // Order follows sim/src/MitHumanoidSpec.cpp:
            // [hip_yaw, hip_abad, hip_pitch, knee, ankle]
            _legJointTrackingGains.set({60.0, 50.0, 55.0, 60.0, 40.0},
                                       {15.0, 10.0, 7.0, 9.0, 10.0});
            // _legJointTrackingGains.setConstant(
            //     static_cast<Eigen::Index>(_params->legs.front().joints.q_idx.size()), 50.0, 20.0);
            // Order follows sim/src/MitHumanoidSpec.cpp:
            // [shoulder_pitch, shoulder_abad, shoulder_yaw, elbow]
            _armJointTrackingGains.set({30.0, 8.0, 8.0, 2.0},
                                       {5.0, 1., 1.0, 0.75});
            break;
        case RobotType::UNITREE_G1:
        case RobotType::UNITREE_H1:
            if (_params->legs.empty() || _params->arms.empty()) {
                throw std::runtime_error("RobotParams is missing limb data for gain initialization");
            }
            _legJointTrackingGains.setConstant(
                static_cast<Eigen::Index>(_params->legs.front().joints.q_idx.size()), 20.0, 16.0);
            _armJointTrackingGains.setConstant(
                static_cast<Eigen::Index>(_params->arms.front().joints.q_idx.size()), 3.0, 2.0);
            break;
        default:
            throw std::runtime_error("Unsupported robot type for joint tracking gains");
    }
}

void RobotRunner::setupStep(const StateEstimate<double>& state) {
    if (state.legs.size() != _model.numLegs()) {
        throw std::invalid_argument("StateEstimate leg count does not match RobotModel");
    }
    if (state.arms.size() != _model.numArms()) {
        throw std::invalid_argument("StateEstimate arm count does not match RobotModel");
    }

    _legController->zeroCommand();
    _legController->zeroData();
    _armController->zeroCommand();
    _armController->zeroData();

    for (std::size_t leg = 0; leg < state.legs.size(); ++leg) {
        const auto& legState = state.legs[leg];
        const Eigen::Index q_size =
            static_cast<Eigen::Index>(_model.legQIndices(static_cast<int>(leg)).size());
        const Eigen::Index qd_size =
            static_cast<Eigen::Index>(_model.legQdIndices(static_cast<int>(leg)).size());
        const Eigen::Index tau_size =
            static_cast<Eigen::Index>(_model.legActuatorIndices(static_cast<int>(leg)).size());

        if (legState.q.size() != q_size || legState.qd.size() != qd_size) {
            throw std::invalid_argument("RobotLegState joint dimension does not match RobotModel");
        }

        _legController->setLegJointData(static_cast<int>(leg), legState.q, legState.qd);

        if (legState.tauEstimate.size() == tau_size) {
            _legController->setLegTauEstimate(static_cast<int>(leg), legState.tauEstimate);
        } else if (legState.tauEstimate.size() != 0) {
            throw std::invalid_argument(
                "RobotLegState tauEstimate dimension does not match RobotModel");
        }
    }

    for (std::size_t arm = 0; arm < state.arms.size(); ++arm) {
        const auto& armState = state.arms[arm];
        const Eigen::Index q_size =
            static_cast<Eigen::Index>(_model.armQIndices(static_cast<int>(arm)).size());
        const Eigen::Index qd_size =
            static_cast<Eigen::Index>(_model.armQdIndices(static_cast<int>(arm)).size());
        const Eigen::Index tau_size =
            static_cast<Eigen::Index>(_model.armActuatorIndices(static_cast<int>(arm)).size());

        if (armState.q.size() != q_size || armState.qd.size() != qd_size) {
            throw std::invalid_argument("RobotArmState joint dimension does not match RobotModel");
        }

        _armController->setArmJointData(static_cast<int>(arm), armState.q, armState.qd);

        if (armState.tauEstimate.size() == tau_size) {
            _armController->setArmTauEstimate(static_cast<int>(arm), armState.tauEstimate);
        } else if (armState.tauEstimate.size() != 0) {
            throw std::invalid_argument(
                "RobotArmState tauEstimate dimension does not match RobotModel");
        }
    }
}

void RobotRunner::finalizeStep() {
    _iterations++;
}
