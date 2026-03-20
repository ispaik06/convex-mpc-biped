#include <stdexcept>

#include "RobotRunner.h"

void RobotRunner::init(RobotParams<double>* params) {
    _params = params;
    _model = RobotModel<double>(_params);
    if(!_model.validate()) {
        throw std::runtime_error("Invalid RobotParams");
    }
    _robotType = _params->roboType;
    _legController = std::make_unique<LegController<double>>(_model);
    _armController = std::make_unique<ArmController<double>>(_model);
    _legPosInitializer = std::make_unique<LegPosInitializer<double>>(_params, 3., 0.01);
    _armPosInitializer = std::make_unique<ArmPosInitializer<double>>(_params, 3., 0.01);
}

void RobotRunner::run(const RobotState<double>& state, RobotCommand<double>& command) {
    if (!_legController) {
        throw std::runtime_error("RobotRunner::init must be called before run");
    }

    setupStep(state);

    _legController->setEnabled(true);
    for (std::size_t leg = 0; leg < _legController->numLegs(); ++leg) {
        auto& legCommand = _legController->commands[leg];
        legCommand.kpJoint.setZero(legCommand.dof(), legCommand.dof());
        legCommand.kdJoint.setZero(legCommand.dof(), legCommand.dof());
        legCommand.kpJoint.diagonal().setConstant(10.0);
        legCommand.kdJoint.diagonal().setConstant(1.0);
    }
    _armController->setEnabled(true);
    for (std::size_t leg = 0; leg < _armController->numArms(); ++leg) {
        auto& armCommand = _armController->commands[leg];
        armCommand.kpJoint.setZero(armCommand.dof(), armCommand.dof());
        armCommand.kdJoint.setZero(armCommand.dof(), armCommand.dof());
        armCommand.kpJoint.diagonal().setConstant(10.0);
        armCommand.kdJoint.diagonal().setConstant(3.0);
    }

    _legPosInitializer->IsInitialized(_legController.get());
    _armPosInitializer->IsInitialized(_armController.get());
    composeCommand(command);

    finalizeStep();
}

void RobotRunner::composeCommand(RobotCommand<double>& command) const {
    command.tau.setZero(_model.nu());
    _legController->updateCommand(command.tau);
    _armController->updateCommand(command.tau);
}


void RobotRunner::setupStep(const RobotState<double>& state) {
    if (state.legs.size() != _model.numLegs()) {
        throw std::invalid_argument("RobotState leg count does not match RobotModel");
    }
    if (state.arms.size() != _model.numArms()) {
        throw std::invalid_argument("RobotState arm count does not match RobotModel");
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
