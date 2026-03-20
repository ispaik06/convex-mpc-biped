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
    _jointPosInitializer = std::make_unique<JointPosInitializer<double>>(_params, 3., 0.01);
}

void RobotRunner::run(const RobotState<double>& state, RobotCommand<double>& command) {
    if (!_legController) {
        throw std::runtime_error("RobotRunner::init must be called before run");
    }

    setupStep(state);

    // Apply simple diagonal gains while the posture initializer drives qDes.
    for (std::size_t leg = 0; leg < _legController->numLegs(); ++leg) {
        auto& legCommand = _legController->commands[leg];
        legCommand.kpJoint.setZero(legCommand.dof(), legCommand.dof());
        legCommand.kdJoint.setZero(legCommand.dof(), legCommand.dof());
        legCommand.kpJoint.diagonal().setConstant(5.0);
        legCommand.kdJoint.diagonal().setConstant(1.0);
    }

    _jointPosInitializer->IsInitialized(_legController.get());
    command.tau = _legController->updateCommand();

    finalizeStep();
}


void RobotRunner::setupStep(const RobotState<double>& state) {
    if (state.q.size() != _model.nq() || state.qd.size() != _model.nv()) {
        throw std::invalid_argument("RobotState size does not match RobotModel dimensions");
    }

    if (state.tauEstimate.size() != 0 && state.tauEstimate.size() != _model.nu()) {
        throw std::invalid_argument("RobotState tauEstimate size does not match RobotModel::nu");
    }

    _legController->zeroCommand();
    _legController->zeroData();
    _legController->setEnabled(true);

    if (state.tauEstimate.size() == _model.nu()) {
        _legController->updateJointData(state.q, state.qd, state.tauEstimate);
    } else {
        _legController->updateJointData(state.q, state.qd);
    }
}

void RobotRunner::finalizeStep() {
    _iterations++;
}
