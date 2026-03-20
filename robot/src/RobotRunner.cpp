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
}

void RobotRunner::run() {
    setupStep();

    DVec<double> kpMat;
    DVec<double> kdMat;
    Eigen::Index dof = _legController->
    kpMat.setZero(dof)
    kpMat << 5, 0, 0, 0, 5, 0, 0, 0, 5;
    kdMat << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    for (int leg = 0; leg < _legController->numLegs; ++leg) {
        _legController->commands[leg].kpJoint = kpMat;
        _legController->commands[leg].kdJoint = kdMat;
    }
    _legController->updateCommand();
    


    finalizeStep();
}


void RobotRunner::setupStep() {
    _legController->zeroCommand();
    _legController->zeroData();
    _legController->setEnabled(true);
}

void RobotRunner::finalizeStep() {
    _iterations++;
}
