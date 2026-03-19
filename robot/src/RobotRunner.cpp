#include "RobotRunner.h"

void RobotRunner::init(RobotParams<double>* params) {
    _params = params;
    if(!_model.validate()) {
        throw std::runtime_error("Invalid RobotParams");
    }
    _legController = std::make_unique<LegController<double>>(_model);
}

void RobotRunner::run() {

}


void RobotRunner::setupStep() {

}

void RobotRunner::finalizeStep() {

}
