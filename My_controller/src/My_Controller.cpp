#include "My_Controller.h"

void MyController::initializeController() {

}

void MyController::runController() {
    const double x_dot_des = (_userCommand != nullptr) ? _userCommand->x_dot : 0.0;
    const double y_dot_des = (_userCommand != nullptr) ? _userCommand->y_dot : 0.0;
    const double psi_dot_des = (_userCommand != nullptr) ? _userCommand->psi_dot : 0.0;

    (void)x_dot_des;
    (void)y_dot_des;
    (void)psi_dot_des;
}
