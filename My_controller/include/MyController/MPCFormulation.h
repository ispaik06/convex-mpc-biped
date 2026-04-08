#ifndef MPC_FORMULATION_H
#define MPC_FORMULATION_H

#include "ControlParameters.h"
#include "ReferenceTrajectory.h"
#include "Robot/RobotParams.h"

using Mat13d = Eigen::Matrix<double, 13, 13>;
using Mat13x12d = Eigen::Matrix<double, 13, 12>;

struct MPCFormulationOutput {
    DMat<double> A_qp;
    DMat<double> B_qp;

    MPCFormulationOutput()
        : A_qp(13 * N, 13),
          B_qp(13 * N, 12 * N) {
        A_qp.setZero();
        B_qp.setZero();
    }
};

class MPCFormulation {
public:
    explicit MPCFormulation(const RobotParams<double>* robotParams)
        : _robotParams(robotParams) {}

    MPCFormulationOutput build(const ReferenceTrajectoryOutput& referenceTrajectory) const;

private:
    const RobotParams<double>* _robotParams = nullptr;
};

#endif  // MPC_FORMULATION_H
