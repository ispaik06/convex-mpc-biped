#ifndef MPC_FORMULATION_H
#define MPC_FORMULATION_H

#include "ControllerConfig.h"
#include "ReferenceTrajectory.h"
#include "Robot/RobotParams.h"

using Mat13d = Eigen::Matrix<double, 13, 13>;
using Mat13x12d = Eigen::Matrix<double, 13, 12>;

struct MPCFormulationOutput {
    DMat<double> A_qp;
    DMat<double> B_qp;

    MPCFormulationOutput()
        : A_qp(13 * horizonSteps(), 13),
          B_qp(13 * horizonSteps(), 12 * horizonSteps()) {
        setZero();
    }

    void resizeIfNeeded() {
        const int steps = horizonSteps();
        if (A_qp.rows() == 13 * steps &&
            A_qp.cols() == 13 &&
            B_qp.rows() == 13 * steps &&
            B_qp.cols() == 12 * steps) {
            return;
        }

        A_qp.resize(13 * steps, 13);
        B_qp.resize(13 * steps, 12 * steps);
    }

    void setZero() {
        A_qp.setZero();
        B_qp.setZero();
    }
};

class MPCFormulation {
public:
    explicit MPCFormulation(const RobotParams<double>* robotParams)
        : _robotParams(robotParams) {}

    void build(const ReferenceTrajectoryOutput& referenceTrajectory, MPCFormulationOutput& out);

private:
    void resizeScratchIfNeeded();

    const RobotParams<double>* _robotParams = nullptr;
    vectorAligned<Mat13d> _A_d;
    vectorAligned<Mat13x12d> _B_d;
};

#endif  // MPC_FORMULATION_H
