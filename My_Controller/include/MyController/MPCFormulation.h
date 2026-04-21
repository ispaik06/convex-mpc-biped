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
    vectorAligned<Mat13d> A_c;
    vectorAligned<Mat13x12d> B_c;
    vectorAligned<Mat3<double>> inertia_W;

    MPCFormulationOutput()
        : A_qp(13 * horizonSteps(), 13),
          B_qp(13 * horizonSteps(), 12 * horizonSteps()) {
        resizeIfNeeded();
        setZero();
    }

    void resizeIfNeeded() {
        const int steps = horizonSteps();
        if (A_qp.rows() == 13 * steps &&
            A_qp.cols() == 13 &&
            B_qp.rows() == 13 * steps &&
            B_qp.cols() == 12 * steps &&
            static_cast<int>(A_c.size()) == steps &&
            static_cast<int>(B_c.size()) == steps &&
            static_cast<int>(inertia_W.size()) == steps) {
            return;
        }

        if (A_qp.rows() != 13 * steps || A_qp.cols() != 13) {
            A_qp.resize(13 * steps, 13);
        }
        if (B_qp.rows() != 13 * steps || B_qp.cols() != 12 * steps) {
            B_qp.resize(13 * steps, 12 * steps);
        }
        A_c.assign(steps, Mat13d::Zero());
        B_c.assign(steps, Mat13x12d::Zero());
        inertia_W.assign(steps, Mat3<double>::Zero());
    }

    void setZero() {
        A_qp.setZero();
        B_qp.setZero();
        for (auto& A_c_k : A_c) {
            A_c_k.setZero();
        }
        for (auto& B_c_k : B_c) {
            B_c_k.setZero();
        }
        for (auto& inertia_W_k : inertia_W) {
            inertia_W_k.setZero();
        }
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
