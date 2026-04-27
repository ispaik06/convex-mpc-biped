#ifndef CONVEX_MPC_H
#define CONVEX_MPC_H

#include <Eigen/SparseCore>
#include <OsqpEigen/OsqpEigen.h>

#include "ControllerConfig.h"
#include "GaitScheduler.h"
#include "MPCFormulation.h"

struct ConvexMPCInputView {
    const DMat<double>* A_qp{nullptr};
    const DMat<double>* B_qp{nullptr};
    const DVec<double>* X_ref{nullptr};
    const Vec13<double>* x0{nullptr};
    const DMat<double>* C{nullptr};
    const DVec<double>* C_bound{nullptr};
    const DMat<double>* D{nullptr};
    LocomotionMode locomotionMode{LocomotionMode::Walking};

    static ConvexMPCInputView from(const GaitScheduler& gaitScheduler,
                                   const MPCFormulationOutput& formulation,
                                   const ReferenceTrajectoryOutput& referenceTrajectory,
                                   const Vec13<double>& x0,
                                   LocomotionMode locomotionMode);
};

class ConvexMPC {
public:
    ConvexMPC();

    void updateInput(const GaitScheduler& gaitScheduler,
                     const MPCFormulationOutput& formulation,
                     const ReferenceTrajectoryOutput& referenceTrajectory,
                     const Vec13<double>& x0,
                     LocomotionMode locomotionMode);

    void clear();

    const ConvexMPCInputView& input() const;
    bool hasInput() const;
    const DMat<double>& L() const;
    const DMat<double>& K() const;
    const Vec12<double>& optimalWrench() const;
    const DVec<double>& optimalWrenchHorizon() const;
    bool hasSolution() const;

    void buildQP();
    void solve();

private:
    int numVars() const;
    int numIneq() const;
    int numEq() const;
    int numCons() const;

    bool initializeSolver();
    bool updateSolverData();
    void buildHessianMatrix(const DMat<double>& P);
    void buildConstraintMatrix(const DMat<double>& C, const DMat<double>& D);
    void updateWarmStart();
    void validateInputDimensions(const ConvexMPCInputView& input) const;
    static const StateWeightMat& stateWeightForMode(LocomotionMode mode);
    static const InputWeightMat& inputWeightForMode(LocomotionMode mode);

    ConvexMPCInputView _input;
    bool _hasInput{false};
    bool _qpReady{false};
    bool _solverInitialized{false};
    bool _hasPreviousSolution{false};

    OsqpEigen::Solver _solver;

    Eigen::SparseMatrix<c_float> _hessian;
    Eigen::SparseMatrix<c_float> _constraintMatrix;
    DVec<c_float> _gradient;
    DVec<c_float> _lowerBound;
    DVec<c_float> _upperBound;
    DVec<c_float> _warmStart;
    DVec<c_float> _lastSolution;
    DMat<double> _hessianDense;
    DMat<double> _weightedB;
    DVec<double> _gradientDense;
    DVec<double> _statePrediction;
    DVec<double> _stateError;
    DVec<double> _weightedStateError;
    Vec12<double> _optimalWrench = Vec12<double>::Zero();
    DVec<double> _optimalWrenchHorizon;
};

#endif  // CONVEX_MPC_H
