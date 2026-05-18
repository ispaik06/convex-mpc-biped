#ifndef CONVEX_MPC_H
#define CONVEX_MPC_H

#include <cstdint>
#include <iosfwd>
#include <Eigen/SparseCore>
#include <OsqpEigen/OsqpEigen.h>

#include <vector>

#include "ControllerConfig.h"
#include "GaitScheduler.h"
#include "MPCFormulation.h"
#include "Utilities/Timing.h"

struct ConvexMPCInputView {
    const DMat<double>* A_qp{nullptr};
    const DMat<double>* B_qp{nullptr};
    const DVec<double>* X_ref{nullptr};
    const Vec13<double>* x0{nullptr};
    const GaitScheduler* gaitScheduler{nullptr};
    const DVec<double>* C_bound{nullptr};
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
    void printProfilingSummary(std::ostream& out) const;

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
    void buildConstraintMatrix(const GaitScheduler& gaitScheduler);
    std::vector<int> contactConstraintSignature(const GaitScheduler& gaitScheduler) const;
    bool hasUsableWarmStart() const;
    void updateWarmStart();
    void validateInputDimensions(const ConvexMPCInputView& input) const;
    static const StateWeightMat& stateWeightForMode(LocomotionMode mode);
    static const InputWeightMat& inputWeightForMode(LocomotionMode mode);

    ConvexMPCInputView _input;
    bool _hasInput{false};
    bool _qpReady{false};
    bool _solverInitialized{false};
    bool _hasPreviousSolution{false};
    bool _hasContactConstraintSignature{false};
    bool _skipWarmStartForCurrentSolve{false};
    std::vector<int> _contactConstraintSignature;

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
    profiling::TimingStats _buildQpTime;
    profiling::TimingStats _stateProjectionTime;
    profiling::TimingStats _weightedAssemblyTime;
    profiling::TimingStats _hessianAssemblyTime;
    profiling::TimingStats _gradientAssemblyTime;
    profiling::TimingStats _sparseHessianTime;
    profiling::TimingStats _sparseConstraintTime;
    profiling::TimingStats _osqpInitTime;
    profiling::TimingStats _osqpUpdateTime;
    profiling::TimingStats _solveTime;
    profiling::TimingStats _osqpSolveCallTime;
    profiling::TimingStats _solutionExtractTime;
    std::uint64_t _coldStartCount{0};
    std::uint64_t _contactSignatureChangeCount{0};
    std::uint64_t _solveRetryCount{0};
    bool _loggedNormalWarmStart{false};
    bool _loggedShiftedWarmStart{false};
};

#endif  // CONVEX_MPC_H
