#include "ConvexMPC.h"

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace {
template <typename T>
void requireNonNull(const T* ptr, const char* name) {
    if (ptr == nullptr) {
        std::ostringstream oss;
        oss << "ConvexMPC input pointer is null for " << name;
        throw std::runtime_error(oss.str());
    }
}

void requireShape(const Eigen::Index actualRows,
                  const Eigen::Index actualCols,
                  const Eigen::Index expectedRows,
                  const Eigen::Index expectedCols,
                  const char* name) {
    if (actualRows != expectedRows || actualCols != expectedCols) {
        std::ostringstream oss;
        oss << "ConvexMPC input dimension mismatch for " << name
            << ": expected " << expectedRows << "x" << expectedCols
            << ", got " << actualRows << "x" << actualCols;
        throw std::runtime_error(oss.str());
    }
}

void requireSize(const Eigen::Index actualSize,
                 const Eigen::Index expectedSize,
                 const char* name) {
    if (actualSize != expectedSize) {
        std::ostringstream oss;
        oss << "ConvexMPC input dimension mismatch for " << name
            << ": expected " << expectedSize
            << ", got " << actualSize;
        throw std::runtime_error(oss.str());
    }
}

Eigen::SparseMatrix<c_float> makeUpperTriangularPattern(const int size) {
    std::vector<Eigen::Triplet<c_float>> triplets;
    triplets.reserve(size * (size + 1) / 2);

    for (int col = 0; col < size; ++col) {
        for (int row = 0; row <= col; ++row) {
            triplets.emplace_back(row, col, c_float(0));
        }
    }

    Eigen::SparseMatrix<c_float> sparse(size, size);
    sparse.setFromTriplets(triplets.begin(), triplets.end());
    sparse.makeCompressed();
    return sparse;
}

Eigen::SparseMatrix<c_float> makeFullPattern(const int rows, const int cols) {
    std::vector<Eigen::Triplet<c_float>> triplets;
    triplets.reserve(rows * cols);

    for (int col = 0; col < cols; ++col) {
        for (int row = 0; row < rows; ++row) {
            triplets.emplace_back(row, col, c_float(0));
        }
    }

    Eigen::SparseMatrix<c_float> sparse(rows, cols);
    sparse.setFromTriplets(triplets.begin(), triplets.end());
    sparse.makeCompressed();
    return sparse;
}

void fillUpperTriangularValues(const DMat<double>& dense, Eigen::SparseMatrix<c_float>& sparse) {
    if (dense.rows() != sparse.rows() || dense.cols() != sparse.cols()) {
        throw std::runtime_error("fillUpperTriangularValues received mismatched dimensions");
    }

    c_float* values = sparse.valuePtr();
    Eigen::Index idx = 0;
    for (Eigen::Index col = 0; col < dense.cols(); ++col) {
        for (Eigen::Index row = 0; row <= col; ++row) {
            values[idx++] = static_cast<c_float>(dense(row, col));
        }
    }
}

void fillConstraintValues(const DMat<double>& C,
                          const DMat<double>& D,
                          Eigen::SparseMatrix<c_float>& sparse) {
    if (C.rows() + D.rows() != sparse.rows() || C.cols() != sparse.cols() ||
        D.cols() != sparse.cols()) {
        throw std::runtime_error("fillConstraintValues received mismatched dimensions");
    }

    c_float* values = sparse.valuePtr();
    Eigen::Index idx = 0;
    for (Eigen::Index col = 0; col < sparse.cols(); ++col) {
        for (Eigen::Index row = 0; row < C.rows(); ++row) {
            values[idx++] = static_cast<c_float>(C(row, col));
        }
        for (Eigen::Index row = 0; row < D.rows(); ++row) {
            values[idx++] = static_cast<c_float>(D(row, col));
        }
    }
}
}  // namespace

ConvexMPC::ConvexMPC()
    : _hessian(makeUpperTriangularPattern(12 * horizonSteps())),
      _constraintMatrix(makeFullPattern(36 * horizonSteps(), 12 * horizonSteps())),
      _gradient(12 * horizonSteps()),
      _lowerBound(36 * horizonSteps()),
      _upperBound(36 * horizonSteps()),
      _warmStart(12 * horizonSteps()),
      _lastSolution(12 * horizonSteps()),
      _hessianDense(12 * horizonSteps(), 12 * horizonSteps()),
      _weightedB(13 * horizonSteps(), 12 * horizonSteps()),
      _gradientDense(12 * horizonSteps()),
      _statePrediction(13 * horizonSteps()),
      _stateError(13 * horizonSteps()),
      _weightedStateError(13 * horizonSteps()) {
    _gradient.setZero();
    _lowerBound.setZero();
    _upperBound.setZero();
    _warmStart.setZero();
    _lastSolution.setZero();
    _hessianDense.setZero();
    _weightedB.setZero();
    _gradientDense.setZero();
    _statePrediction.setZero();
    _stateError.setZero();
    _weightedStateError.setZero();
}

int ConvexMPC::numVars() const {
    return 12 * horizonSteps();
}

int ConvexMPC::numIneq() const {
    return 24 * horizonSteps();
}

int ConvexMPC::numEq() const {
    return 12 * horizonSteps();
}

int ConvexMPC::numCons() const {
    return numIneq() + numEq();
}

ConvexMPCInputView ConvexMPCInputView::from(const GaitScheduler& gaitScheduler,
                                            const MPCFormulationOutput& formulation,
                                            const ReferenceTrajectoryOutput& referenceTrajectory,
                                            const Vec13<double>& x0) {
    ConvexMPCInputView input;
    input.A_qp = &formulation.A_qp;
    input.B_qp = &formulation.B_qp;
    input.X_ref = &referenceTrajectory.X_ref;
    input.x0 = &x0;
    input.C = &gaitScheduler.C;
    input.C_bound = &gaitScheduler.C_bound;
    input.D = &gaitScheduler.D;
    return input;
}

void ConvexMPC::updateInput(const GaitScheduler& gaitScheduler,
                            const MPCFormulationOutput& formulation,
                            const ReferenceTrajectoryOutput& referenceTrajectory,
                            const Vec13<double>& x0) {
    const ConvexMPCInputView input =
        ConvexMPCInputView::from(gaitScheduler, formulation, referenceTrajectory, x0);

    validateInputDimensions(input);
    _input = input;
    _hasInput = true;
    _qpReady = false;
}

void ConvexMPC::clear() {
    _input = ConvexMPCInputView{};
    _hasInput = false;
    _qpReady = false;
    _hasPreviousSolution = false;
    _optimalWrench.setZero();
    _warmStart.setZero();
    _lastSolution.setZero();
}

const ConvexMPCInputView& ConvexMPC::input() const {
    if (!_hasInput) {
        throw std::runtime_error("ConvexMPC input has not been initialized");
    }
    return _input;
}

bool ConvexMPC::hasInput() const {
    return _hasInput;
}

const DMat<double>& ConvexMPC::L() const {
    return getL();
}

const DMat<double>& ConvexMPC::K() const {
    return getK();
}

const Vec12<double>& ConvexMPC::optimalWrench() const {
    return _optimalWrench;
}

void ConvexMPC::buildQP() {
    if (!_hasInput) {
        throw std::runtime_error("ConvexMPC::buildQP requires initialized input");
    }

    const DMat<double>& A_qp = *_input.A_qp;
    const DMat<double>& B_qp = *_input.B_qp;
    const DVec<double>& X_ref = *_input.X_ref;
    const Vec13<double>& x0 = *_input.x0;
    const DMat<double>& C = *_input.C;
    const DVec<double>& C_bound = *_input.C_bound;
    const DMat<double>& D = *_input.D;
    const StateWeightMat& stateWeight = getControllerConfig().mpc.stateWeight;
    const InputWeightMat& inputWeight = getControllerConfig().mpc.inputWeight;
    const int steps = horizonSteps();

    _statePrediction.noalias() = A_qp * x0;
    _stateError = _statePrediction;
    _stateError -= X_ref;

    for (int k = 0; k < steps; ++k) {
        const Eigen::Index stateOffset = static_cast<Eigen::Index>(13 * k);

        _weightedB.middleRows(stateOffset, 13).noalias() =
            stateWeight * B_qp.middleRows(stateOffset, 13);
        _weightedStateError.segment(stateOffset, 13).noalias() =
            stateWeight * _stateError.segment(stateOffset, 13);
    }

    _hessianDense.noalias() = 2.0 * (B_qp.transpose() * _weightedB);
    for (int k = 0; k < steps; ++k) {
        const Eigen::Index inputOffset = static_cast<Eigen::Index>(12 * k);
        _hessianDense.block(inputOffset, inputOffset, 12, 12) += 2.0 * inputWeight;
    }
    _hessianDense = 0.5 * (_hessianDense + _hessianDense.transpose());

    _gradientDense.noalias() = 2.0 * (B_qp.transpose() * _weightedStateError);

    buildHessianMatrix(_hessianDense);
    buildConstraintMatrix(C, D);

    _gradient = _gradientDense.cast<c_float>();
    _lowerBound.head(numIneq()).setConstant(-OsqpEigen::INFTY);
    _upperBound.head(numIneq()) = C_bound.cast<c_float>();
    _lowerBound.tail(numEq()).setZero();
    _upperBound.tail(numEq()).setZero();

    const bool success = _solverInitialized ? updateSolverData() : initializeSolver();
    if (!success) {
        throw std::runtime_error("ConvexMPC failed to setup/update OsqpEigen solver");
    }

    _qpReady = true;
}

void ConvexMPC::solve() {
    if (!_hasInput) {
        throw std::runtime_error("ConvexMPC::solve requires initialized input");
    }

    if (!_qpReady) {
        buildQP();
    }

    if (_hasPreviousSolution && !_solver.setPrimalVariable(_warmStart)) {
        throw std::runtime_error("ConvexMPC failed to set OsqpEigen warm start");
    }

    const auto exitFlag = _solver.solveProblem();
    if (exitFlag != OsqpEigen::ErrorExitFlag::NoError) {
        throw std::runtime_error("ConvexMPC OsqpEigen solveProblem failed");
    }

    const auto status = _solver.getStatus();
    if (status != OsqpEigen::Status::Solved &&
        status != OsqpEigen::Status::SolvedInaccurate) {
        throw std::runtime_error("ConvexMPC OsqpEigen did not converge to a valid solution");
    }

    const auto& solution = _solver.getSolution();
    if (solution.size() != numVars()) {
        throw std::runtime_error("ConvexMPC received solution with unexpected dimension");
    }

    _lastSolution = solution;
    updateWarmStart();
    _hasPreviousSolution = true;
    _optimalWrench = solution.segment(0, 12).cast<double>();
}

void ConvexMPC::validateInputDimensions(const ConvexMPCInputView& input) const {
    requireNonNull(input.A_qp, "A_qp");
    requireNonNull(input.B_qp, "B_qp");
    requireNonNull(input.X_ref, "X_ref");
    requireNonNull(input.x0, "x0");
    requireNonNull(input.C, "C");
    requireNonNull(input.C_bound, "C_bound");
    requireNonNull(input.D, "D");

    requireShape(input.A_qp->rows(), input.A_qp->cols(), 13 * horizonSteps(), 13, "A_qp");
    requireShape(input.B_qp->rows(), input.B_qp->cols(),
                 13 * horizonSteps(), 12 * horizonSteps(), "B_qp");
    requireSize(input.X_ref->size(), 13 * horizonSteps(), "X_ref");
    requireSize(input.x0->size(), 13, "x0");
    requireShape(input.C->rows(), input.C->cols(),
                 24 * horizonSteps(), 12 * horizonSteps(), "C");
    requireSize(input.C_bound->size(), 24 * horizonSteps(), "C_bound");
    requireShape(input.D->rows(), input.D->cols(),
                 12 * horizonSteps(), 12 * horizonSteps(), "D");
}

bool ConvexMPC::initializeSolver() {
    if (_solver.isInitialized()) {
        _solver.clearSolver();
    }

    _solver.settings()->setVerbosity(false);
    _solver.settings()->setWarmStart(true);
    _solver.settings()->setPolish(false);
    _solver.settings()->setMaxIteration(200);
    _solver.settings()->setAdaptiveRho(true);

    _solver.data()->setNumberOfVariables(numVars());
    _solver.data()->setNumberOfConstraints(numCons());

    if (!_solver.data()->setHessianMatrix(_hessian) ||
        !_solver.data()->setGradient(_gradient) ||
        !_solver.data()->setLinearConstraintsMatrix(_constraintMatrix) ||
        !_solver.data()->setBounds(_lowerBound, _upperBound)) {
        _solverInitialized = false;
        return false;
    }

    _solverInitialized = _solver.initSolver();
    return _solverInitialized;
}

bool ConvexMPC::updateSolverData() {
    if (!_solverInitialized) {
        return initializeSolver();
    }

    if (_solver.updateHessianMatrix(_hessian) &&
        _solver.updateGradient(_gradient) &&
        _solver.updateLinearConstraintsMatrix(_constraintMatrix) &&
        _solver.updateBounds(_lowerBound, _upperBound)) {
        return true;
    }

    _solverInitialized = false;
    return initializeSolver();
}

void ConvexMPC::buildHessianMatrix(const DMat<double>& P) {
    fillUpperTriangularValues(P, _hessian);
}

void ConvexMPC::buildConstraintMatrix(const DMat<double>& C, const DMat<double>& D) {
    fillConstraintValues(C, D, _constraintMatrix);
}

void ConvexMPC::updateWarmStart() {
    if (_lastSolution.size() != numVars()) {
        throw std::runtime_error("ConvexMPC warm start dimension mismatch");
    }

    _warmStart.head(numVars() - 12) = _lastSolution.segment(12, numVars() - 12);
    _warmStart.tail(12) = _lastSolution.tail(12);
}
