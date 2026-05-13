#include "ConvexMPC.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "Utilities/MatrixUtils.h"
#include "Utilities/Timing.h"

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

Eigen::SparseMatrix<c_float> makeBlockLocalConstraintPattern(const int steps) {
    constexpr int kVarsPerStep = 12;
    constexpr int kIneqPerStep = 24;
    constexpr int kEqPerStep = 12;
    constexpr int kConsPerStep = kIneqPerStep + kEqPerStep;

    const int vars = kVarsPerStep * steps;
    const int ineqRows = kIneqPerStep * steps;
    const int cons = kConsPerStep * steps;

    std::vector<Eigen::Triplet<c_float>> triplets;

    // Conservative block-local superset:
    // For each horizon step k, only constraints at step k may depend on u_k.
    //
    // A_osqp = [ C ]
    //          [ D ]
    //
    // C block for step k: rows [24k, 24k+23], cols [12k, 12k+11]
    // D block for step k: rows [24N+12k, 24N+12k+11], cols [12k, 12k+11]
    triplets.reserve(static_cast<size_t>(steps) * kConsPerStep * kVarsPerStep);

    for (int k = 0; k < steps; ++k) {
        const int col0 = kVarsPerStep * k;
        const int cRow0 = kIneqPerStep * k;
        const int dRow0 = ineqRows + kEqPerStep * k;

        for (int localCol = 0; localCol < kVarsPerStep; ++localCol) {
            const int col = col0 + localCol;

            for (int localRow = 0; localRow < kIneqPerStep; ++localRow) {
                triplets.emplace_back(cRow0 + localRow, col, c_float(0));
            }

            for (int localRow = 0; localRow < kEqPerStep; ++localRow) {
                triplets.emplace_back(dRow0 + localRow, col, c_float(0));
            }
        }
    }

    Eigen::SparseMatrix<c_float> sparse(cons, vars);
    sparse.setFromTriplets(triplets.begin(), triplets.end());
    sparse.makeCompressed();
    return sparse;
}

Eigen::SparseMatrix<c_float> makeRowLevelConstraintPattern(const int steps) {
    constexpr int kInputDim = 12;
    constexpr int kIneqPerStep = 24;
    constexpr int kEqPerStep = 12;

    const int vars = kInputDim * steps;
    const int ineqRows = kIneqPerStep * steps;
    const int eqRows = kEqPerStep * steps;
    const int cons = ineqRows + eqRows;

    std::vector<Eigen::Triplet<c_float>> triplets;

    // Yaw-rotated GaitScheduler pattern:
    //
    // Per step:
    // C: 60 entries
    // D: 16 entries
    // total: 76 entries
    triplets.reserve(static_cast<std::size_t>(steps) * 76);

    auto add = [&triplets](const int row, const int col) {
        triplets.emplace_back(row, col, c_float(0));
    };

    // C_unit row-level pattern in local 6D foot wrench coordinates:
    // local foot wrench = [Fx, Fy, Fz, Mx, My, Mz]
    const std::array<std::vector<int>, 12> cUnitCols = {{
        // Friction pyramid in foot-local frame, rotated into world frame.
        // Fx_F = c Fx_W + s Fy_W
        // Fy_F = -s Fx_W + c Fy_W
        // Therefore each tangential friction row can touch Fx_W, Fy_W, Fz_W.
        {0, 1, 2}, // +Fx_F - mu Fz <= 0
        {0, 1, 2}, // -Fx_F - mu Fz <= 0
        {0, 1, 2}, // +Fy_F - mu Fz <= 0
        {0, 1, 2}, // -Fy_F - mu Fz <= 0

        // Normal force bounds.
        // Yaw rotation does not change z.
        {2},       // +Fz <= Fz_max
        {2},       // -Fz <= -Fz_min

        // CoP bounds in foot-local frame, rotated into world frame.
        // Mx_F = c Mx_W + s My_W
        // My_F = -s Mx_W + c My_W
        // Therefore each CoP row can touch Fz_W, Mx_W, My_W.
        {2, 3, 4}, // +Mx_F - footHalfWidth  Fz <= 0
        {2, 3, 4}, // -Mx_F - footHalfWidth  Fz <= 0
        {2, 3, 4}, // +My_F - footHalfLength Fz <= 0
        {2, 3, 4}, // -My_F - footHalfLength Fz <= 0

        // Torsional friction.
        // Yaw-only rotation keeps Mz unchanged.
        {2, 5},    // +Mz - mu_t Fz <= 0
        {2, 5},    // -Mz - mu_t Fz <= 0
    }};

    // Map local 6D foot wrench columns into the 12D input block:
    //
    // u_k = [
    //   left force  0,1,2,
    //   right force 3,4,5,
    //   left torque 6,7,8,
    //   right torque 9,10,11
    // ]
    const std::array<int, 6> leftMap  = {0, 1, 2, 6, 7, 8};
    const std::array<int, 6> rightMap = {3, 4, 5, 9, 10, 11};

    for (int k = 0; k < steps; ++k) {
        const int u0 = kInputDim * k;
        const int cRow0 = kIneqPerStep * k;
        const int dRow0 = ineqRows + kEqPerStep * k;

        // ------------------------------------------------------------
        // C rows.
        // Left foot inequality rows: local rows 0..11.
        // Right foot inequality rows: local rows 12..23.
        // ------------------------------------------------------------
        for (int r = 0; r < 12; ++r) {
            for (const int localCol : cUnitCols[r]) {
                add(cRow0 + r, u0 + leftMap[localCol]);
            }
        }

        for (int r = 0; r < 12; ++r) {
            for (const int localCol : cUnitCols[r]) {
                add(cRow0 + 12 + r, u0 + rightMap[localCol]);
            }
        }

        // ------------------------------------------------------------
        // D rows.
        //
        // Force zero equality in swing:
        //   left force rows 0..2, right force rows 3..5
        //
        // Torque zero equality in swing:
        //   left torque rows 6..8, right torque rows 9..11
        //
        // NoRollMoment stance constraint can overwrite:
        //   left D row 6 with foot x-axis dot left torque = 0
        //   right D row 9 with foot x-axis dot right torque = 0
        //
        // Therefore row 6 and row 9 need 3 torque columns each.
        // ------------------------------------------------------------

        // Left force swing zero: diagonal only.
        add(dRow0 + 0, u0 + 0);
        add(dRow0 + 1, u0 + 1);
        add(dRow0 + 2, u0 + 2);

        // Right force swing zero: diagonal only.
        add(dRow0 + 3, u0 + 3);
        add(dRow0 + 4, u0 + 4);
        add(dRow0 + 5, u0 + 5);

        // Left torque:
        // row 6 may be either identity col 6 or roll axis cols 6,7,8.
        add(dRow0 + 6, u0 + 6);
        add(dRow0 + 6, u0 + 7);
        add(dRow0 + 6, u0 + 8);

        // rows 7,8 are only swing torque zero diagonal.
        add(dRow0 + 7, u0 + 7);
        add(dRow0 + 8, u0 + 8);

        // Right torque:
        // row 9 may be either identity col 9 or roll axis cols 9,10,11.
        add(dRow0 + 9, u0 + 9);
        add(dRow0 + 9, u0 + 10);
        add(dRow0 + 9, u0 + 11);

        // rows 10,11 are only swing torque zero diagonal.
        add(dRow0 + 10, u0 + 10);
        add(dRow0 + 11, u0 + 11);
    }

    Eigen::SparseMatrix<c_float> sparse(cons, vars);
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

// void fillConstraintValues(const DMat<double>& C, const DMat<double>& D, Eigen::SparseMatrix<c_float>& sparse) {
//     constexpr int kVarsPerStep = 12;
//     constexpr int kIneqPerStep = 24;
//     constexpr int kEqPerStep = 12;

//     if (C.rows() + D.rows() != sparse.rows() ||
//         C.cols() != sparse.cols() ||
//         D.cols() != sparse.cols()) {
//         throw std::runtime_error("fillConstraintValues received mismatched dimensions");
//     }

//     if (C.rows() % kIneqPerStep != 0 ||
//         D.rows() % kEqPerStep != 0 ||
//         sparse.cols() % kVarsPerStep != 0) {
//         throw std::runtime_error("fillConstraintValues received invalid MPC block dimensions");
//     }

//     const int steps = static_cast<int>(sparse.cols() / kVarsPerStep);

//     if (C.rows() != kIneqPerStep * steps ||
//         D.rows() != kEqPerStep * steps) {
//         throw std::runtime_error("fillConstraintValues received inconsistent horizon dimensions");
//     }

//     c_float* values = sparse.valuePtr();
//     Eigen::Index idx = 0;

//     // This must match makeBlockLocalConstraintPattern().
//     // Eigen's compressed sparse matrix is column-major by default.
//     for (int k = 0; k < steps; ++k) {
//         const Eigen::Index col0 = static_cast<Eigen::Index>(kVarsPerStep * k);
//         const Eigen::Index cRow0 = static_cast<Eigen::Index>(kIneqPerStep * k);
//         const Eigen::Index dRow0 = static_cast<Eigen::Index>(kEqPerStep * k);

//         for (int localCol = 0; localCol < kVarsPerStep; ++localCol) {
//             const Eigen::Index col = col0 + localCol;

//             for (int localRow = 0; localRow < kIneqPerStep; ++localRow) {
//                 values[idx++] = static_cast<c_float>(C(cRow0 + localRow, col));
//             }

//             for (int localRow = 0; localRow < kEqPerStep; ++localRow) {
//                 values[idx++] = static_cast<c_float>(D(dRow0 + localRow, col));
//             }
//         }
//     }

//     if (idx != sparse.nonZeros()) {
//         throw std::runtime_error("fillConstraintValues did not fill the expected number of sparse entries");
//     }
// }

void fillConstraintValues(const DMat<double>& C,
                          const DMat<double>& D,
                          Eigen::SparseMatrix<c_float>& sparse) {
    if (C.rows() + D.rows() != sparse.rows() ||
        C.cols() != sparse.cols() ||
        D.cols() != sparse.cols()) {
        throw std::runtime_error("fillConstraintValues received mismatched dimensions");
    }

#ifndef NDEBUG
    // Safety check:
    // If future constraints are added to C or D outside the row-level pattern,
    // this detects it instead of silently dropping those entries.
    constexpr double kPatternTolerance = 1e-12;

    auto hasSlot = [&sparse](const Eigen::Index row, const Eigen::Index col) {
        for (Eigen::SparseMatrix<c_float>::InnerIterator it(sparse, static_cast<int>(col)); it; ++it) {
            if (it.row() == row) {
                return true;
            }
        }
        return false;
    };

    for (Eigen::Index col = 0; col < C.cols(); ++col) {
        for (Eigen::Index row = 0; row < C.rows(); ++row) {
            if (std::abs(C(row, col)) > kPatternTolerance && !hasSlot(row, col)) {
                std::ostringstream oss;
                oss << "Constraint sparse pattern missing C entry at row="
                    << row << ", col=" << col << ", value=" << C(row, col);
                throw std::runtime_error(oss.str());
            }
        }

        for (Eigen::Index row = 0; row < D.rows(); ++row) {
            const Eigen::Index sparseRow = C.rows() + row;
            if (std::abs(D(row, col)) > kPatternTolerance && !hasSlot(sparseRow, col)) {
                std::ostringstream oss;
                oss << "Constraint sparse pattern missing D entry at row="
                    << row << ", col=" << col << ", value=" << D(row, col);
                throw std::runtime_error(oss.str());
            }
        }
    }
#endif

    c_float* values = sparse.valuePtr();
    Eigen::Index idx = 0;

    // Eigen SparseMatrix is column-major by default.
    // This fills values in the actual compressed sparse storage order,
    // so it does not depend on triplet insertion order.
    for (int outer = 0; outer < sparse.outerSize(); ++outer) {
        for (Eigen::SparseMatrix<c_float>::InnerIterator it(sparse, outer); it; ++it) {
            const Eigen::Index row = it.row();
            const Eigen::Index col = it.col();

            if (row < C.rows()) {
                values[idx++] = static_cast<c_float>(C(row, col));
            } else {
                values[idx++] = static_cast<c_float>(D(row - C.rows(), col));
            }
        }
    }

    if (idx != sparse.nonZeros()) {
        throw std::runtime_error("fillConstraintValues did not fill all sparse entries");
    }
}

const char* osqpStatusName(const OsqpEigen::Status status) {
    switch (status) {
        case OsqpEigen::Status::DualInfeasibleInaccurate:
            return "DualInfeasibleInaccurate";
        case OsqpEigen::Status::PrimalInfeasibleInaccurate:
            return "PrimalInfeasibleInaccurate";
        case OsqpEigen::Status::SolvedInaccurate:
            return "SolvedInaccurate";
        case OsqpEigen::Status::Solved:
            return "Solved";
        case OsqpEigen::Status::MaxIterReached:
            return "MaxIterReached";
        case OsqpEigen::Status::PrimalInfeasible:
            return "PrimalInfeasible";
        case OsqpEigen::Status::DualInfeasible:
            return "DualInfeasible";
        case OsqpEigen::Status::Sigint:
            return "Sigint";
        case OsqpEigen::Status::TimeLimitReached:
            return "TimeLimitReached";
        case OsqpEigen::Status::NonCvx:
            return "NonCvx";
        case OsqpEigen::Status::Unsolved:
            return "Unsolved";
    }
    return "Unknown";
}

const char* osqpErrorExitFlagName(const OsqpEigen::ErrorExitFlag flag) {
    switch (flag) {
        case OsqpEigen::ErrorExitFlag::NoError:
            return "NoError";
        case OsqpEigen::ErrorExitFlag::DataValidationError:
            return "DataValidationError";
        case OsqpEigen::ErrorExitFlag::SettingsValidationError:
            return "SettingsValidationError";
        case OsqpEigen::ErrorExitFlag::LinsysSolverLoadError:
            return "LinsysSolverLoadError";
        case OsqpEigen::ErrorExitFlag::LinsysSolverInitError:
            return "LinsysSolverInitError";
        case OsqpEigen::ErrorExitFlag::NonCvxError:
            return "NonCvxError";
        case OsqpEigen::ErrorExitFlag::MemAllocError:
            return "MemAllocError";
        case OsqpEigen::ErrorExitFlag::WorkspaceNotInitError:
            return "WorkspaceNotInitError";
    }
    return "Unknown";
}

bool isSolvedStatus(const OsqpEigen::Status status) {
    return status == OsqpEigen::Status::Solved ||
           status == OsqpEigen::Status::SolvedInaccurate;
}

std::string formatOsqpStatus(const OsqpEigen::Status status) {
    std::ostringstream oss;
    oss << osqpStatusName(status) << "(" << static_cast<int>(status) << ")";
    return oss.str();
}

std::string formatOsqpExitFlag(const OsqpEigen::ErrorExitFlag flag) {
    std::ostringstream oss;
    oss << osqpErrorExitFlagName(flag) << "(" << static_cast<int>(flag) << ")";
    return oss.str();
}

StateWeightMat bodyYawStateCost(const StateWeightMat& stateWeight,
                                const double psiRef) {
    const Mat3<double> R_BW = Rz(psiRef).transpose();

    Mat13d T = Mat13d::Identity();
    T.block<3, 3>(3, 3) = R_BW;   // COM position error: world to body-yaw.
    T.block<3, 3>(6, 6) = R_BW;   // Angular velocity error: world to body-yaw.
    T.block<3, 3>(9, 9) = R_BW;   // COM velocity error: world to body-yaw.

    return T.transpose() * stateWeight * T;
}
}  // namespace

ConvexMPC::ConvexMPC()
    : _hessian(makeUpperTriangularPattern(12 * horizonSteps())),
      _constraintMatrix(makeRowLevelConstraintPattern(horizonSteps())),
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
      _weightedStateError(13 * horizonSteps()),
      _optimalWrenchHorizon(12 * horizonSteps()) {
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
    _optimalWrenchHorizon.setZero();
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
                                            const Vec13<double>& x0,
                                            const LocomotionMode locomotionMode) {
    ConvexMPCInputView input;
    input.A_qp = &formulation.A_qp;
    input.B_qp = &formulation.B_qp;
    input.X_ref = &referenceTrajectory.X_ref;
    input.x0 = &x0;
    input.C = &gaitScheduler.C;
    input.C_bound = &gaitScheduler.C_bound;
    input.D = &gaitScheduler.D;
    input.locomotionMode = locomotionMode;
    return input;
}

void ConvexMPC::updateInput(const GaitScheduler& gaitScheduler,
                            const MPCFormulationOutput& formulation,
                            const ReferenceTrajectoryOutput& referenceTrajectory,
                            const Vec13<double>& x0,
                            const LocomotionMode locomotionMode) {
    const ConvexMPCInputView input =
        ConvexMPCInputView::from(gaitScheduler, formulation, referenceTrajectory, x0, locomotionMode);

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
    _hasContactConstraintSignature = false;
    _skipWarmStartForCurrentSolve = false;
    _contactConstraintSignature.clear();
    _optimalWrench.setZero();
    _optimalWrenchHorizon.setZero();
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
    return getL(_input.locomotionMode);
}

const DMat<double>& ConvexMPC::K() const {
    return getK(_input.locomotionMode);
}

const Vec12<double>& ConvexMPC::optimalWrench() const {
    return _optimalWrench;
}

const DVec<double>& ConvexMPC::optimalWrenchHorizon() const {
    return _optimalWrenchHorizon;
}

bool ConvexMPC::hasSolution() const {
    return _hasPreviousSolution;
}

const StateWeightMat& ConvexMPC::stateWeightForMode(const LocomotionMode mode) {
    const auto& mpc = getControllerConfig().mpc;
    return mode == LocomotionMode::Standing ? mpc.standingStateWeight : mpc.walkingStateWeight;
}

const InputWeightMat& ConvexMPC::inputWeightForMode(const LocomotionMode mode) {
    const auto& mpc = getControllerConfig().mpc;
    return mode == LocomotionMode::Standing ? mpc.standingInputWeight : mpc.walkingInputWeight;
}

void ConvexMPC::buildQP() {
    if (!_hasInput) {
        throw std::runtime_error("ConvexMPC::buildQP requires initialized input");
    }

    profiling::ScopedTimer buildTimer(_buildQpTime);

    const DMat<double>& A_qp = *_input.A_qp;
    const DMat<double>& B_qp = *_input.B_qp;
    const DVec<double>& X_ref = *_input.X_ref;
    const Vec13<double>& x0 = *_input.x0;
    const DMat<double>& C = *_input.C;
    const DVec<double>& C_bound = *_input.C_bound;
    const DMat<double>& D = *_input.D;
    const StateWeightMat& stateWeight = stateWeightForMode(_input.locomotionMode);
    const InputWeightMat& inputWeight = inputWeightForMode(_input.locomotionMode);
    const int steps = horizonSteps();

    {
        profiling::ScopedTimer timer(_stateProjectionTime);
        _statePrediction.noalias() = A_qp * x0;
        _stateError = _statePrediction;
        _stateError -= X_ref;
    }

    {
        profiling::ScopedTimer timer(_weightedAssemblyTime);
        for (int k = 0; k < steps; ++k) {
            const Eigen::Index stateOffset = static_cast<Eigen::Index>(13 * k);
            const double psiRef = X_ref[stateOffset + 2];
            const StateWeightMat stateCost = bodyYawStateCost(stateWeight, psiRef);

            _weightedB.middleRows(stateOffset, 13).noalias() =
                stateCost * B_qp.middleRows(stateOffset, 13);
            _weightedStateError.segment(stateOffset, 13).noalias() =
                stateCost * _stateError.segment(stateOffset, 13);
        }
    }

    {
        profiling::ScopedTimer timer(_hessianAssemblyTime);
        _hessianDense.noalias() = 2.0 * (B_qp.transpose() * _weightedB);
        for (int k = 0; k < steps; ++k) {
            const Eigen::Index inputOffset = static_cast<Eigen::Index>(12 * k);
            _hessianDense.block(inputOffset, inputOffset, 12, 12) += 2.0 * inputWeight;
        }
        _hessianDense = 0.5 * (_hessianDense + _hessianDense.transpose());
    }

    {
        profiling::ScopedTimer timer(_gradientAssemblyTime);
        _gradientDense.noalias() = 2.0 * (B_qp.transpose() * _weightedStateError);
    }

    {
        profiling::ScopedTimer timer(_sparseHessianTime);
        buildHessianMatrix(_hessianDense);
    }
    {
        profiling::ScopedTimer timer(_sparseConstraintTime);
        buildConstraintMatrix(C, D);
    }

    const std::vector<int> currentContactSignature = contactConstraintSignature(D);
    const bool contactSignatureChanged =
        _hasContactConstraintSignature &&
        currentContactSignature != _contactConstraintSignature;

    _gradient = _gradientDense.cast<c_float>();
    _lowerBound.head(numIneq()).setConstant(-OsqpEigen::INFTY);
    _upperBound.head(numIneq()) = C_bound.cast<c_float>();
    _lowerBound.tail(numEq()).setZero();
    _upperBound.tail(numEq()).setZero();

    const bool shouldColdInitialize = !_solverInitialized || contactSignatureChanged;
    if (contactSignatureChanged) {
        ++_contactSignatureChangeCount;
    }
    if (shouldColdInitialize) {
        ++_coldStartCount;
    }
    const bool success = shouldColdInitialize ? initializeSolver() : updateSolverData();
    if (!success) {
        throw std::runtime_error("ConvexMPC failed to setup/update OsqpEigen solver");
    }

    _contactConstraintSignature = currentContactSignature;
    _hasContactConstraintSignature = true;
    _skipWarmStartForCurrentSolve = contactSignatureChanged;
    _qpReady = true;
}

void ConvexMPC::solve() {
    if (!_hasInput) {
        throw std::runtime_error("ConvexMPC::solve requires initialized input");
    }

    if (!_qpReady) {
        buildQP();
    }

    profiling::ScopedTimer solveTimer(_solveTime);

    auto solveCurrentProblem = [this](const bool useWarmStart,
                                      const char* context) -> OsqpEigen::Status {
        if (useWarmStart && _hasPreviousSolution && !_solver.setPrimalVariable(_warmStart)) {
            std::ostringstream oss;
            oss << "ConvexMPC failed to set OsqpEigen warm start"
                << " context=" << context;
            throw std::runtime_error(oss.str());
        }

        OsqpEigen::ErrorExitFlag exitFlag;
        {
            profiling::ScopedTimer timer(_osqpSolveCallTime);
            exitFlag = _solver.solveProblem();
        }
        if (exitFlag != OsqpEigen::ErrorExitFlag::NoError) {
            std::ostringstream oss;
            oss << "ConvexMPC OsqpEigen solveProblem failed"
                << " context=" << context
                << " exit_flag=" << formatOsqpExitFlag(exitFlag);
            throw std::runtime_error(oss.str());
        }
        return _solver.getStatus();
    };

    const auto firstStatus = solveCurrentProblem(!_skipWarmStartForCurrentSolve, "initial");
        if (!isSolvedStatus(firstStatus)) {
            ++_solveRetryCount;
            if (!initializeSolver()) {
                std::ostringstream oss;
                oss << "ConvexMPC OsqpEigen fresh retry initialize failed"
                    << " first_status=" << formatOsqpStatus(firstStatus);
                throw std::runtime_error(oss.str());
        }

        const auto retryStatus = solveCurrentProblem(false, "fresh_cold_retry");
        if (!isSolvedStatus(retryStatus)) {
            std::ostringstream oss;
            oss << "ConvexMPC OsqpEigen did not converge to a valid solution"
                << " first_status=" << formatOsqpStatus(firstStatus)
                << " retry_status=" << formatOsqpStatus(retryStatus);
            throw std::runtime_error(oss.str());
        }

        std::cerr << "[MPC] OSQP fresh cold retry recovered"
                  << " first_status=" << formatOsqpStatus(firstStatus)
                  << " retry_status=" << formatOsqpStatus(retryStatus)
                  << std::endl;
    }

    const auto& solution = _solver.getSolution();
    if (solution.size() != numVars()) {
        throw std::runtime_error("ConvexMPC received solution with unexpected dimension");
    }

    {
        profiling::ScopedTimer timer(_solutionExtractTime);
        _lastSolution = solution;
        updateWarmStart();
        _hasPreviousSolution = true;
        _optimalWrenchHorizon = solution.cast<double>();
        _optimalWrench = solution.segment(0, 12).cast<double>();
        _skipWarmStartForCurrentSolve = false;
    }
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
    profiling::ScopedTimer timer(_osqpInitTime);

    if (_solver.isInitialized()) {
        _solver.clearSolver();
    }
    _solver.data()->clearHessianMatrix();
    _solver.data()->clearLinearConstraintsMatrix();

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
    profiling::ScopedTimer timer(_osqpUpdateTime);

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

std::vector<int> ConvexMPC::contactConstraintSignature(const DMat<double>& D) const {
    std::vector<int> signature;
    signature.reserve(static_cast<std::size_t>(2 * horizonSteps()));
    for (int k = 0; k < horizonSteps(); ++k) {
        const Eigen::Index offset = static_cast<Eigen::Index>(12 * k);
        const bool leftForceZeroEquality =
            D.block(offset + 0, offset + 0, 3, 3).norm() > 1e-9;
        const bool rightForceZeroEquality =
            D.block(offset + 3, offset + 3, 3, 3).norm() > 1e-9;
        signature.push_back(leftForceZeroEquality ? 0 : 1);
        signature.push_back(rightForceZeroEquality ? 0 : 1);
    }
    return signature;
}

void ConvexMPC::updateWarmStart() {
    if (_lastSolution.size() != numVars()) {
        throw std::runtime_error("ConvexMPC warm start dimension mismatch");
    }

    _warmStart.head(numVars() - 12) = _lastSolution.segment(12, numVars() - 12);
    _warmStart.tail(12) = _lastSolution.tail(12);
}

void ConvexMPC::printProfilingSummary(std::ostream& out) const {
    if (!profiling::enabled()) {
        return;
    }

    out << profiling::formatTimingStats("qp_build", _buildQpTime) << '\n';
    out << "qp_cold_starts: " << _coldStartCount << '\n';
    out << "qp_contact_signature_changes: " << _contactSignatureChangeCount << '\n';
    out << "qp_solve_retries: " << _solveRetryCount << '\n';
    out << profiling::formatTimingStats("qp_state_projection", _stateProjectionTime) << '\n';
    out << profiling::formatTimingStats("qp_weighted_assembly", _weightedAssemblyTime) << '\n';
    out << profiling::formatTimingStats("qp_hessian_assembly", _hessianAssemblyTime) << '\n';
    out << profiling::formatTimingStats("qp_gradient_assembly", _gradientAssemblyTime) << '\n';
    out << profiling::formatTimingStats("qp_sparse_hessian", _sparseHessianTime) << '\n';
    out << profiling::formatTimingStats("qp_sparse_constraint", _sparseConstraintTime) << '\n';
    out << profiling::formatTimingStats("osqp_init", _osqpInitTime) << '\n';
    out << profiling::formatTimingStats("osqp_update", _osqpUpdateTime) << '\n';
    out << profiling::formatTimingStats("qp_solve", _solveTime) << '\n';
    out << profiling::formatTimingStats("osqp_solve_call", _osqpSolveCallTime) << '\n';
    out << profiling::formatTimingStats("qp_solution_extract", _solutionExtractTime) << '\n';
}
