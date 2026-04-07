#include "ConvexMPC.h"

#include <sstream>
#include <stdexcept>

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
}  // namespace

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
}

void ConvexMPC::clear() {
    _input = ConvexMPCInputView{};
    _hasInput = false;
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

void ConvexMPC::buildQP() {
    if (!_hasInput) {
        throw std::runtime_error("ConvexMPC::buildQP requires initialized input");
    }

    throw std::logic_error("ConvexMPC::buildQP is not implemented yet");
}

void ConvexMPC::solve() {
    if (!_hasInput) {
        throw std::runtime_error("ConvexMPC::solve requires initialized input");
    }

    throw std::logic_error("ConvexMPC::solve is not implemented yet");
}

void ConvexMPC::validateInputDimensions(const ConvexMPCInputView& input) const {
    requireNonNull(input.A_qp, "A_qp");
    requireNonNull(input.B_qp, "B_qp");
    requireNonNull(input.X_ref, "X_ref");
    requireNonNull(input.x0, "x0");
    requireNonNull(input.C, "C");
    requireNonNull(input.C_bound, "C_bound");
    requireNonNull(input.D, "D");

    requireShape(input.A_qp->rows(), input.A_qp->cols(), 13 * N, 13, "A_qp");
    requireShape(input.B_qp->rows(), input.B_qp->cols(), 13 * N, 12 * N, "B_qp");
    requireShape(input.X_ref->rows(), input.X_ref->cols(), 13, N, "X_ref");
    requireSize(input.x0->size(), 13, "x0");
    requireShape(input.C->rows(), input.C->cols(), 24 * N, 12 * N, "C");
    requireSize(input.C_bound->size(), 24 * N, "C_bound");
    requireShape(input.D->rows(), input.D->cols(), 12 * N, 12 * N, "D");
}
