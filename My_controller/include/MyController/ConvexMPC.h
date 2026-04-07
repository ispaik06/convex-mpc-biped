#ifndef CONVEX_MPC_H
#define CONVEX_MPC_H

#include "GaitScheduler.h"
#include "MPCFormulation.h"

struct ConvexMPCInputView {
    const DMat<double>* A_qp{nullptr};
    const DMat<double>* B_qp{nullptr};
    const DMat<double>* X_ref{nullptr};
    const Vec13<double>* x0{nullptr};
    const DMat<double>* C{nullptr};
    const DVec<double>* C_bound{nullptr};
    const DMat<double>* D{nullptr};

    static ConvexMPCInputView from(const GaitScheduler& gaitScheduler,
                                   const MPCFormulationOutput& formulation,
                                   const ReferenceTrajectoryOutput& referenceTrajectory,
                                   const Vec13<double>& x0);
};

class ConvexMPC {
public:
    ConvexMPC() = default;

    void updateInput(const GaitScheduler& gaitScheduler,
                     const MPCFormulationOutput& formulation,
                     const ReferenceTrajectoryOutput& referenceTrajectory,
                     const Vec13<double>& x0);

    void clear();

    const ConvexMPCInputView& input() const;
    bool hasInput() const;

    void buildQP();
    void solve();

private:
    void validateInputDimensions(const ConvexMPCInputView& input) const;

    ConvexMPCInputView _input;
    bool _hasInput{false};
    
};

#endif  // CONVEX_MPC_H
