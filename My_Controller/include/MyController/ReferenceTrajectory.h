#ifndef REFERENCE_TRAJECTORY_H
#define REFERENCE_TRAJECTORY_H

#include "ControllerConfig.h"
#include "HorizonClock.h"
#include "SwingFootPlanner.h"
#include "Utilities/UserCommand.h"
#include "cppTypes.h"

struct ReferenceTrajectoryOutput {
    DVec<double> X_ref;
    D3Mat<double> r_left;
    D3Mat<double> r_right;
    DVec<double> psi;
    DVec<double> tk;

    ReferenceTrajectoryOutput()
        : X_ref(13 * horizonSteps()),
          r_left(3, horizonSteps()),
          r_right(3, horizonSteps()),
          psi(horizonSteps()),
          tk(horizonSteps()) {
        setZero();
    }

    void resizeIfNeeded() {
        const int steps = horizonSteps();
        if (X_ref.size() == 13 * steps &&
            r_left.cols() == steps &&
            r_right.cols() == steps &&
            psi.size() == steps &&
            tk.size() == steps) {
            return;
        }

        X_ref.resize(13 * steps);
        r_left.resize(3, steps);
        r_right.resize(3, steps);
        psi.resize(steps);
        tk.resize(steps);
    }

    void setZero() {
        X_ref.setZero();
        r_left.setZero();
        r_right.setZero();
        psi.setZero();
        tk.setZero();
    }
};

class ReferenceTrajectory {
public:
    ReferenceTrajectory(const UserCommand* userCommand,
                        const Vec13<double>& x0,
                        const DesiredFootPositions& desiredFootPositions,
                        const HorizonClock* horizonClock)
        : _userCommand(userCommand),
          _x0(x0),
          _desiredFootPositions(desiredFootPositions),
          _horizonClock(horizonClock) {}

    void build(ReferenceTrajectoryOutput& out) const;

private:
    const UserCommand* _userCommand = nullptr;
    Vec13<double> _x0 = Vec13<double>::Zero();
    DesiredFootPositions _desiredFootPositions;
    const HorizonClock* _horizonClock = nullptr;
};

#endif  // REFERENCE_TRAJECTORY_H
