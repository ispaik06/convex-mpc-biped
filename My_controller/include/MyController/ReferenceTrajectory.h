#ifndef REFERNCE_TRAJECTORY_H
#define REFERNCE_TRAJECTORY_H

#include "ControlFSM.h"
#include "Utilities/UserCommand.h"
#include "cppTypes.h"

struct ReferenceTrajectoryOutput {
    DMat<double> X_ref;
    D3Mat<double> r_left;
    D3Mat<double> r_right;
    DVec<double> psi;
    DVec<double> tk;

    ReferenceTrajectoryOutput()
        : X_ref(13, N),
          r_left(3, N),
          r_right(3, N),
          psi(N),
          tk(N) {
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
                        const PosFootDes& posFootDes,
                        double t0)
        : _userCommand(userCommand),
          _x0(x0),
          _posFootDes(posFootDes),
          _t0(t0) {}

    ReferenceTrajectoryOutput build() const;

private:
    const UserCommand* _userCommand = nullptr;
    Vec13<double> _x0 = Vec13<double>::Zero();
    PosFootDes _posFootDes;
    double _t0{0.0};
};

#endif  // REFERNCE_TRAJECTORY_H
