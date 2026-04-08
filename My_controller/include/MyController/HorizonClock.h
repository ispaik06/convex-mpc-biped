#ifndef HORIZON_CLOCK_H
#define HORIZON_CLOCK_H

#include <cmath>
#include <stdexcept>

#include "ControlParameters.h"

class HorizonClock {
public:
    explicit HorizonClock(const double t0 = 0.0)
        : _t0(t0) {}

    void reset(const double t0 = 0.0) {
        _t0 = t0;
    }

    void sync(const double t) {
        if (!std::isfinite(t)) {
            throw std::runtime_error("HorizonClock::sync received non-finite time");
        }

        while (t - _t0 >= T_cycle) {
            _t0 += T_cycle;
        }
    }

    double t0() const {
        return _t0;
    }

    double tk(const int k) const {
        return _t0 + static_cast<double>(k) * dt_mpc;
    }

private:
    double _t0{0.0};
};

#endif  // HORIZON_CLOCK_H
