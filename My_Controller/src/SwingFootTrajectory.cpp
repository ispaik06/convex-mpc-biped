#include <algorithm>
#include <stdexcept>

#include "SwingFootTrajectory.h"

void SwingFootTrajectory::reset(const Vec3<double>& pInit,
                                const Vec3<double>& pFinal,
                                const double swingHeight,
                                const double swingTime) {
    if (swingTime <= 0.0) {
        throw std::invalid_argument("SwingFootTrajectory requires positive swingTime");
    }

    _pInit = pInit;
    _pFinal = pFinal;
    _swingHeight = swingHeight;
    _swingTime = swingTime;
    _remainingTime = swingTime;
    _active = true;
    updateOutputs();
}

void SwingFootTrajectory::setFinalPosition(const Vec3<double>& pFinal) {
    _pFinal = pFinal;
    if (_active) {
        updateOutputs();
    }
}

void SwingFootTrajectory::advance(const double dt) {
    if (!_active) {
        return;
    }

    _remainingTime = std::max(0.0, _remainingTime - std::max(0.0, dt));
    updateOutputs();
    if (_remainingTime <= 0.0) {
        _active = false;
    }
}

const Vec3<double>& SwingFootTrajectory::position() const {
    return _p;
}

const Vec3<double>& SwingFootTrajectory::velocity() const {
    return _v;
}

const Vec3<double>& SwingFootTrajectory::acceleration() const {
    return _a;
}

double SwingFootTrajectory::remainingTime() const {
    return _remainingTime;
}

double SwingFootTrajectory::phase() const {
    if (_swingTime <= 0.0) {
        return 1.0;
    }
    return 1.0 - (_remainingTime / _swingTime);
}

bool SwingFootTrajectory::active() const {
    return _active;
}

bool SwingFootTrajectory::finished() const {
    return _remainingTime <= 0.0;
}

void SwingFootTrajectory::deactivate() {
    _remainingTime = 0.0;
    _active = false;
    updateOutputs();
}

void SwingFootTrajectory::updateOutputs() {
    const double s = std::clamp(phase(), 0.0, 1.0);
    const double ds_dt = (_swingTime > 0.0) ? (1.0 / _swingTime) : 0.0;
    const double d2s_dt2 = 0.0;

    const double blendXY = smoothBlend(s);
    const double blendXYDot = smoothBlendDot(s);
    const double blendXYDdot = smoothBlendDdot(s);

    const Vec3<double> delta = _pFinal - _pInit;

    _p = _pInit;
    _p.x() = (1.0 - blendXY) * _pInit.x() + blendXY * _pFinal.x();
    _p.y() = (1.0 - blendXY) * _pInit.y() + blendXY * _pFinal.y();

    _v.setZero();
    _v.x() = blendXYDot * delta.x() * ds_dt;
    _v.y() = blendXYDot * delta.y() * ds_dt;

    _a.setZero();
    _a.x() = delta.x() * (blendXYDdot * ds_dt * ds_dt + blendXYDot * d2s_dt2);
    _a.y() = delta.y() * (blendXYDdot * ds_dt * ds_dt + blendXYDot * d2s_dt2);

    if (s <= 0.5) {
        const double u = 2.0 * s;
        const double du_dt = 2.0 * ds_dt;
        const double blendZ = smoothBlend(u);
        const double blendZDot = smoothBlendDot(u);
        const double blendZDdot = smoothBlendDdot(u);
        const double zMid = _pFinal.z() + _swingHeight;
        const double zDelta = zMid - _pInit.z();

        _p.z() = (1.0 - blendZ) * _pInit.z() + blendZ * zMid;
        _v.z() = blendZDot * zDelta * du_dt;
        _a.z() = zDelta * blendZDdot * du_dt * du_dt;
    } else {
        const double u = 2.0 * s - 1.0;
        const double du_dt = 2.0 * ds_dt;
        const double blendZ = smoothBlend(u);
        const double blendZDot = smoothBlendDot(u);
        const double blendZDdot = smoothBlendDdot(u);
        const double zMid = _pFinal.z() + _swingHeight;
        const double zDelta = _pFinal.z() - zMid;

        _p.z() = (1.0 - blendZ) * zMid + blendZ * _pFinal.z();
        _v.z() = blendZDot * zDelta * du_dt;
        _a.z() = zDelta * blendZDdot * du_dt * du_dt;
    }
}

double SwingFootTrajectory::smoothBlend(const double s) {
    return 3.0 * s * s - 2.0 * s * s * s;
}

double SwingFootTrajectory::smoothBlendDot(const double s) {
    return 6.0 * s - 6.0 * s * s;
}

double SwingFootTrajectory::smoothBlendDdot(const double s) {
    return 6.0 - 12.0 * s;
}
