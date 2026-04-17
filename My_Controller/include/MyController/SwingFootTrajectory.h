#ifndef SWING_FOOT_TRAJECTORY_H
#define SWING_FOOT_TRAJECTORY_H

#include "cppTypes.h"

class SwingFootTrajectory {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void reset(const Vec3<double>& pInit,
               const Vec3<double>& pFinal,
               double swingHeight,
               double swingTime);

    void setFinalPosition(const Vec3<double>& pFinal);
    void advance(double dt);

    const Vec3<double>& position() const;
    const Vec3<double>& velocity() const;
    const Vec3<double>& acceleration() const;

    double remainingTime() const;
    double phase() const;
    bool active() const;
    bool finished() const;
    void deactivate();

private:
    void updateOutputs();
    static double smoothBlend(double s);
    static double smoothBlendDot(double s);
    static double smoothBlendDdot(double s);

    Vec3<double> _pInit = Vec3<double>::Zero();
    Vec3<double> _pFinal = Vec3<double>::Zero();
    Vec3<double> _p = Vec3<double>::Zero();
    Vec3<double> _v = Vec3<double>::Zero();
    Vec3<double> _a = Vec3<double>::Zero();
    double _swingHeight{0.06};
    double _swingTime{1.0};
    double _remainingTime{0.0};
    bool _active{false};
};

#endif  // SWING_FOOT_TRAJECTORY_H
