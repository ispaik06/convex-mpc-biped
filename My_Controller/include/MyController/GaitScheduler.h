#ifndef GAIT_SCHEDULER_H
#define GAIT_SCHEDULER_H

#include "ContactSchedule.h"
#include "ControllerConfig.h"
#include "HorizonClock.h"
#include "Robot/RobotParams.h"
#include <limits>
#include <vector>

struct GaitConstraintStep {
    bool leftStance{true};
    bool rightStance{true};
    double leftNormalForceMinScale{1.0};
    double rightNormalForceMinScale{1.0};
    double leftFootYaw_W{0.0};
    double rightFootYaw_W{0.0};
    Vec3<double> leftFootXAxis_W = Vec3<double>::UnitX();
    Vec3<double> rightFootXAxis_W = Vec3<double>::UnitX();
};

class GaitScheduler {
public:
    explicit GaitScheduler(const HorizonClock* horizonClock)
        : _horizonClock(horizonClock) {
        rebuildContactConstraintTemplate();
        Ck_bound = Vec24<double>::Zero();
    }

    void setLocomotionMode(LocomotionMode locomotionMode);
    void setFootLocalXAxesWorld(const Vec3<double>& leftFootXAxis_W,
                                const Vec3<double>& rightFootXAxis_W);

    double p(Side, double) const;
    bool c(Side, double) const;
    bool bothFeetStance(double t) const;

    void buildConstraintMatrices(const ContactScheduleOverride* contactOverride = nullptr,
                                 double leftFootYaw_W = std::numeric_limits<double>::quiet_NaN(),
                                 double rightFootYaw_W = std::numeric_limits<double>::quiet_NaN());

    const std::vector<GaitConstraintStep>& constraintSteps() const;
    bool constrainFootRollMoment() const;

    DMat<double> D;
    DMat<double> C;
    DVec<double> C_bound;

private:
    void rebuildContactConstraintTemplate();

    const HorizonClock* _horizonClock = nullptr;
    LocomotionMode _locomotionMode{LocomotionMode::Walking};
    Vec3<double> _leftFootXAxis_W = Vec3<double>::UnitX();
    Vec3<double> _rightFootXAxis_W = Vec3<double>::UnitX();
    bool _constrainFootRollMoment{false};
    std::vector<GaitConstraintStep> _constraintSteps;
    Eigen::Matrix<double, 12, 6> C_unit;
    Vec24<double> Ck_bound;
};

#endif  // GAIT_SCHEDULER_H
