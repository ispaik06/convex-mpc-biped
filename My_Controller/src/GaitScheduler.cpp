#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include "GaitScheduler.h"
#include "Utilities/MatrixUtils.h"

namespace {
double footYawFromXAxisWorld(const Vec3<double>& xAxis_W) {
    if (!xAxis_W.allFinite()) {
        throw std::runtime_error("Foot axis is non-finite");
    }

    const double planarNorm = std::hypot(xAxis_W.x(), xAxis_W.y());
    if (planarNorm <= 1e-9) {
        throw std::runtime_error("Foot axis has no usable yaw component");
    }

    return std::atan2(xAxis_W.y(), xAxis_W.x());
}

double resolvedFootYaw(const Vec3<double>& xAxis_W, const double yaw_W) {
    if (std::isfinite(yaw_W)) {
        return yaw_W;
    }
    return footYawFromXAxisWorld(xAxis_W);
}

void fillYawRotatedFootConstraintBlock(const Eigen::Matrix<double, 12, 6>& localBlock,
                                      const double yaw_W,
                                      Eigen::Matrix<double, 12, 12>& worldBlock,
                                      const int forceColOffset,
                                      const int momentColOffset) {
    const double c = std::cos(yaw_W);
    const double s = std::sin(yaw_W);

    worldBlock.setZero();
    for (int row = 0; row < 12; ++row) {
        const double fx = localBlock(row, 0);
        const double fy = localBlock(row, 1);
        const double fz = localBlock(row, 2);
        const double mx = localBlock(row, 3);
        const double my = localBlock(row, 4);
        const double mz = localBlock(row, 5);

        worldBlock(row, forceColOffset + 0) = c * fx - s * fy;
        worldBlock(row, forceColOffset + 1) = s * fx + c * fy;
        worldBlock(row, forceColOffset + 2) = fz;
        worldBlock(row, momentColOffset + 0) = c * mx - s * my;
        worldBlock(row, momentColOffset + 1) = s * mx + c * my;
        worldBlock(row, momentColOffset + 2) = mz;
    }
}

Vec3<double> normalizedAxisOrThrow(const Vec3<double>& axis, const char* name) {
    if (!axis.allFinite()) {
        throw std::runtime_error(std::string("Non-finite ") + name);
    }

    const double norm = axis.norm();
    if (norm <= 1e-9) {
        throw std::runtime_error(std::string("Degenerate ") + name);
    }
    return axis / norm;
}
}  // namespace

void GaitScheduler::setLocomotionMode(const LocomotionMode locomotionMode) {
    _locomotionMode = locomotionMode;
}

void GaitScheduler::setFootLocalXAxesWorld(const Vec3<double>& leftFootXAxis_W,
                                           const Vec3<double>& rightFootXAxis_W) {
    _leftFootXAxis_W = normalizedAxisOrThrow(leftFootXAxis_W, "left foot local x axis");
    _rightFootXAxis_W = normalizedAxisOrThrow(rightFootXAxis_W, "right foot local x axis");
}

void GaitScheduler::rebuildContactConstraintTemplate() {
    const auto& mpc = getControllerConfig().mpc;
    C_unit <<
         1, 0, -mpc.frictionCoefficient, 0, 0, 0,
        -1, 0, -mpc.frictionCoefficient, 0, 0, 0,
         0, 1, -mpc.frictionCoefficient, 0, 0, 0,
         0, -1, -mpc.frictionCoefficient, 0, 0, 0,
         0, 0, 1, 0, 0, 0,
         0, 0, -1, 0, 0, 0,
         0, 0, -mpc.footHalfWidth, 1, 0, 0,
         0, 0, -mpc.footHalfWidth, -1, 0, 0,
         0, 0, -mpc.footHalfLength, 0, 1, 0,
         0, 0, -mpc.footHalfLength, 0, -1, 0,
         0, 0, -mpc.torsionalFrictionScale * mpc.frictionCoefficient, 0, 0, 1,
         0, 0, -mpc.torsionalFrictionScale * mpc.frictionCoefficient, 0, 0, -1;
}

double GaitScheduler::p(Side i, double t) const {
    if (_locomotionMode == LocomotionMode::Standing) {
        return 0.0;
    }

    if (_horizonClock == nullptr) {
        throw std::runtime_error("GaitScheduler::p requires initialized HorizonClock");
    }

    double phi = 0;
    if (i == Side::Left) phi = 0.5;
    return std::fmod((t - _horizonClock->t0()) / cycleTime() + phi, 1.0);
}

bool GaitScheduler::c(Side i, double t) const {
    if (_locomotionMode == LocomotionMode::Standing) {
        return true;
    }

    const double phase = p(i, t);
    const double stanceFraction = stanceTime() / cycleTime();
    return (0.0 <= phase && phase < stanceFraction);
}

bool GaitScheduler::bothFeetStance(const double t) const {
    return c(Side::Left, t) && c(Side::Right, t);
}

const std::vector<GaitConstraintStep>& GaitScheduler::constraintSteps() const {
    return _constraintSteps;
}

bool GaitScheduler::constrainFootRollMoment() const {
    return _constrainFootRollMoment;
}

void GaitScheduler::buildConstraintMatrices(const ContactScheduleOverride* contactOverride,
                                            const double leftFootYaw_W,
                                            const double rightFootYaw_W) {
    if (_horizonClock == nullptr) {
        throw std::runtime_error(
            "GaitScheduler::buildConstraintMatrices requires initialized HorizonClock");
    }

    const int steps = horizonSteps();
    const auto& mpc = getControllerConfig().mpc;
    _constrainFootRollMoment =
        contactWrenchModel(_locomotionMode) == ContactWrenchModel::NoRollMoment;
    const double resolvedLeftFootYaw_W = resolvedFootYaw(_leftFootXAxis_W, leftFootYaw_W);
    const double resolvedRightFootYaw_W = resolvedFootYaw(_rightFootXAxis_W, rightFootYaw_W);

    C_bound.setZero(24 * steps);
    _constraintSteps.resize(static_cast<std::size_t>(steps));

    for (int k = 0; k < steps; ++k) {
        const double tk = _horizonClock->tk(k);
        Ck_bound = Vec24<double>::Zero();
        bool leftStance = c(Side::Left, tk);
        bool rightStance = c(Side::Right, tk);
        double leftNormalForceMinScale = 1.0;
        double rightNormalForceMinScale = 1.0;
        if (contactOverride != nullptr &&
            k < static_cast<int>(contactOverride->steps.size()) &&
            contactOverride->steps[static_cast<std::size_t>(k)].enabled) {
            const auto& step = contactOverride->steps[static_cast<std::size_t>(k)];
            leftStance = step.leftContact;
            rightStance = step.rightContact;
            leftNormalForceMinScale = std::clamp(step.leftNormalForceMinScale, 0.0, 1.0);
            rightNormalForceMinScale = std::clamp(step.rightNormalForceMinScale, 0.0, 1.0);
        }

        if (leftStance) {
            Ck_bound(4) = mpc.normalForceMax;
            Ck_bound(5) = -leftNormalForceMinScale * mpc.normalForceMin;
        }

        if (rightStance) {
            Ck_bound(16) = mpc.normalForceMax;
            Ck_bound(17) = -rightNormalForceMinScale * mpc.normalForceMin;
        }

        const Eigen::Index cOffset = static_cast<Eigen::Index>(24 * k);
        C_bound.segment(cOffset, 24) = Ck_bound;

        GaitConstraintStep& constraintStep = _constraintSteps[static_cast<std::size_t>(k)];
        constraintStep.leftStance = leftStance;
        constraintStep.rightStance = rightStance;
        constraintStep.leftNormalForceMinScale = leftNormalForceMinScale;
        constraintStep.rightNormalForceMinScale = rightNormalForceMinScale;
        constraintStep.leftFootYaw_W = resolvedLeftFootYaw_W;
        constraintStep.rightFootYaw_W = resolvedRightFootYaw_W;
        constraintStep.leftFootXAxis_W = _leftFootXAxis_W;
        constraintStep.rightFootXAxis_W = _rightFootXAxis_W;
    }

}
