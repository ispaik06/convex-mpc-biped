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

void GaitScheduler::buildConstraintMatrices(const ContactScheduleOverride* contactOverride,
                                            const double leftFootYaw_W,
                                            const double rightFootYaw_W) {
    if (_horizonClock == nullptr) {
        throw std::runtime_error(
            "GaitScheduler::buildConstraintMatrices requires initialized HorizonClock");
    }

    const int steps = horizonSteps();
    const auto& mpc = getControllerConfig().mpc;
    const bool constrainFootRollMoment =
        contactWrenchModel(_locomotionMode) == ContactWrenchModel::NoRollMoment;
    const bool enableCopConstraint = mpc.enableCopConstraint;
    const double resolvedLeftFootYaw_W = resolvedFootYaw(_leftFootXAxis_W, leftFootYaw_W);
    const double resolvedRightFootYaw_W = resolvedFootYaw(_rightFootXAxis_W, rightFootYaw_W);

    D.setZero(12 * steps, 12 * steps);
    C.setZero(24 * steps, 12 * steps);
    C_bound.setZero(24 * steps);

    for (int k = 0; k < steps; ++k) {
        const double tk = _horizonClock->tk(k);
        Ck_bound = Vec24<double>::Zero();
        Mat3<double> S_left = Mat3<double>::Zero();
        Mat3<double> S_right = Mat3<double>::Zero();
        Mat12<double> C_left = Mat12<double>::Zero();
        Mat12<double> C_right = Mat12<double>::Zero();
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

        if (!leftStance) {  // swing
            S_left = Mat3<double>::Identity();
        }
        else {  // stance
            fillYawRotatedFootConstraintBlock(C_unit,
                                             resolvedLeftFootYaw_W,
                                             C_left,
                                             0,
                                             6);
            if (!enableCopConstraint) {
                C_left.block<4, 12>(6, 0).setZero();
            }
            Ck_bound(4) = mpc.normalForceMax;
            Ck_bound(5) = -leftNormalForceMinScale * mpc.normalForceMin;
        }

        if (!rightStance) {  // swing
            S_right = Mat3<double>::Identity();
        }
        else {  // stance
            fillYawRotatedFootConstraintBlock(C_unit,
                                             resolvedRightFootYaw_W,
                                             C_right,
                                             3,
                                             9);
            if (!enableCopConstraint) {
                C_right.block<4, 12>(6, 0).setZero();
            }
            Ck_bound(16) = mpc.normalForceMax;
            Ck_bound(17) = -rightNormalForceMinScale * mpc.normalForceMin;
        }

        Mat12<double> Dk = Mat12<double>::Zero();
        Dk.block<3, 3>(0, 0) = S_left;
        Dk.block<3, 3>(3, 3) = S_right;
        Dk.block<3, 3>(6, 6) = S_left;
        Dk.block<3, 3>(9, 9) = S_right;
        if (constrainFootRollMoment && leftStance) {
            Dk.block<1, 3>(6, 6) = _leftFootXAxis_W.transpose();
        }
        if (constrainFootRollMoment && rightStance) {
            Dk.block<1, 3>(9, 9) = _rightFootXAxis_W.transpose();
        }

        const Eigen::Index dOffset = static_cast<Eigen::Index>(12 * k);
        const Eigen::Index cOffset = static_cast<Eigen::Index>(24 * k);
        D.block(dOffset, dOffset, 12, 12) = Dk;
        C.block(cOffset + 0, dOffset, 12, 12) = C_left;
        C.block(cOffset + 12, dOffset, 12, 12) = C_right;
        C_bound.segment(cOffset, 24) = Ck_bound;
    }
}
