#ifndef SWING_ATTITUDE_CONTROL_H
#define SWING_ATTITUDE_CONTROL_H

#include <cmath>
#include <stdexcept>

#include "Controllers/LegController.h"

inline DVec<double> computeSwingAttitudeLevelTorque(const RobotLegState<double>& legState,
                                                    const LegControllerData<double>& legData,
                                                    const double desiredYaw_W,
                                                    const double rollKp,
                                                    const double rollKd,
                                                    const double pitchKp,
                                                    const double pitchKd,
                                                    const double yawKp,
                                                    const double yawKd) {
    const Eigen::Index dof = legData.dof();
    DVec<double> torque = DVec<double>::Zero(dof);
    const bool rollEnabled = rollKp > 0.0 || rollKd > 0.0;
    const bool pitchEnabled = pitchKp > 0.0 || pitchKd > 0.0;
    const bool yawEnabled = yawKp > 0.0 || yawKd > 0.0;
    if (!rollEnabled && !pitchEnabled && !yawEnabled) {
        return torque;
    }

    if (!legState.hasFootFrame) {
        throw std::runtime_error("Swing attitude control requires foot frame data");
    }
    if (!legData.hasFootData) {
        throw std::runtime_error("Swing attitude control requires foot angular Jacobian data");
    }
    if (legData.Jw_W.rows() != 3 || legData.Jw_W.cols() != dof || legData.qd.size() != dof) {
        throw std::runtime_error("Swing attitude control received inconsistent leg angular data");
    }

    Vec3<double> footX_W = legState.R_WF.col(0);
    Vec3<double> footY_W = legState.R_WF.col(1);
    Vec3<double> footZ_W = legState.R_WF.col(2);
    if (!footX_W.allFinite() || !footY_W.allFinite() || !footZ_W.allFinite() ||
        footX_W.norm() <= 1e-9 || footY_W.norm() <= 1e-9 || footZ_W.norm() <= 1e-9) {
        throw std::runtime_error("Swing attitude control received invalid foot frame axes");
    }
    footX_W.normalize();
    footY_W.normalize();
    footZ_W.normalize();

    const Vec3<double> worldUp = Vec3<double>::UnitZ();
    const Vec3<double> omegaFoot_W = legData.Jw_W * legData.qd;
    Vec3<double> moment_W = Vec3<double>::Zero();

    if (rollEnabled) {
        const double sinRollError = footX_W.dot(footZ_W.cross(worldUp));
        const double cosRollError = footZ_W.dot(worldUp);
        const double rollError = std::atan2(sinRollError, cosRollError);
        const double rollRate = footX_W.dot(omegaFoot_W);
        moment_W += (rollKp * rollError - rollKd * rollRate) * footX_W;
    }

    if (pitchEnabled) {
        const double sinPitchError = footY_W.dot(footZ_W.cross(worldUp));
        const double cosPitchError = footZ_W.dot(worldUp);
        const double pitchError = std::atan2(sinPitchError, cosPitchError);
        const double pitchRate = footY_W.dot(omegaFoot_W);
        moment_W += (pitchKp * pitchError - pitchKd * pitchRate) * footY_W;
    }

    if (yawEnabled) {
        Vec3<double> footXProj_W = footX_W - footX_W.dot(worldUp) * worldUp;
        if (footXProj_W.norm() <= 1e-9) {
            throw std::runtime_error("Swing attitude control received degenerate yaw axis");
        }
        footXProj_W.normalize();
        const Vec3<double> desiredX_W(std::cos(desiredYaw_W), std::sin(desiredYaw_W), 0.0);
        const double sinYawError = worldUp.dot(footXProj_W.cross(desiredX_W));
        const double cosYawError = footXProj_W.dot(desiredX_W);
        const double yawError = std::atan2(sinYawError, cosYawError);
        const double yawRate = worldUp.dot(omegaFoot_W);
        moment_W += (yawKp * yawError - yawKd * yawRate) * worldUp;
    }

    torque = legData.Jw_W.transpose() * moment_W;
    return torque;
}

#endif  // SWING_ATTITUDE_CONTROL_H
