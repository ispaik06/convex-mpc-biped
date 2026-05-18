#include "ReferenceTrajectory.h"

#include <stdexcept>

#include "BodyMotionReference.h"
#include "Utilities/MatrixUtils.h"

void ReferenceTrajectory::build(ReferenceTrajectoryOutput& out) const {
    if (_horizonClock == nullptr) {
        throw std::runtime_error("ReferenceTrajectory::build requires initialized HorizonClock");
    }

    out.resizeIfNeeded();
    out.setZero();

    const double psi_dot_des = (_userCommand != nullptr) ? _userCommand->psi_dot : 0.0;
    const double bodyHeightOffset_m =
        (_userCommand != nullptr) ? _userCommand->body_height_offset_m : 0.0;
    // Yaw only changes x/y; the body height offset is applied directly to the vertical state.
    const Vec3<double> u_des_B(
        _userCommand != nullptr ? _userCommand->x_dot : 0.0,
        _userCommand != nullptr ? _userCommand->y_dot : 0.0,
        0.0);

    const double psi0 = _x0[2];
    const double gravity = _x0[12];
    const double dt = dtMpc();
    const int steps = horizonSteps();
    const YawIntegrationMode yawIntegrationMode =
        getControllerConfig().referenceTrajectory.yawIntegrationMode;

    Vec3<double> p_ref = _x0.template segment<3>(3);
    p_ref[2] += bodyHeightOffset_m;
    Vec3<double> euler_ref = _x0.template segment<3>(0);
    Vec3<double> v_ref_W = Vec3<double>::Zero();
    const Vec2<double> planarCommand_B(u_des_B.x(), u_des_B.y());
    double psi_ref = psi0;

    for (int k = 0; k < steps; ++k) {
        const double tk = _horizonClock->tk(k);
        const double yawSampleTime = (k == 0) ? tk : (tk - 0.5 * dt);
        if (k > 0) {
            psi_ref = BodyMotionReference::advanceYaw(_gaitScheduler,
                                                      psi_ref,
                                                      psi_dot_des,
                                                      dt,
                                                      yawSampleTime,
                                                      yawIntegrationMode);
        }

        const double psi_k = psi_ref;
        v_ref_W = BodyMotionReference::worldVelocity(u_des_B, psi_k);

        if (k > 0) {
            p_ref = BodyMotionReference::advancePlanarPosition(
                p_ref,
                psi_k,
                planarCommand_B,
                dt);
            p_ref[2] = _x0[5] + bodyHeightOffset_m;
        }

        out.tk[k] = tk;
        out.psi[k] = psi_k;
        out.r_left.col(k) = _desiredFootPositions.left_des_W - p_ref;
        out.r_right.col(k) = _desiredFootPositions.right_des_W - p_ref;

        const Eigen::Index stateOffset = static_cast<Eigen::Index>(13 * k);
        auto x_ref_k = out.X_ref.segment(stateOffset, 13);
        x_ref_k.setZero();
        x_ref_k.template segment<3>(0) = euler_ref;
        x_ref_k[2] = psi_k;
        x_ref_k.template segment<3>(3) = p_ref;
        x_ref_k[6] = 0.0;
        x_ref_k[7] = 0.0;
        x_ref_k[8] = BodyMotionReference::yawRate(_gaitScheduler,
                                                  psi_dot_des,
                                                  yawSampleTime,
                                                  yawIntegrationMode);
        x_ref_k.template segment<3>(9) = v_ref_W;
        x_ref_k[12] = gravity;
    }
}
