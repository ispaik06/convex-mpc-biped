#ifndef CONTROLLER_CONFIG_H
#define CONTROLLER_CONFIG_H

#include "cppTypes.h"
#include "RobotController.h"

using StateWeightMat = Eigen::Matrix<double, 13, 13>;
using InputWeightMat = Mat12<double>;

enum class LocomotionMode {
    Walking,
    Standing,
};

enum class TouchdownTargetMode {
    BodyVelocityHalfStance,
    LegacyComYawCorrected,
};

struct TimingParameters {
    double cycle{1.0};
    double swing{0.4};
    double stance{0.6};
    double horizon{0.5};
    int horizonSteps{15};
};

struct ModelParameters {
    double gravity{9.81};
};

struct MPCParameters {
    double frictionCoefficient{0.1};
    double footHalfLength{0.065};
    double footHalfWidth{0.01};
    double torsionalFrictionScale{0.0657};
    double normalForceMax{200.0};
    double normalForceMin{10.0};
    StateWeightMat stateWeight = StateWeightMat::Identity();
    InputWeightMat inputWeight = InputWeightMat::Identity();
    int iterationsBetweenSolve{10};
};

struct SwingParameters {
    Vec3<double> naturalFrequency = Vec3<double>(10.0, 10.0, 10.0);
    Vec3<double> kdDiag = Vec3<double>(15.0, 15.0, 18.0);
    double height{0.06};
    double minRemainingTime{1e-3};
    TouchdownTargetMode touchdownTargetMode{TouchdownTargetMode::BodyVelocityHalfStance};
    FootEndEffectorSource footEndEffectorSource{FootEndEffectorSource::Site};
};

struct FootPlacementParameters {
    double velocityFeedbackGain{0.03};
    double placementClamp{0.3};
    double touchdownHeight{-0.003};
    double nominalLateralOffset{0.065};
    double swingBias{0.0};
};

struct LoggingParameters {
    int gaitStatusInterval{50};
};

struct InitialPoseParameters {
    std::vector<double> legJointOffsets{0.0, 0.0, -0.65, 0.80, -0.35};
    std::vector<double> armJointOffsets{0.0, 0.0, 0.0, -0.65};
};

struct LeftSwingHoldTestParameters {
    TouchdownTargetMode touchdownTargetMode{TouchdownTargetMode::LegacyComYawCorrected};
};

struct ControllerConfig {
    LocomotionMode locomotionMode{LocomotionMode::Walking};
    TimingParameters timing;
    ModelParameters model;
    MPCParameters mpc;
    SwingParameters swing;
    FootPlacementParameters footPlacement;
    LoggingParameters logging;
    InitialPoseParameters initialPose;
    LeftSwingHoldTestParameters leftSwingHoldTest;
};

const ControllerConfig& getControllerConfig();
double cycleTime();
double swingTime();
double stanceTime();
double horizonTime();
int horizonSteps();
double dtMpc();

const DMat<double>& getL();
const DMat<double>& getK();
LocomotionMode locomotionMode();

#endif  // CONTROLLER_CONFIG_H
