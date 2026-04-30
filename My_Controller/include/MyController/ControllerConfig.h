#ifndef CONTROLLER_CONFIG_H
#define CONTROLLER_CONFIG_H

#include <string>
#include <vector>

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

enum class ContactWrenchModel {
    FullWrench,
    NoRollMoment,
};

struct TimingParameters {
    double cycle{1.0};
    double swing{0.4};
    double stance{0.6};
    double horizon{0.5};
    int horizonSteps{15};
};

struct ModelParameters {
    std::string xmlPath;
    std::string auxiliaryXmlPath;
    FootEndEffectorSource footEndEffectorSource{FootEndEffectorSource::Site};
    double gravity{-9.81};
};

struct MPCParameters {
    double frictionCoefficient{0.1};
    double footHalfLength{0.065};
    double footHalfWidth{0.01};
    double torsionalFrictionScale{0.0657};
    double normalForceMax{200.0};
    double normalForceMin{10.0};
    StateWeightMat walkingStateWeight = StateWeightMat::Identity();
    InputWeightMat walkingInputWeight = InputWeightMat::Identity();
    StateWeightMat standingStateWeight = StateWeightMat::Identity();
    InputWeightMat standingInputWeight = InputWeightMat::Identity();
    int iterationsBetweenSolve{10};
    ContactWrenchModel contactWrenchModel{ContactWrenchModel::FullWrench};
};

struct SwingParameters {
    Vec3<double> naturalFrequency = Vec3<double>(10.0, 10.0, 10.0);
    Vec3<double> kdDiag = Vec3<double>(15.0, 15.0, 18.0);
    double height{0.06};
    double minRemainingTime{1e-3};
    double bodyVelocityHalfStanceOffset{0.0};
    double pitchKp{0.0};
    double pitchKd{0.0};
    double yawKp{0.0};
    double yawKd{0.0};
    TouchdownTargetMode touchdownTargetMode{TouchdownTargetMode::BodyVelocityHalfStance};
};

struct FootPlacementParameters {
    double velocityFeedbackGain{0.03};
    double placementClamp{0.3};
    double touchdownHeight{-0.003};
    double nominalLateralOffset{0.065};
    double swingBias{0.0};
};

struct ContactManagerParameters {
    double contactForceOnThreshold{20.0};
    double contactForceOffThreshold{5.0};
    int contactOnConfirmTicks{2};
    int contactOffConfirmTicks{2};
    double contactRampDuration{0.08};
    int contactLockSteps{3};
    double lateContactTimeout{0.20};
    double groundSearchVelocity{0.08};
    double groundSearchMaxDepth{0.04};
    bool enableEarlyContactHandling{true};
    bool enableLateContactHandling{true};
};

struct LoggingParameters {
    int gaitStatusInterval{50};
    std::vector<double> standingMpcDebugTriggerTimes;
};

struct StartupParameters {
    double postInitStandingSettleTime{2.0};
};

struct InitialPoseParameters {
    std::vector<double> legJointOffsets{0.0, 0.0, -0.65, 0.80, -0.35};
    std::vector<double> armJointOffsets{0.0, 0.0, 0.0, -0.65};
    double legInitializationTime{2.0};
    double armInitializationTime{1.0};
    // Optional floating-base pose seed copied from the scene keyframe.
    bool hasBasePose{false};
    Vec3<double> basePosition_W = Vec3<double>::Zero();
    Vec3<double> baseEuler_W = Vec3<double>::Zero();
};

struct GaitSwingHoldTestParameters {
    std::string xmlPath;
    TouchdownTargetMode touchdownTargetMode{TouchdownTargetMode::LegacyComYawCorrected};
};

struct ControllerConfig {
    LocomotionMode locomotionMode{LocomotionMode::Walking};
    TimingParameters timing;
    ModelParameters model;
    MPCParameters mpc;
    SwingParameters swing;
    FootPlacementParameters footPlacement;
    ContactManagerParameters contactManager;
    LoggingParameters logging;
    StartupParameters startup;
    InitialPoseParameters initialPose;
    GaitSwingHoldTestParameters gaitSwingHoldTest;
};

const ControllerConfig& getControllerConfig();
const ControllerConfig& getControllerConfig(RobotType robotType);
void setControllerConfigOverride(const ControllerConfig* config);
void clearControllerConfigOverride();
double cycleTime();
double swingTime();
double stanceTime();
double horizonTime();
int horizonSteps();
double dtMpc();

const DMat<double>& getL();
const DMat<double>& getK();
const DMat<double>& getL(LocomotionMode mode);
const DMat<double>& getK(LocomotionMode mode);
LocomotionMode locomotionMode();
std::string contactWrenchModelName(ContactWrenchModel model);

#endif  // CONTROLLER_CONFIG_H
