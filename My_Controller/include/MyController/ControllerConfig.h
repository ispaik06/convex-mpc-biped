#ifndef CONTROLLER_CONFIG_H
#define CONTROLLER_CONFIG_H

#include <limits>
#include <string>
#include <vector>

#include "cppTypes.h"
#include "RobotController.h"

using StateWeightMat = Eigen::Matrix<double, 13, 13>;
using InputWeightMat = Mat12<double>;

enum class LocomotionMode {
    Walking,
    Standing,
    Interactive,
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
    ContactWrenchModel walkingContactWrenchModel{ContactWrenchModel::FullWrench};
    ContactWrenchModel standingContactWrenchModel{ContactWrenchModel::FullWrench};
};

struct SwingParameters {
    Vec3<double> naturalFrequency = Vec3<double>(10.0, 10.0, 10.0);
    Vec3<double> kdDiag = Vec3<double>(15.0, 15.0, 18.0);
    double height{0.06};
    double minRemainingTime{1e-3};
    double bodyVelocityHalfStanceOffset{0.0};
    double swingFootYawLeadScale{1.0};
    double turnTangentialLeadScale{0.0};
    bool enableStanceFootYawHold{false};
    std::vector<Vec3<double>> nominalFootOffsets_B;
    bool hasStopBrakingOffset{false};
    Vec3<double> stopBrakingOffset_B = Vec3<double>::Zero();
    double stopCapturePointGain{1.0};
    double stopCapturePointMaxOffset{0.20};
    double stopVelocityDeadband{0.02};
    int stopBrakingLatchClearTicks{5};
    double rollKp{0.0};
    double rollKd{0.0};
    double pitchKp{0.0};
    double pitchKd{0.0};
    double yawKp{0.0};
    double yawKd{0.0};
    double stanceYawKp{0.0};
    double stanceYawKd{0.0};
};

struct UserCommandFilterParameters {
    double xDotTau{0.0};
    double yDotTau{0.0};
    double psiDotTau{0.0};
    double standingRollOffsetTau{0.0};
    double standingPitchOffsetTau{0.0};
    double xDotMax{std::numeric_limits<double>::infinity()};
    double yDotMax{std::numeric_limits<double>::infinity()};
    double psiDotMax{std::numeric_limits<double>::infinity()};
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
    double groundSearchTrackingTime{0.08};
    double stanceContactLossFootHeight{0.025};
    bool enableEarlyContactHandling{true};
    bool enableLateContactHandling{true};
};

struct LoggingParameters {
    std::vector<double> standingMpcDebugTriggerTimes;
};

struct StartupParameters {
    double postInitStandingSettleTime{2.0};
};

struct TransitionParameters {
    double brakingSettleSpeedThreshold{0.05};
    int brakingSettleHoldTicks{3};
    double brakingTimeoutSeconds{2.0};
    int brakingTouchdownCount{2};
};

struct RecoveryStepParameters {
    bool enabled{false};
    double commandVelocityDeadband{0.05};
    double capturePointDeadband{0.03};
    double capturePointFilterTau{0.15};
    double yawRateDeadband{0.0};
    double yawRateGain{0.0};
    double yawRateMax{0.0};
    double stepGain{0.7};
    double maxForwardStep{0.10};
    double maxBackwardStep{0.08};
    double maxLateralStep{0.08};
    double bodyOffsetGain{0.4};
    double bodyOffsetMax{0.06};
    double bodyOffsetTau{0.08};
    double velocityGain{0.0};
    double velocityMax{0.0};
    double maxActiveTime{1.0};
    double cooldownTime{0.5};
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
    std::string keyframeName{"copied_state"};
};

struct ControllerConfig {
    LocomotionMode requestedLocomotionMode{LocomotionMode::Walking};
    TimingParameters timing;
    ModelParameters model;
    MPCParameters mpc;
    SwingParameters swing;
    UserCommandFilterParameters userCommandFilter;
    ContactManagerParameters contactManager;
    LoggingParameters logging;
    StartupParameters startup;
    TransitionParameters transition;
    RecoveryStepParameters recoveryStep;
    InitialPoseParameters initialPose;
    GaitSwingHoldTestParameters gaitSwingHoldTest;
};

const ControllerConfig& getControllerConfig();
const ControllerConfig& getControllerConfig(RobotType robotType);
void setControllerConfigOverride(const ControllerConfig* config);
void clearControllerConfigOverride();
UserCommand clampUserCommand(const UserCommand& command);
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
LocomotionMode requestedLocomotionMode();
std::string contactWrenchModelName(ContactWrenchModel model);
ContactWrenchModel contactWrenchModelForMode(const MPCParameters& mpc, LocomotionMode mode);
ContactWrenchModel contactWrenchModel(LocomotionMode mode);

#endif  // CONTROLLER_CONFIG_H
