#include "ControllerConfig.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "RobotConfig.h"

namespace {
thread_local const ControllerConfig* g_controllerConfigOverride = nullptr;

template <typename T>
void readScalarIfPresent(const YAML::Node& node, const char* key, T& value) {
    if (node && node[key]) {
        value = node[key].as<T>();
    }
}

template <typename T>
void readVectorIfPresent(const YAML::Node& node, const char* key, std::vector<T>& values) {
    if (!node || !node[key]) {
        return;
    }

    const YAML::Node valuesNode = node[key];
    if (!valuesNode.IsSequence()) {
        throw std::runtime_error(std::string("Expected YAML sequence for key ") + key);
    }

    values.clear();
    values.reserve(valuesNode.size());
    for (std::size_t idx = 0; idx < valuesNode.size(); ++idx) {
        values.push_back(valuesNode[idx].as<T>());
    }
}

bool isPositiveFinite(const double value) {
    return value > 0.0 && std::isfinite(value);
}

bool isNonNegativeFiniteOrPositiveInfinity(const double value) {
    return (std::isfinite(value) && value >= 0.0) ||
           (std::isinf(value) && value > 0.0);
}

double clampWithOptionalLimit(const double value, const double maxAbs) {
    if (!std::isfinite(maxAbs)) {
        return value;
    }
    return std::clamp(value, -maxAbs, maxAbs);
}

template <typename Derived>
void fillDiagonal(Eigen::MatrixBase<Derived>& matrix,
                  const YAML::Node& diagonalNode,
                  const char* keyName) {
    if (!diagonalNode || !diagonalNode.IsSequence()) {
        throw std::runtime_error(std::string("Expected YAML sequence for key ") + keyName);
    }
    if (diagonalNode.size() != static_cast<std::size_t>(matrix.rows())) {
        throw std::runtime_error(std::string("YAML diagonal length does not match expected size for ")
                                 + keyName);
    }

    matrix.derived().setZero();
    for (Eigen::Index i = 0; i < matrix.rows(); ++i) {
        matrix(i, i) = diagonalNode[static_cast<std::size_t>(i)].as<double>();
    }
}

void readModeWeights(const YAML::Node& node,
                     const char* modeName,
                     StateWeightMat& stateWeight,
                     InputWeightMat& inputWeight) {
    if (!node || !node.IsMap()) {
        throw std::runtime_error(std::string("Missing controller_config.mpc.") + modeName);
    }

    if (node["state_weight_diag"]) {
        fillDiagonal(stateWeight,
                      node["state_weight_diag"],
                      (std::string("mpc.") + modeName + ".state_weight_diag").c_str());
    }
    if (node["input_weight_diag"]) {
        fillDiagonal(inputWeight,
                      node["input_weight_diag"],
                      (std::string("mpc.") + modeName + ".input_weight_diag").c_str());
    }
}

Vec3<double> readVec3(const YAML::Node& node, const char* keyName) {
    if (!node || !node.IsSequence() || node.size() != 3) {
        throw std::runtime_error(std::string("Expected 3-element YAML sequence for key ") + keyName);
    }

    return Vec3<double>(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
}

std::vector<Vec3<double>> readVec3Vector(const YAML::Node& node, const char* keyName) {
    if (!node || !node.IsSequence()) {
        throw std::runtime_error(std::string("Expected YAML sequence for key ") + keyName);
    }

    std::vector<Vec3<double>> values;
    values.reserve(node.size());
    for (std::size_t idx = 0; idx < node.size(); ++idx) {
        values.push_back(readVec3(node[idx], keyName));
    }
    return values;
}

LocomotionMode parseLocomotionMode(const YAML::Node& node, const char* keyName) {
    if (!node || !node.IsScalar()) {
        return LocomotionMode::Walking;
    }

    const std::string mode = node.as<std::string>();
    if (mode == "walking" || mode == "walk") {
        return LocomotionMode::Walking;
    }
    if (mode == "standing" || mode == "stand") {
        return LocomotionMode::Standing;
    }

    throw std::runtime_error(std::string("Invalid ") + keyName +
                             ". Expected one of: walking, walk, standing, stand");
}

FootEndEffectorSource parseFootEndEffectorSource(const YAML::Node& node) {
    if (!node || !node.IsScalar()) {
        throw std::runtime_error(
            "Missing or invalid model.foot_end_effector_source. Expected site or collision_geom_center");
    }

    const std::string mode = node.as<std::string>();
    if (mode == "site") {
        return FootEndEffectorSource::Site;
    }
    if (mode == "collision_geom_center") {
        return FootEndEffectorSource::CollisionGeomCenter;
    }

    throw std::runtime_error(
        "Invalid model.foot_end_effector_source. Expected site or collision_geom_center");
}

ContactWrenchModel parseContactWrenchModel(const YAML::Node& node,
                                           const char* keyName,
                                           const ContactWrenchModel defaultModel =
                                               ContactWrenchModel::FullWrench) {
    if (!node || !node.IsScalar()) {
        return defaultModel;
    }

    const std::string model = node.as<std::string>();
    if (model == "full_wrench" || model == "model1") {
        return ContactWrenchModel::FullWrench;
    }
    if (model == "no_roll_moment" || model == "model2") {
        return ContactWrenchModel::NoRollMoment;
    }

    throw std::runtime_error(std::string("Invalid ") + keyName +
                             ". Expected full_wrench or no_roll_moment");
}

TouchdownTargetUpdateMode parseTouchdownTargetUpdateMode(const YAML::Node& node) {
    if (!node || !node.IsScalar()) {
        return TouchdownTargetUpdateMode::Fixed;
    }

    const std::string mode = node.as<std::string>();
    if (mode == "fixed") {
        return TouchdownTargetUpdateMode::Fixed;
    }
    if (mode == "realtime" || mode == "real_time") {
        return TouchdownTargetUpdateMode::Realtime;
    }

    throw std::runtime_error(
        "Invalid swing.touchdown_target_update_mode. Expected fixed or realtime");
}

ControllerConfig loadControllerConfigFromYaml(const RobotType robotType) {
    ControllerConfig params;

    const std::string configPath = robotConfigPath(robotType);

    YAML::Node config;
    try {
        config = YAML::LoadFile(configPath);
    } catch (const YAML::Exception& exception) {
        throw std::runtime_error("Failed to load controller config YAML from " + configPath
                                 + ": " + exception.what());
    }

    const YAML::Node requestedLocomotionMode =
        config["requested_locomotion_mode"] ? config["requested_locomotion_mode"]
                                            : config["locomotion_mode"];
    params.requestedLocomotionMode =
        parseLocomotionMode(requestedLocomotionMode, "requested_locomotion_mode");

    const YAML::Node timing = config["timing"];
    readScalarIfPresent(timing, "cycle", params.timing.cycle);
    readScalarIfPresent(timing, "swing", params.timing.swing);
    readScalarIfPresent(timing, "stance", params.timing.stance);
    readScalarIfPresent(timing, "horizon", params.timing.horizon);
    readScalarIfPresent(timing, "horizon_steps", params.timing.horizonSteps);

    const YAML::Node model = config["model"];
    readScalarIfPresent(model, "xml_path", params.model.xmlPath);
    readScalarIfPresent(model, "auxiliary_xml_path", params.model.auxiliaryXmlPath);
    params.model.footEndEffectorSource =
        parseFootEndEffectorSource(model["foot_end_effector_source"]);
    readScalarIfPresent(model, "gravity", params.model.gravity);

    const YAML::Node mpc = config["mpc"];
    readScalarIfPresent(mpc, "friction_coefficient", params.mpc.frictionCoefficient);
    readScalarIfPresent(mpc, "foot_half_length", params.mpc.footHalfLength);
    readScalarIfPresent(mpc, "foot_half_width", params.mpc.footHalfWidth);
    readScalarIfPresent(mpc, "torsional_friction_scale", params.mpc.torsionalFrictionScale);
    readScalarIfPresent(mpc, "normal_force_max", params.mpc.normalForceMax);
    readScalarIfPresent(mpc, "normal_force_min", params.mpc.normalForceMin);
    readScalarIfPresent(mpc, "iterations_between_solve", params.mpc.iterationsBetweenSolve);
    const ContactWrenchModel defaultContactWrenchModel =
        parseContactWrenchModel(mpc["contact_wrench_model"], "mpc.contact_wrench_model");
    params.mpc.walkingContactWrenchModel =
        parseContactWrenchModel(mpc["walking"]["contact_wrench_model"],
                                "mpc.walking.contact_wrench_model",
                                defaultContactWrenchModel);
    params.mpc.standingContactWrenchModel =
        parseContactWrenchModel(mpc["standing"]["contact_wrench_model"],
                                "mpc.standing.contact_wrench_model",
                                defaultContactWrenchModel);
    readModeWeights(mpc["walking"],
                    "walking",
                    params.mpc.walkingStateWeight,
                    params.mpc.walkingInputWeight);
    readModeWeights(mpc["standing"],
                    "standing",
                    params.mpc.standingStateWeight,
                    params.mpc.standingInputWeight);

    const YAML::Node swing = config["swing"];
    if (swing && swing["natural_frequency"]) {
        params.swing.naturalFrequency =
            readVec3(swing["natural_frequency"], "swing.natural_frequency");
    }
    if (swing && swing["kd_diag"]) {
        params.swing.kdDiag = readVec3(swing["kd_diag"], "swing.kd_diag");
    }
    readScalarIfPresent(swing, "height", params.swing.height);
    readScalarIfPresent(swing, "min_remaining_time", params.swing.minRemainingTime);
    readScalarIfPresent(swing,
                        "body_velocity_half_stance_offset",
                        params.swing.bodyVelocityHalfStanceOffset);
    if (swing && swing["swing_foot_yaw_lead_scale"]) {
        params.swing.swingFootYawLeadScale =
            swing["swing_foot_yaw_lead_scale"].as<double>();
    } else {
        readScalarIfPresent(swing,
                            "touchdown_yaw_lead_scale",
                            params.swing.swingFootYawLeadScale);
    }
    if (swing && swing["nominal_foot_offsets_B"]) {
        params.swing.nominalFootOffsets_B =
            readVec3Vector(swing["nominal_foot_offsets_B"], "swing.nominal_foot_offsets_B");
    }
    if (swing && swing["stop_braking_offset_B"]) {
        params.swing.hasStopBrakingOffset = true;
        params.swing.stopBrakingOffset_B =
            readVec3(swing["stop_braking_offset_B"], "swing.stop_braking_offset_B");
    }
    params.swing.touchdownTargetUpdateMode =
        parseTouchdownTargetUpdateMode(swing ? swing["touchdown_target_update_mode"] : YAML::Node{});
    readScalarIfPresent(swing, "stop_capture_point_gain", params.swing.stopCapturePointGain);
    readScalarIfPresent(swing,
                        "stop_capture_point_max_offset",
                        params.swing.stopCapturePointMaxOffset);
    readScalarIfPresent(swing, "stop_velocity_deadband", params.swing.stopVelocityDeadband);
    readScalarIfPresent(swing, "pitch_kp", params.swing.pitchKp);
    readScalarIfPresent(swing, "pitch_kd", params.swing.pitchKd);
    readScalarIfPresent(swing, "yaw_kp", params.swing.yawKp);
    readScalarIfPresent(swing, "yaw_kd", params.swing.yawKd);

    const YAML::Node userCommandFilter = config["user_command_filter"];
    readScalarIfPresent(userCommandFilter, "x_dot_tau", params.userCommandFilter.xDotTau);
    readScalarIfPresent(userCommandFilter, "y_dot_tau", params.userCommandFilter.yDotTau);
    readScalarIfPresent(userCommandFilter, "psi_dot_tau", params.userCommandFilter.psiDotTau);
    readScalarIfPresent(userCommandFilter, "z_dot_tau", params.userCommandFilter.zDotTau);
    readScalarIfPresent(userCommandFilter,
                        "standing_roll_offset_tau",
                        params.userCommandFilter.standingRollOffsetTau);
    readScalarIfPresent(userCommandFilter,
                        "standing_pitch_offset_tau",
                        params.userCommandFilter.standingPitchOffsetTau);
    readScalarIfPresent(userCommandFilter, "x_dot_max", params.userCommandFilter.xDotMax);
    readScalarIfPresent(userCommandFilter, "y_dot_max", params.userCommandFilter.yDotMax);
    readScalarIfPresent(userCommandFilter, "psi_dot_max", params.userCommandFilter.psiDotMax);

    const YAML::Node contactManager = config["contact_manager"];
    readScalarIfPresent(contactManager,
                        "contact_force_on_threshold",
                        params.contactManager.contactForceOnThreshold);
    readScalarIfPresent(contactManager,
                        "contact_force_off_threshold",
                        params.contactManager.contactForceOffThreshold);
    readScalarIfPresent(contactManager,
                        "contact_on_confirm_ticks",
                        params.contactManager.contactOnConfirmTicks);
    readScalarIfPresent(contactManager,
                        "contact_off_confirm_ticks",
                        params.contactManager.contactOffConfirmTicks);
    readScalarIfPresent(contactManager,
                        "contact_ramp_duration",
                        params.contactManager.contactRampDuration);
    readScalarIfPresent(contactManager,
                        "contact_lock_steps",
                        params.contactManager.contactLockSteps);
    readScalarIfPresent(contactManager,
                        "late_contact_timeout",
                        params.contactManager.lateContactTimeout);
    readScalarIfPresent(contactManager,
                        "ground_search_velocity",
                        params.contactManager.groundSearchVelocity);
    readScalarIfPresent(contactManager,
                        "ground_search_max_depth",
                        params.contactManager.groundSearchMaxDepth);
    readScalarIfPresent(contactManager,
                        "ground_search_tracking_time",
                        params.contactManager.groundSearchTrackingTime);
    readScalarIfPresent(contactManager,
                        "stance_contact_loss_foot_height",
                        params.contactManager.stanceContactLossFootHeight);
    readScalarIfPresent(contactManager,
                        "enable_early_contact_handling",
                        params.contactManager.enableEarlyContactHandling);
    readScalarIfPresent(contactManager,
                        "enable_late_contact_handling",
                        params.contactManager.enableLateContactHandling);

    const YAML::Node logging = config["logging"];
    readVectorIfPresent(logging,
                        "standing_mpc_debug_trigger_times",
                        params.logging.standingMpcDebugTriggerTimes);
    std::sort(params.logging.standingMpcDebugTriggerTimes.begin(),
              params.logging.standingMpcDebugTriggerTimes.end());

    const YAML::Node startup = config["startup"];
    readScalarIfPresent(startup,
                        "post_init_standing_settle_time",
                        params.startup.postInitStandingSettleTime);
    if (startup && !startup["post_init_standing_settle_time"]) {
        readScalarIfPresent(startup,
                            "standing_settle_time",
                            params.startup.postInitStandingSettleTime);
    }

    const YAML::Node initialPose = config["initial_pose"];
    readVectorIfPresent(initialPose, "leg_joint_offsets", params.initialPose.legJointOffsets);
    readVectorIfPresent(initialPose, "arm_joint_offsets", params.initialPose.armJointOffsets);
    const bool hasBasePosition = initialPose && initialPose["base_position_W"];
    const bool hasBaseEuler = initialPose && initialPose["base_rpy_W"];
    if (hasBasePosition != hasBaseEuler) {
        throw std::runtime_error(
            "initial_pose.base_position_W and initial_pose.base_rpy_W must be provided together");
    }
    if (hasBasePosition) {
        params.initialPose.basePosition_W = readVec3(initialPose["base_position_W"],
                                                     "initial_pose.base_position_W");
        params.initialPose.baseEuler_W =
            readVec3(initialPose["base_rpy_W"], "initial_pose.base_rpy_W");
        params.initialPose.hasBasePose = true;
    }
    readScalarIfPresent(initialPose,
                        "leg_initialization_time",
                        params.initialPose.legInitializationTime);
    readScalarIfPresent(initialPose,
                        "arm_initialization_time",
                        params.initialPose.armInitializationTime);
    if (!isPositiveFinite(params.initialPose.legInitializationTime) ||
        !isPositiveFinite(params.initialPose.armInitializationTime)) {
        throw std::runtime_error("initial_pose initialization times must be finite and positive");
    }
    if (params.startup.postInitStandingSettleTime < 0.0 ||
        !std::isfinite(params.startup.postInitStandingSettleTime)) {
        throw std::runtime_error(
            "startup.post_init_standing_settle_time must be finite and non-negative");
    }

    const YAML::Node gaitSwingHoldTest = config["gait_swing_hold_test"];
    readScalarIfPresent(gaitSwingHoldTest,
                        "xml_path",
                        params.gaitSwingHoldTest.xmlPath);

    if (params.timing.cycle <= 0.0 || params.timing.swing <= 0.0 || params.timing.stance <= 0.0 ||
        params.timing.horizon <= 0.0 || params.timing.horizonSteps <= 0) {
        throw std::runtime_error("Controller timing parameters must be positive");
    }
    if (std::abs((params.timing.swing + params.timing.stance) - params.timing.cycle) > 1e-9) {
        throw std::runtime_error("Controller timing must satisfy cycle = swing + stance");
    }
    if (params.mpc.normalForceMax < params.mpc.normalForceMin) {
        throw std::runtime_error("MPC normal force max must be >= min");
    }
    if (!std::isfinite(params.swing.bodyVelocityHalfStanceOffset) ||
        !std::isfinite(params.swing.swingFootYawLeadScale) ||
        (!params.swing.nominalFootOffsets_B.empty() &&
         !std::all_of(params.swing.nominalFootOffsets_B.begin(),
                      params.swing.nominalFootOffsets_B.end(),
                      [](const Vec3<double>& offset) { return offset.allFinite(); })) ||
        (params.swing.hasStopBrakingOffset && !params.swing.stopBrakingOffset_B.allFinite()) ||
        !std::isfinite(params.swing.stopCapturePointGain) ||
        !std::isfinite(params.swing.stopCapturePointMaxOffset) ||
        !std::isfinite(params.swing.stopVelocityDeadband) ||
        !std::isfinite(params.userCommandFilter.xDotTau) ||
        !std::isfinite(params.userCommandFilter.yDotTau) ||
        !std::isfinite(params.userCommandFilter.psiDotTau) ||
        !std::isfinite(params.userCommandFilter.zDotTau) ||
        !std::isfinite(params.userCommandFilter.standingRollOffsetTau) ||
        !std::isfinite(params.userCommandFilter.standingPitchOffsetTau) ||
        !isNonNegativeFiniteOrPositiveInfinity(params.userCommandFilter.xDotMax) ||
        !isNonNegativeFiniteOrPositiveInfinity(params.userCommandFilter.yDotMax) ||
        !isNonNegativeFiniteOrPositiveInfinity(params.userCommandFilter.psiDotMax) ||
        !std::isfinite(params.swing.pitchKp) || !std::isfinite(params.swing.pitchKd) ||
        !std::isfinite(params.swing.yawKp) || !std::isfinite(params.swing.yawKd) ||
        params.swing.bodyVelocityHalfStanceOffset < 0.0 ||
        params.swing.swingFootYawLeadScale < 0.0 ||
        params.swing.stopCapturePointGain < 0.0 ||
        params.swing.stopCapturePointMaxOffset < 0.0 ||
        params.swing.stopVelocityDeadband < 0.0 ||
        params.userCommandFilter.xDotTau < 0.0 ||
        params.userCommandFilter.yDotTau < 0.0 ||
        params.userCommandFilter.psiDotTau < 0.0 ||
        params.userCommandFilter.zDotTau < 0.0 ||
        params.userCommandFilter.standingRollOffsetTau < 0.0 ||
        params.userCommandFilter.standingPitchOffsetTau < 0.0 ||
        params.swing.pitchKp < 0.0 || params.swing.pitchKd < 0.0 ||
        params.swing.yawKp < 0.0 || params.swing.yawKd < 0.0) {
        throw std::runtime_error(
            "swing.body_velocity_half_stance_offset, swing.swing_foot_yaw_lead_scale, "
            "swing.nominal_foot_offsets_B, swing.stop_braking_offset_B, "
            "swing.stop_capture_point_gain, swing.stop_capture_point_max_offset, "
            "swing.stop_velocity_deadband, "
            "user_command_filter.x_dot_tau, "
            "user_command_filter.y_dot_tau, user_command_filter.psi_dot_tau, "
            "user_command_filter.z_dot_tau, user_command_filter.standing_roll_offset_tau, "
            "user_command_filter.standing_pitch_offset_tau must be finite; "
            "user_command_filter.x_dot_max, user_command_filter.y_dot_max, and "
            "user_command_filter.psi_dot_max must be finite or positive infinity; "
            "offsets, time constants, and "
            "stop-braking gains must be non-negative and attitude gains must be "
            "non-negative");
    }
    if (!std::isfinite(params.contactManager.contactForceOnThreshold) ||
        !std::isfinite(params.contactManager.contactForceOffThreshold) ||
        !std::isfinite(params.contactManager.contactRampDuration) ||
        !std::isfinite(params.contactManager.lateContactTimeout) ||
        !std::isfinite(params.contactManager.groundSearchVelocity) ||
        !std::isfinite(params.contactManager.groundSearchMaxDepth) ||
        !std::isfinite(params.contactManager.groundSearchTrackingTime) ||
        !std::isfinite(params.contactManager.stanceContactLossFootHeight) ||
        params.contactManager.contactForceOnThreshold < 0.0 ||
        params.contactManager.contactForceOffThreshold < 0.0 ||
        params.contactManager.contactForceOnThreshold < params.contactManager.contactForceOffThreshold ||
        params.contactManager.contactOnConfirmTicks <= 0 ||
        params.contactManager.contactOffConfirmTicks <= 0 ||
        params.contactManager.contactRampDuration < 0.0 ||
        params.contactManager.contactLockSteps < 0 ||
        params.contactManager.lateContactTimeout < 0.0 ||
        params.contactManager.groundSearchVelocity < 0.0 ||
        params.contactManager.groundSearchMaxDepth < 0.0 ||
        params.contactManager.groundSearchTrackingTime <= 0.0 ||
        params.contactManager.stanceContactLossFootHeight < 0.0) {
        throw std::runtime_error(
            "contact_manager thresholds, ticks, ramp, lock steps, timeout, and ground-search parameters are invalid");
    }

    return params;
}
}  // namespace

void setControllerConfigOverride(const ControllerConfig* config) {
    g_controllerConfigOverride = config;
}

void clearControllerConfigOverride() {
    g_controllerConfigOverride = nullptr;
}

const ControllerConfig& getControllerConfig() {
    return getControllerConfig(activeRobotType());
}

const ControllerConfig& getControllerConfig(const RobotType robotType) {
    if (g_controllerConfigOverride != nullptr) {
        return *g_controllerConfigOverride;
    }

    static std::map<RobotType, ControllerConfig> paramsByRobot;
    auto it = paramsByRobot.find(robotType);
    if (it == paramsByRobot.end()) {
        it = paramsByRobot.emplace(robotType, loadControllerConfigFromYaml(robotType)).first;
    }
    return it->second;
}

double cycleTime() {
    return getControllerConfig().timing.cycle;
}

double swingTime() {
    return getControllerConfig().timing.swing;
}

double stanceTime() {
    return getControllerConfig().timing.stance;
}

double horizonTime() {
    return getControllerConfig().timing.horizon;
}

int horizonSteps() {
    return getControllerConfig().timing.horizonSteps;
}

double dtMpc() {
    return horizonTime() / static_cast<double>(horizonSteps());
}

UserCommand clampUserCommand(const UserCommand& command) {
    const auto& filter = getControllerConfig().userCommandFilter;
    UserCommand clamped = command;
    clamped.x_dot = clampWithOptionalLimit(clamped.x_dot, filter.xDotMax);
    clamped.y_dot = clampWithOptionalLimit(clamped.y_dot, filter.yDotMax);
    clamped.psi_dot = clampWithOptionalLimit(clamped.psi_dot, filter.psiDotMax);
    return clamped;
}

namespace {
const StateWeightMat& stateWeightForMode(const MPCParameters& mpc, const LocomotionMode mode) {
    return mode == LocomotionMode::Standing ? mpc.standingStateWeight : mpc.walkingStateWeight;
}

const InputWeightMat& inputWeightForMode(const MPCParameters& mpc, const LocomotionMode mode) {
    return mode == LocomotionMode::Standing ? mpc.standingInputWeight : mpc.walkingInputWeight;
}
}  // namespace

const DMat<double>& getL(const LocomotionMode mode) {
    static std::map<RobotType, std::map<LocomotionMode, DMat<double>>> cache;
    const RobotType robotType = activeRobotType();
    auto& robotCache = cache[robotType];
    auto it = robotCache.find(mode);
    if (it == robotCache.end()) {
        const auto& config = getControllerConfig(robotType);
        const int steps = config.timing.horizonSteps;
        DMat<double> out = DMat<double>::Zero(13 * steps, 13 * steps);
        const StateWeightMat& weight = stateWeightForMode(config.mpc, mode);
        for (int k = 0; k < steps; ++k) {
            out.block(13 * k, 13 * k, 13, 13) = weight;
        }
        it = robotCache.emplace(mode, std::move(out)).first;
    }

    return it->second;
}

const DMat<double>& getK(const LocomotionMode mode) {
    static std::map<RobotType, std::map<LocomotionMode, DMat<double>>> cache;
    const RobotType robotType = activeRobotType();
    auto& robotCache = cache[robotType];
    auto it = robotCache.find(mode);
    if (it == robotCache.end()) {
        const auto& config = getControllerConfig(robotType);
        const int steps = config.timing.horizonSteps;
        DMat<double> out = DMat<double>::Zero(12 * steps, 12 * steps);
        const InputWeightMat& weight = inputWeightForMode(config.mpc, mode);
        for (int k = 0; k < steps; ++k) {
            out.block(12 * k, 12 * k, 12, 12) = weight;
        }
        it = robotCache.emplace(mode, std::move(out)).first;
    }

    return it->second;
}

const DMat<double>& getL() {
    return getL(requestedLocomotionMode());
}

const DMat<double>& getK() {
    return getK(requestedLocomotionMode());
}

LocomotionMode requestedLocomotionMode() {
    return getControllerConfig().requestedLocomotionMode;
}

ContactWrenchModel contactWrenchModelForMode(const MPCParameters& mpc,
                                             const LocomotionMode mode) {
    return mode == LocomotionMode::Standing
               ? mpc.standingContactWrenchModel
               : mpc.walkingContactWrenchModel;
}

ContactWrenchModel contactWrenchModel(const LocomotionMode mode) {
    return contactWrenchModelForMode(getControllerConfig().mpc, mode);
}

std::string contactWrenchModelName(const ContactWrenchModel model) {
    switch (model) {
        case ContactWrenchModel::FullWrench:
            return "full_wrench";
        case ContactWrenchModel::NoRollMoment:
            return "no_roll_moment";
    }
    return "unknown";
}

std::string touchdownTargetUpdateModeName(const TouchdownTargetUpdateMode mode) {
    switch (mode) {
        case TouchdownTargetUpdateMode::Fixed:
            return "fixed";
        case TouchdownTargetUpdateMode::Realtime:
            return "realtime";
    }
    return "unknown";
}
