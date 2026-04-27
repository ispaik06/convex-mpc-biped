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

Vec3<double> readVec3(const YAML::Node& node, const char* keyName) {
    if (!node || !node.IsSequence() || node.size() != 3) {
        throw std::runtime_error(std::string("Expected 3-element YAML sequence for key ") + keyName);
    }

    return Vec3<double>(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
}

LocomotionMode parseLocomotionMode(const YAML::Node& node) {
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

    throw std::runtime_error(
        "Invalid locomotion_mode. Expected one of: walking, walk, standing, stand");
}

TouchdownTargetMode parseTouchdownTargetMode(const YAML::Node& node) {
    if (!node || !node.IsScalar()) {
        return TouchdownTargetMode::BodyVelocityHalfStance;
    }

    const std::string mode = node.as<std::string>();
    if (mode == "body_velocity_half_stance") {
        return TouchdownTargetMode::BodyVelocityHalfStance;
    }
    if (mode == "legacy_com_yaw_corrected") {
        return TouchdownTargetMode::LegacyComYawCorrected;
    }

    throw std::runtime_error(
        "Invalid swing.touchdown_target_mode. Expected body_velocity_half_stance or legacy_com_yaw_corrected");
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

ContactWrenchModel parseContactWrenchModel(const YAML::Node& node) {
    if (!node || !node.IsScalar()) {
        return ContactWrenchModel::FullWrench;
    }

    const std::string model = node.as<std::string>();
    if (model == "full_wrench" || model == "model1") {
        return ContactWrenchModel::FullWrench;
    }
    if (model == "no_roll_moment" || model == "model2") {
        return ContactWrenchModel::NoRollMoment;
    }

    throw std::runtime_error(
        "Invalid mpc.contact_wrench_model. Expected full_wrench or no_roll_moment");
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

    params.locomotionMode = parseLocomotionMode(config["locomotion_mode"]);

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
    params.mpc.contactWrenchModel = parseContactWrenchModel(mpc["contact_wrench_model"]);
    if (mpc && mpc["state_weight_diag"]) {
        fillDiagonal(params.mpc.stateWeight, mpc["state_weight_diag"], "mpc.state_weight_diag");
    }
    if (mpc && mpc["input_weight_diag"]) {
        fillDiagonal(params.mpc.inputWeight, mpc["input_weight_diag"], "mpc.input_weight_diag");
    }

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
    readScalarIfPresent(swing, "pitch_kp", params.swing.pitchKp);
    readScalarIfPresent(swing, "pitch_kd", params.swing.pitchKd);
    params.swing.touchdownTargetMode = parseTouchdownTargetMode(swing["touchdown_target_mode"]);

    const YAML::Node footPlacement = config["foot_placement"];
    readScalarIfPresent(footPlacement,
                        "velocity_feedback_gain",
                        params.footPlacement.velocityFeedbackGain);
    readScalarIfPresent(footPlacement,
                        "placement_clamp",
                        params.footPlacement.placementClamp);
    readScalarIfPresent(footPlacement,
                        "touchdown_height",
                        params.footPlacement.touchdownHeight);
    readScalarIfPresent(footPlacement,
                        "nominal_lateral_offset",
                        params.footPlacement.nominalLateralOffset);
    readScalarIfPresent(footPlacement, "swing_bias", params.footPlacement.swingBias);

    const YAML::Node logging = config["logging"];
    readScalarIfPresent(logging, "gait_status_interval", params.logging.gaitStatusInterval);
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

    const YAML::Node leftSwingHoldTest = config["left_swing_hold_test"];
    readScalarIfPresent(leftSwingHoldTest,
                        "xml_path",
                        params.leftSwingHoldTest.xmlPath);
    params.leftSwingHoldTest.touchdownTargetMode =
        parseTouchdownTargetMode(leftSwingHoldTest["touchdown_target_mode"]);

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
        !std::isfinite(params.swing.pitchKp) || !std::isfinite(params.swing.pitchKd) ||
        params.swing.pitchKp < 0.0 || params.swing.pitchKd < 0.0) {
        throw std::runtime_error(
            "swing.body_velocity_half_stance_offset, swing.pitch_kp, and swing.pitch_kd must be finite; pitch gains must be non-negative");
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

const DMat<double>& getL() {
    static std::map<RobotType, DMat<double>> cache;
    const RobotType robotType = activeRobotType();
    auto it = cache.find(robotType);
    if (it == cache.end()) {
        const auto& config = getControllerConfig(robotType);
        const int steps = config.timing.horizonSteps;
        DMat<double> out = DMat<double>::Zero(13 * steps, 13 * steps);
        for (int k = 0; k < steps; ++k) {
            out.block(13 * k, 13 * k, 13, 13) = config.mpc.stateWeight;
        }
        it = cache.emplace(robotType, std::move(out)).first;
    }

    return it->second;
}

const DMat<double>& getK() {
    static std::map<RobotType, DMat<double>> cache;
    const RobotType robotType = activeRobotType();
    auto it = cache.find(robotType);
    if (it == cache.end()) {
        const auto& config = getControllerConfig(robotType);
        const int steps = config.timing.horizonSteps;
        DMat<double> out = DMat<double>::Zero(12 * steps, 12 * steps);
        for (int k = 0; k < steps; ++k) {
            out.block(12 * k, 12 * k, 12, 12) = config.mpc.inputWeight;
        }
        it = cache.emplace(robotType, std::move(out)).first;
    }

    return it->second;
}

LocomotionMode locomotionMode() {
    return getControllerConfig().locomotionMode;
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
