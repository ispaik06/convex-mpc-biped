#include "ControllerConfig.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace {
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
        return FootEndEffectorSource::Site;
    }

    const std::string mode = node.as<std::string>();
    if (mode == "site") {
        return FootEndEffectorSource::Site;
    }
    if (mode == "body_com" || mode == "com") {
        return FootEndEffectorSource::BodyCom;
    }

    throw std::runtime_error(
        "Invalid swing.foot_end_effector_source. Expected site or body_com");
}

ControllerConfig loadControllerConfigFromYaml() {
    ControllerConfig params;

    const std::string configPath = std::string(PROJECT_ROOT_DIR) + "/config/my_controller.yaml";

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
    readScalarIfPresent(model, "gravity", params.model.gravity);

    const YAML::Node mpc = config["mpc"];
    readScalarIfPresent(mpc, "friction_coefficient", params.mpc.frictionCoefficient);
    readScalarIfPresent(mpc, "foot_half_length", params.mpc.footHalfLength);
    readScalarIfPresent(mpc, "foot_half_width", params.mpc.footHalfWidth);
    readScalarIfPresent(mpc, "torsional_friction_scale", params.mpc.torsionalFrictionScale);
    readScalarIfPresent(mpc, "normal_force_max", params.mpc.normalForceMax);
    readScalarIfPresent(mpc, "normal_force_min", params.mpc.normalForceMin);
    readScalarIfPresent(mpc, "iterations_between_solve", params.mpc.iterationsBetweenSolve);
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
    params.swing.touchdownTargetMode = parseTouchdownTargetMode(swing["touchdown_target_mode"]);
    params.swing.footEndEffectorSource = parseFootEndEffectorSource(swing["foot_end_effector_source"]);

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

    const YAML::Node initialPose = config["initial_pose"];
    readVectorIfPresent(initialPose, "leg_joint_offsets", params.initialPose.legJointOffsets);
    readVectorIfPresent(initialPose, "arm_joint_offsets", params.initialPose.armJointOffsets);

    const YAML::Node leftSwingHoldTest = config["left_swing_hold_test"];
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

    return params;
}
}  // namespace

const ControllerConfig& getControllerConfig() {
    static const ControllerConfig params = loadControllerConfigFromYaml();
    return params;
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
    static const DMat<double> L = [] {
        const int steps = horizonSteps();
        DMat<double> out = DMat<double>::Zero(13 * steps, 13 * steps);
        for (int k = 0; k < steps; ++k) {
            out.block(13 * k, 13 * k, 13, 13) = getControllerConfig().mpc.stateWeight;
        }
        return out;
    }();

    return L;
}

const DMat<double>& getK() {
    static const DMat<double> K = [] {
        const int steps = horizonSteps();
        DMat<double> out = DMat<double>::Zero(12 * steps, 12 * steps);
        for (int k = 0; k < steps; ++k) {
            out.block(12 * k, 12 * k, 12, 12) = getControllerConfig().mpc.inputWeight;
        }
        return out;
    }();

    return K;
}

LocomotionMode locomotionMode() {
    return getControllerConfig().locomotionMode;
}
