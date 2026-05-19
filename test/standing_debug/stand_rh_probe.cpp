#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "ConvexMPC.h"
#include "GaitScheduler.h"
#include "HorizonClock.h"
#include "MPCFormulation.h"
#include "ReferenceTrajectory.h"
#include "ControllerConfig.h"
#include "Robot/RobotParams.h"
#include "RobotConfig.h"
#include "Utilities/MatrixUtils.h"
#include "Utilities/UserCommand.h"

namespace {
using json = nlohmann::ordered_json;

constexpr int kStateDim = 13;
constexpr int kInputDim = 12;
constexpr int kDefaultRolloutSteps = 60;

const std::array<const char*, kStateDim> kStateNames = {
    "roll", "pitch", "yaw",
    "px", "py", "pz",
    "omega_x", "omega_y", "omega_z",
    "vx", "vy", "vz",
    "g",
};

const std::array<const char*, kInputDim> kInputNames = {
    "L_Fx", "L_Fy", "L_Fz",
    "R_Fx", "R_Fy", "R_Fz",
    "L_Mx", "L_My", "L_Mz",
    "R_Mx", "R_My", "R_Mz",
};

struct OutputPaths {
    std::filesystem::path report;
    std::filesystem::path csv;
    std::filesystem::path plotsDir;
    std::filesystem::path statesPlot;
    std::filesystem::path wrenchPlot;
    std::filesystem::path metricsPlot;
};

struct Options {
    std::filesystem::path logPath;
    int steps{kDefaultRolloutSteps};
};

struct RolloutRow {
    int step{0};
    double time{0.0};
    double simTime{0.0};
    bool solveOk{false};
    Vec13<double> state = Vec13<double>::Zero();
    Vec13<double> reference = Vec13<double>::Zero();
    Vec13<double> error = Vec13<double>::Zero();
    Vec13<double> nextState = Vec13<double>::Zero();
    Vec12<double> firstWrench = Vec12<double>::Zero();
    double errorNormRpy{0.0};
    double errorNormPosition{0.0};
    double errorNormOmega{0.0};
    double errorNormVelocity{0.0};
    double errorNormAll{0.0};
    double weightedHorizonErrorNorm{0.0};
    double inputNorm{0.0};
    double weightedInputNorm{0.0};
};

struct RolloutResult {
    std::vector<RolloutRow> rows;
    Vec13<double> fixedReference = Vec13<double>::Zero();
    DesiredFootPositions desiredFootPositions;
    Vec3<double> leftFootXAxis_W = Vec3<double>::UnitX();
    Vec3<double> rightFootXAxis_W = Vec3<double>::UnitX();
    double leftTouchdownYaw_W{std::numeric_limits<double>::quiet_NaN()};
    double rightTouchdownYaw_W{std::numeric_limits<double>::quiet_NaN()};
    UserCommand userCommand;
    LocomotionMode locomotionMode{LocomotionMode::Standing};
    double sourceControllerTime{0.0};
    double sourceClockT0{0.0};
    int sourceHorizonSteps{0};
    double sourceDtMpc{0.0};
    std::optional<double> firstWrenchDeltaToLog;
    std::optional<double> firstHorizonWrenchDeltaToLog;
    std::optional<double> firstHorizonStateDeltaToLog;
    std::string failureMessage;
};

bool isMpcDebugLogFilename(const std::string& name) {
    const bool standingLog = name.rfind("standing_mpc_debug_", 0) == 0;
    const bool walkingLog = name.rfind("walking_mpc_debug_", 0) == 0;
    return (standingLog || walkingLog) &&
           std::filesystem::path(name).extension() == ".json";
}

std::string locomotionModeName(const LocomotionMode mode) {
    switch (mode) {
        case LocomotionMode::Walking:
            return "walking";
        case LocomotionMode::Standing:
            return "standing";
        case LocomotionMode::Interactive:
            return "interactive";
    }
    return "unknown";
}

std::string shortLocomotionPrefix(const LocomotionMode mode) {
    switch (mode) {
        case LocomotionMode::Walking:
            return "walk";
        case LocomotionMode::Standing:
        case LocomotionMode::Interactive:
            return "stand";
    }
    return "unknown";
}

std::string debugDirectoryNameForMode(const LocomotionMode mode) {
    switch (mode) {
        case LocomotionMode::Walking:
            return "walking_mpc";
        case LocomotionMode::Standing:
        case LocomotionMode::Interactive:
            return "standing_mpc";
    }
    return "standing_mpc";
}

std::vector<std::filesystem::path> mpcDebugLogDirectories() {
    const std::filesystem::path debugRoot =
        std::filesystem::path(PROJECT_ROOT_DIR) / "logs" / "debug";
    return {
        debugRoot / "standing_mpc",
        debugRoot / "walking_mpc",
    };
}

std::string timestampToken() {
    using clock = std::chrono::system_clock;
    const std::time_t time = clock::to_time_t(clock::now());

    std::tm localTime {};
    localtime_r(&time, &localTime);

    std::ostringstream out;
    out << std::put_time(&localTime, "%Y%m%d_%H%M%S");
    return out.str();
}

OutputPaths defaultOutputPaths(const LocomotionMode locomotionMode) {
    const std::string timestamp = timestampToken();
    const std::string prefix = shortLocomotionPrefix(locomotionMode) + "_rh_";
    const std::filesystem::path dir =
        std::filesystem::path(PROJECT_ROOT_DIR) / "logs" / "debug" /
        debugDirectoryNameForMode(locomotionMode) / "receding_horizon";
    const std::filesystem::path reportDir = dir / "reports";
    const std::filesystem::path csvDir = dir / "csv";
    const std::filesystem::path plotDir = dir / "plots";
    const std::filesystem::path plotRunDir = plotDir / (prefix + timestamp);
    std::filesystem::create_directories(reportDir);
    std::filesystem::create_directories(csvDir);
    std::filesystem::create_directories(plotRunDir);
    std::filesystem::create_directories(plotDir);

    OutputPaths paths;
    paths.report = reportDir / (prefix + timestamp + ".md");
    paths.csv = csvDir / (prefix + timestamp + ".csv");
    paths.plotsDir = plotRunDir;
    paths.statesPlot = plotRunDir / "states.png";
    paths.wrenchPlot = plotRunDir / "wrench.png";
    paths.metricsPlot = plotRunDir / "metrics.png";
    return paths;
}

std::filesystem::path latestDebugLogPath() {
    std::filesystem::path best;
    std::filesystem::file_time_type bestWriteTime{};
    bool hasBest = false;
    for (const std::filesystem::path& dir : mpcDebugLogDirectories()) {
        if (!std::filesystem::exists(dir)) {
            continue;
        }
        for (const auto& entry : std::filesystem::directory_iterator(dir)) {
            if (!entry.is_regular_file()) {
                continue;
            }

            const std::string name = entry.path().filename().string();
            if (!isMpcDebugLogFilename(name)) {
                continue;
            }

            const std::filesystem::file_time_type writeTime =
                std::filesystem::last_write_time(entry.path());
            if (!hasBest || writeTime > bestWriteTime) {
                best = entry.path();
                bestWriteTime = writeTime;
                hasBest = true;
            }
        }
    }

    if (best.empty()) {
        throw std::runtime_error(
            "No MPC debug JSON file found under logs/debug/standing_mpc or logs/debug/walking_mpc");
    }
    return best;
}

std::string joinNames(const std::array<const char*, kStateDim>& names) {
    std::ostringstream out;
    for (std::size_t i = 0; i < names.size(); ++i) {
        if (i > 0) {
            out << ',';
        }
        out << names[i];
    }
    return out.str();
}

std::string joinInputNames() {
    std::ostringstream out;
    for (std::size_t i = 0; i < kInputNames.size(); ++i) {
        if (i > 0) {
            out << ',';
        }
        out << kInputNames[i];
    }
    return out.str();
}

std::vector<double> readJsonVector(const json& value, const std::string& name) {
    const json* source = &value;
    if (value.is_object() && value.contains("data")) {
        source = &value.at("data");
    }
    if (!source->is_array()) {
        throw std::runtime_error(name + " is not a JSON array");
    }

    std::vector<double> out;
    if (!source->empty() && source->front().is_array()) {
        for (const auto& row : *source) {
            if (!row.is_array()) {
                throw std::runtime_error(name + " has mixed array nesting");
            }
            for (const auto& item : row) {
                out.push_back(item.get<double>());
            }
        }
        return out;
    }

    out.reserve(source->size());
    for (const auto& item : *source) {
        out.push_back(item.get<double>());
    }
    return out;
}

Vec3<double> vec3FromJson(const json& value, const std::string& name);

std::vector<Vec3<double>> readVec3VectorFromJson(const json& value, const std::string& name) {
    if (!value.is_array()) {
        throw std::runtime_error(name + " is not a JSON array");
    }

    std::vector<Vec3<double>> out;
    out.reserve(value.size());
    for (std::size_t i = 0; i < value.size(); ++i) {
        out.push_back(vec3FromJson(value.at(i), name + "[" + std::to_string(i) + "]"));
    }
    return out;
}

DMat<double> readJsonMatrix(const json& value, const std::string& name) {
    if (!value.is_object() || !value.contains("rows") ||
        !value.contains("cols") || !value.contains("data")) {
        throw std::runtime_error(name + " is not a matrix object");
    }

    const int rows = value.at("rows").get<int>();
    const int cols = value.at("cols").get<int>();
    const json& data = value.at("data");
    if (rows <= 0 || cols <= 0 || !data.is_array()) {
        throw std::runtime_error(name + " has invalid matrix metadata");
    }

    DMat<double> out(rows, cols);
    if (!data.empty() && data.front().is_array()) {
        if (static_cast<int>(data.size()) != rows) {
            throw std::runtime_error(name + " row count does not match metadata");
        }
        for (int row = 0; row < rows; ++row) {
            if (!data[static_cast<std::size_t>(row)].is_array() ||
                static_cast<int>(data[static_cast<std::size_t>(row)].size()) != cols) {
                throw std::runtime_error(name + " column count does not match metadata");
            }
            for (int col = 0; col < cols; ++col) {
                out(row, col) =
                    data[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)].get<double>();
            }
        }
        return out;
    }

    if (static_cast<int>(data.size()) != rows * cols) {
        throw std::runtime_error(name + " flat data length does not match rows*cols");
    }
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            out(row, col) = data[static_cast<std::size_t>(row * cols + col)].get<double>();
        }
    }
    return out;
}

RobotType robotTypeFromString(const std::string& value) {
    if (value == "mit_humanoid") {
        return RobotType::MIT_HUMANOID;
    }
    if (value == "unitree_g1") {
        return RobotType::UNITREE_G1;
    }
    if (value == "unitree_h1") {
        return RobotType::UNITREE_H1;
    }
    throw std::runtime_error("Invalid controller_config.robot_type: " + value);
}

std::string robotTypeLabelFromLog(const json& log) {
    if (log.contains("metadata") && log.at("metadata").is_object() &&
        log.at("metadata").contains("robot_type") &&
        log.at("metadata").at("robot_type").is_string()) {
        return log.at("metadata").at("robot_type").get<std::string>();
    }
    if (log.contains("controller_config") && log.at("controller_config").is_object() &&
        log.at("controller_config").contains("robot_type") &&
        log.at("controller_config").at("robot_type").is_string()) {
        return log.at("controller_config").at("robot_type").get<std::string>();
    }
    return "unknown";
}

LocomotionMode locomotionModeFromString(const std::string& value) {
    if (value == "walking" || value == "walk") {
        return LocomotionMode::Walking;
    }
    if (value == "standing" || value == "stand") {
        return LocomotionMode::Standing;
    }
    if (value == "interactive" || value == "general") {
        return LocomotionMode::Interactive;
    }
    throw std::runtime_error("Invalid locomotion mode: " + value);
}

LocomotionMode locomotionModeFromStateString(const std::string& value) {
    if (value == "walking") {
        return LocomotionMode::Walking;
    }
    if (value == "standing" || value == "standing_settle") {
        return LocomotionMode::Standing;
    }
    throw std::runtime_error("Invalid metadata.locomotion_state: " + value);
}

FootEndEffectorSource footEndEffectorSourceFromString(const std::string& value) {
    if (value == "site") {
        return FootEndEffectorSource::Site;
    }
    if (value == "collision_geom_center") {
        return FootEndEffectorSource::CollisionGeomCenter;
    }
    throw std::runtime_error("Invalid controller_config.model.foot_end_effector_source: " + value);
}

ContactWrenchModel contactWrenchModelFromString(const std::string& value) {
    if (value == "full_wrench" || value == "model1") {
        return ContactWrenchModel::FullWrench;
    }
    if (value == "no_roll_moment" || value == "model2") {
        return ContactWrenchModel::NoRollMoment;
    }
    throw std::runtime_error("Invalid controller_config.mpc.contact_wrench_model: " + value);
}

YawIntegrationMode yawIntegrationModeFromString(const std::string& value) {
    if (value == "single_support" || value == "single" || value == "stance_single") {
        return YawIntegrationMode::SingleSupport;
    }
    if (value == "double_support" || value == "double" || value == "both_feet") {
        return YawIntegrationMode::DoubleSupport;
    }
    if (value == "always" || value == "continuous") {
        return YawIntegrationMode::Always;
    }
    throw std::runtime_error(
        "Invalid controller_config.reference_trajectory.yaw_integration_mode: " + value);
}

Vec3<double> vec3FromJson(const json& value, const std::string& name);
std::vector<Vec3<double>> readVec3VectorFromJson(const json& value, const std::string& name);

std::optional<ControllerConfig> controllerConfigFromLog(const json& log) {
    if (!log.contains("controller_config") || !log.at("controller_config").is_object()) {
        return std::nullopt;
    }

    const json& cfg = log.at("controller_config");
    ControllerConfig out;

    if (cfg.contains("robot_type") && cfg.at("robot_type").is_string()) {
        const RobotType robotType = robotTypeFromString(cfg.at("robot_type").get<std::string>());
        setActiveRobotType(robotType);
    }

    if (cfg.contains("requested_locomotion_mode") &&
        cfg.at("requested_locomotion_mode").is_string()) {
        out.requestedLocomotionMode = locomotionModeFromString(
            cfg.at("requested_locomotion_mode").get<std::string>());
    } else if (cfg.contains("locomotion_mode") && cfg.at("locomotion_mode").is_string()) {
        out.requestedLocomotionMode =
            locomotionModeFromString(cfg.at("locomotion_mode").get<std::string>());
    }

    if (cfg.contains("timing") && cfg.at("timing").is_object()) {
        const json& timing = cfg.at("timing");
        if (timing.contains("cycle")) {
            out.timing.cycle = timing.at("cycle").get<double>();
        }
        if (timing.contains("swing")) {
            out.timing.swing = timing.at("swing").get<double>();
        }
        if (timing.contains("stance")) {
            out.timing.stance = timing.at("stance").get<double>();
        }
        if (timing.contains("horizon")) {
            out.timing.horizon = timing.at("horizon").get<double>();
        }
        if (timing.contains("horizon_steps")) {
            out.timing.horizonSteps = timing.at("horizon_steps").get<int>();
        }
    }

    if (cfg.contains("model") && cfg.at("model").is_object()) {
        const json& model = cfg.at("model");
        if (model.contains("xml_path") && model.at("xml_path").is_string()) {
            out.model.xmlPath = model.at("xml_path").get<std::string>();
        }
        if (model.contains("auxiliary_xml_path") && model.at("auxiliary_xml_path").is_string()) {
            out.model.auxiliaryXmlPath = model.at("auxiliary_xml_path").get<std::string>();
        }
        if (model.contains("foot_end_effector_source") &&
            model.at("foot_end_effector_source").is_string()) {
            out.model.footEndEffectorSource =
                footEndEffectorSourceFromString(
                    model.at("foot_end_effector_source").get<std::string>());
        }
        if (model.contains("gravity")) {
            out.model.gravity = model.at("gravity").get<double>();
        }
    }

    if (cfg.contains("mpc") && cfg.at("mpc").is_object()) {
        const json& mpc = cfg.at("mpc");
        if (mpc.contains("friction_coefficient")) {
            out.mpc.frictionCoefficient = mpc.at("friction_coefficient").get<double>();
        }
        if (mpc.contains("foot_half_length")) {
            out.mpc.footHalfLength = mpc.at("foot_half_length").get<double>();
        }
        if (mpc.contains("foot_half_width")) {
            out.mpc.footHalfWidth = mpc.at("foot_half_width").get<double>();
        }
        if (mpc.contains("torsional_friction_scale")) {
            out.mpc.torsionalFrictionScale = mpc.at("torsional_friction_scale").get<double>();
        }
        if (mpc.contains("normal_force_max")) {
            out.mpc.normalForceMax = mpc.at("normal_force_max").get<double>();
        }
        if (mpc.contains("normal_force_min")) {
            out.mpc.normalForceMin = mpc.at("normal_force_min").get<double>();
        }
        if (mpc.contains("use_shifted_warm_start")) {
            out.mpc.useShiftedWarmStart = mpc.at("use_shifted_warm_start").get<bool>();
        }
        if (mpc.contains("iterations_between_solve")) {
            out.mpc.iterationsBetweenSolve = mpc.at("iterations_between_solve").get<int>();
        }
        ContactWrenchModel defaultContactWrenchModel = ContactWrenchModel::FullWrench;
        if (mpc.contains("contact_wrench_model") && mpc.at("contact_wrench_model").is_string()) {
            defaultContactWrenchModel =
                contactWrenchModelFromString(mpc.at("contact_wrench_model").get<std::string>());
        }
        out.mpc.walkingContactWrenchModel = defaultContactWrenchModel;
        out.mpc.standingContactWrenchModel = defaultContactWrenchModel;

        auto readModeWeights = [&](const json& modeNode,
                                   StateWeightMat& stateWeight,
                                   InputWeightMat& inputWeight,
                                   ContactWrenchModel& contactWrenchModel,
                                   const std::string& modeName) {
            if (!modeNode.is_object()) {
                return false;
            }
            if (modeNode.contains("contact_wrench_model") &&
                modeNode.at("contact_wrench_model").is_string()) {
                contactWrenchModel =
                    contactWrenchModelFromString(
                        modeNode.at("contact_wrench_model").get<std::string>());
            }
            if (modeNode.contains("state_weight_diag")) {
                const std::vector<double> diag = readJsonVector(
                    modeNode.at("state_weight_diag"),
                    "controller_config.mpc." + modeName + ".state_weight_diag");
                if (diag.size() != 13) {
                    throw std::runtime_error("controller_config.mpc." + modeName +
                                             ".state_weight_diag must have size 13");
                }
                stateWeight.setZero();
                for (int i = 0; i < 13; ++i) {
                    stateWeight(i, i) = diag[static_cast<std::size_t>(i)];
                }
            }
            if (modeNode.contains("input_weight_diag")) {
                const std::vector<double> diag = readJsonVector(
                    modeNode.at("input_weight_diag"),
                    "controller_config.mpc." + modeName + ".input_weight_diag");
                if (diag.size() != 12) {
                    throw std::runtime_error("controller_config.mpc." + modeName +
                                             ".input_weight_diag must have size 12");
                }
                inputWeight.setZero();
                for (int i = 0; i < 12; ++i) {
                    inputWeight(i, i) = diag[static_cast<std::size_t>(i)];
                }
            }
            return true;
        };

        const bool hasWalking = mpc.contains("walking") &&
                                readModeWeights(mpc.at("walking"),
                                                out.mpc.walkingStateWeight,
                                                out.mpc.walkingInputWeight,
                                                out.mpc.walkingContactWrenchModel,
                                                "walking");
        const bool hasStanding = mpc.contains("standing") &&
                                 readModeWeights(mpc.at("standing"),
                                                 out.mpc.standingStateWeight,
                                                 out.mpc.standingInputWeight,
                                                 out.mpc.standingContactWrenchModel,
                                                 "standing");

        if (!hasWalking && !hasStanding) {
            if (mpc.contains("state_weight_diag") || mpc.contains("input_weight_diag")) {
                readModeWeights(mpc,
                                out.mpc.walkingStateWeight,
                                out.mpc.walkingInputWeight,
                                out.mpc.walkingContactWrenchModel,
                                "walking");
                out.mpc.standingStateWeight = out.mpc.walkingStateWeight;
                out.mpc.standingInputWeight = out.mpc.walkingInputWeight;
                out.mpc.standingContactWrenchModel = out.mpc.walkingContactWrenchModel;
            }
        }
    }

    if (cfg.contains("swing") && cfg.at("swing").is_object()) {
        const json& swing = cfg.at("swing");
        if (swing.contains("natural_frequency")) {
            out.swing.naturalFrequency =
                vec3FromJson(swing.at("natural_frequency"),
                             "controller_config.swing.natural_frequency");
        }
        if (swing.contains("kd_diag")) {
            out.swing.kdDiag = vec3FromJson(swing.at("kd_diag"), "controller_config.swing.kd_diag");
        }
        if (swing.contains("height")) {
            out.swing.height = swing.at("height").get<double>();
        }
        if (swing.contains("min_remaining_time")) {
            out.swing.minRemainingTime = swing.at("min_remaining_time").get<double>();
        }
        if (swing.contains("body_velocity_half_stance_offset")) {
            out.swing.bodyVelocityHalfStanceOffset =
                swing.at("body_velocity_half_stance_offset").get<double>();
        }
        if (swing.contains("swing_foot_yaw_lead_scale")) {
            out.swing.swingFootYawLeadScale =
                swing.at("swing_foot_yaw_lead_scale").get<double>();
        } else if (swing.contains("touchdown_yaw_lead_scale")) {
            out.swing.swingFootYawLeadScale =
                swing.at("touchdown_yaw_lead_scale").get<double>();
        }
        if (swing.contains("turn_tangential_lead_scale")) {
            out.swing.turnTangentialLeadScale =
                swing.at("turn_tangential_lead_scale").get<double>();
        }
        if (swing.contains("enable_stance_foot_yaw_hold")) {
            out.swing.enableStanceFootYawHold =
                swing.at("enable_stance_foot_yaw_hold").get<bool>();
        }
        if (swing.contains("nominal_foot_offsets_B")) {
            out.swing.nominalFootOffsets_B =
                readVec3VectorFromJson(swing.at("nominal_foot_offsets_B"),
                                       "controller_config.swing.nominal_foot_offsets_B");
        }
        if (swing.contains("stop_braking_offset_B")) {
            out.swing.hasStopBrakingOffset = true;
            out.swing.stopBrakingOffset_B =
                vec3FromJson(swing.at("stop_braking_offset_B"),
                             "controller_config.swing.stop_braking_offset_B");
        }
        if (swing.contains("stop_capture_point_gain")) {
            out.swing.stopCapturePointGain =
                swing.at("stop_capture_point_gain").get<double>();
        }
        if (swing.contains("stop_capture_point_max_offset")) {
            out.swing.stopCapturePointMaxOffset =
                swing.at("stop_capture_point_max_offset").get<double>();
        }
        if (swing.contains("stop_velocity_deadband")) {
            out.swing.stopVelocityDeadband =
                swing.at("stop_velocity_deadband").get<double>();
        }
        if (swing.contains("stop_braking_latch_clear_ticks")) {
            out.swing.stopBrakingLatchClearTicks =
                swing.at("stop_braking_latch_clear_ticks").get<int>();
        }
        if (swing.contains("roll_kp")) {
            out.swing.rollKp = swing.at("roll_kp").get<double>();
        }
        if (swing.contains("roll_kd")) {
            out.swing.rollKd = swing.at("roll_kd").get<double>();
        }
        if (swing.contains("pitch_kp")) {
            out.swing.pitchKp = swing.at("pitch_kp").get<double>();
        }
        if (swing.contains("pitch_kd")) {
            out.swing.pitchKd = swing.at("pitch_kd").get<double>();
        }
        if (swing.contains("yaw_kp")) {
            out.swing.yawKp = swing.at("yaw_kp").get<double>();
        }
        if (swing.contains("yaw_kd")) {
            out.swing.yawKd = swing.at("yaw_kd").get<double>();
        }
        if (swing.contains("stance_yaw_kp")) {
            out.swing.stanceYawKp = swing.at("stance_yaw_kp").get<double>();
        }
        if (swing.contains("stance_yaw_kd")) {
            out.swing.stanceYawKd = swing.at("stance_yaw_kd").get<double>();
        }
    }

    if (cfg.contains("user_command_filter") && cfg.at("user_command_filter").is_object()) {
        const json& userCommandFilter = cfg.at("user_command_filter");
        if (userCommandFilter.contains("x_dot_tau")) {
            out.userCommandFilter.xDotTau = userCommandFilter.at("x_dot_tau").get<double>();
        }
        if (userCommandFilter.contains("y_dot_tau")) {
            out.userCommandFilter.yDotTau = userCommandFilter.at("y_dot_tau").get<double>();
        }
        if (userCommandFilter.contains("psi_dot_tau")) {
            out.userCommandFilter.psiDotTau = userCommandFilter.at("psi_dot_tau").get<double>();
        }
        if (userCommandFilter.contains("standing_roll_offset_tau")) {
            out.userCommandFilter.standingRollOffsetTau =
                userCommandFilter.at("standing_roll_offset_tau").get<double>();
        }
        if (userCommandFilter.contains("standing_pitch_offset_tau")) {
            out.userCommandFilter.standingPitchOffsetTau =
                userCommandFilter.at("standing_pitch_offset_tau").get<double>();
        }
        if (userCommandFilter.contains("x_dot_max") && userCommandFilter.at("x_dot_max").is_number()) {
            out.userCommandFilter.xDotMax = userCommandFilter.at("x_dot_max").get<double>();
        }
        if (userCommandFilter.contains("y_dot_max") && userCommandFilter.at("y_dot_max").is_number()) {
            out.userCommandFilter.yDotMax = userCommandFilter.at("y_dot_max").get<double>();
        }
        if (userCommandFilter.contains("psi_dot_max") &&
            userCommandFilter.at("psi_dot_max").is_number()) {
            out.userCommandFilter.psiDotMax = userCommandFilter.at("psi_dot_max").get<double>();
        }
    }

    if (cfg.contains("reference_trajectory") && cfg.at("reference_trajectory").is_object()) {
        const json& referenceTrajectory = cfg.at("reference_trajectory");
        if (referenceTrajectory.contains("yaw_integration_mode") &&
            referenceTrajectory.at("yaw_integration_mode").is_string()) {
            out.referenceTrajectory.yawIntegrationMode =
                yawIntegrationModeFromString(
                    referenceTrajectory.at("yaw_integration_mode").get<std::string>());
        }
    }

    if (cfg.contains("logging") && cfg.at("logging").is_object()) {
        const json& logging = cfg.at("logging");
        if (logging.contains("standing_mpc_debug_trigger_times")) {
            out.logging.standingMpcDebugTriggerTimes =
                readJsonVector(logging.at("standing_mpc_debug_trigger_times"),
                               "controller_config.logging.standing_mpc_debug_trigger_times");
        }
        std::sort(out.logging.standingMpcDebugTriggerTimes.begin(),
                  out.logging.standingMpcDebugTriggerTimes.end());
    }

    if (cfg.contains("locomotion_transition") && cfg.at("locomotion_transition").is_object()) {
        const json& transition = cfg.at("locomotion_transition");
        if (transition.contains("braking_settle_speed_threshold") &&
            transition.at("braking_settle_speed_threshold").is_number()) {
            out.transition.brakingSettleSpeedThreshold =
                transition.at("braking_settle_speed_threshold").get<double>();
        }
        if (transition.contains("braking_settle_yaw_rate_threshold") &&
            transition.at("braking_settle_yaw_rate_threshold").is_number()) {
            out.transition.brakingSettleYawRateThreshold =
                transition.at("braking_settle_yaw_rate_threshold").get<double>();
        }
        if (transition.contains("braking_settle_average_window") &&
            transition.at("braking_settle_average_window").is_number()) {
            out.transition.brakingSettleAverageWindow =
                transition.at("braking_settle_average_window").get<double>();
        }
        if (transition.contains("braking_settle_hold_ticks") &&
            transition.at("braking_settle_hold_ticks").is_number_integer()) {
            out.transition.brakingSettleHoldTicks =
                transition.at("braking_settle_hold_ticks").get<int>();
        }
        if (transition.contains("braking_timeout_seconds") &&
            transition.at("braking_timeout_seconds").is_number()) {
            out.transition.brakingTimeoutSeconds =
                transition.at("braking_timeout_seconds").get<double>();
        }
        if (transition.contains("braking_touchdown_count") &&
            transition.at("braking_touchdown_count").is_number_integer()) {
            out.transition.brakingTouchdownCount =
                transition.at("braking_touchdown_count").get<int>();
        }
    }

    if (cfg.contains("initial_pose") && cfg.at("initial_pose").is_object()) {
        const json& initialPose = cfg.at("initial_pose");
        const bool hasBasePosition = initialPose.contains("base_position_W");
        const bool hasBaseEuler = initialPose.contains("base_rpy_W");
        if (hasBasePosition != hasBaseEuler) {
            throw std::runtime_error(
                "controller_config.initial_pose.base_position_W and "
                "controller_config.initial_pose.base_rpy_W must be provided together");
        }
        if (hasBasePosition) {
            out.initialPose.hasBasePose = true;
            out.initialPose.basePosition_W =
                vec3FromJson(initialPose.at("base_position_W"),
                             "controller_config.initial_pose.base_position_W");
            out.initialPose.baseEuler_W =
                vec3FromJson(initialPose.at("base_rpy_W"),
                             "controller_config.initial_pose.base_rpy_W");
        }
        if (initialPose.contains("leg_joint_offsets")) {
            out.initialPose.legJointOffsets =
                readJsonVector(initialPose.at("leg_joint_offsets"),
                               "controller_config.initial_pose.leg_joint_offsets");
        }
        if (initialPose.contains("arm_joint_offsets")) {
            out.initialPose.armJointOffsets =
                readJsonVector(initialPose.at("arm_joint_offsets"),
                               "controller_config.initial_pose.arm_joint_offsets");
        }
        if (initialPose.contains("leg_initialization_time")) {
            out.initialPose.legInitializationTime =
                initialPose.at("leg_initialization_time").get<double>();
        }
        if (initialPose.contains("arm_initialization_time")) {
            out.initialPose.armInitializationTime =
                initialPose.at("arm_initialization_time").get<double>();
        }
    }

    if (cfg.contains("gait_swing_hold_test") && cfg.at("gait_swing_hold_test").is_object()) {
        const json& gaitSwingHoldTest = cfg.at("gait_swing_hold_test");
        if (gaitSwingHoldTest.contains("xml_path") && gaitSwingHoldTest.at("xml_path").is_string()) {
            out.gaitSwingHoldTest.xmlPath = gaitSwingHoldTest.at("xml_path").get<std::string>();
        }
        if (gaitSwingHoldTest.contains("keyframe_name") &&
            gaitSwingHoldTest.at("keyframe_name").is_string()) {
            out.gaitSwingHoldTest.keyframeName =
                gaitSwingHoldTest.at("keyframe_name").get<std::string>();
        }
    }

    return out;
}

Vec13<double> vec13FromJson(const json& value, const std::string& name) {
    const std::vector<double> raw = readJsonVector(value, name);
    if (static_cast<int>(raw.size()) != kStateDim) {
        throw std::runtime_error(name + " has size " + std::to_string(raw.size()) +
                                 ", expected 13");
    }

    Vec13<double> out = Vec13<double>::Zero();
    for (int i = 0; i < kStateDim; ++i) {
        out[i] = raw[static_cast<std::size_t>(i)];
    }
    return out;
}

Vec3<double> vec3FromJson(const json& value, const std::string& name) {
    const std::vector<double> raw = readJsonVector(value, name);
    if (raw.size() != 3) {
        throw std::runtime_error(name + " must have size 3");
    }

    return Vec3<double>(
        raw[0],
        raw[1],
        raw[2]);
}

Mat3<double> mat3FromJson(const json& value, const std::string& name) {
    const DMat<double> raw = readJsonMatrix(value, name);
    if (raw.rows() != 3 || raw.cols() != 3) {
        throw std::runtime_error(name + " must be 3x3");
    }

    return raw;
}

DVec<double> dvecFromJsonVector(const json& value, const std::string& name) {
    const std::vector<double> raw = readJsonVector(value, name);
    DVec<double> out(static_cast<Eigen::Index>(raw.size()));
    for (Eigen::Index i = 0; i < out.size(); ++i) {
        out[i] = raw[static_cast<std::size_t>(i)];
    }
    return out;
}

std::optional<DVec<double>> optionalSolutionVector(const json& log,
                                                   const char* vectorKey,
                                                   const char* matrixKey) {
    if (!log.contains("solution") || !log.at("solution").is_object()) {
        return std::nullopt;
    }

    const json& solution = log.at("solution");
    if (solution.contains(vectorKey)) {
        return dvecFromJsonVector(solution.at(vectorKey), std::string("solution.") + vectorKey);
    }
    if (solution.contains(matrixKey)) {
        return dvecFromJsonVector(solution.at(matrixKey), std::string("solution.") + matrixKey);
    }
    return std::nullopt;
}

double metadataDoubleOr(const json& metadata, const char* key, const double fallback) {
    if (!metadata.is_object() || !metadata.contains(key) || metadata.at(key).is_null()) {
        return fallback;
    }
    return metadata.at(key).get<double>();
}

int metadataIntOr(const json& metadata, const char* key, const int fallback) {
    if (!metadata.is_object() || !metadata.contains(key) || metadata.at(key).is_null()) {
        return fallback;
    }
    return metadata.at(key).get<int>();
}

double jsonDoubleOr(const json& value, const char* key, const double fallback) {
    if (!value.is_object() || !value.contains(key) || value.at(key).is_null()) {
        return fallback;
    }
    return value.at(key).get<double>();
}

UserCommand userCommandFromLog(const json& log) {
    UserCommand out;
    if (!log.contains("user_command") || !log.at("user_command").is_object()) {
        return out;
    }

    const json& command = log.at("user_command");
    out.x_dot = jsonDoubleOr(command, "x_dot", 0.0);
    out.y_dot = jsonDoubleOr(command, "y_dot", 0.0);
    out.psi_dot = jsonDoubleOr(command, "psi_dot", 0.0);
    out.body_height_offset_m = jsonDoubleOr(command, "body_height_offset_m", 0.0);
    out.standing_roll_offset_rad =
        jsonDoubleOr(command, "standing_roll_offset_rad", 0.0);
    out.standing_pitch_offset_rad =
        jsonDoubleOr(command, "standing_pitch_offset_rad", 0.0);
    out.standing_mpc_debug_log_request =
        static_cast<unsigned long long>(jsonDoubleOr(command, "standing_mpc_debug_log_request", 0.0));
    out.locomotion_mode_toggle_request =
        static_cast<unsigned long long>(jsonDoubleOr(command, "locomotion_mode_toggle_request", 0.0));
    return out;
}

double maxAbsDelta(const DVec<double>& left, const DVec<double>& right) {
    if (left.size() != right.size()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double maxDelta = 0.0;
    for (Eigen::Index i = 0; i < left.size(); ++i) {
        maxDelta = std::max(maxDelta, std::abs(left[i] - right[i]));
    }
    return maxDelta;
}

double maxAbsDeltaFirstBlock(const DVec<double>& horizon, const Vec12<double>& first) {
    if (horizon.size() < kInputDim) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double maxDelta = 0.0;
    for (int i = 0; i < kInputDim; ++i) {
        maxDelta = std::max(maxDelta, std::abs(horizon[i] - first[i]));
    }
    return maxDelta;
}

double weightedHorizonErrorNorm(const DVec<double>& horizonError,
                                const StateWeightMat& weight) {
    const int steps = horizonSteps();
    if (horizonError.size() != kStateDim * steps) {
        throw std::runtime_error("weightedHorizonErrorNorm received unexpected dimension");
    }

    double sum = 0.0;
    for (int k = 0; k < steps; ++k) {
        const Vec13<double> error = horizonError.segment<kStateDim>(k * kStateDim);
        const Vec13<double> weighted = weight * error;
        sum += weighted.squaredNorm();
    }
    return std::sqrt(sum);
}

double weightedInputNorm(const DVec<double>& wrenchHorizon, const InputWeightMat& weight) {
    const int steps = horizonSteps();
    if (wrenchHorizon.size() != kInputDim * steps) {
        throw std::runtime_error("weightedInputNorm received unexpected dimension");
    }

    double sum = 0.0;
    for (int k = 0; k < steps; ++k) {
        const Vec12<double> wrench = wrenchHorizon.segment<kInputDim>(k * kInputDim);
        const double value = wrench.transpose() * weight * wrench;
        sum += std::max(0.0, value);
    }
    return std::sqrt(sum);
}

double vectorNormRange(const Vec13<double>& value, const int begin, const int count) {
    return value.segment(begin, count).norm();
}

RobotParams<double> robotParamsFromLog(const json& log) {
    RobotParams<double> params;
    if (log.contains("controller_config") &&
        log.at("controller_config").is_object() &&
        log.at("controller_config").contains("robot_type") &&
        log.at("controller_config").at("robot_type").is_string()) {
        params.roboType =
            robotTypeFromString(log.at("controller_config").at("robot_type").get<std::string>());
    } else {
        params.roboType = RobotType::MIT_HUMANOID;
    }
    params.bodyMass = log.at("model").at("mass").get<double>();
    params.bodyInertia =
        mat3FromJson(log.at("model").at("body_inertia_yaw_frame"),
                     "model.body_inertia_yaw_frame");
    if (log.at("model").contains("body_com_location_yaw_frame")) {
        params.bodyComLocation =
            vec3FromJson(log.at("model").at("body_com_location_yaw_frame"),
                         "model.body_com_location_yaw_frame");
    }
    return params;
}

DesiredFootPositions desiredFootPositionsFromLog(const json& log) {
    const json& feet = log.at("feet").at("desired_foot_pos_W");
    DesiredFootPositions out;
    out.left_des_W = vec3FromJson(feet.at("left"), "feet.desired_foot_pos_W.left");
    out.right_des_W = vec3FromJson(feet.at("right"), "feet.desired_foot_pos_W.right");
    return out;
}

Vec3<double> footXAxisFromLegJson(const json& leg) {
    if (leg.contains("foot_x_axis_W")) {
        return vec3FromJson(leg.at("foot_x_axis_W"), "feet.legs.foot_x_axis_W");
    }
    if (leg.contains("R_WF")) {
        const DMat<double> rotation = readJsonMatrix(leg.at("R_WF"), "feet.legs.R_WF");
        if (rotation.rows() != 3 || rotation.cols() != 3) {
            throw std::runtime_error("feet.legs.R_WF must be 3x3");
        }
        return rotation.col(0);
    }
    return Vec3<double>::UnitX();
}

void readFootLocalXAxesFromLog(const json& log,
                               Vec3<double>& leftFootXAxis_W,
                               Vec3<double>& rightFootXAxis_W) {
    leftFootXAxis_W = Vec3<double>::UnitX();
    rightFootXAxis_W = Vec3<double>::UnitX();

    if (!log.contains("feet") || !log.at("feet").contains("legs")) {
        return;
    }

    for (const auto& leg : log.at("feet").at("legs")) {
        if (!leg.contains("side") || !leg.at("side").is_string()) {
            continue;
        }

        const std::string side = leg.at("side").get<std::string>();
        if (side == "left") {
            leftFootXAxis_W = footXAxisFromLegJson(leg);
        } else if (side == "right") {
            rightFootXAxis_W = footXAxisFromLegJson(leg);
        }
    }
}

double footYawFromXAxis(const Vec3<double>& xAxis_W) {
    const double planarNorm = std::hypot(xAxis_W.x(), xAxis_W.y());
    if (!(planarNorm > 1e-9)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return std::atan2(xAxis_W.y(), xAxis_W.x());
}

double touchdownYawFromLegJson(const json& leg, const Vec3<double>& xAxisFallback_W) {
    if (leg.contains("touchdown_yaw_W") && !leg.at("touchdown_yaw_W").is_null()) {
        return leg.at("touchdown_yaw_W").get<double>();
    }
    return footYawFromXAxis(xAxisFallback_W);
}

Vec13<double> fixedReferenceFromLog(const json& log) {
    const DMat<double> xRefByStep =
        readJsonMatrix(log.at("reference_trajectory").at("X_ref_by_step"),
                       "reference_trajectory.X_ref_by_step");
    if (xRefByStep.rows() < 1 || xRefByStep.cols() != kStateDim) {
        throw std::runtime_error("reference_trajectory.X_ref_by_step has invalid shape");
    }

    Vec13<double> out = Vec13<double>::Zero();
    for (int i = 0; i < kStateDim; ++i) {
        out[i] = xRefByStep(0, i);
    }
    return out;
}

double clockT0FromLog(const json& log, const double fallback) {
    if (log.contains("metadata") &&
        log.at("metadata").is_object() &&
        log.at("metadata").contains("horizon_clock_t0") &&
        !log.at("metadata").at("horizon_clock_t0").is_null()) {
        return log.at("metadata").at("horizon_clock_t0").get<double>();
    }

    if (!log.contains("reference_trajectory") ||
        !log.at("reference_trajectory").contains("tk")) {
        return fallback;
    }

    const std::vector<double> tk =
        readJsonVector(log.at("reference_trajectory").at("tk"), "reference_trajectory.tk");
    if (tk.empty()) {
        return fallback;
    }
    return tk.front();
}

LocomotionMode locomotionModeFromLog(const json& log, const LocomotionMode fallback) {
    if (log.contains("metadata") &&
        log.at("metadata").is_object() &&
        log.at("metadata").contains("locomotion_mode") &&
        log.at("metadata").at("locomotion_mode").is_string()) {
        const std::string value = log.at("metadata").at("locomotion_mode").get<std::string>();
        return locomotionModeFromString(value);
    }
    if (log.contains("metadata") &&
        log.at("metadata").is_object() &&
        log.at("metadata").contains("locomotion_state") &&
        log.at("metadata").at("locomotion_state").is_string()) {
        return locomotionModeFromStateString(
            log.at("metadata").at("locomotion_state").get<std::string>());
    }
    if (log.contains("controller_config") &&
        log.at("controller_config").is_object() &&
        log.at("controller_config").contains("effective_locomotion_mode") &&
        log.at("controller_config").at("effective_locomotion_mode").is_string()) {
        return locomotionModeFromString(
            log.at("controller_config").at("effective_locomotion_mode").get<std::string>());
    }
    if (log.contains("controller_config") &&
        log.at("controller_config").is_object() &&
        log.at("controller_config").contains("requested_locomotion_mode") &&
        log.at("controller_config").at("requested_locomotion_mode").is_string()) {
        return locomotionModeFromString(
            log.at("controller_config").at("requested_locomotion_mode").get<std::string>());
    }
    if (log.contains("controller_config") &&
        log.at("controller_config").is_object() &&
        log.at("controller_config").contains("locomotion_mode") &&
        log.at("controller_config").at("locomotion_mode").is_string()) {
        return locomotionModeFromString(
            log.at("controller_config").at("locomotion_mode").get<std::string>());
    }
    return fallback;
}

double nominalSeedHeightFromReference(const Vec13<double>& referenceTarget,
                                      const UserCommand& command) {
    return referenceTarget[5] - command.body_height_offset_m;
}

Vec13<double> referenceSeedForStep(const Vec13<double>& state,
                                   const Vec13<double>& referenceTarget,
                                   const UserCommand& command,
                                   const LocomotionMode locomotionMode) {
    Vec13<double> seed = state;
    if (locomotionMode == LocomotionMode::Standing) {
        seed.segment<3>(0) = referenceTarget.segment<3>(0);
        seed.segment<3>(3) = referenceTarget.segment<3>(3);
        seed[5] = nominalSeedHeightFromReference(referenceTarget, command);
    } else {
        seed[0] = referenceTarget[0];
        seed[1] = referenceTarget[1];
        seed[5] = nominalSeedHeightFromReference(referenceTarget, command);
    }
    seed[12] = referenceTarget[12];
    return seed;
}

RolloutRow makeRowSkeleton(const int step,
                           const double sourceControllerTime,
                           const Vec13<double>& state,
                           const Vec13<double>& reference) {
    RolloutRow row;
    row.step = step;
    row.time = static_cast<double>(step) * dtMpc();
    row.simTime = sourceControllerTime + row.time;
    row.state = state;
    row.reference = reference;
    row.error = state - reference;
    row.errorNormRpy = vectorNormRange(row.error, 0, 3);
    row.errorNormPosition = vectorNormRange(row.error, 3, 3);
    row.errorNormOmega = vectorNormRange(row.error, 6, 3);
    row.errorNormVelocity = vectorNormRange(row.error, 9, 3);
    row.errorNormAll = row.error.head<12>().norm();
    return row;
}

RolloutResult runRollout(const json& log, const int rolloutSteps) {
    const json metadata = log.value("metadata", json::object());
    const ControllerConfig& config = getControllerConfig();

    RolloutResult result;
    result.fixedReference = fixedReferenceFromLog(log);
    result.desiredFootPositions = desiredFootPositionsFromLog(log);
    readFootLocalXAxesFromLog(log, result.leftFootXAxis_W, result.rightFootXAxis_W);
    if (log.contains("feet") && log.at("feet").contains("legs")) {
        for (const auto& leg : log.at("feet").at("legs")) {
            if (!leg.contains("side") || !leg.at("side").is_string()) {
                continue;
            }
            const std::string side = leg.at("side").get<std::string>();
            if (side == "left") {
                result.leftTouchdownYaw_W =
                    touchdownYawFromLegJson(leg, result.leftFootXAxis_W);
            } else if (side == "right") {
                result.rightTouchdownYaw_W =
                    touchdownYawFromLegJson(leg, result.rightFootXAxis_W);
            }
        }
    }
    result.userCommand = userCommandFromLog(log);
    result.locomotionMode = locomotionModeFromLog(log, config.requestedLocomotionMode);
    result.sourceControllerTime = metadataDoubleOr(metadata, "controller_time", 0.0);
    result.sourceClockT0 = clockT0FromLog(log, result.sourceControllerTime);
    result.sourceHorizonSteps = metadataIntOr(metadata, "horizon_steps", 0);
    result.sourceDtMpc = metadataDoubleOr(metadata, "dt_mpc", 0.0);

    RobotParams<double> robotParams = robotParamsFromLog(log);
    HorizonClock horizonClock(result.sourceClockT0);
    GaitScheduler gaitScheduler(&horizonClock);
    gaitScheduler.setLocomotionMode(result.locomotionMode);
    gaitScheduler.setFootLocalXAxesWorld(result.leftFootXAxis_W, result.rightFootXAxis_W);
    MPCFormulation formulation(&robotParams);
    MPCFormulationOutput formulationOutput;
    ReferenceTrajectoryOutput referenceOutput;
    ConvexMPC mpc;

    Vec13<double> state = vec13FromJson(log.at("initial_state").at("x0"), "initial_state.x0");
    Vec13<double> referenceTarget = result.fixedReference;
    const std::optional<DVec<double>> loggedWrenchHorizon =
        optionalSolutionVector(log, "wrench_horizon_vector", "wrench_horizon");
    const std::optional<DVec<double>> loggedPredictedHorizon =
        optionalSolutionVector(log, "predicted_state_horizon_vector", "predicted_state_horizon");

    result.rows.reserve(static_cast<std::size_t>(std::max(rolloutSteps, 0)));
    for (int step = 0; step < rolloutSteps; ++step) {
        horizonClock.reset(result.sourceClockT0 + static_cast<double>(step) * dtMpc());

        const Vec13<double> referenceSeed = referenceSeedForStep(state,
                                                                 referenceTarget,
                                                                 result.userCommand,
                                                                 result.locomotionMode);
        ReferenceTrajectory(
            &result.userCommand,
            referenceSeed,
            result.desiredFootPositions,
            &horizonClock,
            &gaitScheduler)
            .build(referenceOutput);

        RolloutRow row = makeRowSkeleton(
            step,
            result.sourceControllerTime,
            state,
            referenceOutput.X_ref.segment<kStateDim>(0));

        try {
            const double leftFootYaw_W =
                gaitScheduler.c(Side::Left, horizonClock.t0())
                    ? footYawFromXAxis(result.leftFootXAxis_W)
                    : result.leftTouchdownYaw_W;
            const double rightFootYaw_W =
                gaitScheduler.c(Side::Right, horizonClock.t0())
                    ? footYawFromXAxis(result.rightFootXAxis_W)
                    : result.rightTouchdownYaw_W;

            gaitScheduler.buildConstraintMatrices(nullptr, leftFootYaw_W, rightFootYaw_W);
            formulation.build(referenceOutput, formulationOutput);
            mpc.updateInput(
                gaitScheduler,
                formulationOutput,
                referenceOutput,
                state,
                result.locomotionMode);
            mpc.solve();
        } catch (const std::exception& exception) {
            row.solveOk = false;
            result.failureMessage = exception.what();
            result.rows.push_back(row);
            break;
        }

        const DVec<double>& wrenchHorizon = mpc.optimalWrenchHorizon();
        const DVec<double> predictedHorizon =
            formulationOutput.A_qp * state + formulationOutput.B_qp * wrenchHorizon;
        const DVec<double> horizonError = predictedHorizon - referenceOutput.X_ref;

        row.solveOk = true;
        row.nextState = predictedHorizon.segment<kStateDim>(0);
        row.firstWrench = wrenchHorizon.segment<kInputDim>(0);
        row.weightedHorizonErrorNorm =
            weightedHorizonErrorNorm(horizonError,
                                     result.locomotionMode == LocomotionMode::Standing
                                         ? config.mpc.standingStateWeight
                                         : config.mpc.walkingStateWeight);
        row.inputNorm = wrenchHorizon.norm();
        row.weightedInputNorm =
            weightedInputNorm(wrenchHorizon,
                              result.locomotionMode == LocomotionMode::Standing
                                  ? config.mpc.standingInputWeight
                                  : config.mpc.walkingInputWeight);

        if (step == 0) {
            if (loggedWrenchHorizon.has_value()) {
                result.firstHorizonWrenchDeltaToLog =
                    maxAbsDelta(wrenchHorizon, *loggedWrenchHorizon);
                result.firstWrenchDeltaToLog =
                    maxAbsDeltaFirstBlock(*loggedWrenchHorizon, row.firstWrench);
            }
            if (loggedPredictedHorizon.has_value()) {
                result.firstHorizonStateDeltaToLog =
                    maxAbsDelta(predictedHorizon, *loggedPredictedHorizon);
            }
        }

        state = row.nextState;
        result.rows.push_back(row);
    }

    return result;
}

void writeVec3Row(std::ostream& out, const std::string& label, const Vec3<double>& value) {
    out << "| `" << label << "` | "
        << value[0] << " | "
        << value[1] << " | "
        << value[2] << " |\n";
}

void writeStateVectorTable(std::ostream& out,
                           const std::string& title,
                           const Vec13<double>& value) {
    out << "### " << title << "\n\n"
        << "| State | Value |\n"
        << "| --- | ---: |\n";
    for (int i = 0; i < kStateDim; ++i) {
        out << "| `" << kStateNames[static_cast<std::size_t>(i)] << "` | "
            << value[i] << " |\n";
    }
    out << "\n";
}

void writeOptionalMetricRow(std::ostream& out,
                            const std::string& label,
                            const std::optional<double>& value) {
    out << "| `" << label << "` | ";
    if (value.has_value() && std::isfinite(*value)) {
        out << *value;
    } else {
        out << "n/a";
    }
    out << " |\n";
}

Vec13<double> finalStateFromResult(const RolloutResult& result) {
    if (result.rows.empty()) {
        return Vec13<double>::Zero();
    }
    const RolloutRow& last = result.rows.back();
    return last.solveOk ? last.nextState : last.state;
}

double maxAbsTrajectoryError(const RolloutResult& result, const int index) {
    double maxValue = 0.0;
    for (const RolloutRow& row : result.rows) {
        maxValue = std::max(maxValue, std::abs(row.error[index]));
    }
    if (!result.rows.empty() && result.rows.back().solveOk) {
        const Vec13<double> finalError = result.rows.back().nextState - result.rows.back().reference;
        maxValue = std::max(maxValue, std::abs(finalError[index]));
    }
    return maxValue;
}

void writeReport(std::ostream& out,
                 const std::filesystem::path& logPath,
                 const Options& options,
                 const std::string& robotType,
                 const RolloutResult& result,
                 const OutputPaths& outputPaths) {
    out << std::fixed << std::setprecision(9);
    out << "# Receding Horizon Report\n\n"
        << "## Source\n\n"
        << "| Field | Value |\n"
        << "| --- | --- |\n"
        << "| Source log | `" << std::filesystem::absolute(logPath).string() << "` |\n"
        << "| Robot type | `" << robotType << "` |\n"
        << "| Locomotion mode | `" << locomotionModeName(result.locomotionMode) << "` |\n"
        << "| Contact wrench model | `"
        << contactWrenchModelName(contactWrenchModel(result.locomotionMode)) << "` |\n\n"
        << "## Method\n\n"
        << "This is an SRB-only true receding-horizon replay. At each step it solves MPC again using "
        << "the logged command and fixed logged foot points, then advances the reduced state with the "
        << "first solved wrench.\n\n"
        << "$$\n"
        << "x_{k+1} = A_{qp} x_k + B_{qp} w_k\n"
        << "$$\n\n"
        << "The first-solve comparison uses the logged `controller_config` snapshot when available; "
        << "otherwise it falls back to the current `config/my_controller.yaml`.\n\n"
        << "## Summary\n\n"
        << "| Metric | Value |\n"
        << "| --- | ---: |\n"
        << "| requested rollout steps | " << options.steps << " |\n"
        << "| completed rows | " << result.rows.size() << " |\n"
        << "| config horizon steps | " << horizonSteps() << " |\n"
        << "| source horizon steps | " << result.sourceHorizonSteps << " |\n"
        << "| config dt_mpc | " << dtMpc() << " |\n"
        << "| source dt_mpc | " << result.sourceDtMpc << " |\n"
        << "| source controller time | " << result.sourceControllerTime << " |\n"
        << "| source clock t0 | " << result.sourceClockT0 << " |\n\n";

    out << "## Logged Foot Points\n\n"
        << "| Quantity | x | y | z |\n"
        << "| --- | ---: | ---: | ---: |\n";
    writeVec3Row(out, "desired_left_foot_W", result.desiredFootPositions.left_des_W);
    writeVec3Row(out, "desired_right_foot_W", result.desiredFootPositions.right_des_W);
    writeVec3Row(out, "left_foot_x_axis_W", result.leftFootXAxis_W);
    writeVec3Row(out, "right_foot_x_axis_W", result.rightFootXAxis_W);
    out << "\n";

    writeStateVectorTable(out, "Initial Reference", result.fixedReference);
    writeStateVectorTable(out, "Final State", finalStateFromResult(result));
    writeStateVectorTable(out, "Final Error", finalStateFromResult(result) - result.fixedReference);

    out << "## First Solve Delta To Source Log\n\n"
        << "| Metric | Value |\n"
        << "| --- | ---: |\n";
    writeOptionalMetricRow(out, "first_wrench_max_abs_delta", result.firstWrenchDeltaToLog);
    writeOptionalMetricRow(out, "wrench_horizon_max_abs_delta", result.firstHorizonWrenchDeltaToLog);
    writeOptionalMetricRow(out,
                           "predicted_horizon_state_max_abs_delta",
                           result.firstHorizonStateDeltaToLog);
    out << "\n";

    out << "## Max Abs State Error Over Rollout\n\n"
        << "| State | Max abs error |\n"
        << "| --- | ---: |\n";
    for (int i = 0; i < 12; ++i) {
        out << "| `" << kStateNames[static_cast<std::size_t>(i)] << "` | "
            << maxAbsTrajectoryError(result, i) << " |\n";
    }
    out << "\n";

    if (!result.failureMessage.empty()) {
        out << "## Failure\n\n"
            << "`" << result.failureMessage << "`\n\n";
    }

    out << "## Outputs\n\n"
        << "| Artifact | Path |\n"
        << "| --- | --- |\n"
        << "| CSV | `" << std::filesystem::absolute(outputPaths.csv).string() << "` |\n"
        << "| Plots directory | `" << std::filesystem::absolute(outputPaths.plotsDir).string() << "` |\n"
        << "| States plot | `" << std::filesystem::absolute(outputPaths.statesPlot).string() << "` |\n"
        << "| Wrench plot | `" << std::filesystem::absolute(outputPaths.wrenchPlot).string() << "` |\n"
        << "| Metrics plot | `" << std::filesystem::absolute(outputPaths.metricsPlot).string() << "` |\n";
}

void writeCsvHeader(std::ostream& out) {
    out << "step,time,sim_time,solve_ok";
    for (const char* name : kStateNames) {
        out << ',' << name;
    }
    for (const char* name : kStateNames) {
        out << ",ref_" << name;
    }
    for (const char* name : kStateNames) {
        out << ",err_" << name;
    }
    for (const char* name : kStateNames) {
        out << ",next_" << name;
    }
    for (const char* name : kInputNames) {
        out << ',' << name;
    }
    out << ",error_norm_rpy,error_norm_position,error_norm_omega,error_norm_velocity,"
        << "error_norm_all,weighted_horizon_error_norm,input_norm,weighted_input_norm\n";
}

template <typename Derived>
void writeEigenVectorCsv(std::ostream& out, const Eigen::MatrixBase<Derived>& value) {
    for (Eigen::Index i = 0; i < value.size(); ++i) {
        out << ',' << value.derived()[i];
    }
}

void writeCsv(std::ostream& out,
              const std::filesystem::path& logPath,
              const Options& options,
              const std::string& robotType,
              const RolloutResult& result) {
    out << std::setprecision(17);
    out << "# source_json_file=" << logPath.filename().string() << '\n';
    out << "# source_json_path=" << std::filesystem::absolute(logPath).string() << '\n';
    out << "# robot_type=" << robotType << '\n';
    out << "# locomotion_mode=" << locomotionModeName(result.locomotionMode) << '\n';
    out << "# requested_rollout_steps=" << options.steps << '\n';
    out << "# config_horizon_steps=" << horizonSteps() << '\n';
    out << "# config_dt_mpc=" << dtMpc() << '\n';
    out << "# source_controller_time=" << result.sourceControllerTime << '\n';
    out << "# source_clock_t0=" << result.sourceClockT0 << '\n';
    out << "# contact_wrench_model="
        << contactWrenchModelName(contactWrenchModel(result.locomotionMode)) << '\n';
    out << "# state_order=" << joinNames(kStateNames) << '\n';
    out << "# input_order=" << joinInputNames() << '\n';
    writeCsvHeader(out);

    for (const RolloutRow& row : result.rows) {
        out << row.step << ','
            << row.time << ','
            << row.simTime << ','
            << (row.solveOk ? 1 : 0);
        writeEigenVectorCsv(out, row.state);
        writeEigenVectorCsv(out, row.reference);
        writeEigenVectorCsv(out, row.error);
        writeEigenVectorCsv(out, row.nextState);
        writeEigenVectorCsv(out, row.firstWrench);
        out << ',' << row.errorNormRpy
            << ',' << row.errorNormPosition
            << ',' << row.errorNormOmega
            << ',' << row.errorNormVelocity
            << ',' << row.errorNormAll
            << ',' << row.weightedHorizonErrorNorm
            << ',' << row.inputNorm
            << ',' << row.weightedInputNorm
            << '\n';
    }
}

std::string shellQuote(const std::filesystem::path& path) {
    std::string quoted = "'";
    for (const char c : path.string()) {
        if (c == '\'') {
            quoted += "'\\''";
        } else {
            quoted += c;
        }
    }
    quoted += "'";
    return quoted;
}

void runPlotScript(const OutputPaths& outputPaths) {
    const std::filesystem::path scriptPath =
        std::filesystem::path(PROJECT_ROOT_DIR) / "test" / "standing_debug" / "plot_stand_rh.py";
    const char* pythonEnv = std::getenv("PYTHON");
    const std::string python = (pythonEnv != nullptr && pythonEnv[0] != '\0') ? pythonEnv : "python";
    const std::string command =
        python + " " + shellQuote(scriptPath) + " " +
        shellQuote(outputPaths.csv) + " " +
        shellQuote(outputPaths.statesPlot) + " " +
        shellQuote(outputPaths.wrenchPlot) + " " +
        shellQuote(outputPaths.metricsPlot);
    const int status = std::system(command.c_str());
    if (status == 0) {
        std::cout << "states plot: "
                  << std::filesystem::absolute(outputPaths.statesPlot).string() << "\n";
        std::cout << "wrench plot: "
                  << std::filesystem::absolute(outputPaths.wrenchPlot).string() << "\n";
        std::cout << "metrics plot: "
                  << std::filesystem::absolute(outputPaths.metricsPlot).string() << "\n";
    } else {
        std::cerr << "plot: failed to run " << scriptPath
                  << " with exit status " << status << "\n";
    }
}

Options parseArgs(int argc, char** argv) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: stand_rh_probe [-n STEPS] [mpc_debug.json]\n"
                      << "  If the log path is omitted, the latest standing_mpc or walking_mpc log is used.\n";
            std::exit(EXIT_SUCCESS);
        }
        if (arg == "-n") {
            if (index + 1 >= argc) {
                throw std::runtime_error("-n requires a positive integer");
            }
            options.steps = std::stoi(argv[++index]);
            if (options.steps <= 0) {
                throw std::runtime_error("-n requires a positive integer");
            }
            continue;
        }
        if (!options.logPath.empty()) {
            throw std::runtime_error("Unexpected extra argument: " + arg);
        }
        options.logPath = arg;
    }

    if (options.logPath.empty()) {
        options.logPath = latestDebugLogPath();
    }
    return options;
}
}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parseArgs(argc, argv);

        std::ifstream logStream(options.logPath);
        if (!logStream.is_open()) {
            throw std::runtime_error("Failed to open debug log: " + options.logPath.string());
        }

        json log;
        logStream >> log;

        const std::optional<ControllerConfig> loggedConfig = controllerConfigFromLog(log);
        if (loggedConfig.has_value()) {
            setControllerConfigOverride(&*loggedConfig);
        }
        const std::string robotType = robotTypeLabelFromLog(log);

        const RolloutResult result = runRollout(log, options.steps);
        const OutputPaths outputPaths = defaultOutputPaths(result.locomotionMode);

        writeReport(std::cout, options.logPath, options, robotType, result, outputPaths);

        std::ofstream report(outputPaths.report, std::ios::out | std::ios::trunc);
        if (!report.is_open()) {
            throw std::runtime_error("Failed to open report output: " + outputPaths.report.string());
        }
        writeReport(report, options.logPath, options, robotType, result, outputPaths);
        report.close();
        if (!report.good()) {
            throw std::runtime_error("Failed while writing report output: " + outputPaths.report.string());
        }
        std::cout << "report: " << std::filesystem::absolute(outputPaths.report).string() << "\n";

        std::ofstream csv(outputPaths.csv, std::ios::out | std::ios::trunc);
        if (!csv.is_open()) {
            throw std::runtime_error("Failed to open CSV output: " + outputPaths.csv.string());
        }
        writeCsv(csv, options.logPath, options, robotType, result);
        csv.close();
        if (!csv.good()) {
            throw std::runtime_error("Failed while writing CSV output: " + outputPaths.csv.string());
        }
        std::cout << "csv: " << std::filesystem::absolute(outputPaths.csv).string() << "\n";

        runPlotScript(outputPaths);
        clearControllerConfigOverride();
        return EXIT_SUCCESS;
    } catch (const std::exception& exception) {
        std::cerr << "stand_rh_probe: " << exception.what() << "\n";
        return EXIT_FAILURE;
    }
}
