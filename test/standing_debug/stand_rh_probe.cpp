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
#include "Robot/RobotParams.h"
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
    double sourceControllerTime{0.0};
    double sourceClockT0{0.0};
    int sourceHorizonSteps{0};
    double sourceDtMpc{0.0};
    std::optional<double> firstWrenchDeltaToLog;
    std::optional<double> firstHorizonWrenchDeltaToLog;
    std::optional<double> firstHorizonStateDeltaToLog;
    std::string failureMessage;
};

std::string timestampToken() {
    using clock = std::chrono::system_clock;
    const std::time_t time = clock::to_time_t(clock::now());

    std::tm localTime {};
    localtime_r(&time, &localTime);

    std::ostringstream out;
    out << std::put_time(&localTime, "%Y%m%d_%H%M%S");
    return out.str();
}

OutputPaths defaultOutputPaths() {
    const std::string timestamp = timestampToken();
    const std::filesystem::path dir =
        std::filesystem::path(PROJECT_ROOT_DIR) / "logs" / "debug" /
        "standing_mpc" / "receding_horizon";
    const std::filesystem::path txtDir = dir / "txt";
    const std::filesystem::path csvDir = dir / "csv";
    const std::filesystem::path plotDir = dir / "plots";
    const std::filesystem::path plotRunDir = plotDir / ("stand_rh_" + timestamp);
    std::filesystem::create_directories(txtDir);
    std::filesystem::create_directories(csvDir);
    std::filesystem::create_directories(plotRunDir);
    std::filesystem::create_directories(plotDir);

    OutputPaths paths;
    paths.report = txtDir / ("stand_rh_" + timestamp + ".txt");
    paths.csv = csvDir / ("stand_rh_" + timestamp + ".csv");
    paths.plotsDir = plotRunDir;
    paths.statesPlot = plotRunDir / "states.png";
    paths.wrenchPlot = plotRunDir / "wrench.png";
    paths.metricsPlot = plotRunDir / "metrics.png";
    return paths;
}

std::filesystem::path latestDebugLogPath() {
    const std::filesystem::path dir =
        std::filesystem::path(PROJECT_ROOT_DIR) / "logs" / "debug" / "standing_mpc";
    if (!std::filesystem::exists(dir)) {
        throw std::runtime_error("Debug log directory does not exist: " + dir.string());
    }

    std::filesystem::path best;
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (!entry.is_regular_file()) {
            continue;
        }

        const std::string name = entry.path().filename().string();
        if (name.rfind("standing_mpc_debug_", 0) != 0 || entry.path().extension() != ".json") {
            continue;
        }

        if (best.empty() || entry.path().filename().string() > best.filename().string()) {
            best = entry.path();
        }
    }

    if (best.empty()) {
        throw std::runtime_error("No standing_mpc_debug_*.json file found in " + dir.string());
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

double weightedHorizonErrorNorm(const DVec<double>& horizonError) {
    const int steps = horizonSteps();
    if (horizonError.size() != kStateDim * steps) {
        throw std::runtime_error("weightedHorizonErrorNorm received unexpected dimension");
    }

    const StateWeightMat& weight = getControllerConfig().mpc.stateWeight;
    double sum = 0.0;
    for (int k = 0; k < steps; ++k) {
        const Vec13<double> error = horizonError.segment<kStateDim>(k * kStateDim);
        const Vec13<double> weighted = weight * error;
        sum += weighted.squaredNorm();
    }
    return std::sqrt(sum);
}

double weightedInputNorm(const DVec<double>& wrenchHorizon) {
    const int steps = horizonSteps();
    if (wrenchHorizon.size() != kInputDim * steps) {
        throw std::runtime_error("weightedInputNorm received unexpected dimension");
    }

    const InputWeightMat& weight = getControllerConfig().mpc.inputWeight;
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
    params.roboType = RobotType::MIT_HUMANOID;
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

Vec13<double> referenceSeedForStep(const Vec13<double>& state,
                                   const Vec13<double>& fixedReference) {
    Vec13<double> seed = state;
    seed.segment<3>(0) = fixedReference.segment<3>(0);
    seed.segment<3>(3) = fixedReference.segment<3>(3);
    seed[12] = fixedReference[12];
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

    RolloutResult result;
    result.fixedReference = fixedReferenceFromLog(log);
    result.desiredFootPositions = desiredFootPositionsFromLog(log);
    result.sourceControllerTime = metadataDoubleOr(metadata, "controller_time", 0.0);
    result.sourceClockT0 = clockT0FromLog(log, result.sourceControllerTime);
    result.sourceHorizonSteps = metadataIntOr(metadata, "horizon_steps", 0);
    result.sourceDtMpc = metadataDoubleOr(metadata, "dt_mpc", 0.0);

    RobotParams<double> robotParams = robotParamsFromLog(log);
    HorizonClock horizonClock(result.sourceClockT0);
    GaitScheduler gaitScheduler(&horizonClock);
    gaitScheduler.setLocomotionMode(LocomotionMode::Standing);
    MPCFormulation formulation(&robotParams);
    MPCFormulationOutput formulationOutput;
    ReferenceTrajectoryOutput referenceOutput;
    ConvexMPC mpc;
    UserCommand standingCommand;

    Vec13<double> state = vec13FromJson(log.at("initial_state").at("x0"), "initial_state.x0");
    const std::optional<DVec<double>> loggedWrenchHorizon =
        optionalSolutionVector(log, "wrench_horizon_vector", "wrench_horizon");
    const std::optional<DVec<double>> loggedPredictedHorizon =
        optionalSolutionVector(log, "predicted_state_horizon_vector", "predicted_state_horizon");

    result.rows.reserve(static_cast<std::size_t>(std::max(rolloutSteps, 0)));
    for (int step = 0; step < rolloutSteps; ++step) {
        horizonClock.reset(result.sourceClockT0 + static_cast<double>(step) * dtMpc());

        const Vec13<double> referenceSeed = referenceSeedForStep(state, result.fixedReference);
        ReferenceTrajectory(
            &standingCommand,
            referenceSeed,
            result.desiredFootPositions,
            &horizonClock)
            .build(referenceOutput);

        RolloutRow row = makeRowSkeleton(
            step,
            result.sourceControllerTime,
            state,
            referenceOutput.X_ref.segment<kStateDim>(0));

        try {
            gaitScheduler.buildConstraintMatrices();
            formulation.build(referenceOutput, formulationOutput);
            mpc.updateInput(gaitScheduler, formulationOutput, referenceOutput, state);
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
        row.weightedHorizonErrorNorm = weightedHorizonErrorNorm(horizonError);
        row.inputNorm = wrenchHorizon.norm();
        row.weightedInputNorm = weightedInputNorm(wrenchHorizon);

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

void writeVector(std::ostream& out, const std::string& label, const Vec3<double>& value) {
    out << "  " << label << ": ["
        << value[0] << ", "
        << value[1] << ", "
        << value[2] << "]\n";
}

void writeStateVector(std::ostream& out, const std::string& label, const Vec13<double>& value) {
    out << "  " << label << ": [";
    for (int i = 0; i < kStateDim; ++i) {
        if (i > 0) {
            out << ", ";
        }
        out << kStateNames[static_cast<std::size_t>(i)] << '=' << value[i];
    }
    out << "]\n";
}

void writeOptionalDouble(std::ostream& out, const std::string& label,
                         const std::optional<double>& value) {
    out << "  " << label << ": ";
    if (value.has_value() && std::isfinite(*value)) {
        out << *value;
    } else {
        out << "n/a";
    }
    out << "\n";
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
                 const RolloutResult& result,
                 const OutputPaths& outputPaths) {
    out << std::fixed << std::setprecision(9);
    out << "[stand_rh_probe]\n"
        << "  source_log: " << std::filesystem::absolute(logPath).string() << "\n"
        << "  requested_rollout_steps: " << options.steps << "\n"
        << "  completed_rows: " << result.rows.size() << "\n"
        << "  config_horizon_steps: " << horizonSteps() << "\n"
        << "  source_horizon_steps: " << result.sourceHorizonSteps << "\n"
        << "  config_dt_mpc: " << dtMpc() << "\n"
        << "  source_dt_mpc: " << result.sourceDtMpc << "\n"
        << "  source_controller_time: " << result.sourceControllerTime << "\n"
        << "  source_clock_t0: " << result.sourceClockT0 << "\n"
        << "  rollout_model: SRB-only true receding horizon, fixed standing reference, fixed foot points\n\n";

    writeVector(out, "desired_left_foot_W", result.desiredFootPositions.left_des_W);
    writeVector(out, "desired_right_foot_W", result.desiredFootPositions.right_des_W);
    writeStateVector(out, "fixed_reference", result.fixedReference);
    writeStateVector(out, "final_state", finalStateFromResult(result));
    writeStateVector(out, "final_error", finalStateFromResult(result) - result.fixedReference);
    out << "\n";

    out << "first_solve_delta_to_source_log\n";
    out << "  note: comparison uses the current config/my_controller.yaml; "
        << "nonzero values are expected if weights/constraints changed after the source log was captured\n";
    writeOptionalDouble(out, "first_wrench_max_abs_delta", result.firstWrenchDeltaToLog);
    writeOptionalDouble(out, "wrench_horizon_max_abs_delta", result.firstHorizonWrenchDeltaToLog);
    writeOptionalDouble(out, "predicted_horizon_state_max_abs_delta",
                        result.firstHorizonStateDeltaToLog);
    out << "\n";

    out << "max_abs_state_error_over_rollout\n";
    for (int i = 0; i < 12; ++i) {
        out << "  " << kStateNames[static_cast<std::size_t>(i)] << ": "
            << maxAbsTrajectoryError(result, i) << "\n";
    }
    out << "\n";

    if (!result.failureMessage.empty()) {
        out << "failure: " << result.failureMessage << "\n\n";
    }

    out << "outputs\n"
        << "  csv: " << std::filesystem::absolute(outputPaths.csv).string() << "\n"
        << "  plots_dir: " << std::filesystem::absolute(outputPaths.plotsDir).string() << "\n"
        << "  states_plot: " << std::filesystem::absolute(outputPaths.statesPlot).string() << "\n"
        << "  wrench_plot: " << std::filesystem::absolute(outputPaths.wrenchPlot).string() << "\n"
        << "  metrics_plot: " << std::filesystem::absolute(outputPaths.metricsPlot).string() << "\n";
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
              const RolloutResult& result) {
    out << std::setprecision(17);
    out << "# source_json_file=" << logPath.filename().string() << '\n';
    out << "# source_json_path=" << std::filesystem::absolute(logPath).string() << '\n';
    out << "# requested_rollout_steps=" << options.steps << '\n';
    out << "# config_horizon_steps=" << horizonSteps() << '\n';
    out << "# config_dt_mpc=" << dtMpc() << '\n';
    out << "# source_controller_time=" << result.sourceControllerTime << '\n';
    out << "# source_clock_t0=" << result.sourceClockT0 << '\n';
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
            std::cout << "Usage: stand_rh_probe [-n STEPS] [standing_mpc_debug.json]\n"
                      << "  If the log path is omitted, the latest logs/debug/standing_mpc log is used.\n";
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
        const OutputPaths outputPaths = defaultOutputPaths();

        std::ifstream logStream(options.logPath);
        if (!logStream.is_open()) {
            throw std::runtime_error("Failed to open debug log: " + options.logPath.string());
        }

        json log;
        logStream >> log;

        const RolloutResult result = runRollout(log, options.steps);

        writeReport(std::cout, options.logPath, options, result, outputPaths);

        std::ofstream report(outputPaths.report, std::ios::out | std::ios::trunc);
        if (!report.is_open()) {
            throw std::runtime_error("Failed to open report output: " + outputPaths.report.string());
        }
        writeReport(report, options.logPath, options, result, outputPaths);
        report.close();
        if (!report.good()) {
            throw std::runtime_error("Failed while writing report output: " + outputPaths.report.string());
        }
        std::cout << "report: " << std::filesystem::absolute(outputPaths.report).string() << "\n";

        std::ofstream csv(outputPaths.csv, std::ios::out | std::ios::trunc);
        if (!csv.is_open()) {
            throw std::runtime_error("Failed to open CSV output: " + outputPaths.csv.string());
        }
        writeCsv(csv, options.logPath, options, result);
        csv.close();
        if (!csv.good()) {
            throw std::runtime_error("Failed while writing CSV output: " + outputPaths.csv.string());
        }
        std::cout << "csv: " << std::filesystem::absolute(outputPaths.csv).string() << "\n";

        runPlotScript(outputPaths);
        return EXIT_SUCCESS;
    } catch (const std::exception& exception) {
        std::cerr << "stand_rh_probe: " << exception.what() << "\n";
        return EXIT_FAILURE;
    }
}
