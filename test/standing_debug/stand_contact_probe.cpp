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
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include <nlohmann/json.hpp>

#include "MujocoRobotBindings.h"
#include "RobotConfig.h"
#include "Utilities/MatrixUtils.h"
#include "setupRobotParams.h"

namespace {
using json = nlohmann::ordered_json;

struct WrenchAtPoint {
    Vec3<double> force_W = Vec3<double>::Zero();
    Vec3<double> moment_W = Vec3<double>::Zero();
    int contactCount = 0;
};

struct FootContactResult {
    std::string side;
    int footBodyId = -1;
    int footSiteId = -1;
    Vec3<double> footCollisionCenter_W = Vec3<double>::Zero();
    Vec3<double> footSite_W = Vec3<double>::Zero();
    Vec3<double> desiredForce_W = Vec3<double>::Zero();
    Vec3<double> desiredMoment_W = Vec3<double>::Zero();
    WrenchAtPoint atFootCollisionCenter;
    WrenchAtPoint atFootSite;
};

struct ContactProbeOutputPaths {
    std::filesystem::path report;
    std::filesystem::path csv;
    std::filesystem::path plot;
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

bool isMpcDebugLogFilename(const std::string& name) {
    const bool standingLog = name.rfind("standing_mpc_debug_", 0) == 0;
    const bool walkingLog = name.rfind("walking_mpc_debug_", 0) == 0;
    return (standingLog || walkingLog) &&
           std::filesystem::path(name).extension() == ".json";
}

std::string normalizedLocomotionMode(const std::string& mode) {
    if (mode == "walking" || mode == "walk") {
        return "walking";
    }
    if (mode == "standing" || mode == "stand") {
        return "standing";
    }
    return "unknown";
}

std::string shortLocomotionPrefix(const std::string& mode) {
    return normalizedLocomotionMode(mode) == "walking" ? "walk" : "stand";
}

std::string debugDirectoryNameForMode(const std::string& mode) {
    return normalizedLocomotionMode(mode) == "walking" ? "walking_mpc" : "standing_mpc";
}

std::vector<std::filesystem::path> mpcDebugLogDirectories() {
    const std::filesystem::path debugRoot =
        std::filesystem::path(PROJECT_ROOT_DIR) / "logs" / "debug";
    return {
        debugRoot / "standing_mpc",
        debugRoot / "walking_mpc",
    };
}

ContactProbeOutputPaths defaultOutputPaths(const std::string& locomotionMode) {
    const std::string timestamp = timestampToken();
    const std::string prefix = shortLocomotionPrefix(locomotionMode) + "_contact_probe_";
    const std::filesystem::path dir =
        std::filesystem::path(PROJECT_ROOT_DIR) / "logs" / "debug" /
        debugDirectoryNameForMode(locomotionMode) / "contact_probe";
    const std::filesystem::path reportDir = dir / "reports";
    const std::filesystem::path csvDir = dir / "csv";
    const std::filesystem::path plotDir = dir / "plots";
    std::filesystem::create_directories(dir);
    std::filesystem::create_directories(reportDir);
    std::filesystem::create_directories(csvDir);
    std::filesystem::create_directories(plotDir);

    ContactProbeOutputPaths paths;
    paths.report = reportDir / (prefix + timestamp + ".md");
    paths.csv = csvDir / (prefix + timestamp + ".csv");
    paths.plot = plotDir / (prefix + timestamp + ".png");
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

std::string readJsonStringOr(const json& value,
                             const std::string& key,
                             const std::string& fallback) {
    if (!value.is_object() || !value.contains(key) || !value.at(key).is_string()) {
        return fallback;
    }
    return value.at(key).get<std::string>();
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
    throw std::runtime_error("Invalid robot_type: " + value);
}

std::string resolveModelPathOr(const std::string& modelPathFallback, const std::string& value) {
    if (value.empty()) {
        return modelPathFallback;
    }
    return resolveProjectPath(value);
}

FootEndEffectorSource footEndEffectorSourceFromString(const std::string& value) {
    if (value == "site") {
        return FootEndEffectorSource::Site;
    }
    if (value == "collision_geom_center") {
        return FootEndEffectorSource::CollisionGeomCenter;
    }
    throw std::runtime_error("Invalid foot_end_effector_source: " + value);
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
    out.reserve(source->size());
    for (const auto& item : *source) {
        out.push_back(item.get<double>());
    }
    return out;
}

void copyVectorToMujoco(const std::vector<double>& src,
                        mjtNum* dst,
                        const int expectedSize,
                        const std::string& name) {
    if (static_cast<int>(src.size()) != expectedSize) {
        throw std::runtime_error(name + " size does not match MuJoCo model dimension");
    }

    for (int i = 0; i < expectedSize; ++i) {
        dst[i] = static_cast<mjtNum>(src[static_cast<std::size_t>(i)]);
    }
}

std::vector<double> applyCtrlWithClamp(const mjModel* model,
                                       mjData* data,
                                       const std::vector<double>& tauCommand) {
    if (static_cast<int>(tauCommand.size()) != model->nu) {
        throw std::runtime_error("robot_state.full_tau_command size does not match model->nu");
    }

    std::vector<double> applied(static_cast<std::size_t>(model->nu), 0.0);
    for (int i = 0; i < model->nu; ++i) {
        double tau = tauCommand[static_cast<std::size_t>(i)];
        if (model->actuator_ctrllimited[i]) {
            const double lo = static_cast<double>(model->actuator_ctrlrange[2 * i + 0]);
            const double hi = static_cast<double>(model->actuator_ctrlrange[2 * i + 1]);
            tau = std::clamp(tau, lo, hi);
        }
        data->ctrl[i] = static_cast<mjtNum>(tau);
        applied[static_cast<std::size_t>(i)] = tau;
    }
    return applied;
}

Vec3<double> readMujocoVec3(const mjtNum* raw) {
    Vec3<double> out = Vec3<double>::Zero();
    for (int i = 0; i < 3; ++i) {
        out[i] = static_cast<double>(raw[i]);
    }
    return out;
}

Vec3<double> readCollisionGeomCenterWorld(const mjData* data, const std::vector<int>& geomIds) {
    if (geomIds.empty()) {
        throw std::runtime_error("Foot binding has no collision geoms");
    }

    Vec3<double> center = Vec3<double>::Zero();
    for (const int geomId : geomIds) {
        center += readMujocoVec3(data->geom_xpos + 3 * geomId);
    }
    return center / static_cast<double>(geomIds.size());
}

Vec3<double> readSiteWorld(const mjData* data, const int siteId) {
    return readMujocoVec3(data->site_xpos + 3 * siteId);
}

Mat3<double> contactFrameRowsWorld(const mjContact& contact) {
    Mat3<double> frame = Mat3<double>::Zero();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            frame(row, col) = static_cast<double>(contact.frame[3 * row + col]);
        }
    }
    return frame;
}

void addContactWrench(WrenchAtPoint& wrench,
                      const Vec3<double>& force_W,
                      const Vec3<double>& momentAtContact_W,
                      const Vec3<double>& contactPoint_W,
                      const Vec3<double>& referencePoint_W) {
    wrench.force_W += force_W;
    wrench.moment_W += momentAtContact_W + (contactPoint_W - referencePoint_W).cross(force_W);
    ++wrench.contactCount;
}

std::string sideName(const Side side) {
    switch (side) {
        case Side::Left:
            return "left";
        case Side::Right:
            return "right";
        case Side::FL:
            return "front_left";
        case Side::FR:
            return "front_right";
        case Side::BL:
            return "back_left";
        case Side::BR:
            return "back_right";
    }
    return "unknown";
}

Vec3<double> vec3FromWrenchBlock(const std::vector<double>& firstWrench,
                                 const Side side,
                                 const bool momentBlock) {
    if (firstWrench.size() != 12) {
        throw std::runtime_error("solution.first_wrench must have size 12");
    }

    int offset = 0;
    if (momentBlock) {
        offset = (side == Side::Left) ? 6 : 9;
    } else {
        offset = (side == Side::Left) ? 0 : 3;
    }

    Vec3<double> out = Vec3<double>::Zero();
    for (int i = 0; i < 3; ++i) {
        out[i] = firstWrench[static_cast<std::size_t>(offset + i)];
    }
    return out;
}

void accumulateFootContacts(const mjModel* model,
                            const mjData* data,
                            const MujocoEndEffectorBinding& footBinding,
                            FootContactResult& result) {
    for (int contactIndex = 0; contactIndex < data->ncon; ++contactIndex) {
        const mjContact& contact = data->contact[contactIndex];
        const int geom1Body = model->geom_bodyid[contact.geom1];
        const int geom2Body = model->geom_bodyid[contact.geom2];
        const bool footIsGeom1 = geom1Body == footBinding.bodyId;
        const bool footIsGeom2 = geom2Body == footBinding.bodyId;
        if (!footIsGeom1 && !footIsGeom2) {
            continue;
        }

        mjtNum contactForceLocal[6] = {};
        mj_contactForce(model, data, contactIndex, contactForceLocal);

        const Mat3<double> frameRows_W = contactFrameRowsWorld(contact);
        const Vec3<double> forceNormalPositive_W =
            frameRows_W.transpose() *
            Vec3<double>(static_cast<double>(contactForceLocal[0]),
                         static_cast<double>(contactForceLocal[1]),
                         static_cast<double>(contactForceLocal[2]));
        const Vec3<double> momentNormalPositive_W =
            frameRows_W.transpose() *
            Vec3<double>(static_cast<double>(contactForceLocal[3]),
                         static_cast<double>(contactForceLocal[4]),
                         static_cast<double>(contactForceLocal[5]));

        // MuJoCo contact normal points from geom1 to geom2. Treat the positive
        // contact-frame wrench as acting on geom2, and negate it for geom1.
        const double signForFoot = footIsGeom2 ? 1.0 : -1.0;
        const Vec3<double> forceOnFoot_W = signForFoot * forceNormalPositive_W;
        const Vec3<double> momentOnFootAtContact_W = signForFoot * momentNormalPositive_W;
        const Vec3<double> contactPoint_W = readMujocoVec3(contact.pos);

        addContactWrench(result.atFootCollisionCenter,
                         forceOnFoot_W,
                         momentOnFootAtContact_W,
                         contactPoint_W,
                         result.footCollisionCenter_W);
        addContactWrench(result.atFootSite,
                         forceOnFoot_W,
                         momentOnFootAtContact_W,
                         contactPoint_W,
                         result.footSite_W);
    }
}

void writeVec3Row(std::ostream& out, const std::string& label, const Vec3<double>& value) {
    out << "| `" << label << "` | "
        << value[0] << " | " << value[1] << " | " << value[2] << " |\n";
}

void writeVec3TableHeader(std::ostream& out) {
    out << "| Quantity | x | y | z |\n"
        << "| --- | ---: | ---: | ---: |\n";
}

void writeWrenchComparison(std::ostream& out,
                           const std::string& label,
                           const WrenchAtPoint& measured,
                           const Vec3<double>& desiredForce,
                           const Vec3<double>& desiredMoment,
                           const bool sameMomentReference,
                           const std::string& desiredReferencePoint) {
    out << "### " << label << "\n\n"
        << "| Field | Value |\n"
        << "| --- | ---: |\n"
        << "| contacts | " << measured.contactCount << " |\n\n";
    writeVec3TableHeader(out);
    writeVec3Row(out, "force_W", measured.force_W);
    writeVec3Row(out, "force_error_W", measured.force_W - desiredForce);
    writeVec3Row(out, "moment_W", measured.moment_W);
    if (sameMomentReference) {
        writeVec3Row(out, "moment_error_W", measured.moment_W - desiredMoment);
    } else {
        out << "| `moment_error_W` | n/a | n/a | n/a |\n\n"
            << "Moment error is not computed here because the desired moment is about `"
            << desiredReferencePoint << "`.\n";
    }
    out << "\n";
}

void writeCsvRowsForVec3(std::ostream& out,
                         const std::string& side,
                         const std::string& measurementPoint,
                         const std::string& quantity,
                         const Vec3<double>& desired,
                         const Vec3<double>& measured,
                         const int contactCount,
                         const bool comparable) {
    const char* axisNames[3] = {"x", "y", "z"};
    for (int axis = 0; axis < 3; ++axis) {
        out << side << ','
            << measurementPoint << ','
            << quantity << ','
            << axisNames[axis] << ',';
        if (comparable) {
            out << desired[axis] << ','
                << measured[axis] << ','
                << measured[axis] - desired[axis] << ',';
        } else {
            out << ','
                << measured[axis] << ','
                << ',';
        }
        out << contactCount << ','
            << (comparable ? 1 : 0)
            << '\n';
    }
}

void writePlotCsv(std::ostream& out,
                  const std::filesystem::path& logPath,
                  const std::string& robotType,
                  const std::string& locomotionMode,
                  const std::string& desiredReferencePoint,
                  const std::vector<FootContactResult>& results) {
    out << std::setprecision(17);
    out << "# source_json_file=" << logPath.filename().string() << '\n';
    out << "# source_json_path=" << std::filesystem::absolute(logPath).string() << '\n';
    out << "# robot_type=" << robotType << '\n';
    out << "# locomotion_mode=" << locomotionMode << '\n';
    out << "side,measurement_point,quantity,axis,desired,measured,error,contacts,comparable\n";

    for (const FootContactResult& result : results) {
        writeCsvRowsForVec3(out,
                            result.side,
                            "foot_site",
                            "force",
                            result.desiredForce_W,
                            result.atFootSite.force_W,
                            result.atFootSite.contactCount,
                            true);
        writeCsvRowsForVec3(out,
                            result.side,
                            "foot_collision_geom_center",
                            "force",
                            result.desiredForce_W,
                            result.atFootCollisionCenter.force_W,
                            result.atFootCollisionCenter.contactCount,
                            true);
        writeCsvRowsForVec3(out,
                            result.side,
                            "foot_site",
                            "moment",
                            result.desiredMoment_W,
                            result.atFootSite.moment_W,
                            result.atFootSite.contactCount,
                            desiredReferencePoint == "foot_site");
        writeCsvRowsForVec3(out,
                            result.side,
                            "foot_collision_geom_center",
                            "moment",
                            result.desiredMoment_W,
                            result.atFootCollisionCenter.moment_W,
                            result.atFootCollisionCenter.contactCount,
                            desiredReferencePoint == "foot_collision_geom_center");
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

void runPlotScript(const std::filesystem::path& csvPath, const std::filesystem::path& plotPath) {
    const std::filesystem::path scriptPath =
        std::filesystem::path(PROJECT_ROOT_DIR) / "test" / "standing_debug" / "plot_contact_probe.py";
    const char* pythonEnv = std::getenv("PYTHON");
    const std::string python = (pythonEnv != nullptr && pythonEnv[0] != '\0') ? pythonEnv : "python";
    const std::string command =
        python + " " + shellQuote(scriptPath) + " " + shellQuote(csvPath) + " " + shellQuote(plotPath);
    const int status = std::system(command.c_str());
    if (status == 0) {
        std::cout << "plot: " << std::filesystem::absolute(plotPath).string() << "\n";
    } else {
        std::cerr << "plot: failed to run " << scriptPath
                  << " with exit status " << status << "\n";
    }
}

void writeReport(std::ostream& out,
                 const std::filesystem::path& logPath,
                 const std::string& modelPath,
                 const int totalContacts,
                 const double maxCtrlClampDelta,
                 const std::string& robotType,
                 const std::string& locomotionMode,
                 const std::string& footSource,
                 const std::string& desiredReferencePoint,
                 const std::string& contactWrenchModel,
                 const std::vector<FootContactResult>& results) {
    out << std::fixed << std::setprecision(9);
    out << "# Contact Probe Report\n\n"
        << "## Source\n\n"
        << "| Field | Value |\n"
        << "| --- | --- |\n"
        << "| Source log | `" << std::filesystem::absolute(logPath).string() << "` |\n"
        << "| Robot type | `" << robotType << "` |\n"
        << "| Locomotion mode | `" << locomotionMode << "` |\n"
        << "| MuJoCo model | `" << modelPath << "` |\n"
        << "| Foot end-effector source | `" << footSource << "` |\n"
        << "| Desired wrench reference point | `" << desiredReferencePoint << "` |\n"
        << "| Contact wrench model | `" << contactWrenchModel << "` |\n\n"
        << "## Method\n\n"
        << "The probe restores the logged generalized state, applies `full_tau_command`, "
        << "runs `mj_forward`, and compares the MuJoCo contact wrench against the first MPC wrench.\n\n"
        << "$$\n"
        << "e_F = F_{measured} - F_{desired}, \\qquad "
        << "e_M = M_{measured} - M_{desired}\n"
        << "$$\n\n"
        << "Positive contact-frame wrench is treated as acting on `geom2`; the report flips the sign "
        << "when the foot is `geom1`.\n\n"
        << "## Summary\n\n"
        << "| Metric | Value |\n"
        << "| --- | ---: |\n"
        << "| total contacts | " << totalContacts << " |\n"
        << "| max ctrl clamp delta | " << maxCtrlClampDelta << " |\n\n";

    for (const FootContactResult& result : results) {
        out << "## " << result.side << " foot\n\n"
            << "| Field | Value |\n"
            << "| --- | ---: |\n"
            << "| body id | " << result.footBodyId << " |\n"
            << "| site id | " << result.footSiteId << " |\n\n"
            << "### Desired Wrench\n\n";
        writeVec3TableHeader(out);
        writeVec3Row(out, "desired_force_W", result.desiredForce_W);
        writeVec3Row(out, "desired_moment_W_about_" + desiredReferencePoint,
                     result.desiredMoment_W);
        out << "\n";

        writeWrenchComparison(out,
                              "Measured at foot site",
                              result.atFootSite,
                              result.desiredForce_W,
                              result.desiredMoment_W,
                              desiredReferencePoint == "foot_site",
                              desiredReferencePoint);
        writeWrenchComparison(out,
                              "Measured at foot collision-geom center",
                              result.atFootCollisionCenter,
                              result.desiredForce_W,
                              result.desiredMoment_W,
                              desiredReferencePoint == "foot_collision_geom_center",
                              desiredReferencePoint);
    }
}
}  // namespace

int main(int argc, char** argv) {
    if (argc > 2 || (argc >= 2 && std::string(argv[1]) == "--help")) {
        std::cout << "Usage: stand_contact_probe [mpc_debug.json]\n"
                  << "  If the log path is omitted, the latest standing_mpc or walking_mpc log is used.\n";
        return (argc > 2) ? EXIT_FAILURE : EXIT_SUCCESS;
    }

    const std::filesystem::path logPath =
        (argc >= 2) ? std::filesystem::path(argv[1]) : latestDebugLogPath();

    std::ifstream logStream(logPath);
    if (!logStream.is_open()) {
        throw std::runtime_error("Failed to open debug log: " + logPath.string());
    }

    json log;
    logStream >> log;

    const json metadata = log.value("metadata", json::object());
    const json* controllerConfig = nullptr;
    if (log.contains("controller_config") && log.at("controller_config").is_object()) {
        controllerConfig = &log.at("controller_config");
    }

    const std::string robotTypeLabel = controllerConfig != nullptr
                                           ? readJsonStringOr(*controllerConfig, "robot_type",
                                                             readJsonStringOr(metadata, "robot_type",
                                                                              "unknown"))
                                           : readJsonStringOr(metadata, "robot_type", "unknown");
    const RobotType robotType = robotTypeFromString(robotTypeLabel);
    const std::string locomotionMode =
        normalizedLocomotionMode(readJsonStringOr(metadata, "locomotion_mode", "standing"));
    const json* controllerModel = nullptr;
    if (controllerConfig != nullptr && controllerConfig->contains("model") &&
        controllerConfig->at("model").is_object()) {
        controllerModel = &controllerConfig->at("model");
    }

    const RobotRuntimeConfig& runtimeConfig = getRobotRuntimeConfig(robotType);
    const std::string footSourceLabelFallback =
        readJsonStringOr(metadata,
                         "foot_end_effector_source",
                         runtimeConfig.footEndEffectorSource == FootEndEffectorSource::Site
                             ? "site"
                             : "collision_geom_center");
    const std::string modelPath = controllerModel != nullptr
                                      ? resolveModelPathOr(runtimeConfig.modelXmlPath,
                                                           readJsonStringOr(*controllerModel,
                                                                            "xml_path", ""))
                                      : runtimeConfig.modelXmlPath;
    const FootEndEffectorSource footSource = controllerModel != nullptr
                                                 ? footEndEffectorSourceFromString(
                                                       readJsonStringOr(*controllerModel,
                                                                        "foot_end_effector_source",
                                                                        footSourceLabelFallback))
                                                 : runtimeConfig.footEndEffectorSource;
    const std::string footSourceLabel =
        controllerModel != nullptr ? readJsonStringOr(*controllerModel,
                                                      "foot_end_effector_source",
                                                      footSourceLabelFallback)
                                   : footSourceLabelFallback;
    const std::string desiredReferencePoint =
        readJsonStringOr(metadata, "desired_wrench_reference_point", "unknown");
    const std::string contactWrenchModel =
        readJsonStringOr(metadata, "contact_wrench_model", "unknown");
    const ContactProbeOutputPaths outputPaths = defaultOutputPaths(locomotionMode);

    const std::vector<double> qpos =
        readJsonVector(log.at("robot_state").at("full_qpos"), "robot_state.full_qpos");
    const std::vector<double> qvel =
        readJsonVector(log.at("robot_state").at("full_qvel"), "robot_state.full_qvel");
    const std::vector<double> tauCommand =
        readJsonVector(log.at("robot_state").at("full_tau_command"), "robot_state.full_tau_command");
    const std::vector<double> firstWrench =
        readJsonVector(log.at("solution").at("first_wrench"), "solution.first_wrench");

    if (mjVERSION_HEADER != mj_version()) {
        throw std::runtime_error("MuJoCo header/library version mismatch");
    }

    std::array<char, 1024> error{};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_loadXML(modelPath.c_str(), nullptr, error.data(), error.size()),
        mj_deleteModel);
    if (!model) {
        throw std::runtime_error("mj_loadXML failed for " + modelPath + ": " + error.data());
    }

    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
    if (!data) {
        throw std::runtime_error("mj_makeData failed");
    }

    const auto robotSetup =
        setupRobotParams<double>(robotType, model.get(), footSource);
    const RobotParams<double>& params = robotSetup.params;
    const MujocoRobotBindings& bindings = robotSetup.bindings;
    if (params.legs.size() != bindings.feet.size()) {
        throw std::runtime_error("Robot params and MuJoCo foot bindings do not match");
    }

    copyVectorToMujoco(qpos, data->qpos, model->nq, "robot_state.full_qpos");
    copyVectorToMujoco(qvel, data->qvel, model->nv, "robot_state.full_qvel");
    mj_normalizeQuat(model.get(), data->qpos);
    const std::vector<double> appliedCtrl = applyCtrlWithClamp(model.get(), data.get(), tauCommand);
    mj_forward(model.get(), data.get());

    double maxCtrlClampDelta = 0.0;
    for (std::size_t i = 0; i < appliedCtrl.size(); ++i) {
        maxCtrlClampDelta = std::max(maxCtrlClampDelta, std::abs(appliedCtrl[i] - tauCommand[i]));
    }

    std::vector<FootContactResult> results;
    results.reserve(params.legs.size());
    for (std::size_t leg = 0; leg < params.legs.size(); ++leg) {
        const LegParams<double>& legParams = params.legs[leg];
        const MujocoEndEffectorBinding& footBinding = bindings.feet[leg];

        FootContactResult result;
        result.side = sideName(legParams.side);
        result.footBodyId = footBinding.bodyId;
        result.footSiteId = footBinding.siteId;
        result.footCollisionCenter_W =
            readCollisionGeomCenterWorld(data.get(), footBinding.collisionGeomIds);
        result.footSite_W = readSiteWorld(data.get(), footBinding.siteId);
        result.desiredForce_W = vec3FromWrenchBlock(firstWrench, legParams.side, false);
        result.desiredMoment_W = vec3FromWrenchBlock(firstWrench, legParams.side, true);

        accumulateFootContacts(model.get(), data.get(), footBinding, result);
        results.push_back(result);
    }

    writeReport(std::cout,
                logPath,
                modelPath,
                data->ncon,
                maxCtrlClampDelta,
                robotTypeLabel,
                locomotionMode,
                footSourceLabel,
                desiredReferencePoint,
                contactWrenchModel,
                results);

    std::ofstream report(outputPaths.report, std::ios::out | std::ios::trunc);
    if (!report.is_open()) {
        throw std::runtime_error("Failed to open report output: " + outputPaths.report.string());
    }
    writeReport(report,
                logPath,
                modelPath,
                data->ncon,
                maxCtrlClampDelta,
                robotTypeLabel,
                locomotionMode,
                footSourceLabel,
                desiredReferencePoint,
                contactWrenchModel,
                results);
    std::cout << "report: " << std::filesystem::absolute(outputPaths.report).string() << "\n";

    std::ofstream csv(outputPaths.csv, std::ios::out | std::ios::trunc);
    if (!csv.is_open()) {
        throw std::runtime_error("Failed to open plot CSV output: " + outputPaths.csv.string());
    }
    writePlotCsv(csv, logPath, robotTypeLabel, locomotionMode, desiredReferencePoint, results);
    csv.close();
    if (!csv.good()) {
        throw std::runtime_error("Failed while writing plot CSV output: " + outputPaths.csv.string());
    }
    std::cout << "csv: " << std::filesystem::absolute(outputPaths.csv).string() << "\n";
    runPlotScript(outputPaths.csv, outputPaths.plot);

    return EXIT_SUCCESS;
}
