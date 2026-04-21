#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
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
    Vec3<double> footCom_W = Vec3<double>::Zero();
    Vec3<double> footSite_W = Vec3<double>::Zero();
    Vec3<double> desiredForce_W = Vec3<double>::Zero();
    Vec3<double> desiredMoment_W = Vec3<double>::Zero();
    WrenchAtPoint atFootCom;
    WrenchAtPoint atFootSite;
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

std::filesystem::path defaultReportPath() {
    const std::filesystem::path dir =
        std::filesystem::path(PROJECT_ROOT_DIR) / "logs" / "debug" / "standing_mpc" / "contact_probe";
    std::filesystem::create_directories(dir);
    return dir / ("stand_contact_probe_" + timestampToken() + ".txt");
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

std::string readJsonStringOr(const json& value,
                             const std::string& key,
                             const std::string& fallback) {
    if (!value.is_object() || !value.contains(key) || !value.at(key).is_string()) {
        return fallback;
    }
    return value.at(key).get<std::string>();
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

Vec3<double> readBodyComWorld(const mjData* data, const int bodyId) {
    return readMujocoVec3(data->xipos + 3 * bodyId);
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

        addContactWrench(result.atFootCom,
                         forceOnFoot_W,
                         momentOnFootAtContact_W,
                         contactPoint_W,
                         result.footCom_W);
        addContactWrench(result.atFootSite,
                         forceOnFoot_W,
                         momentOnFootAtContact_W,
                         contactPoint_W,
                         result.footSite_W);
    }
}

void printVec3(std::ostream& out, const std::string& label, const Vec3<double>& value) {
    out << "    " << label << ": ["
        << value[0] << ", " << value[1] << ", " << value[2] << "]\n";
}

void printWrenchComparison(std::ostream& out,
                           const std::string& label,
                           const WrenchAtPoint& measured,
                           const Vec3<double>& desiredForce,
                           const Vec3<double>& desiredMoment,
                           const bool sameMomentReference,
                           const std::string& desiredReferencePoint) {
    out << "  " << label << " contacts=" << measured.contactCount << "\n";
    printVec3(out, "force_W", measured.force_W);
    printVec3(out, "force_error_W", measured.force_W - desiredForce);
    printVec3(out, "moment_W", measured.moment_W);
    if (sameMomentReference) {
        printVec3(out, "moment_error_W", measured.moment_W - desiredMoment);
    } else {
        out << "    moment_error_W: not computed; desired moment is about "
            << desiredReferencePoint << "\n";
    }
}

void writeReport(std::ostream& out,
                 const std::filesystem::path& logPath,
                 const std::string& modelPath,
                 const int totalContacts,
                 const double maxCtrlClampDelta,
                 const std::string& footSource,
                 const std::string& desiredReferencePoint,
                 const std::vector<FootContactResult>& results) {
    out << std::fixed << std::setprecision(9);
    out << "[stand_contact_probe]\n"
        << "  log: " << std::filesystem::absolute(logPath).string() << "\n"
        << "  model: " << modelPath << "\n"
        << "  total_contacts: " << totalContacts << "\n"
        << "  max_ctrl_clamp_delta: " << maxCtrlClampDelta << "\n"
        << "  foot_end_effector_source: " << footSource << "\n"
        << "  desired_wrench_reference_point: " << desiredReferencePoint << "\n"
        << "  sign convention: positive contact-frame wrench is treated as acting on geom2\n\n";

    for (const FootContactResult& result : results) {
        out << result.side << " foot"
            << " body_id=" << result.footBodyId
            << " site_id=" << result.footSiteId << "\n";
        printVec3(out, "desired_force_W", result.desiredForce_W);
        printVec3(out, "desired_moment_W_about_" + desiredReferencePoint, result.desiredMoment_W);
        printWrenchComparison(out,
                              "measured_at_foot_site",
                              result.atFootSite,
                              result.desiredForce_W,
                              result.desiredMoment_W,
                              desiredReferencePoint == "foot_site",
                              desiredReferencePoint);
        printWrenchComparison(out,
                              "measured_at_foot_link_com",
                              result.atFootCom,
                              result.desiredForce_W,
                              result.desiredMoment_W,
                              desiredReferencePoint == "foot_link_com",
                              desiredReferencePoint);
        out << "\n";
    }
}
}  // namespace

int main(int argc, char** argv) {
    if (argc > 2 || (argc >= 2 && std::string(argv[1]) == "--help")) {
        std::cout << "Usage: stand_contact_probe [standing_mpc_debug.json]\n"
                  << "  If the log path is omitted, the latest logs/debug/standing_mpc log is used.\n";
        return (argc > 2) ? EXIT_FAILURE : EXIT_SUCCESS;
    }

    const std::filesystem::path logPath =
        (argc >= 2) ? std::filesystem::path(argv[1]) : latestDebugLogPath();
    const std::filesystem::path reportPath = defaultReportPath();

    std::ifstream logStream(logPath);
    if (!logStream.is_open()) {
        throw std::runtime_error("Failed to open debug log: " + logPath.string());
    }

    json log;
    logStream >> log;

    const json metadata = log.value("metadata", json::object());
    const std::string footSource =
        readJsonStringOr(metadata, "foot_end_effector_source", "unknown");
    const std::string desiredReferencePoint =
        readJsonStringOr(metadata, "desired_wrench_reference_point", "unknown");

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

    const std::string modelPath = std::string(PROJECT_ROOT_DIR) + "/models/mit_humanoid/scene.xml";
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
        setupRobotParams<double>(RobotType::MIT_HUMANOID, model.get(), FootEndEffectorSource::Site);
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
        result.footCom_W = readBodyComWorld(data.get(), footBinding.bodyId);
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
                footSource,
                desiredReferencePoint,
                results);

    std::ofstream report(reportPath, std::ios::out | std::ios::trunc);
    if (!report.is_open()) {
        throw std::runtime_error("Failed to open report output: " + reportPath.string());
    }
    writeReport(report,
                logPath,
                modelPath,
                data->ncon,
                maxCtrlClampDelta,
                footSource,
                desiredReferencePoint,
                results);
    std::cout << "report: " << std::filesystem::absolute(reportPath).string() << "\n";

    return EXIT_SUCCESS;
}
