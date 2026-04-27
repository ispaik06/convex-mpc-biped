#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "ControllerConfig.h"
#include "MujocoCheaterStateReader.h"
#include "RobotConfig.h"
#include "SimulationConfig.h"
#include "StateEstimator/StateEstimator.h"
#include "Utilities/MatrixUtils.h"
#include "setupRobotParams.h"

namespace {
constexpr const char* kInitialKeyframeName = "copied_state";

Vec3<double> readSitePosition(const mjData* data, const int siteId) {
    if (siteId < 0) {
        throw std::runtime_error("Invalid site id");
    }

    Vec3<double> pos = Vec3<double>::Zero();
    for (int i = 0; i < 3; ++i) {
        pos[i] = static_cast<double>(data->site_xpos[3 * siteId + i]);
    }
    return pos;
}

Vec3<double> readCollisionGeomCenterPosition(const mjData* data,
                                             const std::vector<int>& geomIds) {
    if (geomIds.empty()) {
        throw std::runtime_error("No collision geom ids are bound");
    }

    Vec3<double> pos = Vec3<double>::Zero();
    for (const int geomId : geomIds) {
        if (geomId < 0) {
            throw std::runtime_error("Invalid geom id");
        }
        for (int i = 0; i < 3; ++i) {
            pos[i] += static_cast<double>(data->geom_xpos[3 * geomId + i]);
        }
    }
    return pos / static_cast<double>(geomIds.size());
}

std::string sideName(const Side side) {
    switch (side) {
        case Side::Left:
            return "left";
        case Side::Right:
            return "right";
        default:
            return "unknown";
    }
}

void applyCopiedStateQpos(const mjModel* model, mjData* data) {
    const int keyId = mj_name2id(model, mjOBJ_KEY, kInitialKeyframeName);
    if (keyId < 0) {
        throw std::runtime_error(std::string("Failed to find MuJoCo keyframe: ") +
                                 kInitialKeyframeName);
    }

    mj_resetData(model, data);
    const mjtNum* keyQpos = model->key_qpos + keyId * model->nq;
    for (int i = 0; i < model->nq; ++i) {
        data->qpos[i] = keyQpos[i];
    }
    mj_forward(model, data);
}
}  // namespace

int main(int argc, char** argv) {
    if (argc < 2 || argc > 3) {
        std::cout << "Usage: main_left_swing_hold_nominal_test [robot-id] [csv-path]\n"
                  << "\t robot-id: m for MIT humanoid\n"
                  << "\t csv-path: optional output csv path (default: build/left_swing_hold_nominal_pose.csv)\n";
        return EXIT_FAILURE;
    }

    if (argv[1][0] != 'm') {
        std::cout << "Only MIT humanoid is supported by this probe.\n";
        return EXIT_FAILURE;
    }

    const std::string csvPath = (argc == 3)
                                    ? std::string(argv[2])
                                    : std::string(PROJECT_ROOT_DIR) + "/build/left_swing_hold_nominal_pose.csv";

    setActiveRobotType(RobotType::MIT_HUMANOID);
    const auto& controllerConfig = getControllerConfig(RobotType::MIT_HUMANOID);
    const auto& runtimeConfig = getRobotRuntimeConfig(RobotType::MIT_HUMANOID);
    const std::string modelPath = resolveProjectPath(
        controllerConfig.leftSwingHoldTest.xmlPath.empty()
            ? runtimeConfig.modelXmlPath
            : controllerConfig.leftSwingHoldTest.xmlPath);
    if (mjVERSION_HEADER != mj_version()) {
        throw std::runtime_error("MuJoCo header/library version mismatch");
    }

    std::array<char, 1024> error{};
    mjModel* model = mj_loadXML(modelPath.c_str(), nullptr, error.data(), error.size());
    if (model == nullptr) {
        throw std::runtime_error("mj_loadXML failed for " + modelPath + ": " + error.data());
    }

    mjData* data = mj_makeData(model);
    if (data == nullptr) {
        mj_deleteModel(model);
        throw std::runtime_error("mj_makeData failed");
    }

    try {
        configureSimulationModel(model);

        auto robotSetup =
            setupRobotParams<double>(RobotType::MIT_HUMANOID, model, FootEndEffectorSource::Site);
        RobotParams<double>& params = robotSetup.params;
        const MujocoRobotBindings& bindings = robotSetup.bindings;

        if (bindings.feet.size() != params.legs.size()) {
            throw std::runtime_error("Mujoco bindings do not match leg count");
        }

        CheaterState<double> cheaterState;
        StateEstimate<double> stateEstimate;
        cheaterState.resize(params);
        stateEstimate.resize(params);

        applyCopiedStateQpos(model, data);
        updateReducedBodyMassPropertiesFromData(model, data, bindings, params);
        fillCheaterState(model, data, params, bindings, cheaterState);
        stateEstimate.copyFrom(cheaterState);
        const Mat3<double> R_WT = stateEstimate.torsoQuat_W.toRotationMatrix();
        stateEstimate.psi = std::atan2(R_WT(1, 0), R_WT(0, 0));

        const auto& config = getControllerConfig();
        const Mat3<double> R_WB = Rz(stateEstimate.psi);
        const Mat3<double> R_BW = R_WB.transpose();
        const Vec3<double> p_com_W = stateEstimate.torsoPos_W + R_WB * params.bodyComLocation;

        std::ofstream csv(csvPath, std::ios::out | std::ios::trunc);
        if (!csv.is_open()) {
            throw std::runtime_error("Failed to open CSV output at " + csvPath);
        }
        csv << "leg,"
               "site_Wx,site_Wy,site_Wz,"
               "collision_geom_center_Wx,collision_geom_center_Wy,collision_geom_center_Wz,"
               "site_Bx,site_By,site_Bz,"
               "collision_geom_center_Bx,collision_geom_center_By,collision_geom_center_Bz,"
               "legacy_nominal_Bx,legacy_nominal_By,legacy_nominal_Bz,"
               "delta_site_legacy_x,delta_site_legacy_y,delta_site_legacy_z,"
               "delta_collision_geom_center_legacy_x,delta_collision_geom_center_legacy_y,delta_collision_geom_center_legacy_z\n";

        std::cout << std::fixed << std::setprecision(9);
        std::cout << "[LeftSwingHoldNominal] state from copied_state qpos\n";
        std::cout << "  model: " << modelPath << "\n";
        std::cout << "  key:   " << kInitialKeyframeName << "\n";
        std::cout << "  csv:   " << csvPath << "\n";
        std::cout << "  sim_time: " << stateEstimate.time << "\n\n";

        for (std::size_t leg = 0; leg < params.legs.size(); ++leg) {
            const auto& legParams = params.legs[leg];
            const auto& footBinding = bindings.feet[leg];

            const Vec3<double> sitePos_W = readSitePosition(data, footBinding.siteId);
            const Vec3<double> collisionCenter_W =
                readCollisionGeomCenterPosition(data, footBinding.collisionGeomIds);
            const Vec3<double> sitePos_B = R_BW * (sitePos_W - p_com_W);
            const Vec3<double> collisionCenter_B = R_BW * (collisionCenter_W - p_com_W);

            const Vec3<double> hipWorld =
                stateEstimate.torsoPos_W +
                stateEstimate.torsoQuat_W.toRotationMatrix() * legParams.hipLocationFromBody;
            Vec3<double> legacyNominal_B = R_BW * (hipWorld - p_com_W);
            legacyNominal_B[1] +=
                (legParams.side == Side::Left ? 1.0 : -1.0) * config.footPlacement.nominalLateralOffset;

            const Vec3<double> deltaSite = sitePos_B - legacyNominal_B;
            const Vec3<double> deltaCollisionCenter = collisionCenter_B - legacyNominal_B;

            const Vec3<double> reconstructedSite_W = p_com_W + R_WB * sitePos_B;
            if ((reconstructedSite_W - sitePos_W).norm() > 1e-9) {
                throw std::runtime_error("Foot pose round-trip check failed");
            }

            std::cout << "[" << sideName(legParams.side) << "]\n";
            std::cout << "  site_W                 = " << sitePos_W.transpose() << "\n";
            std::cout << "  collision_center_W     = " << collisionCenter_W.transpose() << "\n";
            std::cout << "  site_B                 = " << sitePos_B.transpose() << "\n";
            std::cout << "  collision_center_B     = " << collisionCenter_B.transpose() << "\n";
            std::cout << "  legacy_nominal_B       = " << legacyNominal_B.transpose() << "\n";
            std::cout << "  delta_site_legacy      = " << deltaSite.transpose() << "\n";
            std::cout << "  delta_collision_legacy = " << deltaCollisionCenter.transpose() << "\n\n";

            csv << sideName(legParams.side) << ","
                << sitePos_W.x() << ","
                << sitePos_W.y() << ","
                << sitePos_W.z() << ","
                << collisionCenter_W.x() << ","
                << collisionCenter_W.y() << ","
                << collisionCenter_W.z() << ","
                << sitePos_B.x() << ","
                << sitePos_B.y() << ","
                << sitePos_B.z() << ","
                << collisionCenter_B.x() << ","
                << collisionCenter_B.y() << ","
                << collisionCenter_B.z() << ","
                << legacyNominal_B.x() << ","
                << legacyNominal_B.y() << ","
                << legacyNominal_B.z() << ","
                << deltaSite.x() << ","
                << deltaSite.y() << ","
                << deltaSite.z() << ","
                << deltaCollisionCenter.x() << ","
                << deltaCollisionCenter.y() << ","
                << deltaCollisionCenter.z() << "\n";
        }

        csv.close();

        mj_deleteData(data);
        mj_deleteModel(model);
        return EXIT_SUCCESS;
    } catch (...) {
        mj_deleteData(data);
        mj_deleteModel(model);
        throw;
    }
}
