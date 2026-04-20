#include <algorithm>
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
#include "RobotController.h"
#include "RobotRunner.h"
#include "StateEstimator/StateEstimator.h"
#include "Utilities/MatrixUtils.h"
#include "setupRobotParams.h"

namespace {
class NominalProbeController : public RobotController {
public:
    NominalProbeController() {
        setFootEndEffectorSource(FootEndEffectorSource::Site);
    }

    void initializeController() override {}
    void runController() override {}
};

void applyRobotCommand(const mjModel* model, mjData* data, const RobotCommand<double>& command) {
    if (model == nullptr || data == nullptr) {
        throw std::runtime_error("applyRobotCommand received null MuJoCo pointers");
    }
    if (command.tau.size() != model->nu) {
        throw std::runtime_error("RobotCommand torque dimension does not match model->nu");
    }

    for (int i = 0; i < model->nu; ++i) {
        const double tau = command.tau[i];
        if (model->actuator_ctrllimited[i]) {
            const double lo = static_cast<double>(model->actuator_ctrlrange[2 * i + 0]);
            const double hi = static_cast<double>(model->actuator_ctrlrange[2 * i + 1]);
            data->ctrl[i] = std::clamp(tau, lo, hi);
        } else {
            data->ctrl[i] = tau;
        }
    }
}

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

Vec3<double> readBodyComPosition(const mjData* data, const int bodyId) {
    if (bodyId < 0) {
        throw std::runtime_error("Invalid body id");
    }

    Vec3<double> pos = Vec3<double>::Zero();
    for (int i = 0; i < 3; ++i) {
        pos[i] = static_cast<double>(data->xipos[3 * bodyId + i]);
    }
    return pos;
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

    const std::string modelPath = std::string(PROJECT_ROOT_DIR) + "/models/mit_humanoid/scene.xml";
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
        model->opt.timestep = 0.002;
        model->opt.integrator = mjINT_IMPLICITFAST;

        auto robotSetup =
            setupRobotParams<double>(RobotType::MIT_HUMANOID, model, FootEndEffectorSource::Site);
        RobotParams<double>& params = robotSetup.params;
        const MujocoRobotBindings& bindings = robotSetup.bindings;

        if (bindings.feet.size() != params.legs.size()) {
            throw std::runtime_error("Mujoco bindings do not match leg count");
        }

        mj_forward(model, data);

        NominalProbeController controller;
        RobotRunner robotRunner(&controller);
        UserCommand userCommand;
        robotRunner.init(&params, model->opt.timestep, &userCommand);

        CheaterState<double> cheaterState;
        StateEstimate<double> stateEstimate;
        RobotCommand<double> robotCommand;
        cheaterState.resize(params);
        stateEstimate.resize(params);
        robotCommand.resize(model->nu);

        std::size_t stepCount = 0;
        constexpr std::size_t kMaxInitSteps = 10000;
        while (!robotRunner.legInitializationComplete()) {
            fillCheaterState(model, data, params, bindings, cheaterState);
            stateEstimate.copyFrom(cheaterState);
            const Mat3<double> R_WT_step = stateEstimate.torsoQuat_W.toRotationMatrix();
            stateEstimate.psi = std::atan2(R_WT_step(1, 0), R_WT_step(0, 0));

            robotRunner.run(stateEstimate, robotCommand);
            applyRobotCommand(model, data, robotCommand);
            mj_step(model, data);

            ++stepCount;
            if (stepCount > kMaxInitSteps) {
                throw std::runtime_error("LegPosInitializer did not complete within the step limit");
            }
        }

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
               "foot_link_com_Wx,foot_link_com_Wy,foot_link_com_Wz,"
               "site_Bx,site_By,site_Bz,"
               "foot_link_com_Bx,foot_link_com_By,foot_link_com_Bz,"
               "legacy_nominal_Bx,legacy_nominal_By,legacy_nominal_Bz,"
               "delta_site_legacy_x,delta_site_legacy_y,delta_site_legacy_z,"
               "delta_foot_link_com_legacy_x,delta_foot_link_com_legacy_y,delta_foot_link_com_legacy_z\n";

        std::cout << std::fixed << std::setprecision(9);
        std::cout << "[LeftSwingHoldNominal] state after LegPosInitializer completion\n";
        std::cout << "  model: " << modelPath << "\n";
        std::cout << "  csv:   " << csvPath << "\n";
        std::cout << "  init_steps: " << stepCount << ", sim_time: " << stateEstimate.time << "\n\n";

        for (std::size_t leg = 0; leg < params.legs.size(); ++leg) {
            const auto& legParams = params.legs[leg];
            const auto& footBinding = bindings.feet[leg];

            const Vec3<double> sitePos_W = readSitePosition(data, footBinding.siteId);
            const Vec3<double> footLinkCom_W = readBodyComPosition(data, footBinding.bodyId);
            const Vec3<double> sitePos_B = R_BW * (sitePos_W - p_com_W);
            const Vec3<double> footLinkCom_B = R_BW * (footLinkCom_W - p_com_W);

            const Vec3<double> hipWorld =
                stateEstimate.torsoPos_W +
                stateEstimate.torsoQuat_W.toRotationMatrix() * legParams.hipLocationFromBody;
            Vec3<double> legacyNominal_B = R_BW * (hipWorld - p_com_W);
            legacyNominal_B[1] +=
                (legParams.side == Side::Left ? 1.0 : -1.0) * config.footPlacement.nominalLateralOffset;

            const Vec3<double> deltaSite = sitePos_B - legacyNominal_B;
            const Vec3<double> deltaFootLinkCom = footLinkCom_B - legacyNominal_B;

            const Vec3<double> reconstructedSite_W = p_com_W + R_WB * sitePos_B;
            if ((reconstructedSite_W - sitePos_W).norm() > 1e-9) {
                throw std::runtime_error("Foot pose round-trip check failed");
            }

            std::cout << "[" << sideName(legParams.side) << "]\n";
            std::cout << "  site_W                 = " << sitePos_W.transpose() << "\n";
            std::cout << "  foot_link_com_W        = " << footLinkCom_W.transpose() << "\n";
            std::cout << "  site_B                 = " << sitePos_B.transpose() << "\n";
            std::cout << "  foot_link_com_B        = " << footLinkCom_B.transpose() << "\n";
            std::cout << "  legacy_nominal_B       = " << legacyNominal_B.transpose() << "\n";
            std::cout << "  delta_site_legacy      = " << deltaSite.transpose() << "\n";
            std::cout << "  delta_footcom_legacy   = " << deltaFootLinkCom.transpose() << "\n\n";

            csv << sideName(legParams.side) << ","
                << sitePos_W.x() << ","
                << sitePos_W.y() << ","
                << sitePos_W.z() << ","
                << footLinkCom_W.x() << ","
                << footLinkCom_W.y() << ","
                << footLinkCom_W.z() << ","
                << sitePos_B.x() << ","
                << sitePos_B.y() << ","
                << sitePos_B.z() << ","
                << footLinkCom_B.x() << ","
                << footLinkCom_B.y() << ","
                << footLinkCom_B.z() << ","
                << legacyNominal_B.x() << ","
                << legacyNominal_B.y() << ","
                << legacyNominal_B.z() << ","
                << deltaSite.x() << ","
                << deltaSite.y() << ","
                << deltaSite.z() << ","
                << deltaFootLinkCom.x() << ","
                << deltaFootLinkCom.y() << ","
                << deltaFootLinkCom.z() << "\n";
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
