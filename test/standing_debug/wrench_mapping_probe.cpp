#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
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

FootEndEffectorSource footSourceFromLog(const json& log) {
    const json metadata = log.value("metadata", json::object());
    const json controllerConfig = log.value("controller_config", json::object());
    const json modelConfig = controllerConfig.value("model", json::object());
    const std::string value =
        readJsonStringOr(metadata,
                         "foot_end_effector_source",
                         readJsonStringOr(modelConfig, "foot_end_effector_source", "site"));
    if (value == "site" || value == "foot_site") {
        return FootEndEffectorSource::Site;
    }
    if (value == "collision_geom_center" || value == "foot_collision_geom_center") {
        return FootEndEffectorSource::CollisionGeomCenter;
    }
    throw std::runtime_error("unsupported foot_end_effector_source: " + value);
}

std::string footSourceName(const FootEndEffectorSource source) {
    switch (source) {
        case FootEndEffectorSource::Site:
            return "site";
        case FootEndEffectorSource::CollisionGeomCenter:
            return "collision_geom_center";
    }
    return "unknown";
}

Vec3<double> readMujocoVec3(const mjtNum* raw) {
    Vec3<double> out = Vec3<double>::Zero();
    for (int i = 0; i < 3; ++i) {
        out[i] = static_cast<double>(raw[i]);
    }
    return out;
}

Vec3<double> collisionGeomCenterWorld(const mjData* data, const std::vector<int>& geomIds) {
    if (geomIds.empty()) {
        throw std::runtime_error("foot has no collision geoms for collision center Jacobian");
    }

    Vec3<double> center = Vec3<double>::Zero();
    for (const int geomId : geomIds) {
        center += readMujocoVec3(&data->geom_xpos[3 * geomId]);
    }
    return center / static_cast<double>(geomIds.size());
}

DMat<double> selectJacobianColumns(const std::vector<mjtNum>& raw,
                                   const std::vector<int>& columnIndices,
                                   const int nv,
                                   const std::string& name) {
    DMat<double> out(3, static_cast<Eigen::Index>(columnIndices.size()));
    for (std::size_t col = 0; col < columnIndices.size(); ++col) {
        const int sourceCol = columnIndices[col];
        if (sourceCol < 0 || sourceCol >= nv) {
            throw std::runtime_error(name + " column index out of MuJoCo nv range");
        }
        for (int row = 0; row < 3; ++row) {
            out(row, static_cast<Eigen::Index>(col)) =
                static_cast<double>(raw[static_cast<std::size_t>(row * nv + sourceCol)]);
        }
    }
    return out;
}

void computeFootJacobians(const mjModel* model,
                          mjData* data,
                          const MujocoEndEffectorBinding& foot,
                          const FootEndEffectorSource footSource,
                          const std::vector<int>& qvelIndices,
                          DMat<double>& Jv_W,
                          DMat<double>& Jw_W) {
    std::vector<mjtNum> jacp(static_cast<std::size_t>(3 * model->nv), mjtNum(0));
    std::vector<mjtNum> jacr(static_cast<std::size_t>(3 * model->nv), mjtNum(0));

    switch (footSource) {
        case FootEndEffectorSource::Site:
            if (foot.siteId < 0) {
                throw std::runtime_error("foot source is site, but no foot site was bound");
            }
            mj_jacSite(model, data, jacp.data(), jacr.data(), foot.siteId);
            break;
        case FootEndEffectorSource::CollisionGeomCenter: {
            const Vec3<double> point_W = collisionGeomCenterWorld(data, foot.collisionGeomIds);
            mj_jac(model, data, jacp.data(), jacr.data(), point_W.data(), foot.bodyId);
            break;
        }
    }

    Jv_W = selectJacobianColumns(jacp, qvelIndices, model->nv, "Jv_W");
    Jw_W = selectJacobianColumns(jacr, qvelIndices, model->nv, "Jw_W");
}

json matrixToJson(const DMat<double>& matrix) {
    json data = json::array();
    for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
        json rowJson = json::array();
        for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
            rowJson.push_back(matrix(row, col));
        }
        data.push_back(std::move(rowJson));
    }

    json out = json::object();
    out["rows"] = matrix.rows();
    out["cols"] = matrix.cols();
    out["data"] = std::move(data);
    return out;
}

json vectorToJson(const DVec<double>& vector) {
    json out = json::array();
    for (Eigen::Index i = 0; i < vector.size(); ++i) {
        out.push_back(vector[i]);
    }
    return out;
}

json buildMappingSection(const std::filesystem::path& logPath) {
    std::ifstream logStream(logPath);
    if (!logStream.is_open()) {
        throw std::runtime_error("failed to open debug log: " + logPath.string());
    }

    json log;
    logStream >> log;

    const FootEndEffectorSource footSource = footSourceFromLog(log);
    const std::string modelPath =
        std::string(PROJECT_ROOT_DIR) + "/models/mit_humanoid/scene.xml";
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

    const std::vector<double> qpos =
        readJsonVector(log.at("robot_state").at("full_qpos"), "robot_state.full_qpos");
    const std::vector<double> qvel =
        readJsonVector(log.at("robot_state").at("full_qvel"), "robot_state.full_qvel");
    copyVectorToMujoco(qpos, data->qpos, model->nq, "robot_state.full_qpos");
    copyVectorToMujoco(qvel, data->qvel, model->nv, "robot_state.full_qvel");
    mj_normalizeQuat(model.get(), data->qpos);
    mj_forward(model.get(), data.get());

    const auto robotSetup =
        setupRobotParams<double>(RobotType::MIT_HUMANOID, model.get(), footSource);
    const RobotParams<double>& params = robotSetup.params;
    const MujocoRobotBindings& bindings = robotSetup.bindings;
    if (params.legs.size() != bindings.feet.size()) {
        throw std::runtime_error("robot params and MuJoCo foot bindings do not match");
    }

    Eigen::Index totalDof = 0;
    for (const auto& leg : params.legs) {
        totalDof += static_cast<Eigen::Index>(leg.joints.qd_idx.size());
    }

    DMat<double> wrenchToTau = DMat<double>::Zero(totalDof, 12);
    Eigen::Index rowOffset = 0;
    for (std::size_t leg = 0; leg < params.legs.size(); ++leg) {
        DMat<double> Jv_W;
        DMat<double> Jw_W;
        computeFootJacobians(model.get(),
                             data.get(),
                             bindings.feet[leg],
                             footSource,
                             params.legs[leg].joints.qd_idx,
                             Jv_W,
                             Jw_W);

        int forceColumn = -1;
        int momentColumn = -1;
        switch (params.legs[leg].side) {
            case Side::Left:
                forceColumn = 0;
                momentColumn = 6;
                break;
            case Side::Right:
                forceColumn = 3;
                momentColumn = 9;
                break;
            default:
                throw std::runtime_error("wrench mapping only supports left/right legs");
        }

        const Eigen::Index dof = Jv_W.cols();
        wrenchToTau.block(rowOffset, forceColumn, dof, 3) = -Jv_W.transpose();
        wrenchToTau.block(rowOffset, momentColumn, dof, 3) = -Jw_W.transpose();
        rowOffset += dof;
    }

    json section = json::object();
    section["source"] = "mujoco_full_model_foot_jacobians";
    section["foot_end_effector_source"] = footSourceName(footSource);
    section["input_order"] = "[F_left(3), F_right(3), M_left(3), M_right(3)]";
    section["output_order"] = "packed leg actuator order";
    section["mapping"] =
        "tau = -blockdiag(Jv_left^T, Jv_right^T) * [F_left, F_right] "
        "- blockdiag(Jw_left^T, Jw_right^T) * [M_left, M_right]";
    section["wrench_to_tau_jacobian"] = matrixToJson(wrenchToTau);

    const json legacySection = log.value("standing_wrench_to_torque", json::object());
    if (legacySection.is_object() && legacySection.contains("actual_leg_tau_vector")) {
        const std::vector<double> actualTau =
            readJsonVector(legacySection.at("actual_leg_tau_vector"), "actual_leg_tau_vector");
        DVec<double> actualTauEigen(static_cast<Eigen::Index>(actualTau.size()));
        for (Eigen::Index i = 0; i < actualTauEigen.size(); ++i) {
            actualTauEigen[i] = actualTau[static_cast<std::size_t>(i)];
        }
        section["actual_leg_tau_vector"] = vectorToJson(actualTauEigen);
    }

    return section;
}
}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 2 || std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
            std::cout << "Usage: wrench_mapping_probe mpc_debug.json\n";
            return argc == 2 ? EXIT_SUCCESS : EXIT_FAILURE;
        }

        const json section = buildMappingSection(std::filesystem::path(argv[1]));
        std::cout << section.dump(2) << '\n';
        return EXIT_SUCCESS;
    } catch (const std::exception& exception) {
        std::cerr << "wrench_mapping_probe: " << exception.what() << "\n";
        return EXIT_FAILURE;
    }
}
