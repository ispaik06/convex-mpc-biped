#include "StandingMpcDebugLogger.h"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <type_traits>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "Utilities/MatrixUtils.h"

namespace {
using json = nlohmann::ordered_json;

constexpr int kStateDim = 13;
constexpr int kInputDim = 12;

struct TimestampStrings {
    std::string filenameToken;
    std::string localTime;
};

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

void ensureDirectory(const std::string& path) {
    struct stat info {};
    if (stat(path.c_str(), &info) == 0) {
        if ((info.st_mode & S_IFDIR) != 0) {
            return;
        }
        throw std::runtime_error(path + " exists but is not a directory");
    }

    if (mkdir(path.c_str(), 0755) != 0 && errno != EEXIST) {
        throw std::runtime_error("Failed to create directory " + path);
    }
}

TimestampStrings makeTimestampStrings() {
    using clock = std::chrono::system_clock;

    const auto now = clock::now();
    const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
    const auto micros =
        std::chrono::duration_cast<std::chrono::microseconds>(now - seconds).count();
    const std::time_t time = clock::to_time_t(now);

    std::tm localTime {};
    localtime_r(&time, &localTime);

    std::ostringstream filename;
    filename << std::put_time(&localTime, "%Y%m%d_%H%M%S");

    std::ostringstream readable;
    readable << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S")
             << '.' << std::setw(6) << std::setfill('0') << micros;

    return TimestampStrings{filename.str(), readable.str()};
}

std::string makeLogPath(const TimestampStrings& timestamp) {
    const std::string root(PROJECT_ROOT_DIR);
    const std::string logsDir = root + "/logs";
    const std::string debugDir = logsDir + "/debug";
    const std::string standingDir = debugDir + "/standing_mpc";

    ensureDirectory(logsDir);
    ensureDirectory(debugDir);
    ensureDirectory(standingDir);

    return standingDir + "/standing_mpc_debug_" + timestamp.filenameToken + ".json";
}

template <typename Scalar>
json scalarToJson(Scalar value) {
    if constexpr (std::is_floating_point_v<Scalar>) {
        if (std::isfinite(value)) {
            return json(value);
        }
        return nullptr;
    } else {
        return json(value);
    }
}

template <typename Derived>
json vectorToJson(const Eigen::MatrixBase<Derived>& vector) {
    json out = json::array();
    for (Eigen::Index i = 0; i < vector.size(); ++i) {
        out.push_back(scalarToJson(vector.derived()(i)));
    }
    return out;
}

template <typename Derived>
json matrixToFlatJson(const Eigen::MatrixBase<Derived>& matrix) {
    json data = json::array();
    for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
        json rowJson = json::array();
        for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
            rowJson.push_back(scalarToJson(matrix.derived()(row, col)));
        }
        data.push_back(std::move(rowJson));
    }

    json out = json::object();
    out["rows"] = matrix.rows();
    out["cols"] = matrix.cols();
    out["data"] = std::move(data);
    return out;
}

json intVectorToJson(const std::vector<int>& values) {
    json out = json::array();
    for (const int value : values) {
        out.push_back(value);
    }
    return out;
}

template <typename Sequence>
json matrixSequenceToFlatJson(const Sequence& matrices) {
    json data = json::array();
    int count = 0;
    int rows = 0;
    int cols = 0;
    for (const auto& matrix : matrices) {
        json matrixJson = json::array();
        if (count == 0) {
            rows = static_cast<int>(matrix.rows());
            cols = static_cast<int>(matrix.cols());
        } else if (matrix.rows() != rows || matrix.cols() != cols) {
            throw std::runtime_error("Matrix sequence dimensions are inconsistent for debug log");
        }

        for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
            json rowJson = json::array();
            for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
                rowJson.push_back(scalarToJson(matrix(row, col)));
            }
            matrixJson.push_back(std::move(rowJson));
        }
        data.push_back(std::move(matrixJson));
        ++count;
    }

    json out = json::object();
    out["count"] = count;
    out["rows"] = rows;
    out["cols"] = cols;
    out["data"] = std::move(data);
    return out;
}

json vectorByStepToFlatJson(const DVec<double>& values, const int rows, const int cols) {
    if (values.size() != rows * cols) {
        throw std::runtime_error("Cannot reshape vector for debug log");
    }

    json data = json::array();
    for (int row = 0; row < rows; ++row) {
        json rowJson = json::array();
        for (int col = 0; col < cols; ++col) {
            rowJson.push_back(scalarToJson(values[row * cols + col]));
        }
        data.push_back(std::move(rowJson));
    }

    json out = json::object();
    out["rows"] = rows;
    out["cols"] = cols;
    out["data"] = std::move(data);
    return out;
}

bool isPrimitiveArray(const json& value) {
    if (!value.is_array()) {
        return false;
    }

    for (const auto& item : value) {
        if (item.is_array() || item.is_object()) {
            return false;
        }
    }
    return true;
}

void writeIndent(std::ostream& out, const int indent) {
    for (int i = 0; i < indent; ++i) {
        out.put(' ');
    }
}

void writePrimitiveArray(std::ostream& out, const json& value, const int indent) {
    constexpr std::size_t kInlineLimit = 16;
    constexpr std::size_t kItemsPerLine = 12;

    if (value.size() <= kInlineLimit) {
        out << value.dump();
        return;
    }

    out << "[\n";
    for (std::size_t i = 0; i < value.size(); ++i) {
        if ((i % kItemsPerLine) == 0) {
            writeIndent(out, indent + 2);
        } else {
            out << ' ';
        }

        out << value[i].dump();
        if (i + 1 < value.size()) {
            out << ',';
        }
        if (((i + 1) % kItemsPerLine) == 0 || i + 1 == value.size()) {
            out << '\n';
        }
    }
    writeIndent(out, indent);
    out << ']';
}

std::string footEndEffectorSourceName(const FootEndEffectorSource source) {
    switch (source) {
        case FootEndEffectorSource::Site:
            return "site";
        case FootEndEffectorSource::BodyCom:
            return "body_com";
    }
    return "unknown";
}

std::string desiredWrenchReferencePointName(const FootEndEffectorSource source) {
    switch (source) {
        case FootEndEffectorSource::Site:
            return "foot_site";
        case FootEndEffectorSource::BodyCom:
            return "foot_link_com";
    }
    return "unknown";
}

void writePrettyJson(std::ostream& out, const json& value, const int indent = 0) {
    if (value.is_object()) {
        if (value.empty()) {
            out << "{}";
            return;
        }

        out << "{\n";
        std::size_t index = 0;
        for (auto it = value.begin(); it != value.end(); ++it) {
            writeIndent(out, indent + 2);
            out << json(it.key()).dump() << ": ";
            writePrettyJson(out, it.value(), indent + 2);
            if (++index < value.size()) {
                out << ',';
            }
            out << '\n';
        }
        writeIndent(out, indent);
        out << '}';
        return;
    }

    if (value.is_array()) {
        if (value.empty() || isPrimitiveArray(value)) {
            writePrimitiveArray(out, value, indent);
            return;
        }

        out << "[\n";
        for (std::size_t i = 0; i < value.size(); ++i) {
            writeIndent(out, indent + 2);
            writePrettyJson(out, value[i], indent + 2);
            if (i + 1 < value.size()) {
                out << ',';
            }
            out << '\n';
        }
        writeIndent(out, indent);
        out << ']';
        return;
    }

    out << value.dump();
}

const std::vector<int>& effectiveActuatorIndices(const JointGroupParams<double>& joints) {
    if (!joints.actuator_idx.empty()) {
        return joints.actuator_idx;
    }
    return joints.qd_idx;
}

template <typename Derived>
void packIndexedValues(DVec<double>& dst,
                       const std::vector<int>& indices,
                       const Eigen::MatrixBase<Derived>& values,
                       const char* name) {
    if (static_cast<Eigen::Index>(indices.size()) != values.size()) {
        throw std::runtime_error(std::string("Packed vector size mismatch for ") + name);
    }

    for (Eigen::Index i = 0; i < values.size(); ++i) {
        const int dstIdx = indices[static_cast<std::size_t>(i)];
        if (dstIdx < 0 || dstIdx >= dst.size()) {
            throw std::runtime_error(std::string(name) + " index is out of range");
        }
        dst[dstIdx] = values.derived()(i);
    }
}

DVec<double> packFullQpos(const StandingMpcDebugSnapshot& snapshot) {
    DVec<double> qpos = snapshot.robotParams.default_qpos;
    if (qpos.size() != snapshot.robotParams.nq) {
        qpos.setZero(snapshot.robotParams.nq);
    }

    if (qpos.size() >= 7) {
        qpos[0] = snapshot.stateEstimate.torsoPos_W.x();
        qpos[1] = snapshot.stateEstimate.torsoPos_W.y();
        qpos[2] = snapshot.stateEstimate.torsoPos_W.z();
        qpos[3] = snapshot.stateEstimate.torsoQuat_W.w();
        qpos[4] = snapshot.stateEstimate.torsoQuat_W.x();
        qpos[5] = snapshot.stateEstimate.torsoQuat_W.y();
        qpos[6] = snapshot.stateEstimate.torsoQuat_W.z();
    }

    for (std::size_t leg = 0; leg < snapshot.robotParams.legs.size(); ++leg) {
        packIndexedValues(qpos,
                          snapshot.robotParams.legs[leg].joints.q_idx,
                          snapshot.stateEstimate.legs[leg].q,
                          "leg qpos");
    }

    for (std::size_t arm = 0; arm < snapshot.robotParams.arms.size(); ++arm) {
        packIndexedValues(qpos,
                          snapshot.robotParams.arms[arm].joints.q_idx,
                          snapshot.stateEstimate.arms[arm].q,
                          "arm qpos");
    }

    return qpos;
}

DVec<double> packFullQvel(const StandingMpcDebugSnapshot& snapshot) {
    DVec<double> qvel = DVec<double>::Zero(snapshot.robotParams.nv);
    if (qvel.size() >= 6) {
        qvel[0] = snapshot.stateEstimate.torsoLinVel_W.x();
        qvel[1] = snapshot.stateEstimate.torsoLinVel_W.y();
        qvel[2] = snapshot.stateEstimate.torsoLinVel_W.z();
        qvel[3] = snapshot.stateEstimate.torsoAngVel_W.x();
        qvel[4] = snapshot.stateEstimate.torsoAngVel_W.y();
        qvel[5] = snapshot.stateEstimate.torsoAngVel_W.z();
    }

    for (std::size_t leg = 0; leg < snapshot.robotParams.legs.size(); ++leg) {
        packIndexedValues(qvel,
                          snapshot.robotParams.legs[leg].joints.qd_idx,
                          snapshot.stateEstimate.legs[leg].qd,
                          "leg qvel");
    }

    for (std::size_t arm = 0; arm < snapshot.robotParams.arms.size(); ++arm) {
        packIndexedValues(qvel,
                          snapshot.robotParams.arms[arm].joints.qd_idx,
                          snapshot.stateEstimate.arms[arm].qd,
                          "arm qvel");
    }

    return qvel;
}

DVec<double> packActualLegTauVector(const StandingMpcDebugSnapshot& snapshot) {
    Eigen::Index totalDof = 0;
    for (const auto& data : snapshot.legController.datas) {
        totalDof += data.dof();
    }

    DVec<double> tau(totalDof);
    Eigen::Index offset = 0;
    for (std::size_t leg = 0; leg < snapshot.legController.commands.size(); ++leg) {
        const DVec<double> legTorque = snapshot.legController.computeLegTorque(static_cast<int>(leg));
        tau.segment(offset, legTorque.size()) = legTorque;
        offset += legTorque.size();
    }

    return tau;
}

DVec<double> packFullTauCommand(const StandingMpcDebugSnapshot& snapshot) {
    DVec<double> tau = DVec<double>::Zero(snapshot.robotParams.nu);

    for (std::size_t leg = 0; leg < snapshot.robotParams.legs.size(); ++leg) {
        const DVec<double> legTorque = snapshot.legController.computeLegTorque(static_cast<int>(leg));
        packIndexedValues(tau,
                          effectiveActuatorIndices(snapshot.robotParams.legs[leg].joints),
                          legTorque,
                          "leg tau");
    }

    if (snapshot.armController != nullptr) {
        for (std::size_t arm = 0; arm < snapshot.robotParams.arms.size(); ++arm) {
            const DVec<double> armTorque =
                snapshot.armController->computeArmTorque(static_cast<int>(arm));
            packIndexedValues(tau,
                              effectiveActuatorIndices(snapshot.robotParams.arms[arm].joints),
                              armTorque,
                              "arm tau");
        }
    }

    return tau;
}

DMat<double> buildWrenchToTorqueJacobian(const StandingFootKinematics<double>& standingFeet) {
    DMat<double> wrenchToTau(standingFeet.Jv_W.cols(), 12);
    wrenchToTau.block(0, 0, standingFeet.Jv_W.cols(), 6) = -standingFeet.Jv_W.transpose();
    wrenchToTau.block(0, 6, standingFeet.Jw_W.cols(), 6) = -standingFeet.Jw_W.transpose();
    return wrenchToTau;
}

Mat3<double> inertiaWorldFromX0(const RobotParams<double>& robotParams,
                                const Vec13<double>& x0) {
    const Mat3<double> R_WB = Rz(x0[2]);
    return R_WB * robotParams.bodyInertia * R_WB.transpose();
}

json buildSnapshotJson(const StandingMpcDebugSnapshot& snapshot,
                       const std::string& logPath,
                       const TimestampStrings& timestamp) {
    const int steps = horizonSteps();
    if (snapshot.wrenchHorizon.size() != kInputDim * steps) {
        throw std::runtime_error("Standing MPC debug log received an unexpected wrench horizon size");
    }
    if (snapshot.formulation.A_qp.rows() != kStateDim * steps ||
        snapshot.formulation.A_qp.cols() != kStateDim ||
        snapshot.formulation.B_qp.rows() != kStateDim * steps ||
        snapshot.formulation.B_qp.cols() != kInputDim * steps) {
        throw std::runtime_error("Standing MPC debug log received unexpected QP matrix dimensions");
    }
    if (!snapshot.stateEstimate.standingFeet.hasFootJacobians) {
        throw std::runtime_error("Standing MPC debug log requires standing foot Jacobians");
    }

    const DVec<double> predictedState =
        snapshot.formulation.A_qp * snapshot.x0 +
        snapshot.formulation.B_qp * snapshot.wrenchHorizon;
    const DMat<double> wrenchToTau =
        buildWrenchToTorqueJacobian(snapshot.stateEstimate.standingFeet);
    const DVec<double> actualLegTau = packActualLegTauVector(snapshot);
    const DVec<double> fullQpos = packFullQpos(snapshot);
    const DVec<double> fullQvel = packFullQvel(snapshot);
    const DVec<double> fullTau = packFullTauCommand(snapshot);

    json root = json::object();

    json metadata = json::object();
    metadata["type"] = "standing_mpc_first_solve_debug";
    metadata["generated_at_local"] = timestamp.localTime;
    metadata["log_path"] = logPath;
    metadata["controller_time"] = snapshot.stateEstimate.time;
    metadata["controller_iteration"] = snapshot.iteration;
    metadata["debug_request_source"] = snapshot.debugRequestSource;
    metadata["debug_request_time"] = scalarToJson(snapshot.debugRequestTime);
    metadata["debug_trigger_time"] = scalarToJson(snapshot.debugTriggerTime);
    metadata["horizon_steps"] = steps;
    metadata["dt_mpc"] = dtMpc();
    metadata["foot_end_effector_source"] =
        footEndEffectorSourceName(getControllerConfig().swing.footEndEffectorSource);
    metadata["desired_wrench_reference_point"] =
        desiredWrenchReferencePointName(getControllerConfig().swing.footEndEffectorSource);
    root["metadata"] = std::move(metadata);

    json model = json::object();
    model["mass"] = snapshot.robotParams.bodyMass;
    model["gravity"] = getControllerConfig().model.gravity;
    model["body_com_location_yaw_frame"] = vectorToJson(snapshot.robotParams.bodyComLocation);
    model["body_inertia_yaw_frame"] = matrixToFlatJson(snapshot.robotParams.bodyInertia);
    model["inertia_world_from_x0"] =
        matrixToFlatJson(inertiaWorldFromX0(snapshot.robotParams, snapshot.x0));
    root["model"] = std::move(model);

    json initialState = json::object();
    initialState["x0"] = vectorToJson(snapshot.x0);
    initialState["psi"] = snapshot.x0[2];
    initialState["torso_pos_W"] = vectorToJson(snapshot.stateEstimate.torsoPos_W);
    initialState["torso_lin_vel_W"] = vectorToJson(snapshot.stateEstimate.torsoLinVel_W);
    initialState["torso_ang_vel_W"] = vectorToJson(snapshot.stateEstimate.torsoAngVel_W);
    root["initial_state"] = std::move(initialState);

    json robotState = json::object();
    robotState["full_qpos"] = vectorToJson(fullQpos);
    robotState["full_qvel"] = vectorToJson(fullQvel);
    robotState["full_tau_command"] = vectorToJson(fullTau);
    root["robot_state"] = std::move(robotState);

    json feet = json::object();
    json desiredFootPos = json::object();
    desiredFootPos["left"] = vectorToJson(snapshot.desiredFootPositions.left_des_W);
    desiredFootPos["right"] = vectorToJson(snapshot.desiredFootPositions.right_des_W);
    feet["desired_foot_pos_W"] = std::move(desiredFootPos);

    json legs = json::array();
    for (std::size_t leg = 0; leg < snapshot.robotParams.legs.size(); ++leg) {
        const auto& legParams = snapshot.robotParams.legs[leg];
        const auto& legState = snapshot.stateEstimate.legs[leg];
        const auto& legCommand = snapshot.legController.commands[leg];

        json legJson = json::object();
        legJson["index"] = leg;
        legJson["side"] = sideName(legParams.side);
        legJson["q_indices"] = intVectorToJson(legParams.joints.q_idx);
        legJson["qd_indices"] = intVectorToJson(legParams.joints.qd_idx);
        legJson["actuator_indices"] = intVectorToJson(legParams.joints.actuator_idx);
        legJson["foot_pos_W"] = vectorToJson(legState.footPos_W);
        legJson["foot_vel_W"] = vectorToJson(legState.footVel_W);
        legJson["q"] = vectorToJson(legState.q);
        legJson["qd"] = vectorToJson(legState.qd);
        legJson["tau_feedforward_command"] = vectorToJson(legCommand.tauFeedForward);
        legs.push_back(std::move(legJson));
    }
    feet["legs"] = std::move(legs);
    root["feet"] = std::move(feet);

    json referenceTrajectory = json::object();
    referenceTrajectory["tk"] = vectorToJson(snapshot.referenceTrajectory.tk);
    referenceTrajectory["psi"] = vectorToJson(snapshot.referenceTrajectory.psi);
    referenceTrajectory["r_left"] = matrixToFlatJson(snapshot.referenceTrajectory.r_left);
    referenceTrajectory["r_right"] = matrixToFlatJson(snapshot.referenceTrajectory.r_right);
    referenceTrajectory["X_ref_by_step"] =
        vectorByStepToFlatJson(snapshot.referenceTrajectory.X_ref, steps, kStateDim);
    root["reference_trajectory"] = std::move(referenceTrajectory);

    json formulation = json::object();
    formulation["input_order"] = "[F_left(3), F_right(3), M_left(3), M_right(3)]";
    formulation["A_c"] = matrixSequenceToFlatJson(snapshot.formulation.A_c);
    formulation["B_c"] = matrixSequenceToFlatJson(snapshot.formulation.B_c);
    formulation["inertia_world"] = matrixSequenceToFlatJson(snapshot.formulation.inertia_W);
    formulation["A_qp"] = matrixToFlatJson(snapshot.formulation.A_qp);
    formulation["B_qp"] = matrixToFlatJson(snapshot.formulation.B_qp);
    root["formulation"] = std::move(formulation);

    json solution = json::object();
    solution["wrench_horizon"] = vectorByStepToFlatJson(snapshot.wrenchHorizon, steps, kInputDim);
    solution["wrench_horizon_vector"] = vectorToJson(snapshot.wrenchHorizon);
    solution["first_wrench"] = vectorToJson(snapshot.wrenchHorizon.head(kInputDim));
    solution["predicted_state_horizon"] =
        vectorByStepToFlatJson(predictedState, steps, kStateDim);
    solution["predicted_state_horizon_vector"] = vectorToJson(predictedState);
    root["solution"] = std::move(solution);

    json wrenchToTauSection = json::object();
    wrenchToTauSection["mapping"] =
        "tau = -[Jv_W^T, Jw_W^T] * [F_left, F_right, M_left, M_right]";
    wrenchToTauSection["standing_Jv_W"] =
        matrixToFlatJson(snapshot.stateEstimate.standingFeet.Jv_W);
    wrenchToTauSection["standing_Jw_W"] =
        matrixToFlatJson(snapshot.stateEstimate.standingFeet.Jw_W);
    wrenchToTauSection["wrench_to_tau_jacobian"] = matrixToFlatJson(wrenchToTau);
    wrenchToTauSection["actual_leg_tau_vector"] = vectorToJson(actualLegTau);
    root["standing_wrench_to_torque"] = std::move(wrenchToTauSection);

    return root;
}
}  // namespace

std::string writeStandingMpcDebugLog(const StandingMpcDebugSnapshot& snapshot) {
    const TimestampStrings timestamp = makeTimestampStrings();
    const std::string logPath = makeLogPath(timestamp);

    const json root = buildSnapshotJson(snapshot, logPath, timestamp);

    std::ofstream out(logPath, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to open standing MPC debug log: " + logPath);
    }

    writePrettyJson(out, root);
    out << '\n';
    if (!out.good()) {
        throw std::runtime_error("Failed while writing standing MPC debug log: " + logPath);
    }

    return logPath;
}
