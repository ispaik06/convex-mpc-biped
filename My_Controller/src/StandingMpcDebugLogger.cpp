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

#include "ControllerConfig.h"
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

std::string debugLogPrefixForMode(const LocomotionMode mode) {
    switch (mode) {
        case LocomotionMode::Walking:
            return "walking_mpc_debug_";
        case LocomotionMode::Standing:
            return "standing_mpc_debug_";
    }
    return "mpc_debug_";
}

std::string debugLogDirectoryNameForMode(const LocomotionMode mode) {
    switch (mode) {
        case LocomotionMode::Walking:
            return "walking_mpc";
        case LocomotionMode::Standing:
            return "standing_mpc";
    }
    return "mpc";
}

std::string makeLogPath(const TimestampStrings& timestamp, const LocomotionMode locomotionMode) {
    const std::string root(PROJECT_ROOT_DIR);
    const std::string logsDir = root + "/logs";
    const std::string debugDir = logsDir + "/debug";
    const std::string mpcDebugDir = debugDir + "/" + debugLogDirectoryNameForMode(locomotionMode);

    ensureDirectory(logsDir);
    ensureDirectory(debugDir);
    ensureDirectory(mpcDebugDir);

    return mpcDebugDir + "/" + debugLogPrefixForMode(locomotionMode) +
           timestamp.filenameToken + ".json";
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
        case FootEndEffectorSource::CollisionGeomCenter:
            return "collision_geom_center";
    }
    return "unknown";
}

std::string desiredWrenchReferencePointName(const FootEndEffectorSource source) {
    switch (source) {
        case FootEndEffectorSource::Site:
            return "foot_site";
        case FootEndEffectorSource::CollisionGeomCenter:
            return "foot_collision_geom_center";
    }
    return "unknown";
}

std::string locomotionModeName(const LocomotionMode mode) {
    switch (mode) {
        case LocomotionMode::Walking:
            return "walking";
        case LocomotionMode::Standing:
            return "standing";
    }
    return "unknown";
}

std::string locomotionStateName(const LocomotionState state) {
    switch (state) {
        case LocomotionState::StandingSettle:
            return "standing_settle";
        case LocomotionState::Standing:
            return "standing";
        case LocomotionState::Walking:
            return "walking";
    }
    return "unknown";
}

std::string legControlModeName(const LegControlMode mode) {
    switch (mode) {
        case LegControlMode::JointPd:
            return "joint_pd";
        case LegControlMode::JointTorque:
            return "joint_torque";
        case LegControlMode::SwingFoot:
            return "swing_foot";
        case LegControlMode::StanceWrench:
            return "stance_wrench";
    }
    return "unknown";
}

std::string robotTypeName(const RobotType type) {
    switch (type) {
        case RobotType::MIT_HUMANOID:
            return "mit_humanoid";
        case RobotType::UNITREE_G1:
            return "unitree_g1";
        case RobotType::UNITREE_H1:
            return "unitree_h1";
    }
    return "unknown";
}

template <typename T>
json stdVectorToJson(const std::vector<T>& values) {
    json out = json::array();
    for (const T& value : values) {
        out.push_back(scalarToJson(value));
    }
    return out;
}

template <typename Derived>
json matrixDiagonalToJson(const Eigen::MatrixBase<Derived>& matrix) {
    if (matrix.rows() != matrix.cols()) {
        throw std::runtime_error("Expected square matrix for diagonal serialization");
    }

    json out = json::array();
    for (Eigen::Index i = 0; i < matrix.rows(); ++i) {
        out.push_back(scalarToJson(matrix.derived()(i, i)));
    }
    return out;
}

json userCommandJson(const UserCommand& command) {
    json out = json::object();
    out["x_dot"] = scalarToJson(command.x_dot);
    out["y_dot"] = scalarToJson(command.y_dot);
    out["psi_dot"] = scalarToJson(command.psi_dot);
    out["z_dot"] = scalarToJson(command.z_dot);
    out["standing_roll_offset_rad"] = scalarToJson(command.standing_roll_offset_rad);
    out["standing_pitch_offset_rad"] = scalarToJson(command.standing_pitch_offset_rad);
    out["standing_mpc_debug_log_request"] = command.standing_mpc_debug_log_request;
    return out;
}

json controllerConfigJson(const ControllerConfig& config,
                          const RobotType robotType,
                          const LocomotionMode locomotionMode) {
    json root = json::object();
    root["robot_type"] = robotTypeName(robotType);
    root["requested_locomotion_mode"] = locomotionModeName(config.requestedLocomotionMode);
    root["effective_locomotion_mode"] = locomotionModeName(locomotionMode);

    json timing = json::object();
    timing["cycle"] = config.timing.cycle;
    timing["swing"] = config.timing.swing;
    timing["stance"] = config.timing.stance;
    timing["horizon"] = config.timing.horizon;
    timing["horizon_steps"] = config.timing.horizonSteps;
    root["timing"] = std::move(timing);

    json model = json::object();
    model["xml_path"] = config.model.xmlPath;
    model["auxiliary_xml_path"] = config.model.auxiliaryXmlPath;
    model["foot_end_effector_source"] =
        footEndEffectorSourceName(config.model.footEndEffectorSource);
    model["gravity"] = config.model.gravity;
    root["model"] = std::move(model);

    json mpc = json::object();
    mpc["friction_coefficient"] = config.mpc.frictionCoefficient;
    mpc["foot_half_length"] = config.mpc.footHalfLength;
    mpc["foot_half_width"] = config.mpc.footHalfWidth;
    mpc["torsional_friction_scale"] = config.mpc.torsionalFrictionScale;
    mpc["normal_force_max"] = config.mpc.normalForceMax;
    mpc["normal_force_min"] = config.mpc.normalForceMin;
    mpc["iterations_between_solve"] = config.mpc.iterationsBetweenSolve;
    json walking = json::object();
    walking["contact_wrench_model"] =
        contactWrenchModelName(config.mpc.walkingContactWrenchModel);
    walking["state_weight_diag"] = matrixDiagonalToJson(config.mpc.walkingStateWeight);
    walking["input_weight_diag"] = matrixDiagonalToJson(config.mpc.walkingInputWeight);
    mpc["walking"] = std::move(walking);
    json standing = json::object();
    standing["contact_wrench_model"] =
        contactWrenchModelName(config.mpc.standingContactWrenchModel);
    standing["state_weight_diag"] = matrixDiagonalToJson(config.mpc.standingStateWeight);
    standing["input_weight_diag"] = matrixDiagonalToJson(config.mpc.standingInputWeight);
    mpc["standing"] = std::move(standing);
    root["mpc"] = std::move(mpc);

    json swing = json::object();
    swing["natural_frequency"] = vectorToJson(config.swing.naturalFrequency);
    swing["kd_diag"] = vectorToJson(config.swing.kdDiag);
    swing["height"] = config.swing.height;
    swing["min_remaining_time"] = config.swing.minRemainingTime;
    swing["body_velocity_half_stance_offset"] = config.swing.bodyVelocityHalfStanceOffset;
    swing["swing_foot_yaw_lead_scale"] = config.swing.swingFootYawLeadScale;
    swing["pitch_kp"] = config.swing.pitchKp;
    swing["pitch_kd"] = config.swing.pitchKd;
    swing["yaw_kp"] = config.swing.yawKp;
    swing["yaw_kd"] = config.swing.yawKd;
    root["swing"] = std::move(swing);

    json contactManager = json::object();
    contactManager["contact_force_on_threshold"] =
        config.contactManager.contactForceOnThreshold;
    contactManager["contact_force_off_threshold"] =
        config.contactManager.contactForceOffThreshold;
    contactManager["contact_on_confirm_ticks"] =
        config.contactManager.contactOnConfirmTicks;
    contactManager["contact_off_confirm_ticks"] =
        config.contactManager.contactOffConfirmTicks;
    contactManager["contact_ramp_duration"] =
        config.contactManager.contactRampDuration;
    contactManager["contact_lock_steps"] =
        config.contactManager.contactLockSteps;
    contactManager["late_contact_timeout"] =
        config.contactManager.lateContactTimeout;
    contactManager["ground_search_velocity"] =
        config.contactManager.groundSearchVelocity;
    contactManager["ground_search_max_depth"] =
        config.contactManager.groundSearchMaxDepth;
    contactManager["enable_early_contact_handling"] =
        config.contactManager.enableEarlyContactHandling;
    contactManager["enable_late_contact_handling"] =
        config.contactManager.enableLateContactHandling;
    root["contact_manager"] = std::move(contactManager);

    json logging = json::object();
    logging["standing_mpc_debug_trigger_times"] =
        stdVectorToJson(config.logging.standingMpcDebugTriggerTimes);
    root["logging"] = std::move(logging);

    json startup = json::object();
    startup["post_init_standing_settle_time"] = config.startup.postInitStandingSettleTime;
    root["startup"] = std::move(startup);

    json initialPose = json::object();
    if (config.initialPose.hasBasePose) {
        initialPose["base_position_W"] = vectorToJson(config.initialPose.basePosition_W);
        initialPose["base_rpy_W"] = vectorToJson(config.initialPose.baseEuler_W);
    }
    initialPose["leg_joint_offsets"] = stdVectorToJson(config.initialPose.legJointOffsets);
    initialPose["arm_joint_offsets"] = stdVectorToJson(config.initialPose.armJointOffsets);
    initialPose["leg_initialization_time"] = config.initialPose.legInitializationTime;
    initialPose["arm_initialization_time"] = config.initialPose.armInitializationTime;
    root["initial_pose"] = std::move(initialPose);

    json gaitSwingHoldTest = json::object();
    if (!config.gaitSwingHoldTest.xmlPath.empty()) {
        gaitSwingHoldTest["xml_path"] = config.gaitSwingHoldTest.xmlPath;
    }
    root["gait_swing_hold_test"] = std::move(gaitSwingHoldTest);

    return root;
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

DMat<double> buildWrenchToTorqueJacobianFromLegs(const StateEstimate<double>& stateEstimate,
                                                 const RobotParams<double>& robotParams) {
    if (stateEstimate.legs.size() != robotParams.legs.size()) {
        throw std::runtime_error("state estimate leg count does not match robot params");
    }

    Eigen::Index totalDof = 0;
    for (const RobotLegState<double>& legState : stateEstimate.legs) {
        totalDof += legState.qd.size();
    }

    DMat<double> wrenchToTau = DMat<double>::Zero(totalDof, 12);
    Eigen::Index rowOffset = 0;
    for (std::size_t leg = 0; leg < stateEstimate.legs.size(); ++leg) {
        const RobotLegState<double>& legState = stateEstimate.legs[leg];
        const Eigen::Index dof = legState.qd.size();
        if (!legState.hasFootJacobians || legState.Jv_W.rows() != 3 ||
            legState.Jw_W.rows() != 3 || legState.Jv_W.cols() != dof ||
            legState.Jw_W.cols() != dof) {
            throw std::runtime_error("per-leg foot Jacobians are not available");
        }

        int forceColumn = -1;
        int momentColumn = -1;
        switch (robotParams.legs[leg].side) {
            case Side::Left:
                forceColumn = 0;
                momentColumn = 6;
                break;
            case Side::Right:
                forceColumn = 3;
                momentColumn = 9;
                break;
            default:
                throw std::runtime_error("wrench reconstruction only supports left/right legs");
        }

        wrenchToTau.block(rowOffset, forceColumn, dof, 3) = -legState.Jv_W.transpose();
        wrenchToTau.block(rowOffset, momentColumn, dof, 3) = -legState.Jw_W.transpose();
        rowOffset += dof;
    }

    return wrenchToTau;
}

json buildWrenchToTorqueSection(const StandingMpcDebugSnapshot& snapshot,
                                const DVec<double>& actualLegTau,
                                const bool hasStandingFootJacobians) {
    json section = json::object();
    section["input_order"] = "[F_left(3), F_right(3), M_left(3), M_right(3)]";
    section["output_order"] = "packed leg actuator order";
    section["actual_leg_tau_vector"] = vectorToJson(actualLegTau);

    try {
        if (hasStandingFootJacobians) {
            const DMat<double> wrenchToTau =
                buildWrenchToTorqueJacobian(snapshot.stateEstimate.standingFeet);
            section["source"] = "standing_combined_foot_jacobians";
            section["mapping"] =
                "tau = -[Jv_W^T, Jw_W^T] * [F_left, F_right, M_left, M_right]";
            section["standing_Jv_W"] =
                matrixToFlatJson(snapshot.stateEstimate.standingFeet.Jv_W);
            section["standing_Jw_W"] =
                matrixToFlatJson(snapshot.stateEstimate.standingFeet.Jw_W);
            section["wrench_to_tau_jacobian"] = matrixToFlatJson(wrenchToTau);
            return section;
        }

        const DMat<double> wrenchToTau =
            buildWrenchToTorqueJacobianFromLegs(snapshot.stateEstimate, snapshot.robotParams);
        section["source"] = "per_leg_foot_jacobians";
        section["mapping"] =
            "tau = -blockdiag(Jv_left^T, Jv_right^T) * [F_left, F_right] "
            "- blockdiag(Jw_left^T, Jw_right^T) * [M_left, M_right]";
        section["wrench_to_tau_jacobian"] = matrixToFlatJson(wrenchToTau);
    } catch (const std::exception& exception) {
        section["source"] = "unavailable";
        section["mapping"] =
            "not available; no compatible standing or per-leg foot Jacobians were active";
        section["unavailable_reason"] = exception.what();
    }

    return section;
}

Mat3<double> inertiaWorldFromX0(const RobotParams<double>& robotParams,
                                const Vec13<double>& x0) {
    const Mat3<double> R_WB = Rz(x0[2]);
    return R_WB * robotParams.bodyInertia * R_WB.transpose();
}

json buildSnapshotJson(const StandingMpcDebugSnapshot& snapshot,
                       const std::string& logPath,
                       const TimestampStrings& timestamp) {
    const ControllerConfig& config = getControllerConfig();
    const int steps = horizonSteps();
    if (snapshot.wrenchHorizon.size() != kInputDim * steps) {
        throw std::runtime_error("MPC debug log received an unexpected wrench horizon size");
    }
    if (snapshot.formulation.A_qp.rows() != kStateDim * steps ||
        snapshot.formulation.A_qp.cols() != kStateDim ||
        snapshot.formulation.B_qp.rows() != kStateDim * steps ||
        snapshot.formulation.B_qp.cols() != kInputDim * steps) {
        throw std::runtime_error("MPC debug log received unexpected QP matrix dimensions");
    }

    const DVec<double> predictedState =
        snapshot.formulation.A_qp * snapshot.x0 +
        snapshot.formulation.B_qp * snapshot.wrenchHorizon;
    const bool hasStandingFootJacobians = snapshot.stateEstimate.standingFeet.hasFootJacobians;
    const DVec<double> actualLegTau = packActualLegTauVector(snapshot);
    const DVec<double> fullQpos = packFullQpos(snapshot);
    const DVec<double> fullQvel = packFullQvel(snapshot);
    const DVec<double> fullTau = packFullTauCommand(snapshot);

    json root = json::object();

    json metadata = json::object();
    metadata["type"] = locomotionModeName(snapshot.locomotionMode) + std::string("_mpc_solve_debug");
    metadata["generated_at_local"] = timestamp.localTime;
    metadata["log_path"] = logPath;
    metadata["locomotion_state"] = locomotionStateName(snapshot.locomotionState);
    metadata["controller_time"] = snapshot.stateEstimate.time;
    metadata["controller_iteration"] = snapshot.iteration;
    metadata["locomotion_mode"] = locomotionModeName(snapshot.locomotionMode);
    metadata["horizon_clock_t0"] = scalarToJson(snapshot.horizonClockT0);
    metadata["debug_request_source"] = snapshot.debugRequestSource;
    metadata["debug_request_time"] = scalarToJson(snapshot.debugRequestTime);
    metadata["debug_trigger_time"] = scalarToJson(snapshot.debugTriggerTime);
    metadata["horizon_steps"] = steps;
    metadata["dt_mpc"] = dtMpc();
    metadata["robot_type"] = robotTypeName(snapshot.robotParams.roboType);
    metadata["foot_end_effector_source"] =
        footEndEffectorSourceName(config.model.footEndEffectorSource);
    metadata["desired_wrench_reference_point"] =
        desiredWrenchReferencePointName(config.model.footEndEffectorSource);
    metadata["contact_wrench_model"] =
        contactWrenchModelName(contactWrenchModelForMode(config.mpc, snapshot.locomotionMode));
    root["metadata"] = std::move(metadata);

    root["controller_config"] =
        controllerConfigJson(config, snapshot.robotParams.roboType, snapshot.locomotionMode);
    root["user_command"] = userCommandJson(snapshot.userCommand);

    json model = json::object();
    model["mass"] = snapshot.robotParams.bodyMass;
    model["gravity"] = config.model.gravity;
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
        legJson["R_WF"] = matrixToFlatJson(legState.R_WF);
        legJson["foot_x_axis_W"] = vectorToJson(legState.R_WF.col(0));
        legJson["raw_contact"] = legState.contact;
        legJson["has_contact_force"] = legState.hasContactForce;
        legJson["contact_force_W"] = vectorToJson(legState.contactForce_W);
        legJson["contact_normal_force"] = scalarToJson(legState.contactNormalForce);
        if (legState.hasFootJacobians) {
            legJson["Jv_W"] = matrixToFlatJson(legState.Jv_W);
            legJson["Jw_W"] = matrixToFlatJson(legState.Jw_W);
        }
        legJson["q"] = vectorToJson(legState.q);
        legJson["qd"] = vectorToJson(legState.qd);
        legJson["command_mode"] = legControlModeName(legCommand.mode);
        legJson["force_feedforward_W"] = vectorToJson(legCommand.forceFeedForward_W);
        legJson["moment_feedforward_W"] = vectorToJson(legCommand.momentFeedForward_W);
        legJson["p_des_W"] = vectorToJson(legCommand.pDes_W);
        legJson["v_des_W"] = vectorToJson(legCommand.vDes_W);
        legJson["a_des_W"] = vectorToJson(legCommand.aDes_W);
        legJson["tau_feedforward_command"] = vectorToJson(legCommand.tauFeedForward);
        legs.push_back(std::move(legJson));
    }
    feet["legs"] = std::move(legs);
    root["feet"] = std::move(feet);

    json contactManager = json::object();
    contactManager["contact_lock_steps"] = config.contactManager.contactLockSteps;
    contactManager["active_contact_lock_steps"] = config.contactManager.contactLockSteps;
    contactManager["far_horizon_uses_nominal_schedule"] = true;
    json contactManagerLegs = json::array();
    for (const auto& legState : snapshot.contactManagerLegs) {
        json legJson = json::object();
        legJson["side"] = sideName(legState.side);
        legJson["scheduled_contact"] = legState.scheduledContact;
        legJson["estimated_contact"] = legState.estimatedContact;
        legJson["active_contact"] = legState.activeContact;
        legJson["early_contact"] = legState.earlyContact;
        legJson["late_contact"] = legState.lateContact;
        legJson["search_mode_active"] = legState.searchModeActive;
        legJson["recovery_failure"] = legState.recoveryFailure;
        legJson["contact_ramp_alpha"] = scalarToJson(legState.contactRampAlpha);
        legJson["late_contact_time"] = scalarToJson(legState.lateContactTime);
        legJson["contact_normal_force"] = scalarToJson(legState.contactNormalForce);
        legJson["frozen_touchdown_position_W"] =
            vectorToJson(legState.frozenTouchdownPosition_W);
        legJson["commanded_foot_target_W"] =
            vectorToJson(legState.commandedFootTarget_W);
        contactManagerLegs.push_back(std::move(legJson));
    }
    contactManager["legs"] = std::move(contactManagerLegs);
    root["contact_manager_state"] = std::move(contactManager);

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

    const json wrenchToTauSection =
        buildWrenchToTorqueSection(snapshot, actualLegTau, hasStandingFootJacobians);
    root["wrench_to_torque"] = wrenchToTauSection;
    root["standing_wrench_to_torque"] = wrenchToTauSection;

    return root;
}
}  // namespace

std::string writeStandingMpcDebugLog(const StandingMpcDebugSnapshot& snapshot) {
    const TimestampStrings timestamp = makeTimestampStrings();
    const std::string logPath = makeLogPath(timestamp, snapshot.locomotionMode);

    const json root = buildSnapshotJson(snapshot, logPath, timestamp);

    std::ofstream out(logPath, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to open MPC debug log: " + logPath);
    }

    writePrettyJson(out, root);
    out << '\n';
    if (!out.good()) {
        throw std::runtime_error("Failed while writing MPC debug log: " + logPath);
    }

    return logPath;
}
