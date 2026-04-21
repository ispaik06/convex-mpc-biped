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
#include <vector>

#include "Utilities/MatrixUtils.h"

namespace {
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
    const std::time_t time = clock::to_time_t(now);

    std::tm localTime {};
    localtime_r(&time, &localTime);

    std::ostringstream filename;
    filename << std::put_time(&localTime, "%Y%m%d_%H%M%S");

    std::ostringstream readable;
    readable << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S")
             << '.' << std::setw(6) << std::setfill('0')
             << std::chrono::duration_cast<std::chrono::microseconds>(now - seconds).count();

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

void indent(std::ostream& out, const int spaces) {
    for (int i = 0; i < spaces; ++i) {
        out << ' ';
    }
}

void writeEscapedString(std::ostream& out, const std::string& value) {
    out << '"';
    for (const char c : value) {
        switch (c) {
            case '\\':
                out << "\\\\";
                break;
            case '"':
                out << "\\\"";
                break;
            case '\n':
                out << "\\n";
                break;
            case '\r':
                out << "\\r";
                break;
            case '\t':
                out << "\\t";
                break;
            default:
                out << c;
                break;
        }
    }
    out << '"';
}

void writeKey(std::ostream& out, const int spaces, const std::string& key) {
    indent(out, spaces);
    writeEscapedString(out, key);
    out << ": ";
}

void writeNumber(std::ostream& out, const double value) {
    if (std::isfinite(value)) {
        out << value;
    } else {
        out << "null";
    }
}

void writeIntVector(std::ostream& out, const std::vector<int>& values) {
    out << '[';
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            out << ", ";
        }
        out << values[i];
    }
    out << ']';
}

template <typename Derived>
void writeVector(std::ostream& out, const Eigen::MatrixBase<Derived>& vector) {
    out << '[';
    for (Eigen::Index i = 0; i < vector.size(); ++i) {
        if (i > 0) {
            out << ", ";
        }
        writeNumber(out, vector.derived()(i));
    }
    out << ']';
}

template <typename Derived>
void writeMatrix(std::ostream& out, const Eigen::MatrixBase<Derived>& matrix, const int spaces) {
    out << "[\n";
    for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
        indent(out, spaces + 2);
        out << '[';
        for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
            if (col > 0) {
                out << ", ";
            }
            writeNumber(out, matrix.derived()(row, col));
        }
        out << ']';
        if (row + 1 < matrix.rows()) {
            out << ',';
        }
        out << '\n';
    }
    indent(out, spaces);
    out << ']';
}

template <typename MatrixSequence>
void writeMatrixSequence(std::ostream& out, const MatrixSequence& matrices, const int spaces) {
    out << "[\n";
    for (std::size_t i = 0; i < matrices.size(); ++i) {
        indent(out, spaces + 2);
        writeMatrix(out, matrices[i], spaces + 2);
        if (i + 1 < matrices.size()) {
            out << ',';
        }
        out << '\n';
    }
    indent(out, spaces);
    out << ']';
}

void writeVectorByStep(std::ostream& out,
                       const DVec<double>& values,
                       const int rows,
                       const int cols,
                       const int spaces) {
    if (values.size() != rows * cols) {
        throw std::runtime_error("Cannot reshape vector for debug log");
    }

    out << "[\n";
    for (int row = 0; row < rows; ++row) {
        indent(out, spaces + 2);
        out << '[';
        for (int col = 0; col < cols; ++col) {
            if (col > 0) {
                out << ", ";
            }
            writeNumber(out, values[row * cols + col]);
        }
        out << ']';
        if (row + 1 < rows) {
            out << ',';
        }
        out << '\n';
    }
    indent(out, spaces);
    out << ']';
}

Eigen::Index totalLegDof(const LegController<double>& legController) {
    Eigen::Index total = 0;
    for (const auto& data : legController.datas) {
        total += data.dof();
    }
    return total;
}

DVec<double> combinedLegTorqueCommand(const LegController<double>& legController) {
    DVec<double> combined(totalLegDof(legController));

    Eigen::Index offset = 0;
    for (const auto& command : legController.commands) {
        combined.segment(offset, command.dof()) = command.tauFeedForward;
        offset += command.dof();
    }

    return combined;
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

void writeLegArray(std::ostream& out,
                   const StandingMpcDebugSnapshot& snapshot,
                   const int spaces) {
    out << "[\n";
    for (std::size_t leg = 0; leg < snapshot.robotParams.legs.size(); ++leg) {
        const auto& legParams = snapshot.robotParams.legs[leg];
        const auto& legState = snapshot.stateEstimate.legs[leg];
        const auto& legCommand = snapshot.legController.commands[leg];

        indent(out, spaces + 2);
        out << "{\n";
        writeKey(out, spaces + 4, "index");
        out << leg << ",\n";
        writeKey(out, spaces + 4, "side");
        writeEscapedString(out, sideName(legParams.side));
        out << ",\n";
        writeKey(out, spaces + 4, "q_indices");
        writeIntVector(out, legParams.joints.q_idx);
        out << ",\n";
        writeKey(out, spaces + 4, "qd_indices");
        writeIntVector(out, legParams.joints.qd_idx);
        out << ",\n";
        writeKey(out, spaces + 4, "actuator_indices");
        writeIntVector(out, legParams.joints.actuator_idx);
        out << ",\n";
        writeKey(out, spaces + 4, "foot_pos_W");
        writeVector(out, legState.footPos_W);
        out << ",\n";
        writeKey(out, spaces + 4, "foot_vel_W");
        writeVector(out, legState.footVel_W);
        out << ",\n";
        writeKey(out, spaces + 4, "q");
        writeVector(out, legState.q);
        out << ",\n";
        writeKey(out, spaces + 4, "qd");
        writeVector(out, legState.qd);
        out << ",\n";
        writeKey(out, spaces + 4, "tau_feedforward_command");
        writeVector(out, legCommand.tauFeedForward);
        out << '\n';
        indent(out, spaces + 2);
        out << '}';
        if (leg + 1 < snapshot.robotParams.legs.size()) {
            out << ',';
        }
        out << '\n';
    }
    indent(out, spaces);
    out << ']';
}

void writeJson(std::ostream& out,
               const StandingMpcDebugSnapshot& snapshot,
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
    const DVec<double> actualLegTau = combinedLegTorqueCommand(snapshot.legController);

    out << std::setprecision(17);
    out << "{\n";

    writeKey(out, 2, "metadata");
    out << "{\n";
    writeKey(out, 4, "format_version");
    out << 1 << ",\n";
    writeKey(out, 4, "type");
    writeEscapedString(out, "standing_mpc_first_solve_debug");
    out << ",\n";
    writeKey(out, 4, "generated_at_local");
    writeEscapedString(out, timestamp.localTime);
    out << ",\n";
    writeKey(out, 4, "log_path");
    writeEscapedString(out, logPath);
    out << ",\n";
    writeKey(out, 4, "controller_time");
    writeNumber(out, snapshot.stateEstimate.time);
    out << ",\n";
    writeKey(out, 4, "controller_iteration");
    out << snapshot.iteration << ",\n";
    writeKey(out, 4, "horizon_steps");
    out << steps << ",\n";
    writeKey(out, 4, "dt_mpc");
    writeNumber(out, dtMpc());
    out << '\n';
    indent(out, 2);
    out << "},\n";

    writeKey(out, 2, "model");
    out << "{\n";
    writeKey(out, 4, "mass");
    writeNumber(out, snapshot.robotParams.bodyMass);
    out << ",\n";
    writeKey(out, 4, "gravity");
    writeNumber(out, getControllerConfig().model.gravity);
    out << ",\n";
    writeKey(out, 4, "body_com_location_yaw_frame");
    writeVector(out, snapshot.robotParams.bodyComLocation);
    out << ",\n";
    writeKey(out, 4, "body_inertia_yaw_frame");
    writeMatrix(out, snapshot.robotParams.bodyInertia, 4);
    out << ",\n";
    writeKey(out, 4, "inertia_world_from_x0");
    writeMatrix(out, inertiaWorldFromX0(snapshot.robotParams, snapshot.x0), 4);
    out << '\n';
    indent(out, 2);
    out << "},\n";

    writeKey(out, 2, "initial_state");
    out << "{\n";
    writeKey(out, 4, "x0");
    writeVector(out, snapshot.x0);
    out << ",\n";
    writeKey(out, 4, "psi");
    writeNumber(out, snapshot.x0[2]);
    out << ",\n";
    writeKey(out, 4, "torso_pos_W");
    writeVector(out, snapshot.stateEstimate.torsoPos_W);
    out << ",\n";
    writeKey(out, 4, "torso_lin_vel_W");
    writeVector(out, snapshot.stateEstimate.torsoLinVel_W);
    out << ",\n";
    writeKey(out, 4, "torso_ang_vel_W");
    writeVector(out, snapshot.stateEstimate.torsoAngVel_W);
    out << '\n';
    indent(out, 2);
    out << "},\n";

    writeKey(out, 2, "feet");
    out << "{\n";
    writeKey(out, 4, "desired_foot_pos_W");
    out << "{\n";
    writeKey(out, 6, "left");
    writeVector(out, snapshot.desiredFootPositions.left_des_W);
    out << ",\n";
    writeKey(out, 6, "right");
    writeVector(out, snapshot.desiredFootPositions.right_des_W);
    out << '\n';
    indent(out, 4);
    out << "},\n";
    writeKey(out, 4, "legs");
    writeLegArray(out, snapshot, 4);
    out << '\n';
    indent(out, 2);
    out << "},\n";

    writeKey(out, 2, "reference_trajectory");
    out << "{\n";
    writeKey(out, 4, "tk");
    writeVector(out, snapshot.referenceTrajectory.tk);
    out << ",\n";
    writeKey(out, 4, "psi");
    writeVector(out, snapshot.referenceTrajectory.psi);
    out << ",\n";
    writeKey(out, 4, "r_left");
    writeMatrix(out, snapshot.referenceTrajectory.r_left, 4);
    out << ",\n";
    writeKey(out, 4, "r_right");
    writeMatrix(out, snapshot.referenceTrajectory.r_right, 4);
    out << ",\n";
    writeKey(out, 4, "X_ref_by_step");
    writeVectorByStep(out, snapshot.referenceTrajectory.X_ref, steps, kStateDim, 4);
    out << '\n';
    indent(out, 2);
    out << "},\n";

    writeKey(out, 2, "formulation");
    out << "{\n";
    writeKey(out, 4, "input_order");
    writeEscapedString(out, "[F_left(3), F_right(3), M_left(3), M_right(3)]");
    out << ",\n";
    writeKey(out, 4, "A_c");
    writeMatrixSequence(out, snapshot.formulation.A_c, 4);
    out << ",\n";
    writeKey(out, 4, "B_c");
    writeMatrixSequence(out, snapshot.formulation.B_c, 4);
    out << ",\n";
    writeKey(out, 4, "inertia_world");
    writeMatrixSequence(out, snapshot.formulation.inertia_W, 4);
    out << ",\n";
    writeKey(out, 4, "A_qp");
    writeMatrix(out, snapshot.formulation.A_qp, 4);
    out << ",\n";
    writeKey(out, 4, "B_qp");
    writeMatrix(out, snapshot.formulation.B_qp, 4);
    out << '\n';
    indent(out, 2);
    out << "},\n";

    writeKey(out, 2, "solution");
    out << "{\n";
    writeKey(out, 4, "wrench_horizon");
    writeVectorByStep(out, snapshot.wrenchHorizon, steps, kInputDim, 4);
    out << ",\n";
    writeKey(out, 4, "wrench_horizon_vector");
    writeVector(out, snapshot.wrenchHorizon);
    out << ",\n";
    writeKey(out, 4, "first_wrench");
    writeVector(out, snapshot.wrenchHorizon.head(kInputDim));
    out << ",\n";
    writeKey(out, 4, "predicted_state_horizon");
    writeVectorByStep(out, predictedState, steps, kStateDim, 4);
    out << ",\n";
    writeKey(out, 4, "predicted_state_horizon_vector");
    writeVector(out, predictedState);
    out << '\n';
    indent(out, 2);
    out << "},\n";

    writeKey(out, 2, "standing_wrench_to_torque");
    out << "{\n";
    writeKey(out, 4, "mapping");
    writeEscapedString(out, "tau = -[Jv_W^T, Jw_W^T] * [F_left, F_right, M_left, M_right]");
    out << ",\n";
    writeKey(out, 4, "standing_Jv_W");
    writeMatrix(out, snapshot.stateEstimate.standingFeet.Jv_W, 4);
    out << ",\n";
    writeKey(out, 4, "standing_Jw_W");
    writeMatrix(out, snapshot.stateEstimate.standingFeet.Jw_W, 4);
    out << ",\n";
    writeKey(out, 4, "wrench_to_tau_jacobian");
    writeMatrix(out, wrenchToTau, 4);
    out << ",\n";
    writeKey(out, 4, "actual_leg_tau_vector");
    writeVector(out, actualLegTau);
    out << '\n';
    indent(out, 2);
    out << "}\n";

    out << "}\n";
}
}  // namespace

std::string writeStandingMpcDebugLog(const StandingMpcDebugSnapshot& snapshot) {
    const TimestampStrings timestamp = makeTimestampStrings();
    const std::string logPath = makeLogPath(timestamp);

    std::ofstream out(logPath);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to open standing MPC debug log: " + logPath);
    }

    writeJson(out, snapshot, logPath, timestamp);
    if (!out.good()) {
        throw std::runtime_error("Failed while writing standing MPC debug log: " + logPath);
    }

    return logPath;
}
