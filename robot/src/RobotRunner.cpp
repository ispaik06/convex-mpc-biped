#include <stdexcept>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "RobotRunner.h"
#include "InitialPoseConfig.h"
#include "JointTrackingConfig.h"
#include "Utilities/Timing.h"
#include "Utilities/MatrixUtils.h"

namespace {
Vec3<double> reducedBodyComWorld(const StateEstimate<double>& state,
                                 const RobotParams<double>& params) {
    return state.torsoPos_W + Rz(state.psi) * params.bodyComLocation;
}

std::string formatTimeSeconds(const double time) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << time;
    return out.str();
}

void applyConfiguredJointTrackingGains(const char* limbName,
                                       const std::vector<double>& kp,
                                       const std::vector<double>& kd,
                                       const Eigen::Index expectedDof,
                                       JointPdGains<double>& gains) {
    if (kp.empty() || kd.empty()) {
        throw std::runtime_error(std::string("Missing joint_tracking.") + limbName +
                                 " kp/kd configuration");
    }

    if (kp.size() != kd.size()) {
        throw std::runtime_error(std::string("Configured ") + limbName +
                                 " joint tracking gains have mismatched kp/kd lengths");
    }

    if (static_cast<Eigen::Index>(kp.size()) != expectedDof) {
        throw std::runtime_error(std::string("Configured ") + limbName +
                                 " joint tracking gains do not match limb dof");
    }

    gains.set(kp, kd);
}
}  // namespace

void RobotRunner::init(RobotParams<double>* params, double timestep, const UserCommand* userCommand) {
    _params = params;
    _model = RobotModel<double>(_params);
    _timestep = timestep;
    _robot_ctrl->_userCommand = userCommand;
    if(!_model.validate()) {
        throw std::runtime_error("Invalid RobotParams");
    }
    _robotType = _params->roboType;
    _legController = std::make_unique<LegController<double>>(_model);
    _armController = std::make_unique<ArmController<double>>(_model);
    _robot_ctrl->_robotModel = &_model;
    _robot_ctrl->_robotType = _robotType;
    _robot_ctrl->_robotParams = _params;
    _robot_ctrl->_legController = _legController.get();
    _robot_ctrl->_armController = _armController.get();

    const auto& initialPoseConfig = getInitialPoseConfig(_params->roboType);
    _legPosInitializer = std::make_unique<LegPosInitializer<double>>(
        _params, initialPoseConfig.legInitializationTime, _timestep);
    _armPosInitializer = std::make_unique<ArmPosInitializer<double>>(
        _params, initialPoseConfig.armInitializationTime, _timestep);
    initializeJointTrackingGains();
}

void RobotRunner::prepareController(const StateEstimate<double>& state) {
    if (_robot_ctrl == nullptr) {
        throw std::runtime_error("RobotRunner requires a RobotController");
    }

    _robot_ctrl->_stateEstimate = &state;
    if (_legInitializationComplete) {
        _robot_ctrl->prepareController();
    }
}

LegDynamicsRequest RobotRunner::legDynamicsRequest() const {
    if (_robot_ctrl == nullptr) {
        return {};
    }
    return _robot_ctrl->legDynamicsRequest();
}

UserCommand RobotRunner::dashboardUserCommand() const {
    if (_robot_ctrl == nullptr) {
        return UserCommand{};
    }
    return _robot_ctrl->dashboardUserCommand();
}

void RobotRunner::run(const StateEstimate<double>& state, RobotCommand<double>& command) {
    if (!_legController) {
        throw std::runtime_error("RobotRunner::init must be called before run");
    }

    _robot_ctrl->_stateEstimate = &state;
    setupStep(state);

    const bool armPosInitialized = _armPosInitializer->IsInitialized(_armController.get());
    _armInitializationComplete = armPosInitialized;
    for (auto& armCommand : _armController->commands) {
        _armJointTrackingGains.applyTo(armCommand);
    }

    const bool wasLegInitializationComplete = _legInitializationComplete;
    const bool legPosInitialized = _legPosInitializer->IsInitialized(_legController.get());
    _legInitializationComplete = legPosInitialized;
    if (!wasLegInitializationComplete && _legInitializationComplete) {
        std::cout << "[RobotRunner] PD leg initializer complete at t="
                  << formatTimeSeconds(state.time) << std::endl;
    }

    if(!legPosInitialized) {
        // Initial pose control
        for (auto& legCommand : _legController->commands) {
            _legJointTrackingGains.applyTo(legCommand);
            legCommand.mode = LegControlMode::JointPd;
        }

        // if ((_iterations % 50) == 0 && _params != nullptr) {
        //     const double reducedComZ = reducedBodyComWorld(state, *_params)[2];
        //     std::cout << "[LegPosInitializer] SRB COM z (world): " << reducedComZ << std::endl;
        // }
    }
    else {
        // Run main Controller
        _robot_ctrl->runController();
    }

    _legController->setEnabled(true);
    _armController->setEnabled(true);
    {
        profiling::ScopedTimer timer(_composeCommandTime);
        composeCommand(command);
    }

    finalizeStep();
}

void RobotRunner::composeCommand(RobotCommand<double>& command) const {
    command.tau.setZero(_model.nu());
    _legController->updateCommand(command.tau);
    _armController->updateCommand(command.tau);
}

void RobotRunner::initializeJointTrackingGains() {
    if (_params == nullptr) {
        throw std::runtime_error("RobotRunner::init must set RobotParams before gains");
    }

    const auto& config = getJointTrackingConfig();

    if (!_params->legs.empty()) {
        applyConfiguredJointTrackingGains("leg",
                                          config.legKp,
                                          config.legKd,
                                          static_cast<Eigen::Index>(_params->legs.front().joints.q_idx.size()),
                                          _legJointTrackingGains);
    } else if (config.hasLegGains()) {
        throw std::runtime_error("joint_tracking.leg configured, but robot has no legs");
    }

    if (!_params->arms.empty()) {
        applyConfiguredJointTrackingGains("arm",
                                          config.armKp,
                                          config.armKd,
                                          static_cast<Eigen::Index>(_params->arms.front().joints.q_idx.size()),
                                          _armJointTrackingGains);
    } else if (config.hasArmGains()) {
        throw std::runtime_error("joint_tracking.arm configured, but robot has no arms");
    }
}

void RobotRunner::setupStep(const StateEstimate<double>& state) {
    if (state.legs.size() != _model.numLegs()) {
        throw std::invalid_argument("StateEstimate leg count does not match RobotModel");
    }
    if (state.arms.size() != _model.numArms()) {
        throw std::invalid_argument("StateEstimate arm count does not match RobotModel");
    }

    _legController->zeroCommand();
    _legController->zeroData();
    _armController->zeroCommand();
    _armController->zeroData();

    for (std::size_t leg = 0; leg < state.legs.size(); ++leg) {
        const auto& legState = state.legs[leg];
        const Eigen::Index q_size =
            static_cast<Eigen::Index>(_model.legQIndices(static_cast<int>(leg)).size());
        const Eigen::Index qd_size =
            static_cast<Eigen::Index>(_model.legQdIndices(static_cast<int>(leg)).size());
        const Eigen::Index tau_size =
            static_cast<Eigen::Index>(_model.legActuatorIndices(static_cast<int>(leg)).size());

        if (legState.q.size() != q_size || legState.qd.size() != qd_size) {
            throw std::invalid_argument("RobotLegState joint dimension does not match RobotModel");
        }

        _legController->setLegJointData(static_cast<int>(leg), legState.q, legState.qd);

        if (legState.tauEstimate.size() == tau_size) {
            _legController->setLegTauEstimate(static_cast<int>(leg), legState.tauEstimate);
        } else if (legState.tauEstimate.size() != 0) {
            throw std::invalid_argument(
                "RobotLegState tauEstimate dimension does not match RobotModel");
        }

        if (legState.hasFootJacobians) {
            _legController->setLegCartesianData(static_cast<int>(leg),
                                                legState.footPos_W,
                                                legState.footVel_W,
                                                legState.Jv_W,
                                                legState.JvDot_W,
                                                legState.Jw_W);
        }

        if (legState.hasLegDynamics) {
            _legController->setLegDynamicsData(static_cast<int>(leg),
                                               legState.massMatrix,
                                               legState.bias);
        }
    }

    for (std::size_t arm = 0; arm < state.arms.size(); ++arm) {
        const auto& armState = state.arms[arm];
        const Eigen::Index q_size =
            static_cast<Eigen::Index>(_model.armQIndices(static_cast<int>(arm)).size());
        const Eigen::Index qd_size =
            static_cast<Eigen::Index>(_model.armQdIndices(static_cast<int>(arm)).size());
        const Eigen::Index tau_size =
            static_cast<Eigen::Index>(_model.armActuatorIndices(static_cast<int>(arm)).size());

        if (armState.q.size() != q_size || armState.qd.size() != qd_size) {
            throw std::invalid_argument("RobotArmState joint dimension does not match RobotModel");
        }

        _armController->setArmJointData(static_cast<int>(arm), armState.q, armState.qd);

        if (armState.tauEstimate.size() == tau_size) {
            _armController->setArmTauEstimate(static_cast<int>(arm), armState.tauEstimate);
        } else if (armState.tauEstimate.size() != 0) {
            throw std::invalid_argument(
                "RobotArmState tauEstimate dimension does not match RobotModel");
        }
    }
}

void RobotRunner::finalizeStep() {
    _iterations++;
}

bool RobotRunner::legInitializationComplete() const {
    return _legInitializationComplete;
}

bool RobotRunner::armInitializationComplete() const {
    return _armInitializationComplete;
}

bool RobotRunner::initializationComplete() const {
    return _legInitializationComplete && _armInitializationComplete;
}

void RobotRunner::printProfilingSummary(std::ostream& out) const {
    if (!profiling::enabled()) {
        return;
    }

    if (_robot_ctrl != nullptr) {
        _robot_ctrl->printProfilingSummary(out);
    }
    out << profiling::formatTimingStats("torque_compose", _composeCommandTime) << '\n';
}

const LegController<double>* RobotRunner::legController() const {
    return _legController.get();
}
