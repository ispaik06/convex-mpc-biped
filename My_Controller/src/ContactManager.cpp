#include "ContactManager.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace {
double clampUnitInterval(const double value) {
    return std::clamp(value, 0.0, 1.0);
}

std::string sideLabel(const Side side) {
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

Side oppositeBipedSide(const Side side) {
    if (side == Side::Left) {
        return Side::Right;
    }
    if (side == Side::Right) {
        return Side::Left;
    }
    return side;
}

int legIndexForSide(const RobotParams<double>& robotParams, const Side side) {
    for (std::size_t leg = 0; leg < robotParams.legs.size(); ++leg) {
        if (robotParams.legs[leg].side == side) {
            return static_cast<int>(leg);
        }
    }
    return -1;
}

Vec3<double> yawRotate(const Vec3<double>& v_B, const double yaw) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return Vec3<double>(
        c * v_B.x() - s * v_B.y(),
        s * v_B.x() + c * v_B.y(),
        v_B.z());
}

double rollFromQuaternion(Quat<double> quat) {
    quat.normalize();
    const double w = quat.w();
    const double x = quat.x();
    const double y = quat.y();
    const double z = quat.z();
    return std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
}
}  // namespace

ContactManager::ContactManager(const ContactManagerParameters& params)
    : _params(params) {}

void ContactManager::ensureRuntimeLayout(const RobotParams<double>& robotParams) {
    if (_legs.size() == robotParams.legs.size()) {
        bool matches = true;
        for (std::size_t leg = 0; leg < robotParams.legs.size(); ++leg) {
            if (_legs[leg].publicState.side != robotParams.legs[leg].side) {
                matches = false;
                break;
            }
        }
        if (matches) {
            return;
        }
    }

    _legs.clear();
    _legs.resize(robotParams.legs.size());
    for (std::size_t leg = 0; leg < robotParams.legs.size(); ++leg) {
        _legs[leg].publicState.side = robotParams.legs[leg].side;
    }
    _hasLastUpdateTime = false;
}

ContactManager::RuntimeLegState& ContactManager::runtimeForSide(const Side side) {
    for (auto& leg : _legs) {
        if (leg.publicState.side == side) {
            return leg;
        }
    }
    throw std::runtime_error("ContactManager cannot find side " + sideLabel(side));
}

const ContactManager::RuntimeLegState& ContactManager::runtimeForSide(const Side side) const {
    for (const auto& leg : _legs) {
        if (leg.publicState.side == side) {
            return leg;
        }
    }
    throw std::runtime_error("ContactManager cannot find side " + sideLabel(side));
}

Vec3<double> ContactManager::nominalDesiredFootPosition(
    const DesiredFootPositions& desiredFootPositions,
    const Side side) const {
    switch (side) {
        case Side::Left:
            return desiredFootPositions.left_des_W;
        case Side::Right:
            return desiredFootPositions.right_des_W;
        default:
            throw std::runtime_error("ContactManager only supports left/right foot targets");
    }
}

bool ContactManager::updateEstimatedContact(RuntimeLegState& runtime,
                                            const RobotLegState<double>& legState) const {
    const bool hasForceSignal = legState.hasContactForce &&
                                std::isfinite(legState.contactNormalForce);
    const double normalForce = hasForceSignal ? legState.contactNormalForce : 0.0;

    if (!runtime.initialized) {
        if (hasForceSignal) {
            runtime.publicState.estimatedContact =
                normalForce >= _params.contactForceOnThreshold;
        } else {
            runtime.publicState.estimatedContact = legState.contact;
        }
        runtime.rawContactEstimate = runtime.publicState.estimatedContact;
        runtime.contactOnTicks = 0;
        runtime.contactOffTicks = 0;
        return runtime.publicState.estimatedContact;
    }

    if (!hasForceSignal) {
        if (legState.contact) {
            ++runtime.contactOnTicks;
            runtime.contactOffTicks = 0;
        } else {
            ++runtime.contactOffTicks;
            runtime.contactOnTicks = 0;
        }
    } else if (runtime.publicState.estimatedContact) {
        if (normalForce <= _params.contactForceOffThreshold) {
            ++runtime.contactOffTicks;
            runtime.contactOnTicks = 0;
        } else {
            runtime.contactOffTicks = 0;
        }
    } else {
        if (normalForce >= _params.contactForceOnThreshold) {
            ++runtime.contactOnTicks;
            runtime.contactOffTicks = 0;
        } else {
            runtime.contactOnTicks = 0;
        }
    }

    if (!runtime.publicState.estimatedContact &&
        runtime.contactOnTicks >= std::max(_params.contactOnConfirmTicks, 1)) {
        runtime.publicState.estimatedContact = true;
        runtime.contactOnTicks = 0;
        runtime.contactOffTicks = 0;
    } else if (runtime.publicState.estimatedContact &&
               runtime.contactOffTicks >= std::max(_params.contactOffConfirmTicks, 1)) {
        runtime.publicState.estimatedContact = false;
        runtime.contactOnTicks = 0;
        runtime.contactOffTicks = 0;
    }

    runtime.rawContactEstimate = hasForceSignal
                                     ? normalForce >= _params.contactForceOnThreshold
                                     : legState.contact;
    return runtime.publicState.estimatedContact;
}

bool ContactManager::shouldHoldLiftoff(RuntimeLegState& runtime,
                                       const ContactManagerLegState& state,
                                       const StateEstimate<double>& stateEstimate,
                                       const RobotParams<double>& robotParams,
                                       const double dt) const {
    const auto& balance = getControllerConfig().walkingBalance;
    if (!balance.enableLiftoffGuard || state.scheduledContact ||
        runtime.liftoffReleasedDuringSwing) {
        runtime.liftoffHoldTime = 0.0;
        return false;
    }

    runtime.liftoffHoldTime += std::max(0.0, dt);

    const int swingLeg = legIndexForSide(robotParams, state.side);
    const int supportLeg = legIndexForSide(robotParams, oppositeBipedSide(state.side));
    if (swingLeg < 0 || supportLeg < 0 ||
        static_cast<std::size_t>(swingLeg) >= stateEstimate.legs.size() ||
        static_cast<std::size_t>(supportLeg) >= stateEstimate.legs.size()) {
        runtime.liftoffReleasedDuringSwing = true;
        runtime.liftoffHoldTime = 0.0;
        return false;
    }

    const Vec3<double> com_W =
        stateEstimate.torsoPos_W + yawRotate(robotParams.bodyComLocation, stateEstimate.psi);
    const double swingFootZ =
        stateEstimate.legs[static_cast<std::size_t>(swingLeg)].footPos_W.z();
    const double swingY =
        stateEstimate.legs[static_cast<std::size_t>(swingLeg)].footPos_W.y();
    const double supportY =
        stateEstimate.legs[static_cast<std::size_t>(supportLeg)].footPos_W.y();
    const double midpointY = 0.5 * (swingY + supportY);
    const double releaseY =
        midpointY + balance.liftoffComLateralFraction * (supportY - midpointY);

    const bool supportIsNegativeSide = supportY < midpointY;
    const bool comReady = supportIsNegativeSide
                              ? com_W.y() <= releaseY + balance.liftoffComLateralTolerance
                              : com_W.y() >= releaseY - balance.liftoffComLateralTolerance;
    const bool rollReady =
        balance.liftoffMaxAbsRoll <= 0.0 ||
        std::abs(rollFromQuaternion(stateEstimate.torsoQuat_W)) <= balance.liftoffMaxAbsRoll;
    const bool rollRateReady =
        balance.liftoffMaxAbsRollRate <= 0.0 ||
        std::abs(stateEstimate.torsoAngVel_W.x()) <= balance.liftoffMaxAbsRollRate;
    const bool heightReady =
        balance.liftoffMinComHeight <= 0.0 || com_W.z() >= balance.liftoffMinComHeight;
    const bool postureReady = rollReady && rollRateReady && heightReady;
    const bool footNearGround =
        balance.liftoffMaxHoldFootHeight > 0.0 &&
        swingFootZ <= balance.liftoffMaxHoldFootHeight;
    if (!postureReady && footNearGround) {
        return true;
    }

    if (!state.estimatedContact || runtime.releasedContactDuringSwing) {
        runtime.liftoffReleasedDuringSwing = true;
        runtime.liftoffHoldTime = 0.0;
        return false;
    }

    const bool timedOut =
        balance.liftoffMaxDelay > 0.0 && runtime.liftoffHoldTime >= balance.liftoffMaxDelay;
    if (postureReady && (comReady || timedOut)) {
        runtime.liftoffReleasedDuringSwing = true;
        runtime.liftoffHoldTime = 0.0;
        return false;
    }

    return true;
}

void ContactManager::reset(const StateEstimate<double>& stateEstimate,
                           const RobotParams<double>& robotParams,
                           const GaitScheduler& gaitScheduler,
                           const LocomotionMode locomotionMode) {
    ensureRuntimeLayout(robotParams);
    if (stateEstimate.legs.size() != robotParams.legs.size()) {
        throw std::runtime_error("ContactManager reset received mismatched leg state count");
    }

    _lastUpdateTime = stateEstimate.time;
    _hasLastUpdateTime = true;

    for (std::size_t leg = 0; leg < robotParams.legs.size(); ++leg) {
        auto& runtime = _legs[leg];
        const auto& legState = stateEstimate.legs[leg];
        ContactManagerLegState& state = runtime.publicState;
        state.side = robotParams.legs[leg].side;
        state.scheduledContact = gaitScheduler.c(state.side, stateEstimate.time);
        state.estimatedContact =
            legState.hasContactForce
                ? legState.contactNormalForce >= _params.contactForceOnThreshold
                : legState.contact;
        state.activeContact =
            locomotionMode == LocomotionMode::Standing ? true : state.scheduledContact;
        state.earlyContact = false;
        state.lateContact = false;
        state.liftoffHold = false;
        state.searchModeActive = false;
        state.recoveryFailure = false;
        state.contactRampAlpha = state.activeContact ? 1.0 : 0.0;
        state.lateContactTime = 0.0;
        state.liftoffHoldTime = 0.0;
        state.contactNormalForce =
            legState.hasContactForce ? legState.contactNormalForce : 0.0;
        state.frozenTouchdownPosition_W = legState.footPos_W;
        state.commandedFootTarget_W = legState.footPos_W;

        runtime.initialized = true;
        runtime.rawContactEstimate = state.estimatedContact;
        runtime.previousActiveContact = state.activeContact;
        runtime.previousScheduledContact = state.scheduledContact;
        runtime.previousLateContact = false;
        runtime.releasedContactDuringSwing = !state.scheduledContact && !state.estimatedContact;
        runtime.liftoffReleasedDuringSwing = !state.scheduledContact;
        runtime.contactOnTicks = 0;
        runtime.contactOffTicks = 0;
        runtime.contactRampTime = state.activeContact ? _params.contactRampDuration : 0.0;
        runtime.searchDepth = 0.0;
        runtime.liftoffHoldTime = 0.0;
    }
}

void ContactManager::update(const StateEstimate<double>& stateEstimate,
                            const RobotParams<double>& robotParams,
                            const GaitScheduler& gaitScheduler,
                            const LocomotionMode locomotionMode,
                            const DesiredFootPositions& nominalDesiredFootPositions,
                            const bool recoveryHoldActive) {
    ensureRuntimeLayout(robotParams);
    if (stateEstimate.legs.size() != robotParams.legs.size()) {
        throw std::runtime_error("ContactManager update received mismatched leg state count");
    }

    const double time = stateEstimate.time;
    const double dt = (_hasLastUpdateTime && time >= _lastUpdateTime)
                          ? (time - _lastUpdateTime)
                          : 0.0;
    _lastUpdateTime = time;
    _hasLastUpdateTime = true;

    for (std::size_t leg = 0; leg < robotParams.legs.size(); ++leg) {
        auto& runtime = _legs[leg];
        const auto& legState = stateEstimate.legs[leg];
        ContactManagerLegState& state = runtime.publicState;
        state.side = robotParams.legs[leg].side;
        state.scheduledContact = gaitScheduler.c(state.side, time);
        state.contactNormalForce =
            legState.hasContactForce ? legState.contactNormalForce : 0.0;
        const bool estimatedContact = updateEstimatedContact(runtime, legState);
        state.estimatedContact = estimatedContact;

        const Vec3<double> nominalTarget =
            nominalDesiredFootPosition(nominalDesiredFootPositions, state.side);

        if (!runtime.initialized) {
            runtime.initialized = true;
            runtime.previousActiveContact = state.scheduledContact;
            runtime.previousScheduledContact = state.scheduledContact;
            runtime.previousLateContact = false;
            runtime.releasedContactDuringSwing =
                !state.scheduledContact && !estimatedContact;
            runtime.liftoffReleasedDuringSwing = !state.scheduledContact;
            runtime.contactRampTime =
                state.scheduledContact ? _params.contactRampDuration : 0.0;
            runtime.searchDepth = 0.0;
            runtime.liftoffHoldTime = 0.0;
            state.frozenTouchdownPosition_W =
                state.scheduledContact ? legState.footPos_W : nominalTarget;
        }

        if (locomotionMode == LocomotionMode::Standing) {
            state.activeContact = true;
            state.earlyContact = false;
            state.lateContact = false;
            state.liftoffHold = false;
            state.searchModeActive = false;
            state.recoveryFailure = false;
            state.contactRampAlpha = 1.0;
            state.lateContactTime = 0.0;
            state.liftoffHoldTime = 0.0;
            state.frozenTouchdownPosition_W = legState.footPos_W;
            state.commandedFootTarget_W = legState.footPos_W;
            runtime.previousActiveContact = true;
            runtime.previousScheduledContact = true;
            runtime.previousLateContact = false;
            runtime.releasedContactDuringSwing = false;
            runtime.liftoffReleasedDuringSwing = false;
            runtime.contactRampTime = _params.contactRampDuration;
            runtime.searchDepth = 0.0;
            runtime.liftoffHoldTime = 0.0;
            continue;
        }

        if (state.scheduledContact) {
            runtime.releasedContactDuringSwing = false;
            runtime.liftoffReleasedDuringSwing = false;
            runtime.liftoffHoldTime = 0.0;
        } else if (!estimatedContact) {
            runtime.releasedContactDuringSwing = true;
        }

        state.earlyContact =
            _params.enableEarlyContactHandling && !state.scheduledContact &&
            estimatedContact && runtime.releasedContactDuringSwing;

        const bool scheduledTouchdown =
            state.scheduledContact && !runtime.previousScheduledContact;
        const bool continuingLateTouchdown =
            state.scheduledContact && runtime.previousLateContact && !estimatedContact;
        const bool lostEstablishedStanceContact =
            state.scheduledContact && !estimatedContact &&
            !scheduledTouchdown && !runtime.previousLateContact &&
            legState.footPos_W.z() > _params.stanceContactLossFootHeight;
        const bool recoveryMissingScheduledContact =
            recoveryHoldActive && state.scheduledContact && !estimatedContact;
        state.lateContact =
            _params.enableLateContactHandling && !estimatedContact &&
            (recoveryMissingScheduledContact ||
             scheduledTouchdown ||
             continuingLateTouchdown ||
             lostEstablishedStanceContact);
        state.liftoffHold =
            shouldHoldLiftoff(runtime, state, stateEstimate, robotParams, dt);
        state.liftoffHoldTime = state.liftoffHold ? runtime.liftoffHoldTime : 0.0;

        if (state.earlyContact && !runtime.previousActiveContact) {
            state.frozenTouchdownPosition_W = legState.footPos_W;
        }

        if (state.lateContact) {
            if (!runtime.previousLateContact) {
                state.frozenTouchdownPosition_W = nominalTarget;
                state.lateContactTime = 0.0;
                runtime.searchDepth = 0.0;
            } else {
                state.lateContactTime += dt;
                runtime.searchDepth += _params.groundSearchVelocity * dt;
            }
            runtime.searchDepth = std::clamp(runtime.searchDepth,
                                             0.0,
                                             _params.groundSearchMaxDepth);
            state.searchModeActive = true;
            state.recoveryFailure =
                _params.lateContactTimeout > 0.0 &&
                state.lateContactTime > _params.lateContactTimeout;

            Vec3<double> searchTarget = state.frozenTouchdownPosition_W;
            searchTarget.z() -= runtime.searchDepth;
            state.commandedFootTarget_W = searchTarget;
        } else {
            state.lateContactTime = 0.0;
            state.searchModeActive = false;
            state.recoveryFailure = false;
            runtime.searchDepth = 0.0;
            state.commandedFootTarget_W =
                state.liftoffHold ? legState.footPos_W
                                  : (state.earlyContact ? state.frozenTouchdownPosition_W
                                                        : nominalTarget);
        }

        if (state.lateContact) {
            state.activeContact = false;
        } else if (state.liftoffHold) {
            state.activeContact = true;
        } else if (state.earlyContact) {
            state.activeContact = true;
        } else {
            state.activeContact = state.scheduledContact;
            if (state.activeContact) {
                state.frozenTouchdownPosition_W = legState.footPos_W;
            }
        }

        if (state.activeContact) {
            if (!runtime.previousActiveContact) {
                runtime.contactRampTime = 0.0;
            } else {
                runtime.contactRampTime += dt;
            }
            state.contactRampAlpha =
                _params.contactRampDuration <= 0.0
                    ? 1.0
                    : clampUnitInterval(runtime.contactRampTime / _params.contactRampDuration);
        } else {
            runtime.contactRampTime = 0.0;
            state.contactRampAlpha = 0.0;
        }

        runtime.previousActiveContact = state.activeContact;
        runtime.previousScheduledContact = state.scheduledContact;
        runtime.previousLateContact = state.lateContact;
    }
}

bool ContactManager::activeContact(const Side side) const {
    return runtimeForSide(side).publicState.activeContact;
}

bool ContactManager::scheduledContact(const Side side) const {
    return runtimeForSide(side).publicState.scheduledContact;
}

bool ContactManager::estimatedContact(const Side side) const {
    return runtimeForSide(side).publicState.estimatedContact;
}

bool ContactManager::searchModeActive(const Side side) const {
    return runtimeForSide(side).publicState.searchModeActive;
}

double ContactManager::contactRampAlpha(const Side side) const {
    return runtimeForSide(side).publicState.contactRampAlpha;
}

const ContactManagerLegState& ContactManager::legState(const Side side) const {
    return runtimeForSide(side).publicState;
}

vectorAligned<ContactManagerLegState> ContactManager::legStates() const {
    vectorAligned<ContactManagerLegState> states;
    states.reserve(_legs.size());
    for (const auto& runtime : _legs) {
        states.push_back(runtime.publicState);
    }
    return states;
}

DesiredFootPositions ContactManager::managedFootPositions(
    const DesiredFootPositions& nominalDesiredFootPositions) const {
    DesiredFootPositions out = nominalDesiredFootPositions;
    for (const auto& runtime : _legs) {
        const auto& state = runtime.publicState;
        if (!state.searchModeActive && !state.earlyContact && !state.liftoffHold) {
            continue;
        }
        switch (state.side) {
            case Side::Left:
                out.left_des_W = state.commandedFootTarget_W;
                break;
            case Side::Right:
                out.right_des_W = state.commandedFootTarget_W;
                break;
            default:
                break;
        }
    }
    return out;
}

ContactScheduleOverride ContactManager::buildHorizonOverride() const {
    ContactScheduleOverride override;
    const int lockSteps = std::max(_params.contactLockSteps, 0);
    override.steps.resize(static_cast<std::size_t>(lockSteps));

    bool leftContact = true;
    bool rightContact = true;
    double leftScale = 1.0;
    double rightScale = 1.0;

    for (const auto& runtime : _legs) {
        const auto& state = runtime.publicState;
        if (state.side == Side::Left) {
            leftContact = state.activeContact;
            leftScale = state.activeContact ? state.contactRampAlpha : 0.0;
        } else if (state.side == Side::Right) {
            rightContact = state.activeContact;
            rightScale = state.activeContact ? state.contactRampAlpha : 0.0;
        }
    }

    for (auto& step : override.steps) {
        step.enabled = true;
        step.leftContact = leftContact;
        step.rightContact = rightContact;
        step.leftNormalForceMinScale = leftScale;
        step.rightNormalForceMinScale = rightScale;
    }

    return override;
}
