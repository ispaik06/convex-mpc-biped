#ifndef SWING_LEG_TEST_CONTROLLER_H
#define SWING_LEG_TEST_CONTROLLER_H

#include <vector>

#include "ControllerConfig.h"
#include "RobotController.h"
#include "StateEstimator/StateEstimator.h"
#include "SwingFootTrajectory.h"

class SwingLegTestController : public RobotController {
public:
    SwingLegTestController() = default;
    ~SwingLegTestController() override = default;

    void initializeController() override;
    void runController() override;

private:
    struct LegRuntimeState {
        SwingFootTrajectory trajectory;
        Vec3<double> centerFootPosition = Vec3<double>::Zero();
        Vec3<double> nominalFootFromCom = Vec3<double>::Zero();
    };

    static Mat3<double> makeDiagonal(double x, double y, double z);

    void initializeRuntime();
    Vec3<double> reducedBodyComWorld() const;
    Vec3<double> touchdownTargetWorld(std::size_t legIndex) const;
    void resetTrajectories();
    void configureSwingLeg(std::size_t legIndex);
    void printStatus() const;

    bool _initialized{false};
    std::vector<LegRuntimeState> _legs;
    double _lastTime{0.0};
    double _swingDuration{0.8};
    double _swingHeight{0.06};
    Mat3<double> _swingKp = Mat3<double>::Zero();
    Mat3<double> _swingKd = Mat3<double>::Zero();
    u64 _iteration{0};
};

#endif  // SWING_LEG_TEST_CONTROLLER_H
