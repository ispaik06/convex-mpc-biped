#ifndef ARM_POS_INITIALIZER_H
#define ARM_POS_INITIALIZER_H

#include <cstddef>

#include "Controllers/ArmController.h"
#include "Robot/RobotModel.h"
#include "Utilities/BSplineBasicDynamic.h"

template <typename T>
class ArmPosInitializer {
public:
    ArmPosInitializer(const RobotParams<T>* params, T end_time, float dt);
    ~ArmPosInitializer() = default;

    bool IsInitialized(ArmController<T>*);

private:
    void initializeSpline(const ArmController<T>& arm_ctrl);
    std::size_t totalArmDof() const;

    const RobotParams<T>* _params = nullptr;
    T _end_time{0};
    T _curr_time{0};
    T _dt{0};
    bool _splineInitialized{false};
    std::size_t _totalArmDof{0};
    BS_BasicDyn<T, 3, 1, 2, 2> _jpos_trj;
};

#endif  // ARM_POS_INITIALIZER_H
