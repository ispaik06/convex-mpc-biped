#ifndef LEG_POS_INITIALIZER_H
#define LEG_POS_INITIALIZER_H

#include <cstddef>

#include "Controllers/LegController.h"
#include "Robot/RobotModel.h"
#include "Utilities/BSplineBasicDynamic.h"

template <typename T>
class LegPosInitializer {
public:
    LegPosInitializer(const RobotParams<T>* params, T end_time, float dt);
    ~LegPosInitializer() = default;

    bool IsInitialized(LegController<T>*);

private:
    void initializeSpline(const LegController<T>& leg_ctrl);
    std::size_t totalLegDof() const;

    const RobotParams<T>* _params = nullptr;
    T _end_time{0};
    T _curr_time{0};
    T _dt{0};
    bool _splineInitialized{false};
    std::size_t _totalLegDof{0};
    BS_BasicDyn<T, 3, 1, 2, 2> _jpos_trj;
};


#endif  // LEG_POS_INITIALIZER_H
