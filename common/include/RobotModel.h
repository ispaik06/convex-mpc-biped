#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include <cstddef>

#include "RobotParams.h"

template <typename T>
class RobotModel {
public:
    explicit RobotModel(RobotParams<T>* params) : _params(params) {}

    const RobotParams<T>& params() const { return *_params; }

    bool validate() const;

    int nq() const { return _params->nq; }
    int nv() const { return _params->nv; }
    int nu() const { return _params->nu; }

    std::size_t numLegs() const { return _params->legs.size(); }
    std::size_t numArms() const { return _params->arms.size(); }

    const std::vector<int>& legQIndices(int leg) const;
    const std::vector<int>& legQdIndices(int leg) const;
    const std::vector<int>& legActuatorIndices(int leg) const;
    const std::vector<int>& armQIndices(int arm) const;
    const std::vector<int>& armQdIndices(int arm) const;
    const std::vector<int>& armActuatorIndices(int arm) const;

    DVec<T> getLegQ(const DVec<T>& q, int leg) const;
    DVec<T> getLegQd(const DVec<T>& qd, int leg) const;
    DVec<T> getArmQ(const DVec<T>& q, int arm) const;
    DVec<T> getArmQd(const DVec<T>& qd, int arm) const;

    void setLegTau(int leg, const DVec<T>& tau_leg, DVec<T>& tau_all) const;
    void setArmTau(int arm, const DVec<T>& tau_arm, DVec<T>& tau_all) const;

    const Vec3<T>& hipLocationFromBody(int leg) const;

private:
    RobotParams<T>* _params;
};

#endif  // ROBOT_MODEL_H
