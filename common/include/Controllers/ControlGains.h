#ifndef CONTROL_GAINS_H
#define CONTROL_GAINS_H

#include <initializer_list>
#include <stdexcept>

#include "Types.h"

template <typename T>
struct JointPdGains {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DVec<T> kp;
    DVec<T> kd;

    JointPdGains() = default;

    JointPdGains(std::initializer_list<T> kp_values, std::initializer_list<T> kd_values) {
        set(kp_values, kd_values);
    }

    void set(std::initializer_list<T> kp_values, std::initializer_list<T> kd_values) {
        kp = _makeVector(kp_values);
        kd = _makeVector(kd_values);
        _validate();
    }

    void setConstant(Eigen::Index dof, T kp_value, T kd_value) {
        if (dof < 0) {
            throw std::invalid_argument("JointPdGains received negative dof");
        }

        kp = DVec<T>::Constant(dof, kp_value);
        kd = DVec<T>::Constant(dof, kd_value);
        _validate();
    }

    Eigen::Index dof() const {
        return kp.size();
    }

    bool empty() const {
        return kp.size() == 0 && kd.size() == 0;
    }

    bool validFor(Eigen::Index expected_dof) const {
        return kp.size() == expected_dof && kd.size() == expected_dof;
    }

    void applyTo(DMat<T>& kp_joint, DMat<T>& kd_joint) const {
        _validate();

        kp_joint.setZero(kp.size(), kp.size());
        kd_joint.setZero(kd.size(), kd.size());
        kp_joint.diagonal() = kp;
        kd_joint.diagonal() = kd;
    }

    template <typename Command>
    void applyTo(Command& command) const {
        if (!validFor(command.dof())) {
            throw std::invalid_argument("JointPdGains size does not match command dof");
        }

        applyTo(command.kpJoint, command.kdJoint);
    }

private:
    static DVec<T> _makeVector(std::initializer_list<T> values) {
        DVec<T> result(static_cast<Eigen::Index>(values.size()));
        Eigen::Index idx = 0;
        for (const T value : values) {
            result[idx++] = value;
        }
        return result;
    }

    void _validate() const {
        if (kp.size() != kd.size()) {
            throw std::invalid_argument("JointPdGains kp/kd size mismatch");
        }
    }
};

#endif  // CONTROL_GAINS_H
