#ifndef CONTROL_PARAMETERS_H
#define CONTROL_PARAMETERS_H

#include "cppTypes.h"

using StateWeightMat = Eigen::Matrix<double, 13, 13>;
using InputWeightMat = Mat12<double>;

inline constexpr double T_cycle = 1.0;
inline constexpr double T_swing = 0.4;
inline constexpr double T_stance = 0.6;

inline constexpr double T_horizon = 0.5;
inline constexpr int N = 10;
inline constexpr double dt_mpc = T_horizon / static_cast<double>(N);

inline constexpr double mu = 0.1;

inline constexpr double a = 0.065;
inline constexpr double b = 0.01;
inline constexpr double gamma = 0.0657;

inline constexpr double Fmax = 200.0;
inline constexpr double Fmin = 10.0;

inline const StateWeightMat Qk = StateWeightMat::Identity();
inline const InputWeightMat Rk = InputWeightMat::Identity();

const DMat<double>& getL();
const DMat<double>& getK();

#endif  // CONTROL_PARAMETERS_H
