#ifndef MATRIX_UTILS_H
#define MATRIX_UTILS_H

#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include "cppTypes.h"

template <typename T>
Mat3<T> Rz(const T psi) {
    const T cpsi = std::cos(psi);
    const T spsi = std::sin(psi);

    Mat3<T> R = Mat3<T>::Zero();
    R << cpsi, -spsi, T(0),
         spsi,  cpsi, T(0),
         T(0),  T(0), T(1);
    return R;
}

template <typename T>
Mat3<T> skew(const Vec3<T>& v) {
    Mat3<T> S = Mat3<T>::Zero();
    S << T(0), -v.z(),  v.y(),
         v.z(),  T(0), -v.x(),
        -v.y(),  v.x(),  T(0);
    return S;
}

template <typename T>
DMat<T> blkdiag(const std::vector<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>>& mats) {
    using Mat = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

    int total_rows = 0;
    int total_cols = 0;

    for (const auto& m : mats) {
        total_rows += m.rows();
        total_cols += m.cols();
    }

    Mat result = Mat::Zero(total_rows, total_cols);

    int r = 0;
    int c = 0;

    for (const auto& m : mats) {
        result.block(r, c, m.rows(), m.cols()) = m;
        r += m.rows();
        c += m.cols();
    }

    return result;
}

#endif  // MATRIX_UTILS_H
