#ifndef MATH_H
#define MATH_H

#include <Eigen/Dense>
#include <vector>
#include "cppTypes.h"

template <typename T>
DMat<T> blkdiag(const std::vector<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>>& mats)
{
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

#endif  // MATH_H