#include "ControlParameters.h"

const DMat<double>& getL() {
    static const DMat<double> L = [] {
        DMat<double> out = DMat<double>::Zero(13 * N, 13 * N);
        for (int k = 0; k < N; ++k) {
            out.block(13 * k, 13 * k, 13, 13) = Qk;
        }
        return out;
    }();

    return L;
}

const DMat<double>& getK() {
    static const DMat<double> K = [] {
        DMat<double> out = DMat<double>::Zero(12 * N, 12 * N);
        for (int k = 0; k < N; ++k) {
            out.block(12 * k, 12 * k, 12, 12) = Rk;
        }
        return out;
    }();

    return K;
}
