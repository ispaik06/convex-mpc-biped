#include <sstream>
#include <stdexcept>

#include "GaitScheduler.h"
#include "Utilities/Math.h"


double GaitScheduler::p(Side i, double t) {
    double phi = 0;
    if (i == Side::Left) phi = 0.5;
    return (t - _t0) / T_cycle + phi;
}

bool GaitScheduler::c(Side i, double t) {
    double phase = p(i, t);
    if(0 <= phase && phase < 0.6) return true;
    else return false;
}

void GaitScheduler::init(double t) {
    if (t - _t0 >= T_cycle) _t0 = t;
}

void GaitScheduler::buildConstraintMatrices() {
    D.resize(12*N, 12*N);
    C.resize(24*N, 12*N);
    C_bound.resize(24*N, 1);
    double tk = _t0;

    Mat3<double> S_left, S_right;
    S_left  = S_right = Mat3<double>::Zero();

    DMat<double> D_result(12*N, 12*N);
    DMat<double> C_result(24*N, 12*N);
    DVec<double> C_bound_result(24*N, 1);

    Mat12<double> C_left, C_right;
    C_left = C_right = Mat12<double>::Zero();

    for (int k = 0; k < N; ++k) {
        Ck_bound = Vec24<double>::Zero();
        if (c(Side::Left, tk) == 0) {  // swing
            S_left = Mat3<double>::Ones();
        }
        else {  // stance
            C_left << C_unit, DMat<double>::Zero(12, 6);
            Ck_bound(4) = Fmax;
            Ck_bound(5) = -Fmin;
        }

        if (c(Side::Right, tk) == 0) {  // swing
            S_right = Mat3<double>::Ones();
        }
        else {  // stance
            C_right << DMat<double>::Zero(12, 6), C_unit;
            Ck_bound(16) = Fmax;
            Ck_bound(17) = -Fmin;
        }


        DMat<double> Dk = blkdiag<double>({S_left, S_right, S_left, S_right});
        if (k == 0) D_result = Dk;
        else D_result = blkdiag<double>({D_result, Dk});

        DMat<double> Ck(24, 12);
        Ck.block(0, 0, 12, 12) = C_left;
        Ck.block(12, 0, 12, 12) = C_right;
        if (k == 0) {
            C_result = Ck;
        }
        else {
            C_result = blkdiag<double>({C_result, Ck});
        }
        C_bound_result.segment(24*k, 24) = Ck_bound;

        tk += _dt;
    }

    D = D_result;
    C = C_result;
    C_bound = C_bound_result;
    return;
}
