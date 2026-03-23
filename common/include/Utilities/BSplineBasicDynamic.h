#ifndef B_SPLINE_BASIC_DYNAMIC_H
#define B_SPLINE_BASIC_DYNAMIC_H

#include <algorithm>
#include <cstdio>
#include <stdexcept>
#include <vector>

template <typename T, int DEGREE, int NUM_MIDDLE, int CONST_LEVEL_INI,
          int CONST_LEVEL_FIN>
class BS_BasicDyn {
 public:
  BS_BasicDyn() = default;
  explicit BS_BasicDyn(int dim) { setDimension(dim); }
  ~BS_BasicDyn() = default;

  void setDimension(int dim) {
    if (dim < 0) {
      throw std::invalid_argument("BS_BasicDyn received negative dimension");
    }

    Dim_ = dim;
    NumKnots_ = DEGREE + NUM_MIDDLE + 2 + CONST_LEVEL_INI + CONST_LEVEL_FIN + 1;
    NumCPs_ = NUM_MIDDLE + 2 + CONST_LEVEL_INI + CONST_LEVEL_FIN;
    Knots_.assign(static_cast<std::size_t>(NumKnots_), T(0));
    CPoints_.assign(static_cast<std::size_t>(NumCPs_ * Dim_), T(0));

    if (NumKnots_ < 2 * (DEGREE + 1)) {
      std::printf("Invalid setup (num_knots, degree): %d, %d\n", NumKnots_, DEGREE);
    }
  }

  int dim() const { return Dim_; }

  bool SetParam(T* init, T* fin, T** middle_pt, T fin_time) {
    _requireConfigured();
    _CalcKnot(fin_time);
    _CalcConstrainedCPoints(init, fin, fin_time);
    _CalcCPoints(middle_pt);
    return true;
  }

  bool getCurvePoint(T u, T* ret) {
    _requireConfigured();

    if (u < Knots_.front()) {
      u = Knots_.front();
    } else if (u > Knots_.back()) {
      u = Knots_.back();
    }

    int span = 0;
    if (!_findSpan(span, u)) return false;

    T basis[DEGREE + 1] = {};
    _BasisFuns(basis, span, u);

    for (int m = 0; m < Dim_; ++m) {
      ret[m] = T(0);
      for (int i = 0; i <= DEGREE; ++i) {
        ret[m] += basis[i] * _cp(span - DEGREE + i, m);
      }
    }

    return true;
  }

  bool getCurveDerPoint(T u, int d, T* ret) {
    _requireConfigured();

    if (d > DEGREE) return false;

    if (u < Knots_.front()) {
      u = Knots_.front();
    } else if (u > Knots_.back()) {
      u = Knots_.back();
    }

    std::vector<std::vector<T>> curve_derivs(
        static_cast<std::size_t>(d + 1), std::vector<T>(static_cast<std::size_t>(Dim_), T(0)));

    if (!_CurveDerivsAlg1V(curve_derivs, u, d)) return false;

    for (int m = 0; m < Dim_; ++m) {
      ret[m] = curve_derivs[static_cast<std::size_t>(d)][static_cast<std::size_t>(m)];
    }

    return true;
  }

 private:
  void _requireConfigured() const {
    if (Dim_ <= 0) {
      throw std::runtime_error("BS_BasicDyn dimension must be configured before use");
    }
  }

  bool _isEqual(T x, T y) const {
    const T diff = x - y;
    return diff * diff < static_cast<T>(1.e-10);
  }

  T& _cp(int cp_idx, int dim_idx) {
    return CPoints_[static_cast<std::size_t>(cp_idx * Dim_ + dim_idx)];
  }

  const T& _cp(int cp_idx, int dim_idx) const {
    return CPoints_[static_cast<std::size_t>(cp_idx * Dim_ + dim_idx)];
  }

  void _CalcKnot(T Tf) {
    int idx = 0;
    const int num_mid_knot = NumKnots_ - 2 * DEGREE - 2;
    const T time_step = Tf / static_cast<T>(num_mid_knot + 1);

    for (int j = 0; j < DEGREE + 1; ++j) Knots_[static_cast<std::size_t>(idx++)] = T(0);

    for (int j = 0; j < num_mid_knot; ++j) {
      Knots_[static_cast<std::size_t>(idx)] = Knots_[static_cast<std::size_t>(idx - 1)] + time_step;
      ++idx;
    }

    for (int j = 0; j < DEGREE + 1; ++j) Knots_[static_cast<std::size_t>(idx++)] = Tf;
  }

  bool _CurveDerivsAlg1V(std::vector<std::vector<T>>& CK, T u, int d) {
    int span = 0;
    if (!_findSpan(span, u)) return false;

    std::vector<std::vector<T>> nders(
        static_cast<std::size_t>(d + 1),
        std::vector<T>(static_cast<std::size_t>(DEGREE + 1), T(0)));

    _BasisFunsDers(nders, span, u, d);

    for (int k = 0; k <= d; ++k) {
      std::fill(CK[static_cast<std::size_t>(k)].begin(), CK[static_cast<std::size_t>(k)].end(),
                T(0));

      for (int j = 0; j <= DEGREE; ++j) {
        for (int m = 0; m < Dim_; ++m) {
          CK[static_cast<std::size_t>(k)][static_cast<std::size_t>(m)] +=
              nders[static_cast<std::size_t>(k)][static_cast<std::size_t>(j)] *
              _cp(span - DEGREE + j, m);
        }
      }
    }

    return true;
  }

  bool _BasisFunsDers(std::vector<std::vector<T>>& ders, T u, int n) {
    int span = 0;
    if (!_findSpan(span, u)) return false;

    _BasisFunsDers(ders, span, u, n);
    return true;
  }

  bool _BasisFunsDers(std::vector<std::vector<T>>& ders, int span, T u, int n) {
    std::vector<std::vector<T>> ndu(static_cast<std::size_t>(DEGREE + 1),
                                    std::vector<T>(static_cast<std::size_t>(DEGREE + 1), T(0)));
    std::vector<std::vector<T>> a(2, std::vector<T>(static_cast<std::size_t>(DEGREE + 1), T(0)));

    ndu[0][0] = T(1);
    for (int j = 1; j <= DEGREE; ++j) {
      T saved = T(0);
      for (int r = 0; r < j; ++r) {
        const T left = _Left(span, j - r, u);
        const T right = _Right(span, r + 1, u);

        ndu[static_cast<std::size_t>(j)][static_cast<std::size_t>(r)] = right + left;
        const T temp =
            ndu[static_cast<std::size_t>(r)][static_cast<std::size_t>(j - 1)] /
            ndu[static_cast<std::size_t>(j)][static_cast<std::size_t>(r)];

        ndu[static_cast<std::size_t>(r)][static_cast<std::size_t>(j)] = saved + right * temp;
        saved = left * temp;
      }
      ndu[static_cast<std::size_t>(j)][static_cast<std::size_t>(j)] = saved;
    }

    for (int j = 0; j <= DEGREE; ++j) {
      ders[0][static_cast<std::size_t>(j)] = ndu[static_cast<std::size_t>(j)][static_cast<std::size_t>(DEGREE)];
    }

    for (int r = 0; r <= DEGREE; ++r) {
      int s1 = 0;
      int s2 = 1;
      a[0][0] = T(1);

      for (int k = 1; k <= n; ++k) {
        T dval = T(0);
        const int rk = r - k;
        const int pk = DEGREE - k;

        if (r >= k) {
          a[static_cast<std::size_t>(s2)][0] =
              a[static_cast<std::size_t>(s1)][0] /
              ndu[static_cast<std::size_t>(pk + 1)][static_cast<std::size_t>(rk)];
          dval = a[static_cast<std::size_t>(s2)][0] *
                 ndu[static_cast<std::size_t>(rk)][static_cast<std::size_t>(pk)];
        }

        const int j1 = (rk >= -1) ? 1 : -rk;
        const int j2 = (r - 1 <= pk) ? (k - 1) : (DEGREE - r);

        for (int j = j1; j <= j2; ++j) {
          a[static_cast<std::size_t>(s2)][static_cast<std::size_t>(j)] =
              (a[static_cast<std::size_t>(s1)][static_cast<std::size_t>(j)] -
               a[static_cast<std::size_t>(s1)][static_cast<std::size_t>(j - 1)]) /
              ndu[static_cast<std::size_t>(pk + 1)][static_cast<std::size_t>(rk + j)];
          dval += a[static_cast<std::size_t>(s2)][static_cast<std::size_t>(j)] *
                  ndu[static_cast<std::size_t>(rk + j)][static_cast<std::size_t>(pk)];
        }

        if (r <= pk) {
          a[static_cast<std::size_t>(s2)][static_cast<std::size_t>(k)] =
              -a[static_cast<std::size_t>(s1)][static_cast<std::size_t>(k - 1)] /
              ndu[static_cast<std::size_t>(pk + 1)][static_cast<std::size_t>(r)];
          dval += a[static_cast<std::size_t>(s2)][static_cast<std::size_t>(k)] *
                  ndu[static_cast<std::size_t>(r)][static_cast<std::size_t>(pk)];
        }

        ders[static_cast<std::size_t>(k)][static_cast<std::size_t>(r)] = dval;
        std::swap(s1, s2);
      }
    }

    int r = DEGREE;
    for (int k = 1; k <= n; ++k) {
      for (int j = 0; j <= DEGREE; ++j) {
        ders[static_cast<std::size_t>(k)][static_cast<std::size_t>(j)] *= static_cast<T>(r);
      }
      r *= (DEGREE - k);
    }

    return true;
  }

  void _BasisFuns(T* N, int span, T u) {
    N[0] = T(1);
    for (int j = 1; j <= DEGREE; ++j) {
      T saved = T(0);
      T temp = T(0);
      for (int r = 0; r < j; ++r) {
        const T left = _Left(span, j - r, u);
        const T right = _Right(span, r + 1, u);

        if ((right + left) != T(0)) {
          temp = N[r] / (right + left);
        }

        N[r] = saved + right * temp;
        saved = left * temp;
      }
      N[j] = saved;
    }
  }

  T _Left(int i, int j, T u) const { return u - Knots_[static_cast<std::size_t>(i + 1 - j)]; }

  T _Right(int i, int j, T u) const { return Knots_[static_cast<std::size_t>(i + j)] - u; }

  bool _findSpan(int& ret, T u) const {
    if (u < Knots_.front() || Knots_.back() < u) return false;

    if (_isEqual(u, Knots_.back())) {
      for (int i = NumKnots_ - 2; i > -1; --i) {
        if (Knots_[static_cast<std::size_t>(i)] < u &&
            u <= Knots_[static_cast<std::size_t>(i + 1)]) {
          ret = i;
          return true;
        }
      }
      return false;
    }

    int low = 0;
    int high = NumKnots_ - 1;
    int mid = (low + high) >> 1;

    while (u < Knots_[static_cast<std::size_t>(mid)] ||
           u >= Knots_[static_cast<std::size_t>(mid + 1)]) {
      if (u < Knots_[static_cast<std::size_t>(mid)]) {
        high = mid;
      } else {
        low = mid;
      }
      mid = (low + high) >> 1;
    }

    ret = mid;
    return true;
  }

  void _CalcConstrainedCPoints(T* init, T* fin, T Tf) {
    for (int m = 0; m < Dim_; ++m) {
      _cp(0, m) = init[m];
      _cp(NumCPs_ - 1, m) = fin[m];
    }

    std::vector<std::vector<T>> d_mat(
        static_cast<std::size_t>(CONST_LEVEL_INI + 1),
        std::vector<T>(static_cast<std::size_t>(DEGREE + 1), T(0)));
    _BasisFunsDers(d_mat, T(0), CONST_LEVEL_INI);

    std::vector<T> tmp(static_cast<std::size_t>(Dim_), T(0));
    for (int j = 1; j < CONST_LEVEL_INI + 1; ++j) {
      for (int k = 0; k < Dim_; ++k) {
        tmp[static_cast<std::size_t>(k)] = init[j * Dim_ + k];

        for (int h = j; h > 0; --h) {
          tmp[static_cast<std::size_t>(k)] -=
              d_mat[static_cast<std::size_t>(j)][static_cast<std::size_t>(h - 1)] *
              _cp(h - 1, k);
        }
        _cp(j, k) = tmp[static_cast<std::size_t>(k)] /
                    d_mat[static_cast<std::size_t>(j)][static_cast<std::size_t>(j)];
      }
    }

    std::vector<std::vector<T>> c_mat(
        static_cast<std::size_t>(CONST_LEVEL_FIN + 1),
        std::vector<T>(static_cast<std::size_t>(DEGREE + 1), T(0)));
    _BasisFunsDers(c_mat, Tf, CONST_LEVEL_FIN);

    int idx = 1;
    for (int j = NumCPs_ - 2; j > NumCPs_ - 2 - CONST_LEVEL_FIN; --j) {
      for (int k = 0; k < Dim_; ++k) {
        tmp[static_cast<std::size_t>(k)] = fin[idx * Dim_ + k];

        for (int h = idx; h > 0; --h) {
          tmp[static_cast<std::size_t>(k)] -=
              c_mat[static_cast<std::size_t>(idx)]
                   [static_cast<std::size_t>(CONST_LEVEL_FIN + 2 - h)] *
              _cp(NumCPs_ - h, k);
        }
        _cp(j, k) = tmp[static_cast<std::size_t>(k)] /
                    c_mat[static_cast<std::size_t>(idx)]
                         [static_cast<std::size_t>(CONST_LEVEL_FIN + 1 - idx)];
      }
      ++idx;
    }
  }

  void _CalcCPoints(T** middle_pt) {
    for (int i = 0; i < NUM_MIDDLE; ++i) {
      for (int m = 0; m < Dim_; ++m) {
        _cp(CONST_LEVEL_INI + 1 + i, m) = middle_pt[i][m];
      }
    }
  }

  int Dim_{0};
  int NumKnots_{0};
  int NumCPs_{0};
  std::vector<T> Knots_;
  std::vector<T> CPoints_;
};

#endif  // B_SPLINE_BASIC_DYNAMIC_H
