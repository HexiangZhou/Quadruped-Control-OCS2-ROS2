#pragma once

#include <ocs2_core/Types.h>

#include <utility>

class Task {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Task() = default;

  Task(ocs2::matrix_t a, ocs2::vector_t b, ocs2::matrix_t d, ocs2::vector_t f) : a_(std::move(a)), d_(std::move(d)), b_(std::move(b)), f_(std::move(f)) {}

  explicit Task(size_t numDecisionVars)
      : Task(ocs2::matrix_t::Zero(0, numDecisionVars), ocs2::vector_t::Zero(0), ocs2::matrix_t::Zero(0, numDecisionVars), ocs2::vector_t::Zero(0)) {}

  Task operator+(const Task& rhs) const {
    return {concatenateMatrices(a_, rhs.a_), concatenateVectors(b_, rhs.b_), concatenateMatrices(d_, rhs.d_),
            concatenateVectors(f_, rhs.f_)};
  }

  Task operator*(ocs2::scalar_t rhs) const {  // clang-format off
    return {a_.cols() > 0 ? rhs * a_ : a_,
            b_.cols() > 0 ? rhs * b_ : b_,
            d_.cols() > 0 ? rhs * d_ : d_,
            f_.cols() > 0 ? rhs * f_ : f_};  // clang-format on
  }

  ocs2::matrix_t a_, d_;
  ocs2::vector_t b_, f_;

  static ocs2::matrix_t concatenateMatrices(ocs2::matrix_t m1, ocs2::matrix_t m2) {
    if (m1.cols() <= 0) {
      return m2;
    } else if (m2.cols() <= 0) {
      return m1;
    }
    assert(m1.cols() == m2.cols());
    ocs2::matrix_t res(m1.rows() + m2.rows(), m1.cols());
    res << m1, m2;
    return res;
  }

  static ocs2::vector_t concatenateVectors(const ocs2::vector_t& v1, const ocs2::vector_t& v2) {
    if (v1.cols() <= 0) {
      return v2;
    } else if (v2.cols() <= 0) {
      return v1;
    }
    assert(v1.cols() == v2.cols());
    ocs2::vector_t res(v1.rows() + v2.rows());
    res << v1, v2;
    return res;
  }
};

